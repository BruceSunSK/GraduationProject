#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <Eigen/Core>

#include "global_planning/planners/TSHAstar.h"
#include "global_planning/planners/astar.h"
#include "global_planning/planners/rrt.h"
#include "global_planning/planners/rrtstar.h"
#include "global_planning/planners/genetic_algorithm.h"


struct PlannerMetrics
{
    std::string name = "";

    double path_length = 0.0;                   // 路径长度，单位为米
    size_t node_nums = 0.0;                     // 节点个数
    double average_curvature = 0.0;             // 平均曲率，单位1/m
    double max_curvature = 0.0;                 // 最大曲率，单位1/m
    double min_collision_distance
        = std::numeric_limits<double>::max();   // 最小碰撞距离，单位米
    double planning_time = 0.0;                 // 规划时间，单位ms

    std::vector<double> s_list;
    std::vector<double> curvature_list;
    std::vector<double> collision_distance_list;
};


std::vector<ros::Publisher> processed_map_pubs;
std::vector<ros::Publisher> path_pubs;
std::vector<ros::Publisher> auxiliary_pubs;
std::vector<GlobalPlannerInterface *> planners;
std::vector<std::string> planners_names;
double res = 0.0;
double ori_x = 0.0;
double ori_y = 0.0;
bool start_point_flag = false;
bool end_point_flag = false;
bool map_flag = false;
Map::DistanceMap collision_check_map_;


// 使用最小二乘法拟合二次曲线 y = ax² + bx + c
void fitQuadraticCurve(const std::vector<cv::Point2d> & points,
    double & a, double & b, double & c)
{
    const int n = points.size();
    Eigen::MatrixXd X(n, 3);
    Eigen::VectorXd Y(n);

    for (int i = 0; i < n; ++i)
    {
        const double xi = points[i].x;
        X(i, 0) = xi * xi;
        X(i, 1) = xi;
        X(i, 2) = 1;
        Y(i) = points[i].y;
    }

    Eigen::Vector3d coeff = X.jacobiSvd(Eigen::ComputeThinU | Eigen::ComputeThinV).solve(Y);
    a = coeff[0];
    b = coeff[1];
    c = coeff[2];
}

// 计算曲率 κ = |y''| / (1 + y'²)^(3/2)
double calculateCurvature(double x, double a, double b)
{
    const double y_prime = 2 * a * x + b;       // 一阶导数 dy/dx
    const double y_double_prime = 2 * a;      // 二阶导数 d²y/dx²
    return std::abs(y_double_prime) / std::pow(1 + y_prime * y_prime, 1.5);
}

// 主计算函数（滑动窗口处理）
std::vector<double> computeCurvatures(const std::vector<cv::Point2d> & path, int window_size = 5)
{
    std::vector<double> curvatures;
    const int n = path.size();

    for (int i = 0; i < n; ++i)
    {
        // 确定滑动窗口范围
        int start = std::max(0, i - window_size / 2);
        int end = std::min(n - 1, i + window_size / 2);
        if (end - start + 1 < 3)
        { // 边界处理
            curvatures.push_back(0);
            continue;
        }

        // 提取局部点集
        std::vector<cv::Point2d> local_points;
        for (int j = start; j <= end; ++j)
        {
            local_points.push_back(path[j]);
        }

        // 曲线拟合与曲率计算
        double a, b, c;
        fitQuadraticCurve(local_points, a, b, c);
        curvatures.push_back(calculateCurvature(path[i].x, a, b));
    }

    return curvatures;
}

PlannerMetrics calc_metrics(const std::string & planner_name, const GlobalPlannerInterface * const planner,
                            const std::vector<cv::Point2d> & path)
{
    std::vector<cv::Point2d> real_path;
    PlannerMetrics metrics;
    if (planner_name == "TSHAstar_RRP")
    {
        const TSHAstar::TSHAstarHelper * tsha_helper =
            dynamic_cast<const TSHAstar::TSHAstarHelper *>(planner->getAllInfo());
        real_path = tsha_helper->search.path_simplification.reduced_path;

        metrics.name = planner_name;
        metrics.node_nums = tsha_helper->search.path_simplification.reduced_path_size;
        metrics.planning_time = tsha_helper->search.path_search.cost_time
            + tsha_helper->search.path_simplification.cost_time;
    }
    else if (planner_name == "TSHAstar_BS")
    {
        const TSHAstar::TSHAstarHelper * tsha_helper =
            dynamic_cast<const TSHAstar::TSHAstarHelper *>(planner->getAllInfo());
        real_path = tsha_helper->search.path_smooth.smooth_path;

        metrics.name = planner_name;
        metrics.node_nums = tsha_helper->search.path_smooth.smooth_path_size;
        metrics.planning_time = tsha_helper->search.path_search.cost_time
            + tsha_helper->search.path_simplification.cost_time
            + tsha_helper->search.path_smooth.cost_time;
    }
    else if (planner_name == "TSHAstar_OP")
    {
        const TSHAstar::TSHAstarHelper * tsha_helper =
            dynamic_cast<const TSHAstar::TSHAstarHelper *>(planner->getAllInfo());
        real_path = tsha_helper->search.path_optimization.optimized_path;

        metrics.name = planner_name;
        metrics.node_nums = tsha_helper->search.path_optimization.optimized_path_size;
        metrics.planning_time = tsha_helper->search.path_search.cost_time
            + tsha_helper->search.path_simplification.cost_time
            + tsha_helper->search.path_smooth.cost_time
            + tsha_helper->search.path_optimization.cost_time;
    }
    else if (planner_name == "TSHAstar")
    {
        const TSHAstar::TSHAstarHelper * tsha_helper =
            dynamic_cast<const TSHAstar::TSHAstarHelper *>(planner->getAllInfo());
        real_path = path;
        
        metrics.name = planner_name;
        metrics.node_nums = tsha_helper->search.path_optimization.optimized_path_size;
        metrics.planning_time = tsha_helper->search.path_search.cost_time
            + tsha_helper->search.path_simplification.cost_time
            + tsha_helper->search.path_smooth.cost_time
            + tsha_helper->search.path_optimization.cost_time
            + tsha_helper->sample.path_sample.cost_time
            + tsha_helper->sample.path_dp.total_cost_time
            + tsha_helper->sample.path_qp.cost_time;
    }
    else if (planner_name == "Astar")
    {
        const Astar::AstarHelper * astar_helper =
            dynamic_cast<const Astar::AstarHelper *>(planner->getAllInfo());
        real_path = path;

        metrics.name = planner_name;
        metrics.node_nums = astar_helper->search.node_nums;
        metrics.planning_time = astar_helper->search.cost_time;
    }
    else if (planner_name == "RRT")
    {
        const RRT::RRTHelper * rrt_helper =
            dynamic_cast<const RRT::RRTHelper *>(planner->getAllInfo());
        real_path = path;

        metrics.name = planner_name;
        metrics.node_nums = rrt_helper->sample.node_nums;
        metrics.planning_time = rrt_helper->sample.cost_time;
    }
    else if (planner_name == "RRTstar")
    {
        const RRTstar::RRTstarHelper * rrtstar_helper =
            dynamic_cast<const RRTstar::RRTstarHelper *>(planner->getAllInfo());
        real_path = path;
        
        metrics.name = planner_name;
        metrics.node_nums = rrtstar_helper->sample.node_nums;
        metrics.planning_time = rrtstar_helper->sample.cost_time;
    }

    // 根据路径信息计算路径长度、曲率、最近碰撞距离
    std::vector<double> s_list(real_path.size(), 0.0);
    std::vector<double> curvature_list(real_path.size(), 0.0);
    std::vector<double> collision_distance_list(real_path.size(), 0.0);

    double s = 0.0;
    for (size_t i = 1; i < real_path.size(); i++)
    {
        const double dx = real_path[i].x - real_path[i - 1].x;
        const double dy = real_path[i].y - real_path[i - 1].y;
        const double d = std::sqrt(dx * dx + dy * dy);
        s += d;
        s_list[i] = s;
    }
    metrics.path_length = s;
    metrics.s_list = std::move(s_list);

    for (size_t i = 0; i < real_path.size(); i++)
    {
        const cv::Point2d & point = real_path[i];
        const cv::Point2d point_grid((point.x - ori_x) / res, (point.y - ori_y) / res);
        const double dis = collision_check_map_.IsInside(point_grid) ?
            collision_check_map_.GetDistance(point_grid) * res : 0.0;
        collision_distance_list[i] = dis;
        metrics.min_collision_distance = std::min(metrics.min_collision_distance, dis);
    }
    metrics.collision_distance_list = std::move(collision_distance_list);

    double total_curvature = 0.0;
    for (int i = 1; i < real_path.size() - 1; ++i)
    {
        const cv::Point2d & A = real_path[i - 1];
        const cv::Point2d & B = real_path[i];
        const cv::Point2d & C = real_path[i + 1];

        // 计算三边长度
        double ab = cv::norm(B - A);
        double bc = cv::norm(C - B);
        double ac = cv::norm(C - A);

        // 计算叉积以确定三角形面积
        double cross = (B.x - A.x) * (C.y - A.y) - (B.y - A.y) * (C.x - A.x);
        double delta = 0.5 * std::abs(cross);
        double denominator = ab * bc * ac;

        double curvature = 0.0;
        // 避免除以零
        if (denominator > 1e-10)
        {
            curvature = 4.0 * delta / denominator;
        }

        curvature_list[i] = curvature;
        total_curvature += curvature;
        metrics.max_curvature = std::max(metrics.max_curvature, curvature);
    }
    curvature_list[0] = curvature_list[1];
    curvature_list[real_path.size() - 1] = curvature_list[real_path.size() - 2];
    metrics.average_curvature = total_curvature / (real_path.size() - 2);
    metrics.curvature_list = std::move(curvature_list);

    // metrics.curvature_list = computeCurvatures(real_path);
    // double total_curv = 0.0;
    // double max_curv = 0.0;
    // for (size_t i = 0; i < metrics.curvature_list.size(); i++)
    // {
    //     total_curv += metrics.curvature_list[i];
    //     max_curv = std::max(max_curv, metrics.curvature_list[i]);
    // }
    // metrics.average_curvature = total_curv / metrics.curvature_list.size();
    // metrics.max_curvature = max_curv;

    return metrics;
}

void write_metrics(const std::string & dir_name, const std::vector<PlannerMetrics> & metrics)
{
    // 保存所有路径的数据指标
    const std::string metrics_filename = dir_name + "/all_planner_metrics.csv";
    std::ofstream file(metrics_filename, std::ios::trunc);
    file << "Planner,Path_Length,Node_Nums,Average_Curvature,Max_Curvature,Min_Collision_Distance,Planning_time\n";
    for (const auto & planner : metrics)
    {
        file << planner.name << ","
             << planner.path_length << ","
             << planner.node_nums << ","
             << planner.average_curvature << ","
             << planner.max_curvature << ","
             << planner.min_collision_distance << ","
             << planner.planning_time << "\n";
    }
    file.close();

    // 保存用于绘制图像的路径各个点的值
    for (const auto & planner : metrics)
    {
        const std::string planner_filename = dir_name + "/" + planner.name + ".csv";
        std::ofstream planner_file(planner_filename, std::ios::trunc);
        planner_file << "S,Curvature,Collision_Distance\n";
        for (size_t i = 0; i < planner.s_list.size(); i++)
        {
            planner_file << planner.s_list[i] << ","
                         << planner.curvature_list[i] << ","
                         << planner.collision_distance_list[i] << "\n";
        }
        planner_file.close();
    }
}

void load_map(const std::string & map_path, const double resulution)
{
    cv::Mat map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    int rows = map.rows;
    int cols = map.cols;
    res = resulution;
    ori_x = 0.0;
    ori_y = 0.0;

    for (size_t i = 0; i < planners.size(); i++)
    {
        GlobalPlannerInterface * const planner = planners[i];
        
        planner->setMapInfo(res, ori_x, ori_y);
        map_flag = planner->setMap(map);
        if (map_flag)
        {
            cv::Mat processed_map;
            planner->getProcessedMap(processed_map);
            // 以下这部分代码是从TSHAstar中复制过来的，目的是为了得到distanceTransform之后的图，用以给所有不同的planner的离散点确定最近碰撞距离。
            if (i == 0)
            {
                // 1. 离散代价值的障碍物地图，正常范围在[0, 100]，也包括255未探明的情况
                // 2. 代价值二值化后的障碍物地图，未探明情况视为障碍物。即[0, OBSTACLE_THRESHOLD)值为255，白色，前景；[OBSTACLE_THRESHOLD, 255]值为0，黑色，背景。
                cv::Mat binary_map_;
                cv::threshold(processed_map, binary_map_, 99, 255, cv::ThresholdTypes::THRESH_BINARY_INV);
                // 3. 在二值化地图中进行distanceTransform变换的结果，用于描述每个栅格到障碍物的距离
                cv::Mat distance_map_;
                cv::distanceTransform(binary_map_, distance_map_, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32FC1);
                // 4. 设置采样时所用的地图
                collision_check_map_.SetMap(distance_map_);
            }

            nav_msgs::OccupancyGrid msg;
            msg.header.frame_id = "/map";
            msg.header.stamp = ros::Time::now();
            msg.info.height = rows;
            msg.info.width = cols;
            msg.info.origin.position.x = ori_x;
            msg.info.origin.position.y = ori_y;
            msg.info.resolution = res;
            msg.data.resize(rows * cols);
            for (size_t i = 0; i < rows; i++)
            {
                for (size_t j = 0; j < cols; j++)
                {
                    msg.data[i * cols + j] = processed_map.at<uchar>(i, j);
                }
            }
            processed_map_pubs[i].publish(msg);
        }
    }
}

void pub_path()
{
    std::vector<PlannerMetrics> all_metrics;    // 保存所有planner的实验信息

    for (size_t i = 0; i < planners.size(); i++)
    {
        GlobalPlannerInterface * const planner = planners[i];
        const std::string & planner_name = planners_names[i];
        
        /// 1st 绘制路径信息
        std::vector<cv::Point2d> path;
        std::vector<std::vector<cv::Point2d>> auxiliary_info;
        if (planner->getPath(path, auxiliary_info) == false)
        {
            return;
        }

        // 规划路径信息
        nav_msgs::Path path_msg;
        path_msg.header.frame_id = "map";
        path_msg.header.stamp = ros::Time::now();
        for (cv::Point2d & p : path)
        {
            geometry_msgs::PoseStamped m;
            m.header = path_msg.header;
            m.pose.position.x = p.x;
            m.pose.position.y = p.y;
            m.pose.position.z = 10;
            path_msg.poses.push_back(m);
        }

        /// 2nd 绘制辅助信息 && 3rd 保存实验数据到本地（总共5种内容。路径长度、节点数量、路径平均曲率、最近碰撞距离和计算时间）
        // 实验信息
        PlannerMetrics metrics = calc_metrics(planner_name, planner, path);
        all_metrics.push_back(metrics);
        // 辅助信息
        visualization_msgs::MarkerArray marker_array;
        visualization_msgs::Marker clean_marker;
        clean_marker.header = path_msg.header;
        clean_marker.action = visualization_msgs::Marker::DELETEALL;
        marker_array.markers.push_back(std::move(clean_marker));
        if (planner_name == "TSHAstar")
        {
            // 用于显示
            visualization_msgs::Marker line_marker;
            line_marker.header = path_msg.header;
            line_marker.type = visualization_msgs::Marker::LINE_STRIP;
            line_marker.action = visualization_msgs::Marker::ADD;
            line_marker.pose.orientation.x = 0.0;
            line_marker.pose.orientation.y = 0.0;
            line_marker.pose.orientation.z = 0.0;
            line_marker.pose.orientation.w = 1.0;
            visualization_msgs::Marker points_marker;
            points_marker.header = path_msg.header;
            points_marker.type = visualization_msgs::Marker::CUBE_LIST;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.pose.orientation.x = 0.0;
            points_marker.pose.orientation.y = 0.0;
            points_marker.pose.orientation.z = 0.0;
            points_marker.pose.orientation.w = 1.0;


            // 0. 规划原始节点
            points_marker.ns = "search_raw_nodes";
            points_marker.id = 0;
            points_marker.scale.x = 0.35;
            points_marker.scale.y = 0.35;
            points_marker.scale.z = 0.35;
            points_marker.color.a = 0.9;
            points_marker.color.r = 0.0 / 255.0;
            points_marker.color.g = 0.0 / 255.0;
            points_marker.color.b = 255.0 / 255.0;
            points_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[0].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[0][i].x;
                p.y = auxiliary_info[0][i].y;
                p.z = 1;
                points_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(points_marker);


            // 1. 规划扩展节点
            points_marker.ns = "search_expanded_nodes";
            points_marker.id = 1;
            points_marker.scale.x = 0.2;
            points_marker.scale.y = 0.2;
            points_marker.scale.z = 0.2;
            points_marker.color.a = 1.0;
            points_marker.color.r = 65.0 / 255.0;
            points_marker.color.g = 105.0 / 255.0;
            points_marker.color.b = 255.0 / 255.0;
            points_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[1].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[1][i].x;
                p.y = auxiliary_info[1][i].y;
                p.z = 0;
                points_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(points_marker);


            // 2. 去除冗余点后的关键节点
            points_marker.ns = "search_key_nodes";
            points_marker.id = 2;
            points_marker.scale.x = 0.6;
            points_marker.scale.y = 0.6;
            points_marker.scale.z = 0.6;
            points_marker.color.a = 1.0;
            points_marker.color.r = 255.0 / 255.0;
            points_marker.color.g = 215.0 / 255.0;
            points_marker.color.b = 0.0 / 255.0;
            points_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[2].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[2][i].x;
                p.y = auxiliary_info[2][i].y;
                p.z = 2;
                points_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(points_marker);

            // 3. 进行曲线平滑后的结果
            line_marker.ns = "search_bspline_smooth_path";
            line_marker.id = 3;
            line_marker.scale.x = 0.3;
            line_marker.scale.y = 0.3;
            line_marker.scale.z = 0.3;
            line_marker.color.a = 0.8;
            line_marker.color.r = 255.0 / 255.0;
            line_marker.color.g = 165.0 / 255.0;
            line_marker.color.b = 0.0 / 255.0;
            line_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[3].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[3][i].x;
                p.y = auxiliary_info[3][i].y;
                p.z = 3;
                line_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(line_marker);


            // 4. 平滑后曲线均匀采样点
            points_marker.ns = "search_s_sample_points";
            points_marker.id = 4;
            points_marker.scale.x = 0.3;
            points_marker.scale.y = 0.3;
            points_marker.scale.z = 0.3;
            points_marker.color.a = 1.0;
            points_marker.color.r = 255.0 / 255.0;
            points_marker.color.g = 255.0 / 255.0;
            points_marker.color.b = 0.0 / 255.0;
            points_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[4].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[4][i].x;
                p.y = auxiliary_info[4][i].y;
                p.z = 4;
                points_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(points_marker);


            // 5. 进行数值优化平滑后的结果
            line_marker.ns = "search_optimized_path";
            line_marker.id = 5;
            line_marker.scale.x = 0.3;
            line_marker.scale.y = 0.3;
            line_marker.scale.z = 0.3;
            line_marker.color.a = 0.8;
            line_marker.color.r = 238.0 / 255.0;
            line_marker.color.g = 44.0 / 255.0;
            line_marker.color.b = 44.0 / 255.0;
            line_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[5].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[5][i].x;
                p.y = auxiliary_info[5][i].y;
                p.z = 5;
                line_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(line_marker);


            // 6. dp后的路径和上下边界
            // 6.1 dp路径
            line_marker.ns = "sample_dp_path";
            line_marker.id = 6;
            line_marker.scale.x = 0.3;
            line_marker.scale.y = 0.3;
            line_marker.scale.z = 0.3;
            line_marker.color.a = 0.8;
            line_marker.color.r = 54.0 / 255.0;
            line_marker.color.g = 54.0 / 255.0;
            line_marker.color.b = 54.0 / 255.0;
            line_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[6].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[6][i].x;
                p.y = auxiliary_info[6][i].y;
                p.z = 6;
                line_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(line_marker);
            // 6.2 dp得到的下边界
            line_marker.ns = "sample_dp_lower_bound";
            line_marker.id = 7;
            line_marker.scale.x = 0.4;
            line_marker.scale.y = 0.4;
            line_marker.scale.z = 0.4;
            line_marker.color.a = 0.9;
            line_marker.color.r = 47.0 / 255.0;
            line_marker.color.g = 79.0 / 255.0;
            line_marker.color.b = 79.0 / 255.0;
            line_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[7].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[7][i].x;
                p.y = auxiliary_info[7][i].y;
                p.z = 7;
                line_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(line_marker);
            // 6.3 dp得到的上边界
            line_marker.ns = "sample_dp_upper_bound";
            line_marker.id = 8;
            line_marker.scale.x = 0.4;
            line_marker.scale.y = 0.4;
            line_marker.scale.z = 0.4;
            line_marker.color.a = 0.9;
            line_marker.color.r = 47.0 / 255.0;
            line_marker.color.g = 79.0 / 255.0;
            line_marker.color.b = 79.0 / 255.0;
            line_marker.points.clear();
            for (size_t i = 0; i < auxiliary_info[8].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[8][i].x;
                p.y = auxiliary_info[8][i].y;
                p.z = 7;
                line_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(line_marker);
        }
        else if (planner_name == "Astar")
        {
            // 用于显示
            visualization_msgs::Marker points_marker;
            points_marker.header = path_msg.header;
            points_marker.type = visualization_msgs::Marker::CUBE_LIST;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.pose.orientation.x = 0.0;
            points_marker.pose.orientation.y = 0.0;
            points_marker.pose.orientation.z = 0.0;
            points_marker.pose.orientation.w = 1.0;


            // 0. 规划扩展节点
            points_marker.ns = "expanded_nodes";
            points_marker.id = 0;
            points_marker.scale.x = 0.2;
            points_marker.scale.y = 0.2;
            points_marker.scale.z = 0.2;
            points_marker.color.a = 0.7;
            points_marker.color.r = 65.0 / 255.0;
            points_marker.color.g = 105.0 / 255.0;
            points_marker.color.b = 255.0 / 255.0;
            for (size_t i = 0; i < auxiliary_info[0].size(); i++)
            {
                geometry_msgs::Point p;
                p.x = auxiliary_info[0][i].x;
                p.y = auxiliary_info[0][i].y;
                p.z = 0;
                points_marker.points.push_back(std::move(p));
            }
            marker_array.markers.push_back(points_marker);
        }
        else if (planner_name == "RRT" || planner_name == "RRTstar")
        {
            // 用于显示
            visualization_msgs::Marker lines_marker;
            lines_marker.header = path_msg.header;
            lines_marker.type = visualization_msgs::Marker::LINE_LIST;
            lines_marker.action = visualization_msgs::Marker::ADD;
            lines_marker.pose.orientation.x = 0.0;
            lines_marker.pose.orientation.y = 0.0;
            lines_marker.pose.orientation.z = 0.0;
            lines_marker.pose.orientation.w = 1.0;
            visualization_msgs::Marker points_marker;
            points_marker.header = path_msg.header;
            points_marker.type = visualization_msgs::Marker::CUBE_LIST;
            points_marker.action = visualization_msgs::Marker::ADD;
            points_marker.pose.orientation.x = 0.0;
            points_marker.pose.orientation.y = 0.0;
            points_marker.pose.orientation.z = 0.0;
            points_marker.pose.orientation.w = 1.0;


            // 0. rrt树上所有节点
            lines_marker.ns = "tree_lines";
            lines_marker.id = 0;
            lines_marker.scale.x = 0.2;
            lines_marker.scale.y = 0.2;
            lines_marker.scale.z = 0.2;
            lines_marker.color.a = 0.7;
            lines_marker.color.r = 255.0 / 255.0;
            lines_marker.color.g = 215.0 / 255.0;
            lines_marker.color.b = 0.0 / 255.0;

            points_marker.ns = "tree_nodes";
            points_marker.id = 1;
            points_marker.scale.x = 0.4;
            points_marker.scale.y = 0.4;
            points_marker.scale.z = 0.4;
            points_marker.color.a = 1.0;
            points_marker.color.r = 139.0 / 255.0;
            points_marker.color.g = 105.0 / 255.0;
            points_marker.color.b = 20.0 / 255.0;
            for (size_t i = 0; i < auxiliary_info[0].size(); i++)
            {
                geometry_msgs::Point p1;
                p1.x = auxiliary_info[0][i].x;
                p1.y = auxiliary_info[0][i].y;
                p1.z = 0;
                points_marker.points.push_back(p1);
                p1.z = 5;
                lines_marker.points.push_back(std::move(p1));
                geometry_msgs::Point p2;
                p2.x = auxiliary_info[1][i].x;
                p2.y = auxiliary_info[1][i].y;
                p2.z = 0;
                lines_marker.points.push_back(std::move(p2));
            }
            marker_array.markers.push_back(lines_marker);
            marker_array.markers.push_back(points_marker);
        }
        else if (planner_name == "GA")
        {
        }

        path_pubs[i].publish(path_msg);
        auxiliary_pubs[i].publish(marker_array);
        planner->showAllInfo(true, ros::package::getPath("global_planning") + "/result/test_result/");
    }

    write_metrics(ros::package::getPath("global_planning") + "/csv/", all_metrics);
}

void start_point_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    for (size_t i = 0; i < planners.size(); i++)
    {
        GlobalPlannerInterface * planner = planners[i];

        planner->setStartPoint(msg.pose.pose.position.x, msg.pose.pose.position.y);
        planner->setStartPointYaw(tf2::getYaw(msg.pose.pose.orientation));
    }

    start_point_flag = true;
    if (start_point_flag && end_point_flag && map_flag)
    {
        pub_path();
    }
}

void end_point_callback(const geometry_msgs::PoseStamped & msg)
{
    for (size_t i = 0; i < planners.size(); i++)
    {
        GlobalPlannerInterface * planner = planners[i];

        planner->setEndPoint(msg.pose.position.x, msg.pose.position.y);
        planner->setEndPointYaw(tf2::getYaw(msg.pose.orientation));
    }

    end_point_flag = true;
    if (start_point_flag && end_point_flag && map_flag)
    {
        pub_path();
    }
}

void map_callback(const nav_msgs::OccupancyGrid & msg)
{
    int rows = msg.info.height;
    int cols = msg.info.width;
    res = msg.info.resolution;
    ori_x = msg.info.origin.position.x;
    ori_y = msg.info.origin.position.y;

    for (size_t i = 0; i < planners.size(); i++)
    {
        GlobalPlannerInterface * planner = planners[i];

        planner->setMapInfo(res, ori_x, ori_y);

        cv::Mat map = cv::Mat::zeros(rows, cols, CV_8UC1);
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                map.at<uchar>(i, j) = msg.data[i * cols + j];
            }
        }
        // std::string file_path = ros::package::getPath("global_planning") + "/map/XG_map.png";
        // cv::imwrite(file_path, map);
        map_flag = planner->setMap(map);

        if (map_flag)
        {
            cv::Mat processed_map;
            planner->getProcessedMap(processed_map);

            nav_msgs::OccupancyGrid new_msg;
            new_msg.header.frame_id = msg.header.frame_id;
            new_msg.header.stamp = ros::Time::now();
            new_msg.info = msg.info;
            new_msg.data.resize(msg.data.size());
            for (size_t i = 0; i < rows; i++)
            {
                for (size_t j = 0; j < cols; j++)
                {
                    new_msg.data[i * cols + j] = processed_map.at<uchar>(i, j);
                }
            }
            processed_map_pubs[i].publish(new_msg);
        }
    }
}


int main(int argc, char * argv[])
{
    // 1. 确定规划器类别
    planners_names.clear();
    planners_names.push_back("TSHAstar_OP");    // TSHAstar的截止第一个优化的效果（基本参考线）
    planners_names.push_back("TSHAstar");

    // 2. 初始化ros，根据规划器类别创建相应的规划器对象和ros对象
    ros::init(argc, argv, "test_ros");
    ros::NodeHandle nh("~");

    ros::Subscriber start_point_sub = nh.subscribe("/initialpose", 1, start_point_callback);
    ros::Subscriber end_point_sub   = nh.subscribe("/move_base_simple/goal", 1, end_point_callback);
    ros::Subscriber map_sub         = nh.subscribe("/grid_cost_map/global_occupancy_grid_map", 1, map_callback);
    for (const std::string & planner_name : planners_names)
    {
        ros::Publisher processed_map_pub = nh.advertise<nav_msgs::OccupancyGrid>(planner_name + "/processed_map", 1, true);
        ros::Publisher path_pub          = nh.advertise<nav_msgs::Path>(planner_name + "/path", 1, true);
        ros::Publisher auxiliary_pub     = nh.advertise<visualization_msgs::MarkerArray>(planner_name + "/auxiliary_info", 1, true);
        processed_map_pubs.push_back(std::move(processed_map_pub));
        path_pubs.push_back(std::move(path_pub));
        auxiliary_pubs.push_back(std::move(auxiliary_pub));
    }

    // 3. 循环创建规划器对象，读取参数
    for (const std::string & planner_name : planners_names)
    {
        if (planner_name == "TSHAstar_RRP" || planner_name == "TSHAstar_BS" || planner_name == "TSHAstar_OP" || planner_name == "TSHAstar")
        {
            TSHAstar::TSHAstarParams TSHAstar_params;
            TSHAstar_params.map.KERNEL_SIZE = 15;
            TSHAstar_params.map.EXPANDED_K = 1.3;
            TSHAstar_params.map.EXPANDED_MIN_THRESHOLD = 0;
            TSHAstar_params.map.EXPANDED_MAX_THRESHOLD = 100;
            TSHAstar_params.map.COST_THRESHOLD = 10;
            TSHAstar_params.map.OBSTACLE_THRESHOLD = 100;
            TSHAstar_params.search.path_search.NEIGHBOR_TYPE = TSHAstar::NeighborType::FiveConnected;
            TSHAstar_params.search.path_search.HEURISTICS_TYPE = TSHAstar::HeuristicsType::Euclidean;
            TSHAstar_params.search.path_search.TRAV_COST_K = 2.0;
            TSHAstar_params.search.path_search.TURN_COST_STRAIGHT = 1.0;
            TSHAstar_params.search.path_search.TURN_COST_SLANT = 1.1;
            TSHAstar_params.search.path_search.TURN_COST_VERTICAL = 2.0;
            TSHAstar_params.search.path_search.TURN_COST_REVERSE_SLANT = 3.0;
            TSHAstar_params.search.path_simplification.PATH_SIMPLIFICATION_TYPE = TSHAstar::PathSimplificationType::DPPlus;
            TSHAstar_params.search.path_simplification.DISTANCE_THRESHOLD = 1.5;
            TSHAstar_params.search.path_simplification.ANGLE_THRESHOLD = 10 / 180 * M_PI;
            TSHAstar_params.search.path_simplification.OBSTACLE_THRESHOLD = 70;
            TSHAstar_params.search.path_simplification.LINE_WIDTH = 1.0;
            TSHAstar_params.search.path_simplification.MAX_INTAVAL = 8.0;
            TSHAstar_params.search.path_smooth.PATH_SMOOTH_TYPE = TSHAstar::PathSmoothType::BSpline;
            TSHAstar_params.search.path_smooth.T_STEP = 0.0005;
            TSHAstar_params.search.path_optimization.S_INTERVAL = 4.0;
            TSHAstar_params.search.path_optimization.USE_DUBINS = false;
            TSHAstar_params.search.path_optimization.DUBINS_RADIUS = 2.5;
            TSHAstar_params.search.path_optimization.DUBINS_INTERVAL = 1.5;
            TSHAstar_params.search.path_optimization.DUBINS_LENGTH = 8.0;
            TSHAstar_params.search.path_optimization.REF_WEIGTH_SMOOTH = 100.0;
            TSHAstar_params.search.path_optimization.REF_WEIGTH_LENGTH = 1.0;
            TSHAstar_params.search.path_optimization.REF_WEIGTH_DEVIATION = 50.0;
            TSHAstar_params.search.path_optimization.REF_BUFFER_DISTANCE = 1.0;
            TSHAstar_params.sample.path_sample.LONGITUDIAL_SAMPLE_SPACING = 0.5;
            TSHAstar_params.sample.path_sample.LATERAL_SAMPLE_SPACING = 1.0;
            TSHAstar_params.sample.path_sample.LATERAL_SAMPLE_RANGE = 10.0;
            TSHAstar_params.sample.path_dp.COLLISION_DISTANCE = 1.2;
            TSHAstar_params.sample.path_dp.WARNING_DISTANCE = 7.0;
            TSHAstar_params.sample.path_dp.BOUND_CHECK_INTERVAL = 0.5;
            TSHAstar_params.sample.path_dp.WEIGHT_OFFSET = 50.0;
            TSHAstar_params.sample.path_dp.WEIGHT_OBSTACLE = 100.0;
            TSHAstar_params.sample.path_dp.WEIGHT_ANGLE_CHANGE = 1000.0;
            TSHAstar_params.sample.path_dp.WEIGHT_ANGLE_DIFF = 1.0;
            TSHAstar_params.sample.path_qp.WEIGHT_L = 0.01;
            TSHAstar_params.sample.path_qp.WEIGHT_DL = 1.0;
            TSHAstar_params.sample.path_qp.WEIGHT_DDL = 2000.0;
            TSHAstar_params.sample.path_qp.WEIGHT_DDDL = 4000.0;
            TSHAstar_params.sample.path_qp.WEIGHT_CENTER = 0.0;
            TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_L = 20.0;
            TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_DL = 20.0;
            TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_DDL = 1.0;
            TSHAstar_params.sample.path_qp.DL_LIMIT = 1.0;
            TSHAstar_params.sample.path_qp.VEHICLE_KAPPA_MAX = 0.3;
            TSHAstar_params.sample.path_qp.CENTER_DEVIATION_THRESHOLD = 2.2;
            TSHAstar_params.sample.path_qp.CENTER_BOUNDS_THRESHOLD = 3.2;
            TSHAstar_params.sample.path_qp.CENTER_OBS_COEFFICIENT = 3.0;
            GlobalPlannerInterface * planner = new TSHAstar;
            planner->initParams(TSHAstar_params);
            planners.push_back(planner);
        }
        else if (planner_name == "Astar")
        {
            Astar::AstarParams astar_params;
            astar_params.map.OBSTACLE_THRESHOLD = 30;
            astar_params.cost_function.HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;
            GlobalPlannerInterface * planner = new Astar;
            planner->initParams(astar_params);
            planners.push_back(planner);
        }
        else if (planner_name == "RRT")
        {
            RRT::RRTParams rrt_params;
            rrt_params.map.OBSTACLE_THRESHOLD = 30;
            rrt_params.sample.ITERATOR_TIMES = 100000;
            rrt_params.sample.GOAL_SAMPLE_RATE = 0.1;
            rrt_params.sample.GOAL_DIS_TOLERANCE = 2.0;
            rrt_params.sample.STEP_SIZE = 3.0;
            GlobalPlannerInterface * planner = new RRT;
            planner->initParams(rrt_params);
            planners.push_back(planner);
        }
        else if (planner_name == "RRTstar")
        {
            RRTstar::RRTstarParams rrtstar_params;
            rrtstar_params.map.OBSTACLE_THRESHOLD = 50;
            rrtstar_params.sample.ITERATOR_TIMES = 10000000;
            rrtstar_params.sample.GOAL_SAMPLE_RATE = 0.1;
            rrtstar_params.sample.GOAL_DIS_TOLERANCE = 2.0;
            rrtstar_params.sample.STEP_SIZE = 3.0;
            rrtstar_params.sample.NEAR_DIS = 10.0;
            GlobalPlannerInterface * planner = new RRTstar;
            planner->initParams(rrtstar_params);
            planners.push_back(planner);
        }
        else if (planner_name == "GA")
        {
            // GA规划器
            GA::GAParams ga_params;
            ga_params.map.OBSTACLE_THRESHOLD = 50;
            ga_params.optimization.GENERATION_SIZE = 1000;
            ga_params.optimization.POPULATION_SIZE = 200;
            ga_params.optimization.CHROMOSOME_SIZE = 2;
            ga_params.optimization.CROSSOVER_RATE = 0.7;
            ga_params.optimization.MUTATION_RATE = 0.01;
            GlobalPlannerInterface * planner = new GA;
            planner->initParams(ga_params);
            planners.push_back(planner);
        }
    }

    // 4. 读取地图信息
    // 可以选择手动加载地图，也可以选择订阅地图
    std::string map_path = ros::package::getPath("global_planning") + "/map/XG_map.png";
    // std::string map_path = ros::package::getPath("global_planning") + "/map/map2.png";
    load_map(map_path, 0.4);
    // 5. 直接手动指定起点和终点
    geometry_msgs::PoseWithCovarianceStamped start_point;
    start_point.header.frame_id = "/map";
    start_point.header.stamp = ros::Time::now();
    start_point.pose.pose.position.x = 40.0;
    start_point.pose.pose.position.y = 185.0;
    start_point.pose.pose.position.z = 0.0;
    start_point.pose.pose.orientation.x = 0.0;
    start_point.pose.pose.orientation.y = 0.0;
    start_point.pose.pose.orientation.z = 0.0;
    start_point.pose.pose.orientation.w = 1.0;
    // start_point_callback(start_point);
    geometry_msgs::PoseStamped end_point;
    end_point.header.frame_id = "/map";
    end_point.header.stamp = ros::Time::now();
    end_point.pose.position.x = 875.0;
    end_point.pose.position.y = 60.0;
    end_point.pose.position.z = 0.0;
    end_point.pose.orientation.x = 0.0;
    end_point.pose.orientation.y = 0.0;
    end_point.pose.orientation.z = -0.434185925891;
    end_point.pose.orientation.w = 0.900823279982;
    end_point_callback(end_point);

    // 6. 等待回调函数，开始规划
    ros::spin();
    for (auto && planner : planners)
    {
        delete planner;
    }
    return 0;
}
