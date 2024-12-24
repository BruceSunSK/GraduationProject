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

#include "global_planning/planners/TSHAstar.h"
#include "global_planning/planners/astar.h"
#include "global_planning/planners/rrt.h"
#include "global_planning/planners/rrtstar.h"
#include "global_planning/planners/genetic_algorithm.h"


ros::Publisher processed_map_pub;
ros::Publisher path_pub;
ros::Publisher auxiliary_pub;
GlobalPlannerInterface * planner;
std::string planner_name;
double res = 0.0;
double ori_x = 0.0;
double ori_y = 0.0;
bool start_point_flag = false;
bool end_point_flag = false;
bool map_flag = false;


void load_map(const std::string & map_path, const double resulution)
{
    cv::Mat map = cv::imread(map_path, cv::IMREAD_GRAYSCALE);
    int rows = map.rows;
    int cols = map.cols;
    res = resulution;
    ori_x = 0.0;
    ori_y = 0.0;
    planner->setMapInfo(res, ori_x, ori_y);

    map_flag = planner->setMap(map);

    if (map_flag)
    {
        cv::Mat processed_map;
        planner->getProcessedMap(processed_map);

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
        processed_map_pub.publish(msg);
    }
}

void pub_path()
{
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

    path_pub.publish(path_msg);
    auxiliary_pub.publish(marker_array);
    planner->showAllInfo(true, ros::package::getPath("global_planning") + "/result/test_result/");
}

void start_point_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    start_point_flag = planner->setStartPoint(msg.pose.pose.position.x, msg.pose.pose.position.y) &&
                       planner->setStartPointYaw(tf2::getYaw(msg.pose.pose.orientation));

    if (start_point_flag && end_point_flag && map_flag)
    {
        pub_path();
    }
}

void end_point_callback(const geometry_msgs::PoseStamped & msg)
{
    end_point_flag = planner->setEndPoint(msg.pose.position.x, msg.pose.position.y) &&
                     planner->setEndPointYaw(tf2::getYaw(msg.pose.orientation));

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
    planner->setMapInfo(res, ori_x, ori_y);

    cv::Mat map = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            map.at<uchar>(i, j) = msg.data[i * cols + j];
        }
    }
    std::string file_path = ros::package::getPath("global_planning") + "/map/XG_map.png";
    cv::imwrite(file_path, map);
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
        processed_map_pub.publish(new_msg);
    }
}


int main(int argc, char * argv[])
{
    ros::init(argc, argv, "test_ros");
    ros::NodeHandle nh("~");

    ros::Subscriber start_point_sub = nh.subscribe("/initialpose", 1, start_point_callback);
    ros::Subscriber end_point_sub   = nh.subscribe("/move_base_simple/goal", 1, end_point_callback);
    ros::Subscriber map_sub         = nh.subscribe("/grid_cost_map/global_occupancy_grid_map", 1, map_callback);
    processed_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("processed_map", 1, true);
    path_pub          = nh.advertise<nav_msgs::Path>("path", 1, true);
    auxiliary_pub     = nh.advertise<visualization_msgs::MarkerArray>("auxiliary_info", 1, true);

    planner_name = nh.param<std::string>("planner_name", "TSHAstar");
    if (planner_name == "TSHAstar")
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
        TSHAstar_params.search.path_optimization.REF_WEIGTH_SMOOTH = 300.0;
        TSHAstar_params.search.path_optimization.REF_WEIGTH_LENGTH = 1.0;
        TSHAstar_params.search.path_optimization.REF_WEIGTH_DEVIATION = 10.0;
        TSHAstar_params.search.path_optimization.REF_BUFFER_DISTANCE = 1.0;
        TSHAstar_params.sample.path_sample.LONGITUDIAL_SAMPLE_SPACING = 0.5;
        TSHAstar_params.sample.path_sample.LATERAL_SAMPLE_SPACING = 0.5;
        TSHAstar_params.sample.path_sample.LATERAL_SAMPLE_RANGE = 10.0;
        TSHAstar_params.sample.path_dp.COLLISION_DISTANCE = 1.5;
        TSHAstar_params.sample.path_dp.WARNING_DISTANCE = 5.0;
        TSHAstar_params.sample.path_dp.BOUND_CHECK_INTERVAL = 0.3;
        TSHAstar_params.sample.path_dp.WEIGHT_OFFSET = 50.0;
        TSHAstar_params.sample.path_dp.WEIGHT_OBSTACLE = 100.0;
        TSHAstar_params.sample.path_dp.WEIGHT_ANGLE_CHANGE = 1000.0;
        TSHAstar_params.sample.path_dp.WEIGHT_ANGLE_DIFF = 1.0;
        TSHAstar_params.sample.path_qp.WEIGHT_L = 1.0;
        TSHAstar_params.sample.path_qp.WEIGHT_DL = 100.0;
        TSHAstar_params.sample.path_qp.WEIGHT_DDL = 1000.0;
        TSHAstar_params.sample.path_qp.WEIGHT_DDDL = 7000.0;
        TSHAstar_params.sample.path_qp.WEIGHT_CENTER = 0.6;
        TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_L = 10.0;
        TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_DL = 50.0;
        TSHAstar_params.sample.path_qp.WEIGHT_END_STATE_DDL = 500.0;
        TSHAstar_params.sample.path_qp.DL_LIMIT = 2.0;
        TSHAstar_params.sample.path_qp.VEHICLE_KAPPA_MAX = 0.5;
        TSHAstar_params.sample.path_qp.CENTER_DEVIATION_THRESHOLD = 2.2;
        TSHAstar_params.sample.path_qp.CENTER_BOUNDS_THRESHOLD = 3.2;
        TSHAstar_params.sample.path_qp.CENTER_OBS_COEFFICIENT = 10.0;
        planner = new TSHAstar;
        planner->initParams(TSHAstar_params);
    }
    else if (planner_name == "Astar")
    {
        Astar::AstarParams astar_params;
        astar_params.map.OBSTACLE_THRESHOLD = 50;
        astar_params.cost_function.HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;
        planner = new Astar;
        planner->initParams(astar_params);
    }
    else if (planner_name == "RRT")
    {
        RRT::RRTParams rrt_params;
        rrt_params.map.OBSTACLE_THRESHOLD = 50;
        rrt_params.sample.ITERATOR_TIMES = 100000;
        rrt_params.sample.GOAL_SAMPLE_RATE = 0.1;
        rrt_params.sample.GOAL_DIS_TOLERANCE = 2.0;
        rrt_params.sample.STEP_SIZE = 3.0;
        planner = new RRT;
        planner->initParams(rrt_params);
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
        planner = new RRTstar;
        planner->initParams(rrtstar_params);
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
        planner = new GA;
        planner->initParams(ga_params);
    }

    // 可以选择手动加载地图，也可以选择订阅地图
    std::string map_path = ros::package::getPath("global_planning") + "/map/XG_map.png";
    // std::string map_path = ros::package::getPath("global_planning") + "/map/map2.png";
    load_map(map_path, 0.4);
     
    ros::spin();
    delete planner;
    return 0;
}
