#include <ros/ros.h>
#include <ros/package.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>

#include "global_planning/MCAstar.h"
#include "global_planning/astar.h"
#include "global_planning/rrt.h"
#include "global_planning/rrtstar.h"


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
void pub_path();

void start_point_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    start_point_flag = planner->setStartPoint(msg.pose.pose.position.x, msg.pose.pose.position.y);

    if (start_point_flag && end_point_flag && map_flag)
    {
        pub_path();
    }
}

void end_point_callback(const geometry_msgs::PoseStamped & msg)
{
    end_point_flag = planner->setEndPoint(msg.pose.position.x, msg.pose.position.y);

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
    if (planner_name == "MCAstar")
    {
        // 用于显示
        visualization_msgs::Marker marker;
        marker.header = path_msg.header;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        
        // 0. 规划原始节点
        marker.ns = "raw_nodes";
        marker.id = 0;
        marker.scale.x = 0.35;
        marker.scale.y = 0.35;
        marker.scale.z = 0.35;
        marker.color.a = 0.9;
        marker.color.r = 0.0 / 255.0;
        marker.color.g = 0.0 / 255.0;
        marker.color.b = 255.0 / 255.0;
        marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[0][i].x;
            p.y = auxiliary_info[0][i].y;
            p.z = 1;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);

        
        // 1. 规划扩展节点
        marker.ns = "expanded_nodes";
        marker.id = 1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 65.0 / 255.0;
        marker.color.g = 105.0 / 255.0;
        marker.color.b = 255.0 / 255.0;
        marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[1].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[1][i].x;
            p.y = auxiliary_info[1][i].y;
            p.z = 0;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);

        
        // 2. 去除冗余点后的关键节点
        marker.ns = "key_nodes";
        marker.id = 2;
        marker.scale.x = 0.7;
        marker.scale.y = 0.7;
        marker.scale.z = 0.7;
        marker.color.a = 1.0;
        marker.color.r = 255.0 / 255.0;
        marker.color.g = 215.0 / 255.0;
        marker.color.b = 0.0 / 255.0;
        marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[2].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[2][i].x;
            p.y = auxiliary_info[2][i].y;
            p.z = 2;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);
    }
    else if (planner_name == "Astar")
    {
        // 用于显示
        visualization_msgs::Marker marker;
        marker.header = path_msg.header;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;

        
        // 0. 规划扩展节点
        marker.ns = "expanded_nodes";
        marker.id = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 0.7;
        marker.color.r = 65.0 / 255.0;
        marker.color.g = 105.0 / 255.0;
        marker.color.b = 255.0 / 255.0;
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[0][i].x;
            p.y = auxiliary_info[0][i].y;
            p.z = 0;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);
    }
    else if (planner_name == "RRT" || planner_name == "RRTstar")
    {
        // 用于显示
        visualization_msgs::Marker line_marker;
        line_marker.header = path_msg.header;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        visualization_msgs::Marker point_marker;
        point_marker.header = path_msg.header;
        point_marker.type = visualization_msgs::Marker::CUBE_LIST;
        point_marker.action = visualization_msgs::Marker::ADD;
        point_marker.pose.orientation.x = 0.0;
        point_marker.pose.orientation.y = 0.0;
        point_marker.pose.orientation.z = 0.0;
        point_marker.pose.orientation.w = 1.0;


        // 0. rrt树上所有节点
        line_marker.ns = "tree_lines";
        line_marker.id = 0;
        line_marker.scale.x = 0.2;
        line_marker.scale.y = 0.2;
        line_marker.scale.z = 0.2;
        line_marker.color.a = 0.7;
        line_marker.color.r = 255.0 / 255.0;
        line_marker.color.g = 215.0 / 255.0;
        line_marker.color.b = 0.0 / 255.0;

        point_marker.ns = "tree_nodes";
        point_marker.id = 1;
        point_marker.scale.x = 0.4;
        point_marker.scale.y = 0.4;
        point_marker.scale.z = 0.4;
        point_marker.color.a = 1.0;
        point_marker.color.r = 139.0 / 255.0;
        point_marker.color.g = 105.0 / 255.0;
        point_marker.color.b = 20.0 / 255.0;
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p1;
            p1.x = auxiliary_info[0][i].x;
            p1.y = auxiliary_info[0][i].y;
            p1.z = 0;
            point_marker.points.push_back(p1);
            p1.z = 5;
            line_marker.points.push_back(std::move(p1));
            geometry_msgs::Point p2;
            p2.x = auxiliary_info[1][i].x;
            p2.y = auxiliary_info[1][i].y;
            p2.z = 0;
            line_marker.points.push_back(std::move(p2));
        }
        marker_array.markers.push_back(line_marker);
        marker_array.markers.push_back(point_marker);
    }

    path_pub.publish(path_msg);
    auxiliary_pub.publish(marker_array);
    planner->showAllInfo(true, ros::package::getPath("global_planning") + "/result/test_result/");
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_ros");
    ros::NodeHandle nh("~");

    ros::Subscriber start_point_sub = nh.subscribe("/initialpose", 1, start_point_callback);
    ros::Subscriber end_point_sub   = nh.subscribe("/move_base_simple/goal", 1, end_point_callback);
    ros::Subscriber map_sub         = nh.subscribe("/grid_cost_map/global_occupancy_grid_map", 1, map_callback);
    processed_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("processed_map", 1, true);
    path_pub          = nh.advertise<nav_msgs::Path>("path", 1, true);
    auxiliary_pub     = nh.advertise<visualization_msgs::MarkerArray>("auxiliary_info", 1, true);

    planner_name = "RRTstar";       // MCAstar / Astar / RRT / RRTstar
    if (planner_name == "MCAstar")
    {
        MCAstar::MCAstarParams MCAstar_params;
        MCAstar_params.map_params.EXPANDED_K = 1.3;
        MCAstar_params.map_params.EXPANDED_MIN_THRESHOLD = 0;
        MCAstar_params.map_params.EXPANDED_MAX_THRESHOLD = 100;
        MCAstar_params.map_params.COST_THRESHOLD = 10;
        MCAstar_params.map_params.OBSTACLE_THRESHOLD = 100;
        MCAstar_params.cost_function_params.NEIGHBOR_TYPE = MCAstar::NeighborType::FiveConnected;
        MCAstar_params.cost_function_params.HEURISTICS_TYPE = MCAstar::HeuristicsType::Euclidean;
        MCAstar_params.cost_function_params.TRAV_COST_K = 2.0;
        MCAstar_params.cost_function_params.TURN_COST_STRAIGHT = 1.0;
        MCAstar_params.cost_function_params.TURN_COST_SLANT = 1.1;
        MCAstar_params.cost_function_params.TURN_COST_VERTICAL = 2.0;
        MCAstar_params.cost_function_params.TURN_COST_REVERSE_SLANT = 3.0;
        MCAstar_params.path_simplification_params.PATH_SIMPLIFICATION_TYPE = MCAstar::PathSimplificationType::DPPlus;
        MCAstar_params.path_simplification_params.DISTANCE_THRESHOLD = 1.5;
        MCAstar_params.path_simplification_params.ANGLE_THRESHOLD = 10 / 180 * M_PI;
        MCAstar_params.path_simplification_params.OBSTACLE_THRESHOLD = 70;
        MCAstar_params.path_simplification_params.LINE_WIDTH = 1.0;
        MCAstar_params.path_simplification_params.MAX_INTAVAL = 8.0;
        MCAstar_params.path_smooth_params.PATH_SMOOTH_TYPE = MCAstar::PathSmoothType::BSpline;
        MCAstar_params.path_smooth_params.T_STEP = 0.0005;
        MCAstar_params.downsampling_params.INTERVAL = 0.4;
        planner = new MCAstar;
        planner->initParams(MCAstar_params);
    }
    else if (planner_name == "Astar")
    {
        Astar::AstarParams astar_params;
        astar_params.map_params.OBSTACLE_THRESHOLD = 50;
        astar_params.cost_function_params.HEURISTICS_TYPE = Astar::HeuristicsType::Euclidean;
        planner = new Astar;
        planner->initParams(astar_params);
    }
    else if (planner_name == "RRT")
    {
        RRT::RRTParams rrt_params;
        rrt_params.map_params.OBSTACLE_THRESHOLD = 50;
        rrt_params.sample_params.ITERATOR_TIMES = 100000;
        rrt_params.sample_params.GOAL_SAMPLE_RATE = 0.1;
        rrt_params.sample_params.GOAL_DIS_TOLERANCE = 2.0;
        rrt_params.sample_params.STEP_SIZE = 3.0;
        planner = new RRT;
        planner->initParams(rrt_params);
    }
    else if (planner_name == "RRTstar")
    {
        RRTstar::RRTstarParams rrtstar_params;
        rrtstar_params.map_params.OBSTACLE_THRESHOLD = 50;
        rrtstar_params.sample_params.ITERATOR_TIMES = 10000000;
        rrtstar_params.sample_params.GOAL_SAMPLE_RATE = 0.1;
        rrtstar_params.sample_params.GOAL_DIS_TOLERANCE = 2.0;
        rrtstar_params.sample_params.STEP_SIZE = 3.0;
        rrtstar_params.sample_params.NEAR_DIS = 10.0;
        planner = new RRTstar;
        planner->initParams(rrtstar_params);
    }

    ros::spin();
    delete planner;
    return 0;
}
