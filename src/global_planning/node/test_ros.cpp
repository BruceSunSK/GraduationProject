#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "global_planning/MCAstar.h"
#include "global_planning/astar.h"

ros::Publisher path_pub;
ros::Publisher smooth_path_pub;
ros::Publisher expanded_map_pub;
MCAstar MCAstar;
double res = 0.0;
double ori_x = 0.0;
double ori_y = 0.0;
bool start_point_flag = false;
bool end_point_flag = false;
bool map_flag = false;
void pub_path();

void start_point_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    start_point_flag = MCAstar.setStartPoint(static_cast<int>((msg.pose.pose.position.x - ori_x) / res), 
                                             static_cast<int>((msg.pose.pose.position.y - ori_y) / res));

    if (start_point_flag && end_point_flag && map_flag)
    {
        pub_path();
    }
}

void end_point_callback(const geometry_msgs::PoseStamped & msg)
{
    end_point_flag = MCAstar.setEndPoint(static_cast<int>((msg.pose.position.x - ori_x) / res), 
                                         static_cast<int>((msg.pose.position.y - ori_y) / res));

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
    MCAstar.setMapInfo(res, ori_x, ori_y);

    cv::Mat map = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            map.at<uchar>(i, j) = msg.data[i * cols + j];
        }
    }
    map_flag = MCAstar.setMap(map);

    // nav_msgs::OccupancyGrid new_msg;
    // new_msg.header.frame_id = msg.header.frame_id;
    // new_msg.header.stamp = ros::Time::now();
    // new_msg.info = msg.info;
    // new_msg.data.resize(msg.data.size());
    // for (size_t i = 0; i < rows; i++)
    // {
    //     for (size_t j = 0; j < cols; j++)
    //     {
    //         new_msg.data[i * cols + j] = map.at<uchar>(i, j);
    //     }
    // }
    // expanded_map_pub.publish(new_msg);
}

void pub_path()
{
    std::vector<cv::Point2i> path;
    std::vector<cv::Point2d> smooth_path;
    MCAstar.getRawPath(path);
    MCAstar.getSmoothPath(smooth_path);

    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
    // 原始节点是栅格点，需要手动修正0.5个单位
    for (cv::Point2i & p : path)
    {
        geometry_msgs::PoseStamped m;
        m.header = msg.header;
        m.pose.position.x = (p.x + 0.5) * res + ori_x;
        m.pose.position.y = (p.y + 0.5) * res + ori_y;
        m.pose.position.z = 0;
        msg.poses.push_back(m);
    }
    nav_msgs::Path msg2;
    msg2.header = msg.header;
    for (cv::Point2d & p : smooth_path)
    {
        geometry_msgs::PoseStamped m;
        m.header = msg2.header;
        m.pose.position.x = p.x;
        m.pose.position.y = p.y;
        m.pose.position.z = 0;
        msg2.poses.push_back(m);
    }
    
    path_pub.publish(msg);
    smooth_path_pub.publish(msg2);
}


int main(int argc, char *argv[])
{
    ros::init(argc, argv, "test_ros");
    ros::NodeHandle nh("~");

    ros::Subscriber start_point_sub = nh.subscribe("/initialpose", 1, start_point_callback);
    ros::Subscriber end_point_sub   = nh.subscribe("/move_base_simple/goal", 1, end_point_callback);
    ros::Subscriber map_sub         = nh.subscribe("/grid_cost_map/global_occupancy_grid_map", 1, map_callback);
    path_pub         = nh.advertise<nav_msgs::Path>("path", 1, true);
    smooth_path_pub  = nh.advertise<nav_msgs::Path>("smooth_path", 1, true);
    expanded_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("expanded_map", 1, true);

    MCAstar::MCAstarParams MCAstar_params;
    MCAstar_params.map_params.EXPANDED_K = 1;
    MCAstar_params.map_params.EXPANDED_MIN_THRESHOLD = 0;
    MCAstar_params.map_params.EXPANDED_MAX_THRESHOLD = 100;
    MCAstar_params.map_params.OBSTACLE_THRESHOLD = 100;
    MCAstar_params.cost_function_params.HEURISTICS_TYPE = MCAstar::HeuristicsType::Euclidean;
    MCAstar_params.cost_function_params.TRAV_COST_K = 2.0;
    MCAstar_params.bezier_curve_params.T_STEP = 0.01;
    MCAstar_params.downsampling_params.INTERVAL = 0.3;
    MCAstar.initParams(MCAstar_params);

    ros::spin();
    return 0;
}
