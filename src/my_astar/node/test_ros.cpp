#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include "my_astar/MC_astar.h"

ros::Publisher path_pub;
ros::Publisher smooth_path_pub;
ros::Publisher expanded_map_pub;
MCAstar MC_astar;
double res = 0.0;
double ori_x = 0.0;
double ori_y = 0.0;
bool start_point_flag = false;
bool end_point_flag = false;
bool map_flag = false;
void pub_path();

void start_point_callback(const geometry_msgs::PoseWithCovarianceStamped & msg)
{
    start_point_flag = MC_astar.setStartPoint(static_cast<int>((msg.pose.pose.position.x - ori_x) / res), 
                                              static_cast<int>((msg.pose.pose.position.y - ori_y) / res));

    if (start_point_flag && end_point_flag && map_flag)
    {
        pub_path();
    }
}

void end_point_callback(const geometry_msgs::PoseStamped & msg)
{
    end_point_flag = MC_astar.setEndPoint(static_cast<int>((msg.pose.position.x - ori_x) / res), 
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

    cv::Mat map = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            map.at<uchar>(i, j) = msg.data[i * cols + j];
        }
    }

    static const cv::Mat kernel = (cv::Mat_<float>(15, 15) << 0.10102, 0.10847, 0.11625, 0.12403, 0.13131, 0.13736, 0.14142, 0.14286, 0.14142, 0.13736, 0.13131, 0.12403, 0.11625, 0.10847, 0.10102, 
                                                              0.10847, 0.11785, 0.12804, 0.13868, 0.14907, 0.15811, 0.16440, 0.16667, 0.16440, 0.15811, 0.14907, 0.13868, 0.12804, 0.11785, 0.10847, 
                                                              0.11625, 0.12804, 0.14142, 0.15617, 0.17150, 0.18570, 0.19612, 0.20000, 0.19612, 0.18570, 0.17150, 0.15617, 0.14142, 0.12804, 0.11625, 
                                                              0.12403, 0.13868, 0.15617, 0.17678, 0.20000, 0.22361, 0.24254, 0.25000, 0.24254, 0.22361, 0.20000, 0.17678, 0.15617, 0.13868, 0.12403, 
                                                              0.13131, 0.14907, 0.17150, 0.20000, 0.23570, 0.27735, 0.31623, 0.33333, 0.31623, 0.27735, 0.23570, 0.20000, 0.17150, 0.14907, 0.13131, 
                                                              0.13736, 0.15811, 0.18570, 0.22361, 0.27735, 0.35355, 0.44721, 0.50000, 0.44721, 0.35355, 0.27735, 0.22361, 0.18570, 0.15811, 0.13736, 
                                                              0.14142, 0.16440, 0.19612, 0.24254, 0.31623, 0.44721, 0.70711, 1.00000, 0.70711, 0.44721, 0.31623, 0.24254, 0.19612, 0.16440, 0.14142, 
                                                              0.14286, 0.16667, 0.20000, 0.25000, 0.33333, 0.50000, 1.00000, 1.00000, 1.00000, 0.50000, 0.33333, 0.25000, 0.20000, 0.16667, 0.14286, 
                                                              0.14142, 0.16440, 0.19612, 0.24254, 0.31623, 0.44721, 0.70711, 1.00000, 0.70711, 0.44721, 0.31623, 0.24254, 0.19612, 0.16440, 0.14142, 
                                                              0.13736, 0.15811, 0.18570, 0.22361, 0.27735, 0.35355, 0.44721, 0.50000, 0.44721, 0.35355, 0.27735, 0.22361, 0.18570, 0.15811, 0.13736, 
                                                              0.13131, 0.14907, 0.17150, 0.20000, 0.23570, 0.27735, 0.31623, 0.33333, 0.31623, 0.27735, 0.23570, 0.20000, 0.17150, 0.14907, 0.13131, 
                                                              0.12403, 0.13868, 0.15617, 0.17678, 0.20000, 0.22361, 0.24254, 0.25000, 0.24254, 0.22361, 0.20000, 0.17678, 0.15617, 0.13868, 0.12403, 
                                                              0.11625, 0.12804, 0.14142, 0.15617, 0.17150, 0.18570, 0.19612, 0.20000, 0.19612, 0.18570, 0.17150, 0.15617, 0.14142, 0.12804, 0.11625, 
                                                              0.10847, 0.11785, 0.12804, 0.13868, 0.14907, 0.15811, 0.16440, 0.16667, 0.16440, 0.15811, 0.14907, 0.13868, 0.12804, 0.11785, 0.10847, 
                                                              0.10102, 0.10847, 0.11625, 0.12403, 0.13131, 0.13736, 0.14142, 0.14286, 0.14142, 0.13736, 0.13131, 0.12403, 0.11625, 0.10847, 0.10102);
    ros::Time start = ros::Time::now();
    cv::Mat src = map.clone();
    int kernel_half_width = kernel.cols / 2;
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            uchar & cost = src.at<uchar>(i, j);
            if (cost > 100) // 只对正常范围[0, 100]中的值进行膨胀扩展；(100, 255)未定义区域和255为探索区域不进行扩展
            {
                continue;
            }
            
            for (int m = -kernel_half_width; m <= kernel_half_width; m++)
            {
                if (i + m < 0)
                {
                    continue;
                }
                if (i + m >= rows)
                {
                    break;
                }
                
                for (int n = -kernel_half_width; n <= kernel_half_width; n++)
                {
                    if (j + n < 0)
                    {
                        continue;
                    }
                    if (j + n >= cols)
                    {
                        break;
                    }
                    
                    uchar new_cost = static_cast<uchar>(kernel.at<float>(m + kernel_half_width, n + kernel_half_width) * cost);
                    if (map.at<uchar>(i + m, j + n) < new_cost)
                    {
                        map.at<uchar>(i + m, j + n) = new_cost;
                    }
                }
            }
        }
    }
    ros::Time end = ros::Time::now();
    printf("expand map cost: %.4fs\n", (end - start).toSec());

    nav_msgs::OccupancyGrid new_msg;
    new_msg.header.frame_id = msg.header.frame_id;
    new_msg.header.stamp = ros::Time::now();
    new_msg.info = msg.info;
    new_msg.data.resize(msg.data.size());
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            new_msg.data[i * cols + j] = map.at<uchar>(i, j);
        }
    }
    expanded_map_pub.publish(new_msg);

    map_flag = MC_astar.setMap(map);
}

void pub_path()
{
    std::vector<cv::Point2i> path;
    std::vector<cv::Point2f> smooth_path;
    MC_astar.getRawPath(path);
    MC_astar.getSmoothPath(smooth_path);

    nav_msgs::Path msg;
    msg.header.frame_id = "map";
    msg.header.stamp = ros::Time::now();
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
    for (cv::Point2f & p : smooth_path)
    {
        geometry_msgs::PoseStamped m;
        m.header = msg2.header;
        m.pose.position.x = (p.x + 0.5) * res + ori_x;
        m.pose.position.y = (p.y + 0.5) * res + ori_y;
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
    path_pub         = nh.advertise<nav_msgs::Path>("MC_astar_path", 1, true);
    smooth_path_pub  = nh.advertise<nav_msgs::Path>("MC_astar_smooth_path", 1, true);
    expanded_map_pub = nh.advertise<nav_msgs::OccupancyGrid>("expanded_map", 1, true);

    ros::spin();
    return 0;
}
