#pragma once

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PointStamped.h>
#include <nav_msgs/OccupancyGrid.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_ros/grid_map_ros.hpp>
#include <grid_map_msgs/GridMap.h>
#include <grid_map_filters/MeanInRadiusFilter.hpp>
#include <filters/filter_chain.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

using PointType = pcl::PointXYZI;


class BuildCostMap
{
private:
    ros::NodeHandle nh_;
    std::string global_laser_cloud_topic_;
    std::string local_laser_cloud_topic_;
    std::string global_map_load_path_;      // 如果不为空，则预先从该路径加载预先计算好的png全局代价地图
    double global_map_res_;
    double local_map_res_;
    double local_map_length_;

    ros::Subscriber subGlobalLaserCloud;
    void GlobalLaserCloudCallback(const sensor_msgs::PointCloud2::Ptr & msg);
    ros::Subscriber subLocalLaserCloud;
    void LocalLaserCloudCallback(const sensor_msgs::PointCloud2::Ptr & msg);

    ros::Publisher pubGlobalGridMap;
    ros::Publisher pubGlobalOccupancyGridMap;
    pcl::PointCloud<PointType>::Ptr global_map_laser_cloud;
    filters::FilterChain<grid_map::GridMap> globalMapFilterChain; //滤波器
    grid_map::GridMap global_map;
    bool global_map_finished;

    ros::Publisher pubLocalGridMap;
    ros::Publisher pubLocalOccupancyGridMap;
    pcl::PointCloud<PointType>::Ptr loacl_map_laser_cloud;
    filters::FilterChain<grid_map::GridMap> localMapFilterChain; //滤波器
    grid_map::GridMap local_map;
    grid_map::GridMap local_map_out;
    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

public:
    BuildCostMap(ros::NodeHandle & nh, bool & success);
    ~BuildCostMap();
};


