#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "local_planning/planner/LocalPlanner.h"


class LocalPlanning
{
public:
    LocalPlanning() = delete;
    LocalPlanning(ros::NodeHandle & nh, const ros::Rate & loop_rate);
    LocalPlanning(const LocalPlanning & other) = delete;
    LocalPlanning(LocalPlanning && other) = delete;
    LocalPlanning & operator=(const LocalPlanning & other) = delete;
    LocalPlanning & operator=(LocalPlanning && other) = delete;
    ~LocalPlanning();

    void Run();

private:
    void Initialize();
    void LoadParameters();
    void InitializeSubscribers();
    void InitializePublishers();
    void InitializePlanner();
    bool IsDataReady() const;

private:
    // *********************************************************************************
    // 1. ROS相关
    ros::NodeHandle & nh_;                  // 私有节点句柄（引用）
    ros::Rate loop_rate_;                   // 循环频率

    // 1.1订阅者
    std::string input_global_ref_topic_;    // 全局参考线订阅话题名
    ros::Subscriber sub_global_ref_;        // 全局参考线订阅
    void GlobalReferenceCallback(const nav_msgs::Path::ConstPtr & msg);

    std::string input_costmap_topic_;       // 代价地图订阅话题名
    ros::Subscriber sub_costmap_;           // 代价地图订阅
    void CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg);

    std::string input_vehicle_pose_topic_;  // 车辆位姿订阅话题名
    ros::Subscriber sub_vehicle_pose_;      // 车辆位姿订阅
    void VehiclePoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg);

    std::string input_vehicle_vel_topic_;   // 车辆速度订阅话题名
    ros::Subscriber sub_vehicle_vel_;       // 车辆速度订阅
    void VehicleVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg);

    std::string input_obstacles_topic_;     // 预测障碍物订阅话题名
    ros::Subscriber sub_obstacles_;         // 预测障碍物订阅（预留）
    template<typename T>
    void PredictedObstaclesCallback(const typename T::ConstPtr & msg);

    // 1.2 发布者
    std::string output_local_trajectory_topic_; // 局部轨迹发布话题名
    ros::Publisher pub_local_trajectory_;       // 局部轨迹发布

    std::string output_debug_path_topic_;       // 调试路径发布话题名
    ros::Publisher pub_debug_path_;             // 调试路径发布

    std::string output_debug_speed_topic_;      // 调试速度发布话题名
    ros::Publisher pub_debug_speed_;            // 调试速度发布

    // 1.3 TF相关
    tf2_ros::Buffer tf_buffer_;                 // TF缓冲区
    tf2_ros::TransformListener tf_listener_;    // TF监听器

    // 1.4 状态标志
    bool is_initialized_ = false;               // 是否初始化完成
    
    // *********************************************************************************
    // 2. 数据存储
    nav_msgs::Path global_reference_;               // 全局参考线
    nav_msgs::OccupancyGrid costmap_;               // 代价地图
    geometry_msgs::PoseStamped vehicle_pose_;       // 车辆位姿
    geometry_msgs::TwistStamped vehicle_velocity_;  // 车辆速度

    // 算法实例
    LocalPlanner * planner_ = nullptr;              // LocalPlanner算法实例指针

};
