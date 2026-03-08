#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <tf2/utils.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "perception/PredictedObstacles.h"
#include "global_planning/map/distance_map.h"
#include "global_planning/path/reference_path.h"
#include "local_planning/planners/local_planner_interface.h"
#include "local_planning/planners/local_planner.h"
#include "local_planning/planners/lattice_planner.h"
#include "local_planning/vehicle/data_type.h"


class LocalPlanning
{
public:
    LocalPlanning() = delete;
    LocalPlanning(ros::NodeHandle & nh, const ros::Rate & loop_rate);
    LocalPlanning(const LocalPlanning & other) = delete;
    LocalPlanning(LocalPlanning && other) = delete;
    LocalPlanning & operator=(const LocalPlanning & other) = delete;
    LocalPlanning & operator=(LocalPlanning && other) = delete;
    virtual ~LocalPlanning();

    virtual void Run();

protected:
    virtual void ReferencePathCallback(const nav_msgs::Path::ConstPtr & msg);
    virtual void CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg);
    virtual void VehicleStateCallback(const nav_msgs::Odometry::ConstPtr & msg);
    virtual void PredictedObstaclesCallback(const perception::PredictedObstacles::ConstPtr & msg);
    
    virtual void OnPlanningCompleted(const LocalPlannerResult & result) {}
    
private:
    void Initialize();
    void LoadParameters();
    void InitializeSubscribers();
    void InitializePublishers();
    void InitializePlanner();

protected:
    // 最新接收的数据，供派生类使用
    Path::ReferencePath::Ptr latest_reference_path_;
    Map::MultiMap::Ptr latest_map_;
    Vehicle::State::Ptr latest_vehicle_state_;
    Obstacle::Obstacle::List latest_obstacles_;

private:
    // *********************************************************************************
    // 1. ROS相关
    ros::NodeHandle & nh_;                  // 私有节点句柄（引用）
    ros::Rate loop_rate_;                   // 循环频率

    // 1.1订阅者
    std::string input_ref_path_topic_;      // 全局参考线订阅话题名
    std::string input_costmap_topic_;       // 代价地图订阅话题名
    std::string input_vehicle_state_topic_; // 车辆状态订阅话题名
    std::string input_obstacles_topic_;     // 预测障碍物订阅话题名
    ros::Subscriber sub_ref_path_;          // 全局参考线订阅
    ros::Subscriber sub_costmap_;           // 代价地图订阅
    ros::Subscriber sub_vehicle_state_;     // 车辆状态订阅
    ros::Subscriber sub_obstacles_;         // 预测障碍物订阅（预留）

    // 1.2 发布者
    std::string output_local_trajectory_topic_; // 局部轨迹发布话题名
    ros::Publisher pub_local_trajectory_;       // 局部轨迹发布

    // 1.3 TF相关
    tf2_ros::Buffer tf_buffer_;                 // TF缓冲区
    tf2_ros::TransformListener tf_listener_;    // TF监听器

    // 1.4 状态标志
    bool is_initialized_;                       // 是否初始化完成
    
    // *********************************************************************************
    // 算法实例
    std::string planner_name_;
    std::unique_ptr<LocalPlannerInterface> planner_;     // LocalPlanner算法实例指针

};
