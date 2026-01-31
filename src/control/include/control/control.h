#pragma once
#include <vector>
#include <mutex>
#include <cmath>
#include <algorithm>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "control/controller/pid_controller.h"


class Control
{
public:
    Control(ros::NodeHandle& nh, ros::NodeHandle& private_nh);
    ~Control();
    
    // 主控制循环
    void run();
    
private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber trajectory_sub_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    tf2_ros::Buffer tf_buffer_;
    tf2_ros::TransformListener tf_listener_;
    
    // 数据存储
    nav_msgs::Path current_trajectory_;
    nav_msgs::Odometry current_odom_;
    std::mutex trajectory_mutex_;
    std::mutex odom_mutex_;
    bool has_trajectory_;
    bool has_odom_;
    
    // 控制器
    PIDController * linear_pid_;
    PIDController * angular_pid_;
    
    // 参数
    double lookahead_distance_;
    double max_linear_vel_;
    double max_angular_vel_;
    double control_frequency_;
    double goal_tolerance_;

    // ROS回调函数
    void trajectoryCallback(const nav_msgs::Path::ConstPtr& msg);
    void odometryCallback(const nav_msgs::Odometry::ConstPtr& msg);
    
    // 获取当前目标点
    geometry_msgs::PoseStamped getCurrentTarget();
    
    // 计算控制输出
    void computeControlOutput(const geometry_msgs::PoseStamped& target_pose, 
                              const nav_msgs::Odometry& current_odom,
                              double& v, double& w);
    
    // 寻找最近的轨迹点
    int findNearestPointIndex(const geometry_msgs::Pose& current_pose);
};
