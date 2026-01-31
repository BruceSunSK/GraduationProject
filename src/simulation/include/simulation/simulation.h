#pragma once
#include <mutex>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_srvs/Empty.h>

#include "simulation/model/differential_model.h"


class Simulation
{
public:
    Simulation(ros::NodeHandle & nh, ros::NodeHandle & private_nh);
    ~Simulation();

    // 主仿真循环
    void run();

private:
    // ROS回调函数
    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr & msg);
    bool resetCallback(std_srvs::Empty::Request & req,
        std_srvs::Empty::Response & res);

    // 发布车辆状态
    void publishVehicleState();

    // 更新仿真
    void updateSimulation();

    // 转换车辆状态到ROS消息
    nav_msgs::Odometry stateToOdometry(const VehicleState & state,
        const std::string & frame_id,
        const std::string & child_frame_id);

    // 转换时间戳
    ros::Time chronoToRosTime(const std::chrono::steady_clock::time_point & tp);

    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber cmd_vel_sub_;
    ros::Publisher odom_pub_;
    ros::ServiceServer reset_service_;
    tf2_ros::TransformBroadcaster tf_broadcaster_;

    // 车辆模型
    bool add_noise_;
    DifferentialModel vehicle_model_;

    // 控制输入
    geometry_msgs::Twist current_cmd_vel_;
    ros::Time last_cmd_vel_time_;
    std::mutex cmd_vel_mutex_;

    // 仿真参数
    double simulation_frequency_;
    double cmd_vel_timeout_;
    std::string world_frame_;
    std::string robot_frame_;

    // 仿真状态
    bool is_running_;
    std::chrono::steady_clock::time_point last_update_time_;
};
