#include <cmath>
#include <vector>
#include <mutex>
#include <algorithm>
#include <limits>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "control/controller/pid_controller.h"


class TestControlNode
{
public:
    TestControlNode(ros::NodeHandle & private_nh)
        : private_nh_(private_nh),
        has_odom_(false), test_case_(0), test_iteration_(0),
        last_saved_pose_x_(0.0), last_saved_pose_y_(0.0),
        has_last_saved_pose_(false)
    {
        // 获取参数
        private_nh_.param<double>("lookahead_distance", lookahead_distance_, 0.5);
        private_nh_.param<double>("max_linear_vel", max_linear_vel_, 4.0);
        private_nh_.param<double>("max_angular_vel", max_angular_vel_, 1.5);
        private_nh_.param<double>("control_frequency", control_frequency_, 50.0);
        private_nh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);
        private_nh_.param<double>("min_trajectory_point_distance", min_trajectory_point_distance_, 0.1);
        private_nh_.param<int>("test_case", test_case_, 0);  // 0: 直线, 1: 圆形, 2: 正弦

        // 获取PID参数
        double kp_linear, ki_linear, kd_linear;
        double kp_angular, ki_angular, kd_angular;
        private_nh_.param<double>("pid/linear/kp", kp_linear, 1.0);
        private_nh_.param<double>("pid/linear/ki", ki_linear, 0.01);
        private_nh_.param<double>("pid/linear/kd", kd_linear, 0.05);
        private_nh_.param<double>("pid/angular/kp", kp_angular, 1.5);
        private_nh_.param<double>("pid/angular/ki", ki_angular, 0.01);
        private_nh_.param<double>("pid/angular/kd", kd_angular, 0.1);

        // 初始化PID控制器
        linear_pid_ = new PIDController(kp_linear, ki_linear, kd_linear);
        angular_pid_ = new PIDController(kp_angular, ki_angular, kd_angular);

        // 设置PID限制
        linear_pid_->setOutputLimits(-max_linear_vel_, max_linear_vel_);
        angular_pid_->setOutputLimits(-max_angular_vel_, max_angular_vel_);

        // 初始化订阅者和发布者
        odom_sub_ = nh_.subscribe("odom", 1, &TestControlNode::odometryCallback, this);
        cmd_vel_pub_           = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);
        ref_trajectory_pub_    = nh_.advertise<nav_msgs::Path>("reference_trajectory", 1, true);
        actual_trajectory_pub_ = nh_.advertise<nav_msgs::Path>("actual_trajectory", 1, true);

        // 初始化实际轨迹
        actual_trajectory_.header.frame_id = "/map";
        actual_trajectory_.header.stamp = ros::Time::now();

        // 生成测试轨迹
        generateTestTrajectory();

        ROS_INFO("[TestControlNode]: Test control node initialized");
        ROS_INFO("[TestControlNode]: Test case: %d", test_case_);
        ROS_INFO("[TestControlNode]:    Linear PID: Kp=%.3f, Ki=%.3f, Kd=%.3f",
            kp_linear, ki_linear, kd_linear);
        ROS_INFO("[TestControlNode]:    Angular PID: Kp=%.3f, Ki=%.3f, Kd=%.3f",
            kp_angular, ki_angular, kd_angular);
        ROS_INFO("[TestControlNode]:    Min trajectory point distance: %.3f m", min_trajectory_point_distance_);
    }

    ~TestControlNode()
    {
        delete linear_pid_;
        delete angular_pid_;
    }

    void odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
        current_odom_ = *msg;
        has_odom_ = true;
    }

    // 从control.cpp复制的函数
    int findNearestPointIndex(const geometry_msgs::Pose & current_pose)
    {
        if (reference_trajectory_.poses.empty())
        {
            return -1;
        }

        int nearest_index = 0;
        double min_dis_sq = std::numeric_limits<double>::max();

        for (size_t i = 0; i < reference_trajectory_.poses.size(); ++i)
        {
            double dx = reference_trajectory_.poses[i].pose.position.x - current_pose.position.x;
            double dy = reference_trajectory_.poses[i].pose.position.y - current_pose.position.y;
            double dis_sq = dx * dx + dy * dy;

            if (dis_sq < min_dis_sq)
            {
                min_dis_sq = dis_sq;
                nearest_index = i;
            }
        }

        return nearest_index;
    }

    geometry_msgs::PoseStamped getCurrentTarget()
    {
        if (reference_trajectory_.poses.empty())
        {
            return geometry_msgs::PoseStamped();
        }

        // 获取当前位置
        geometry_msgs::Pose current_pose;
        {
            std::lock_guard<std::mutex> lock(odom_mutex_);
            if (!has_odom_) return geometry_msgs::PoseStamped();
            current_pose = current_odom_.pose.pose;
        }

        // 找到最近点
        int nearest_index = findNearestPointIndex(current_pose);
        if (nearest_index < 0)
        {
            return geometry_msgs::PoseStamped();
        }

        // 寻找前瞻点
        int target_index = nearest_index;
        double accumulated_distance = 0.0;

        for (int i = nearest_index; i < static_cast<int>(reference_trajectory_.poses.size()) - 1; ++i)
        {
            double dx = reference_trajectory_.poses[i + 1].pose.position.x -
                reference_trajectory_.poses[i].pose.position.x;
            double dy = reference_trajectory_.poses[i + 1].pose.position.y -
                reference_trajectory_.poses[i].pose.position.y;
            accumulated_distance += std::hypot(dx, dy);

            if (accumulated_distance >= lookahead_distance_)
            {
                target_index = i + 1;
                break;
            }
        }

        // ROS_INFO("[TestControlNode]: Nearest point index: %d, target point index: %d", nearest_index, target_index);

        // 如果已经到了终点
        if (target_index >= static_cast<int>(reference_trajectory_.poses.size()))
        {
            target_index = reference_trajectory_.poses.size() - 1;
        }

        return reference_trajectory_.poses[target_index];
    }

    void computeControlOutput(const geometry_msgs::PoseStamped & target_pose,
        const nav_msgs::Odometry & current_odom,
        double & v, double & w)
    {
        // 获取当前位置和方向
        double x = current_odom.pose.pose.position.x;
        double y = current_odom.pose.pose.position.y;

        // 从四元数获取偏航角
        tf2::Quaternion q(
            current_odom.pose.pose.orientation.x,
            current_odom.pose.pose.orientation.y,
            current_odom.pose.pose.orientation.z,
            current_odom.pose.pose.orientation.w);
        tf2::Matrix3x3 m(q);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        // 计算到目标点的向量
        double dx = target_pose.pose.position.x - x;
        double dy = target_pose.pose.position.y - y;
        double distance_error = std::hypot(dx, dy);

        double angle_error;
        // ****************** 新增：终点附近处理 ******************
        // 当非常接近终点时，使用更稳定的控制策略
        if (distance_error < 0.3)  // 距离小于0.3m时
        {
            // 从目标点获取目标偏航角
            tf2::Quaternion target_q(
                target_pose.pose.orientation.x,
                target_pose.pose.orientation.y,
                target_pose.pose.orientation.z,
                target_pose.pose.orientation.w);
            tf2::Matrix3x3 target_m(target_q);
            double target_roll, target_pitch, target_yaw;
            target_m.getRPY(target_roll, target_pitch, target_yaw);

            // 计算角度误差
            angle_error = target_yaw - yaw;
        }
        else
        {
            // 计算目标方向（全局坐标系）
            double target_yaw = std::atan2(dy, dx);
            angle_error = target_yaw - yaw;
        }

        // 计算角度误差（归一化到[-pi, pi]）
        while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
        while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

        // 获取目标速度（从轨迹点中提取）
        // 这里假设轨迹点的速度存储在pose的position.z中
        double target_vel = target_pose.pose.position.z;

        // 使用PID计算控制输出
        v = linear_pid_->compute(target_vel, current_odom.twist.twist.linear.x)
            + current_odom.twist.twist.linear.x;

        // 根据角度误差和距离误差计算角速度
        // 当距离较近时，减小角速度增益
        double angular_gain_factor = std::tanh(distance_error / lookahead_distance_);
        w = angular_pid_->compute(0.0, -angle_error * angular_gain_factor);

        // 限制输出
        if (v > max_linear_vel_) v = max_linear_vel_;
        if (v < -max_linear_vel_) v = -max_linear_vel_;
        if (w > max_angular_vel_) w = max_angular_vel_;
        if (w < -max_angular_vel_) w = -max_angular_vel_;
    }

    bool checkGoalReached(const nav_msgs::Odometry & current_odom)
    {
        if (reference_trajectory_.poses.empty()) return false;

        geometry_msgs::Pose & goal_pose = reference_trajectory_.poses.back().pose;
        double dx = goal_pose.position.x - current_odom.pose.pose.position.x;
        double dy = goal_pose.position.y - current_odom.pose.pose.position.y;
        double distance_to_goal = std::hypot(dx, dy);

        return distance_to_goal < goal_tolerance_;
    }

    void generateTestTrajectory()
    {
        reference_trajectory_.header.frame_id = "/map";
        reference_trajectory_.header.stamp = ros::Time::now();
        reference_trajectory_.poses.clear();

        switch (test_case_)
        {
        case 0: // 直线轨迹
            generateStraightTrajectory();
            break;
        case 1: // 圆形轨迹
            generateCircleTrajectory();
            break;
        case 2: // 正弦轨迹
            generateSineTrajectory();
            break;
        default:
            generateStraightTrajectory();
            break;
        }

        // 发布参考轨迹用于可视化
        ref_trajectory_pub_.publish(reference_trajectory_);
    }

    void generateStraightTrajectory()
    {
        double start_x = 0.0, start_y = 0.0;
        double end_x = 10.0, end_y = 0.0;
        double velocity = 0.5;  // m/s
        int num_points = 100;

        for (int i = 0; i <= num_points; ++i)
        {
            double t = static_cast<double>(i) / num_points;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = start_x + t * (end_x - start_x);
            pose.pose.position.y = start_y + t * (end_y - start_y);
            pose.pose.position.z = velocity;  // 速度存储在z中
            pose.pose.orientation.w = 1.0;
            reference_trajectory_.poses.push_back(pose);
        }
    }

    void generateCircleTrajectory()
    {
        double center_x = 0.0, center_y = 5.0;
        double radius = 5.0;
        double velocity = 0.8;  // m/s
        int num_points = 150;

        for (int i = 0; i <= num_points - 10; ++i)
        {
            double angle = 2.0 * M_PI * i / num_points - M_PI_2;
            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = center_x + radius * cos(angle);
            pose.pose.position.y = center_y + radius * sin(angle);
            pose.pose.position.z = velocity;

            // 设置朝向为切线方向
            double yaw = angle + M_PI_2;  // 切线方向
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation = tf2::toMsg(q);

            reference_trajectory_.poses.push_back(pose);
        }
    }

    void generateSineTrajectory()
    {
        double start_x = 0.0, end_x = 10.0;
        double amplitude = 0.5;
        double wavelength = 4.5;
        double phi = M_PI_2;
        double base_velocity = 0.6;  // 基础速度 m/s
        double min_velocity = 0.2;   // 最小速度 m/s
        double max_curvature = 2.0;  // 最大曲率限制
        int num_points = 200;

        for (int i = 0; i <= num_points; ++i)
        {
            double x = start_x + (end_x - start_x) * i / num_points;
            double y = amplitude * sin(2.0 * M_PI * x / wavelength + phi) - amplitude;

            // 计算一阶导数（切线斜率）
            double dy_dx = amplitude * (2.0 * M_PI / wavelength) * cos(2.0 * M_PI * x / wavelength + phi);

            // 计算二阶导数
            double d2y_dx2 = -amplitude * pow(2.0 * M_PI / wavelength, 2) * sin(2.0 * M_PI * x / wavelength + phi);

            // 计算曲率 κ = |y''| / (1 + (y')²)^(3/2)
            double curvature = fabs(d2y_dx2) / pow(1.0 + dy_dx * dy_dx, 1.5);

            // 限制曲率值，防止过大
            curvature = std::min(curvature, max_curvature);

            // 根据曲率调整速度：曲率越大，速度越小
            // 使用反比关系：velocity = base_velocity / (1 + curvature_factor * curvature)
            static const double curvature_factor = 3.0;  // 曲率影响因子，可根据需要调整
            double velocity = base_velocity / (1.0 + curvature_factor * curvature);

            // 确保速度不低于最小值
            velocity = std::max(velocity, min_velocity);

            geometry_msgs::PoseStamped pose;
            pose.header.frame_id = "/map";
            pose.header.stamp = ros::Time::now();
            pose.pose.position.x = x;
            pose.pose.position.y = y;
            pose.pose.position.z = velocity;  // 存储速度值

            // 计算切线方向
            double yaw = atan2(dy_dx, 1.0);
            tf2::Quaternion q;
            q.setRPY(0, 0, yaw);
            pose.pose.orientation = tf2::toMsg(q);

            reference_trajectory_.poses.push_back(pose);
        }
    }

    // 记录实际轨迹 - 按最小距离保存点
    void recordActualTrajectory(const nav_msgs::Odometry & odom)
    {
        double current_x = odom.pose.pose.position.x;
        double current_y = odom.pose.pose.position.y;

        // 检查是否需要保存新点（按最小距离）
        if (has_last_saved_pose_)
        {
            double dx = current_x - last_saved_pose_x_;
            double dy = current_y - last_saved_pose_y_;
            double distance = std::hypot(dx, dy);

            // 如果距离小于最小距离，不保存
            if (distance < min_trajectory_point_distance_)
            {
                return;
            }
        }

        // 创建新的轨迹点
        geometry_msgs::PoseStamped pose;
        pose.header.frame_id = "/map";
        pose.header.stamp = ros::Time::now();
        pose.pose = odom.pose.pose;

        // 保存新点
        actual_trajectory_.poses.push_back(pose);

        // 更新上一次保存的点
        last_saved_pose_x_ = current_x;
        last_saved_pose_y_ = current_y;
        has_last_saved_pose_ = true;

        // 更新轨迹头时间戳
        actual_trajectory_.header.stamp = ros::Time::now();

        // 限制实际轨迹长度，避免内存无限增长
        if (actual_trajectory_.poses.size() > 1000)
        {
            actual_trajectory_.poses.erase(actual_trajectory_.poses.begin());
        }

        // 定期发布实际轨迹
        if (ros::Time::now().toSec() - last_actual_trajectory_pub_time_ > 0.1)  // 10Hz
        {
            actual_trajectory_pub_.publish(actual_trajectory_);
            last_actual_trajectory_pub_time_ = ros::Time::now().toSec();
        }
    }

    void run()
    {
        ros::Rate rate(control_frequency_);

        while (ros::ok())
        {
            // 检查是否有里程计数据
            if (!has_odom_)
            {
                ros::spinOnce();
                rate.sleep();
                continue;
            }

            // 获取当前目标
            geometry_msgs::PoseStamped target_pose = getCurrentTarget();
            if (target_pose.header.frame_id.empty())
            {
                rate.sleep();
                continue;
            }

            // 获取当前里程计数据
            nav_msgs::Odometry current_odom;
            {
                std::lock_guard<std::mutex> lock(odom_mutex_);
                current_odom = current_odom_;
            }

            // 记录实际轨迹
            recordActualTrajectory(current_odom);

            // 计算控制输出
            double v = 0.0, w = 0.0;
            computeControlOutput(target_pose, current_odom, v, w);

            // 检查是否到达终点
            if (checkGoalReached(current_odom))
            {
                test_iteration_++;
                ROS_INFO("[TestControlNode]: Goal reached! Iteration: %d", test_iteration_);

                // 重置PID控制器
                linear_pid_->reset();
                angular_pid_->reset();

                // 可选：重新生成轨迹
                // generateTestTrajectory();

                v = 0.0;
                w = 0.0;
            }

            // 发布控制指令
            geometry_msgs::Twist cmd_vel;
            cmd_vel.linear.x = v;
            cmd_vel.angular.z = w;
            cmd_vel_pub_.publish(cmd_vel);

            // 定期发布参考轨迹
            if (ros::Time::now().toSec() - last_ref_trajectory_pub_time_ > 1.0)
            {
                ref_trajectory_pub_.publish(reference_trajectory_);
                last_ref_trajectory_pub_time_ = ros::Time::now().toSec();
            }

            ros::spinOnce();
            rate.sleep();
        }
    }

private:
    // ROS相关
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;
    ros::Subscriber odom_sub_;
    ros::Publisher cmd_vel_pub_;
    ros::Publisher ref_trajectory_pub_;
    ros::Publisher actual_trajectory_pub_;

    // 轨迹数据
    nav_msgs::Path reference_trajectory_;  // 参考轨迹
    nav_msgs::Path actual_trajectory_;     // 实际轨迹

    // 数据存储
    nav_msgs::Odometry current_odom_;
    std::mutex odom_mutex_;
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
    double min_trajectory_point_distance_;  // 最小轨迹点间距
    int test_case_;

    // 测试状态
    int test_iteration_;
    double last_ref_trajectory_pub_time_;
    double last_actual_trajectory_pub_time_;

    // 轨迹记录相关
    double last_saved_pose_x_;      // 上一次保存的轨迹点x坐标
    double last_saved_pose_y_;      // 上一次保存的轨迹点y坐标
    bool has_last_saved_pose_;      // 是否已经保存过轨迹点
};

int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_control");
    ros::NodeHandle private_nh("~");

    TestControlNode node(private_nh);
    node.run();

    return 0;
}