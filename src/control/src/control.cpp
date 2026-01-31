#include "control/control.h"


Control::Control(ros::NodeHandle & nh, ros::NodeHandle & private_nh)
    : nh_(nh),
    private_nh_(private_nh),
    tf_listener_(tf_buffer_),
    has_trajectory_(false),
    has_odom_(false)
{
    // 获取参数
    private_nh_.param<double>("lookahead_distance", lookahead_distance_, 0.5);
    private_nh_.param<double>("max_linear_vel", max_linear_vel_, 1.0);
    private_nh_.param<double>("max_angular_vel", max_angular_vel_, 1.0);
    private_nh_.param<double>("control_frequency", control_frequency_, 50.0);
    private_nh_.param<double>("goal_tolerance", goal_tolerance_, 0.1);

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
    trajectory_sub_ = nh_.subscribe("trajectory", 1, &Control::trajectoryCallback, this);
    odom_sub_ = nh_.subscribe("odom", 1, &Control::odometryCallback, this);
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    ROS_INFO("Control module initialized");
    ROS_INFO("Linear PID: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp_linear, ki_linear, kd_linear);
    ROS_INFO("Angular PID: Kp=%.3f, Ki=%.3f, Kd=%.3f", kp_angular, ki_angular, kd_angular);
}

Control::~Control()
{
    delete linear_pid_;
    delete angular_pid_;
}

void Control::trajectoryCallback(const nav_msgs::Path::ConstPtr & msg)
{
    std::lock_guard<std::mutex> lock(trajectory_mutex_);
    current_trajectory_ = *msg;
    has_trajectory_ = true;
}

void Control::odometryCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    current_odom_ = *msg;
    has_odom_ = true;
}

int Control::findNearestPointIndex(const geometry_msgs::Pose & current_pose)
{
    if (current_trajectory_.poses.empty())
    {
        return -1;
    }

    int nearest_index = 0;
    double min_dis_sq = std::numeric_limits<double>::max();

    for (size_t i = 0; i < current_trajectory_.poses.size(); ++i)
    {
        double dx = current_trajectory_.poses[i].pose.position.x - current_pose.position.x;
        double dy = current_trajectory_.poses[i].pose.position.y - current_pose.position.y;
        double dis_sq = dx * dx + dy * dy;

        if (dis_sq < min_dis_sq)
        {
            min_dis_sq = dis_sq;
            nearest_index = i;
        }
    }

    return nearest_index;
}

geometry_msgs::PoseStamped Control::getCurrentTarget()
{
    std::lock_guard<std::mutex> lock(trajectory_mutex_);

    if (current_trajectory_.poses.empty())
    {
        return geometry_msgs::PoseStamped();
    }

    // 获取当前位置
    geometry_msgs::Pose current_pose;
    {
        std::lock_guard<std::mutex> lock(odom_mutex_);
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

    for (int i = nearest_index; i < static_cast<int>(current_trajectory_.poses.size()) - 1; ++i)
    {
        double dx = current_trajectory_.poses[i + 1].pose.position.x -
            current_trajectory_.poses[i].pose.position.x;
        double dy = current_trajectory_.poses[i + 1].pose.position.y -
            current_trajectory_.poses[i].pose.position.y;
        accumulated_distance += std::hypot(dx, dy);

        if (accumulated_distance >= lookahead_distance_)
        {
            target_index = i + 1;
            break;
        }
    }

    // 如果已经到了终点
    if (target_index >= static_cast<int>(current_trajectory_.poses.size()))
    {
        target_index = current_trajectory_.poses.size() - 1;
    }

    return current_trajectory_.poses[target_index];
}

void Control::computeControlOutput(const geometry_msgs::PoseStamped & target_pose,
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

    // 计算目标方向（全局坐标系）
    double target_yaw = std::atan2(dy, dx);

    // 计算角度误差（归一化到[-pi, pi]）
    double angle_error = target_yaw - yaw;
    while (angle_error > M_PI) angle_error -= 2.0 * M_PI;
    while (angle_error < -M_PI) angle_error += 2.0 * M_PI;

    // 计算距离误差
    double distance_error = std::hypot(dx, dy);

    // 获取目标速度（从轨迹点中提取）
    // 这里假设轨迹点的速度存储在pose的position.z中
    // 实际应用中可能需要自定义消息类型
    double target_vel = target_pose.pose.position.z;

    // 使用PID计算控制输出
    v = linear_pid_->compute(target_vel, current_odom.twist.twist.linear.x);

    // 根据角度误差和距离误差计算角速度
    // 当距离较近时，减小角速度增益
    double angular_gain_factor = std::tanh(distance_error / lookahead_distance_);
    w = angular_pid_->compute(0.0, angle_error * angular_gain_factor);

    // 限制输出
    if (v > max_linear_vel_) v = max_linear_vel_;
    if (v < -max_linear_vel_) v = -max_linear_vel_;
    if (w > max_angular_vel_) v = max_angular_vel_;
    if (w < -max_angular_vel_) v = -max_angular_vel_;
}

void Control::run()
{
    ros::Rate rate(control_frequency_);

    while (ros::ok())
    {
        // 检查是否有轨迹和里程计数据
        if (!has_trajectory_ || !has_odom_)
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

        // 计算控制输出
        double v = 0.0, w = 0.0;
        computeControlOutput(target_pose, current_odom, v, w);

        // 检查是否到达终点
        {
            std::lock_guard<std::mutex> lock(trajectory_mutex_);
            if (!current_trajectory_.poses.empty())
            {
                geometry_msgs::Pose & goal_pose = current_trajectory_.poses.back().pose;
                double dx = goal_pose.position.x - current_odom.pose.pose.position.x;
                double dy = goal_pose.position.y - current_odom.pose.pose.position.y;
                double distance_to_goal = std::hypot(dx, dy);

                if (distance_to_goal < goal_tolerance_)
                {
                    v = 0.0;
                    w = 0.0;
                    ROS_INFO("Goal reached!");
                }
            }
        }

        // 发布控制指令
        geometry_msgs::Twist cmd_vel;
        cmd_vel.linear.x = v;
        cmd_vel.angular.z = w;
        cmd_vel_pub_.publish(cmd_vel);

        ros::spinOnce();
        rate.sleep();
    }
}