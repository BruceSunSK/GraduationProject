#include "simulation/simulation.h"


Simulation::Simulation(ros::NodeHandle & private_nh) :
    private_nh_(private_nh),
    add_noise_(false),
    is_running_(true),
    last_update_time_(std::chrono::steady_clock::now())
{
    // 获取参数
    private_nh_.param<double>("simulation_frequency", simulation_frequency_, 100.0);
    private_nh_.param<double>("cmd_vel_timeout", cmd_vel_timeout_, 0.5);
    private_nh_.param<std::string>("world_frame", world_frame_, "/map");
    private_nh_.param<std::string>("robot_frame", robot_frame_, "/veh");

    // 获取车辆尺寸参数
    private_nh_.param<double>("vehicle/length", vehicle_length_, 3.8);
    private_nh_.param<double>("vehicle/width", vehicle_width_, 2.0);

    // 获取车辆速度、加速度参数
    double max_linear_vel, max_angular_vel;
    private_nh_.param<double>("vehicle/max_linear_vel", max_linear_vel, 4.0);
    private_nh_.param<double>("vehicle/max_angular_vel", max_angular_vel, 1.5);
    double max_linear_acc, max_angular_acc;
    private_nh_.param<double>("vehicle/max_linear_acc", max_linear_acc, std::numeric_limits<double>::max());
    private_nh_.param<double>("vehicle/max_angular_acc", max_angular_acc, std::numeric_limits<double>::max());

    // 获取初始位置
    double init_x, init_y, init_theta;
    private_nh_.param<double>("initial_position/x", init_x, 0.0);
    private_nh_.param<double>("initial_position/y", init_y, 0.0);
    private_nh_.param<double>("initial_position/theta", init_theta, 0.0);

    // 获取噪声参数
    double linear_noise, angular_noise, position_noise, velocity_noise;
    private_nh_.param<bool>("noise/add_noise", add_noise_, true);
    private_nh_.param<double>("noise/linear_stddev", linear_noise, 0.002);
    private_nh_.param<double>("noise/angular_stddev", angular_noise, 0.005);
    private_nh_.param<double>("noise/position_stddev", position_noise, 0.0005);
    private_nh_.param<double>("noise/velocity_stddev", velocity_noise, 0.001);

    // 初始化车辆模型
    vehicle_model_.initialize(max_linear_vel, max_angular_vel, max_linear_acc, max_angular_acc);
    vehicle_model_.setInitialState(init_x, init_y, init_theta);
    vehicle_model_.setNoiseParameters(linear_noise, angular_noise, position_noise, velocity_noise);

    // 初始化控制输入
    current_cmd_vel_.linear.x = 0.0;
    current_cmd_vel_.angular.z = 0.0;
    last_cmd_vel_time_ = ros::Time::now();

    // 初始化订阅者和发布者
    cmd_vel_sub_      = private_nh.subscribe("/cmd_vel", 1, &Simulation::cmdVelCallback, this);
    initial_pose_sub_ = private_nh.subscribe("/initialpose", 1, &Simulation::initialPoseCallback, this);
    odom_pub_         = private_nh.advertise<nav_msgs::Odometry>("/odom", 10);
    marker_pub_       = private_nh.advertise<visualization_msgs::Marker>("/vehicle_marker", 10);

    // 初始化服务
    reset_service_ = private_nh.advertiseService("reset_simulation", &Simulation::resetCallback, this);

    ROS_INFO("[Simulation]: Simulation module initialized");
    ROS_INFO("[Simulation]:   Simulation frequency: %.1f Hz", simulation_frequency_);
    ROS_INFO("[Simulation]:   Cmd_vel timeout: %.2f s", cmd_vel_timeout_);
    ROS_INFO("[Simulation]:   Vehicle size: %.2f x %.2f m", vehicle_length_, vehicle_width_);
    ROS_INFO("[Simulation]:   Max linear velocity: %.2f m/s", max_linear_vel);
    ROS_INFO("[Simulation]:   Max angular velocity: %.2f rad/s", max_angular_vel);
    if (max_linear_acc < std::numeric_limits<double>::max())
    {
        ROS_INFO("[Simulation]:   Max linear acceleration: %.2f m/s²", max_linear_acc);
    }
    else
    {
        ROS_INFO("[Simulation]:   Max linear acceleration: unlimited");
    }
    if (max_angular_acc < std::numeric_limits<double>::max())
    {
        ROS_INFO("[Simulation]:   Max angular acceleration: %.2f rad/s²", max_angular_acc);
    }
    else
    {
        ROS_INFO("[Simulation]:   Max angular acceleration: unlimited");
    }
    ROS_INFO("[Simulation]:   Initial position: x=%.2f, y=%.2f, theta=%.2f", init_x, init_y, init_theta);
    ROS_INFO("[Simulation]:   Initial noise mode: %s", add_noise_ ? "ON" : "OFF");
}

Simulation::~Simulation()
{
}

void Simulation::cmdVelCallback(const geometry_msgs::Twist::ConstPtr & msg)
{
    std::lock_guard<std::mutex> lock(cmd_vel_mutex_);
    current_cmd_vel_ = *msg;
    last_cmd_vel_time_ = ros::Time::now();
}

void Simulation::initialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
    // 从消息中提取位置和朝向
    double x = msg->pose.pose.position.x;
    double y = msg->pose.pose.position.y;
    double theta = getYawFromQuaternion(msg->pose.pose.orientation);

    // 设置新的初始状态
    vehicle_model_.setInitialState(x, y, theta);
    vehicle_model_.reset();

    ROS_INFO("[Simulation]: Vehicle initial pose set to: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
}

bool Simulation::resetCallback(std_srvs::Empty::Request & req,
    std_srvs::Empty::Response & res)
{
    vehicle_model_.reset();
    last_update_time_ = std::chrono::steady_clock::now();

    ROS_INFO("[Simulation]: Simulation reset to initial state");
    return true;
}

double Simulation::getYawFromQuaternion(const geometry_msgs::Quaternion & q)
{
    tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3 m(tf_q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    return yaw;
}

ros::Time Simulation::chronoToRosTime(const std::chrono::steady_clock::time_point & tp)
{
    auto duration = tp.time_since_epoch();
    auto sec = std::chrono::duration_cast<std::chrono::seconds>(duration);
    auto nsec = std::chrono::duration_cast<std::chrono::nanoseconds>(duration - sec);

    return ros::Time(sec.count(), nsec.count());
}

nav_msgs::Odometry Simulation::stateToOdometry(const VehicleState & state,
    const std::string & frame_id,
    const std::string & child_frame_id)
{
    nav_msgs::Odometry odom_msg;

    odom_msg.header.stamp = chronoToRosTime(state.timestamp);
    odom_msg.header.frame_id = frame_id;
    odom_msg.child_frame_id = child_frame_id;

    // 设置位置
    odom_msg.pose.pose.position.x = state.x;
    odom_msg.pose.pose.position.y = state.y;
    odom_msg.pose.pose.position.z = 0.0;

    // 设置方向（四元数）
    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    odom_msg.pose.pose.orientation = tf2::toMsg(q);

    // 设置速度
    odom_msg.twist.twist.linear.x = state.v;
    odom_msg.twist.twist.linear.y = 0.0;
    odom_msg.twist.twist.linear.z = 0.0;
    odom_msg.twist.twist.angular.x = 0.0;
    odom_msg.twist.twist.angular.y = 0.0;
    odom_msg.twist.twist.angular.z = state.w;

    return odom_msg;
}

void Simulation::publishVehicleMarker(const VehicleState & state)
{
    visualization_msgs::Marker marker;
    marker.header.frame_id = world_frame_;
    marker.header.stamp = chronoToRosTime(state.timestamp);
    marker.ns = "vehicle";
    marker.id = 0;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // 设置位置和朝向
    marker.pose.position.x = state.x;
    marker.pose.position.y = state.y;
    marker.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    marker.pose.orientation = tf2::toMsg(q);

    // 设置尺寸（长、宽、高）
    marker.scale.x = vehicle_length_;  // 长度
    marker.scale.y = vehicle_width_;   // 宽度
    marker.scale.z = 0.5;              // 高度（固定值）

    // 设置颜色（RGBA）
    marker.color.r = 0.2;   // 红色分量
    marker.color.g = 0.6;   // 绿色分量
    marker.color.b = 1.0;   // 蓝色分量
    marker.color.a = 0.8;   // 透明度

    marker.lifetime = ros::Duration(0.1);  // 标记的存活时间

    marker_pub_.publish(marker);
}

void Simulation::updateSimulation()
{
    auto current_time = std::chrono::steady_clock::now();
    std::chrono::duration<double> dt_duration = current_time - last_update_time_;
    double dt = dt_duration.count();

    if (dt <= 0)
    {
        dt = 0.001;  // 最小时间步长
    }

    // 检查控制指令是否超时
    geometry_msgs::Twist cmd_vel_to_use;
    {
        std::lock_guard<std::mutex> lock(cmd_vel_mutex_);

        ros::Time current_ros_time = ros::Time::now();
        double time_since_last_cmd = (current_ros_time - last_cmd_vel_time_).toSec();
        if (time_since_last_cmd > cmd_vel_timeout_)
        {
            // 超时，停止车辆
            cmd_vel_to_use.linear.x = 0.0;
            cmd_vel_to_use.angular.z = 0.0;
        }
        else
        {
            cmd_vel_to_use = current_cmd_vel_;
        }
    }

    // 更新车辆模型
    vehicle_model_.update(cmd_vel_to_use.linear.x, cmd_vel_to_use.angular.z, dt, add_noise_);

    last_update_time_ = current_time;
}

void Simulation::publishVehicleState()
{
    // 获取当前车辆状态
    VehicleState state = vehicle_model_.getCurrentState();

    // 转换为Odometry消息并发布
    nav_msgs::Odometry odom_msg = stateToOdometry(state, world_frame_, robot_frame_);
    odom_pub_.publish(odom_msg);

    // 发布TF变换
    geometry_msgs::TransformStamped transform_stamped;
    transform_stamped.header.stamp = odom_msg.header.stamp;
    transform_stamped.header.frame_id = world_frame_;
    transform_stamped.child_frame_id = robot_frame_;

    transform_stamped.transform.translation.x = state.x;
    transform_stamped.transform.translation.y = state.y;
    transform_stamped.transform.translation.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, state.theta);
    transform_stamped.transform.rotation = tf2::toMsg(q);

    tf_broadcaster_.sendTransform(transform_stamped);

    // 发布车辆可视化标记
    publishVehicleMarker(state);
}

void Simulation::run()
{
    ros::Rate rate(simulation_frequency_);

    ROS_INFO("[Simulation]: Simulation started");
    while (ros::ok() && is_running_)
    {
        // 更新仿真
        updateSimulation();

        // 发布车辆状态
        publishVehicleState();

        // 处理ROS回调
        ros::spinOnce();
        rate.sleep();
    }

    ROS_INFO("[Simulation]: Simulation stopped");
}