#include <vector>
#include <cmath>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseArray.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include "simulation/model/differential_model.h"


// 全局变量用于发布
ros::Publisher odom_pub;
tf2_ros::TransformBroadcaster * tf_broadcaster;
std::string world_frame = "/map";
std::string robot_frame = "/veh";

// 轨迹记录
std::vector<VehicleState> trajectory;

// 发布状态到ROS
void publishState(const VehicleState & state, const std::string & suffix, const ros::Time & stamp)
{
    // 创建Odometry消息
    nav_msgs::Odometry odom_msg;
    odom_msg.header.stamp = stamp;
    odom_msg.header.frame_id = world_frame;
    odom_msg.child_frame_id = robot_frame + suffix;

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
    odom_msg.twist.twist.angular.z = state.w;

    // 发布
    odom_pub.publish(odom_msg);

    // 发布TF
    geometry_msgs::TransformStamped transform;
    transform.header = odom_msg.header;
    transform.child_frame_id = robot_frame + suffix;
    transform.transform.translation.x = state.x;
    transform.transform.translation.y = state.y;
    transform.transform.translation.z = 0.0;
    transform.transform.rotation = odom_msg.pose.pose.orientation;

    tf_broadcaster->sendTransform(transform);
}

// 测试1：直线运动
void testStraightLine(DifferentialModel & model)
{
    printf("=== 测试1: 直线运动 ===\n");

    double dt = 0.02;  // 50Hz
    double total_time = 5.0;
    int steps = static_cast<int>(total_time / dt);

    model.reset();
    trajectory.clear();

    // 前进运动
    for (int i = 0; i < steps; ++i)
    {
        VehicleState state = model.update(0.3, 0.0, dt, true);
        trajectory.push_back(state);

        ros::Time stamp = ros::Time::now();
        publishState(state, "_straight", stamp);

        ros::Duration(dt).sleep();
        ros::spinOnce();
    }

    VehicleState final_state = model.getCurrentState();
    printf("  结束位置: (%.3f, %.3f), 朝向: %.3f rad\n", final_state.x, final_state.y, final_state.theta);
    printf("  理论位置: (%.3f, 0.0), 朝向: 0.0 rad\n", 0.3 * total_time);

    ros::Duration(1.0).sleep();
}

// 测试2：圆周运动
void testCircularMotion(DifferentialModel & model)
{
    printf("=== 测试2: 圆周运动 ===\n");

    double dt = 0.02;
    double total_time = 10.0;
    int steps = static_cast<int>(total_time / dt);

    model.reset();
    trajectory.clear();

    double linear_vel = 0.2;
    double angular_vel = 0.2;

    for (int i = 0; i < steps; ++i)
    {
        VehicleState state = model.update(linear_vel, angular_vel, dt, true);
        trajectory.push_back(state);

        ros::Time stamp = ros::Time::now();
        publishState(state, "_circle", stamp);

        ros::Duration(dt).sleep();
        ros::spinOnce();
    }

    VehicleState final_state = model.getCurrentState();
    double expected_theta = angular_vel * total_time;

    printf("  结束位置: (%.3f, %.3f), 朝向: %.3f rad\n", final_state.x, final_state.y, final_state.theta);
    printf("  理论朝向: %.3f rad\n", expected_theta);

    ros::Duration(1.0).sleep();
}

// 测试3：正方形路径
void testSquarePath(DifferentialModel & model)
{
    printf("=== 测试3: 正方形路径 ===\n");

    double dt = 0.02;
    model.reset();
    trajectory.clear();

    // 定义正方形路径的4条边
    for (int side = 0; side < 4; ++side)
    {
        // 前进3秒
        int forward_steps = static_cast<int>(3.0 / dt);
        for (int i = 0; i < forward_steps; ++i)
        {
            VehicleState state = model.update(0.3, 0.0, dt, true);
            trajectory.push_back(state);

            ros::Time stamp = ros::Time::now();
            publishState(state, "_square", stamp);

            ros::Duration(dt).sleep();
            ros::spinOnce();
        }

        // 转弯90度（最后一边不转弯）
        if (side < 3)
        {
            int turn_steps = static_cast<int>(1.57 / (0.5 * dt));  // 用0.5rad/s转90度
            for (int i = 0; i < turn_steps; ++i)
            {
                VehicleState state = model.update(0.0, 0.5, dt, true);
                trajectory.push_back(state);

                ros::Time stamp = ros::Time::now();
                publishState(state, "_square", stamp);

                ros::Duration(dt).sleep();
                ros::spinOnce();
            }
        }
    }

    VehicleState final_state = model.getCurrentState();
    printf("  结束位置: (%.3f, %.3f), 朝向: %.3f rad\n", final_state.x, final_state.y, final_state.theta);
    printf("  理想情况应回到原点附近\n");

    ros::Duration(1.0).sleep();
}

// 测试4：噪声对比
void testNoiseComparison(DifferentialModel & model)
{
    printf("=== 测试4: 噪声对比 ===\n");

    double dt = 0.02;
    double total_time = 8.0;
    int steps = static_cast<int>(total_time / dt);

    // 创建两个模型：一个无噪声，一个有噪声
    DifferentialModel ideal_model = model;
    DifferentialModel noisy_model = model;

    ideal_model.reset();
    noisy_model.reset();

    // 保存两个轨迹
    std::vector<VehicleState> ideal_traj;
    std::vector<VehicleState> noisy_traj;

    for (int i = 0; i < steps; ++i)
    {
        // 变速度控制
        double time_ratio = static_cast<double>(i) / steps;
        double linear_vel = 0.4 * (1.0 - 0.5 * sin(2.0 * M_PI * time_ratio));
        double angular_vel = 0.3 * sin(M_PI * time_ratio);

        // 理想模型
        VehicleState ideal_state = ideal_model.update(linear_vel, angular_vel, dt, false);
        ideal_traj.push_back(ideal_state);

        // 噪声模型
        VehicleState noisy_state = noisy_model.update(linear_vel, angular_vel, dt, true);
        noisy_traj.push_back(noisy_state);

        // 发布两个状态
        ros::Time stamp = ros::Time::now();
        publishState(ideal_state, "_ideal", stamp);
        publishState(noisy_state, "_noisy", stamp);

        ros::Duration(dt).sleep();
        ros::spinOnce();
    }

    // 计算误差
    if (!ideal_traj.empty() && !noisy_traj.empty())
    {
        VehicleState final_ideal = ideal_traj.back();
        VehicleState final_noisy = noisy_traj.back();

        double pos_error = sqrt(pow(final_noisy.x - final_ideal.x, 2) +
            pow(final_noisy.y - final_ideal.y, 2));

        double theta_error = fabs(final_noisy.theta - final_ideal.theta);
        // 归一化角度误差到[-π, π]
        while (theta_error > M_PI) theta_error -= 2.0 * M_PI;
        while (theta_error < -M_PI) theta_error += 2.0 * M_PI;
        theta_error = fabs(theta_error);

        printf("  理想模型位置: (%.3f, %.3f), 朝向: %.3f\n", final_ideal.x, final_ideal.y, final_ideal.theta);
        printf("  噪声模型位置: (%.3f, %.3f), 朝向: %.3f\n", final_noisy.x, final_noisy.y, final_noisy.theta);
        printf("  位置误差: %.3f m\n", pos_error);
        printf("  朝向误差: %.3f rad (%.1f°)\n", theta_error, theta_error * 180.0 / M_PI);
    }

    ros::Duration(2.0).sleep();
}

// 测试5：极限速度测试
void testLimitVelocity(DifferentialModel & model)
{
    printf("=== 测试5: 极限速度测试 ===\n");

    double dt = 0.02;
    double total_time = 3.0;
    int steps = static_cast<int>(total_time / dt);

    model.reset();
    trajectory.clear();

    // 发送超过限制的速度
    for (int i = 0; i < steps; ++i)
    {
        // 发送超出限制的速度指令
        VehicleState state = model.update(6.0, 6.0, dt, true);  // 超出4.0的限制

        ros::Time stamp = ros::Time::now();
        publishState(state, "_limit", stamp);

        ros::Duration(dt).sleep();
        ros::spinOnce();
    }

    VehicleState final_state = model.getCurrentState();
    printf("  结束速度: v=%.3f m/s, w=%.3f rad/s\n", final_state.v, final_state.w);
    printf("  速度应该被限制在4.0以内\n");

    ros::Duration(1.0).sleep();
}

int main(int argc, char ** argv)
{
    // 初始化ROS
    ros::init(argc, argv, "test_simulation");
    ros::NodeHandle private_nh("~");

    // 获取参数
    private_nh.param<std::string>("world_frame", world_frame, "/map");
    private_nh.param<std::string>("robot_frame", robot_frame, "/veh");

    // 初始化发布者
    odom_pub = private_nh.advertise<nav_msgs::Odometry>("odom", 10);

    // 初始化TF广播器
    tf2_ros::TransformBroadcaster broadcaster;
    tf_broadcaster = &broadcaster;

    // 创建车辆模型
    DifferentialModel vehicle_model;

    // 设置模型参数
    vehicle_model.initialize(4.0, 1.5);
    vehicle_model.setInitialState(0.0, 0.0, 0.0);
    vehicle_model.setNoiseParameters(0.002, 0.005, 0.0005, 0.001);

    printf("开始 DifferentialModel 测试\n");
    ros::Duration(1.0).sleep();  // 等待发布者建立连接

    // 运行测试
    testStraightLine(vehicle_model);
    testCircularMotion(vehicle_model);
    testSquarePath(vehicle_model);
    testNoiseComparison(vehicle_model);
    testLimitVelocity(vehicle_model);

    printf("=== 所有测试完成 ===\n");
    printf("在rviz中观察结果:\n");
    printf("  1. 添加 TF 显示\n");
    printf("  2. 添加 Odometry 显示，话题设置为 ~/odom\n");
    printf("  3. 可以切换显示不同的frame_id后缀来查看不同测试\n");

    // 保持节点运行
    ros::spin();

    return 0;
}