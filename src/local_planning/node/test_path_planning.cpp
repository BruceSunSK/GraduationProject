#include <atomic>
#include <chrono>

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2/utils.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/map/distance_map.h"
#include "global_planning/path/reference_path.h"
#include "global_planning/path/data_type.h" 
#include "local_planning/planners/local_planner.h"
#include "local_planning/vehicle/data_type.h"


// ============================================================================
// 全局数据存储和标志
// Global data storage and flags
// ============================================================================

/// @brief 测试数据命名空间
namespace TestData
{
// 数据接收标志
std::atomic<bool> map_received { false };
std::atomic<bool> reference_path_received { false };
std::atomic<bool> vehicle_state_received { false };

// 数据存储
Map::MultiMap::Ptr map { nullptr };
Path::ReferencePath::Ptr reference_path { nullptr };
Vehicle::State::Ptr vehicle_state { nullptr };

// 测试状态
std::atomic<bool> test_executed { false };
std::atomic<bool> test_success { false };
std::string last_error_msg;

// LocalPlanner实例
std::unique_ptr<LocalPlanner> planner { nullptr };

// 规划结果
std::vector<Path::TrajectoryPoint> trajectory_result;
}

// ============================================================================
// ROS发布者全局变量
// ROS publishers global variables
// ============================================================================

ros::Publisher g_local_trajectory_pub;

// ============================================================================
// ROS话题回调函数 - 简化版本，只设置标志
// ROS topic callback functions - simplified, only set flags
// ============================================================================

/**
 * @brief 代价地图回调函数
 * @param msg 代价地图消息
 */
void CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
    if (TestData::map_received)
    {
        return;
    }

    try
    {
        // 创建多层地图对象
        TestData::map = std::make_shared<Map::MultiMap>();

        // 设置基本地图信息
        TestData::map->rows = msg->info.height;
        TestData::map->cols = msg->info.width;
        TestData::map->resolution = msg->info.resolution;
        TestData::map->origin_x = msg->info.origin.position.x;
        TestData::map->origin_y = msg->info.origin.position.y;

        // 创建代价地图
        cv::Mat cost_map = cv::Mat::zeros(msg->info.height, msg->info.width, CV_8UC1);
        for (size_t i = 0; i < msg->info.height; i++)
        {
            for (size_t j = 0; j < msg->info.width; j++)
            {
                cost_map.at<uchar>(i, j) = msg->data[i * msg->info.width + j];
            }
        }
        TestData::map->cost_map = std::move(cost_map);

        // 创建二值化地图
        cv::Mat binary_map;
        cv::threshold(TestData::map->cost_map, binary_map, 99, 255, cv::THRESH_BINARY_INV);

        // 创建距离地图
        cv::Mat distance_map;
        cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32FC1);
        TestData::map->distance_map.SetMap(distance_map);

        TestData::map_received = true;

        // 设置到规划器中
        if (TestData::planner)
        {
            TestData::planner->SetMap(TestData::map);
        }

        ROS_INFO("Costmap received: %dx%d, resolution: %.2f",
            msg->info.width, msg->info.height, msg->info.resolution);
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("Error processing costmap: %s", e.what());
    }
}

/**
 * @brief 全局参考线回调函数
 * @param msg 参考线消息
 */
void ReferencePathCallback(const nav_msgs::Path::ConstPtr & msg)
{
    if (TestData::reference_path_received)
    {
        return;
    }

    // 即使已经接收过，也重新处理（可能数据更新）
    try
    {
        // 转换nav_msgs::Path到cv::Point2d向量
        std::vector<cv::Point2d> points;
        points.reserve(msg->poses.size());
        for (const auto & pose : msg->poses)
        {
            points.emplace_back(pose.pose.position.x, pose.pose.position.y);
        }

        // 创建参考线对象
        TestData::reference_path = std::make_shared<Path::ReferencePath>(points, 4.0);
        TestData::reference_path_received = true;

        // 设置到规划器中
        if (TestData::planner)
        {
            TestData::planner->SetReferencePath(TestData::reference_path);
        }

        ROS_INFO("Reference path received: %lu points, length: %.2fm",
            msg->poses.size(), TestData::reference_path->GetLength());
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("Error processing reference path: %s", e.what());
    }
}

/**
 * @brief 车辆状态回调函数
 * @param msg 车辆状态消息
 */
void VehicleStateCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    if (TestData::vehicle_state_received)
    {
        return;
    }

    try
    {
        // 创建车辆状态对象
        TestData::vehicle_state = std::make_shared<Vehicle::State>();

        // 提取位置和方向信息
        TestData::vehicle_state->pos.x = msg->pose.pose.position.x;
        TestData::vehicle_state->pos.y = msg->pose.pose.position.y;
        TestData::vehicle_state->pos.theta = tf2::getYaw(msg->pose.pose.orientation);

        // 提取速度信息
        TestData::vehicle_state->v = msg->twist.twist.linear.x;
        TestData::vehicle_state->w = msg->twist.twist.angular.z;

        // 计算曲率（防止除零）
        if (std::abs(TestData::vehicle_state->v) > 1e-6)
        {
            TestData::vehicle_state->pos.kappa = TestData::vehicle_state->w / TestData::vehicle_state->v;
        }
        else
        {
            TestData::vehicle_state->pos.kappa = 0.0;
        }

        TestData::vehicle_state_received = true;

        // 设置到规划器中
        if (TestData::planner)
        {
            TestData::planner->SetVehicleState(TestData::vehicle_state);
        }

        ROS_INFO("Vehicle state received: x=%.2f, y=%.2f, theta=%.2f, v=%.2f",
            TestData::vehicle_state->pos.x,
            TestData::vehicle_state->pos.y,
            TestData::vehicle_state->pos.theta,
            TestData::vehicle_state->v);
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("Error processing vehicle state: %s", e.what());
    }
}

// ============================================================================
// 辅助函数
// Helper functions
// ============================================================================

/**
 * @brief 等待所有数据就绪 - 简化版本，不使用条件变量
 * @param timeout_ms 超时时间（毫秒）
 * @return 如果所有数据就绪返回true，超时返回false
 */
bool WaitForData(int timeout_ms = 10000)
{
    auto start_time = std::chrono::steady_clock::now();

    while (true)
    {
        // 处理回调
        ros::spinOnce();

        // 检查是否所有数据就绪 - 直接检查原子标志
        if (TestData::map_received &&
            TestData::reference_path_received &&
            TestData::vehicle_state_received)
        {
            return true;
        }

        // 检查是否超时
        auto current_time = std::chrono::steady_clock::now();
        auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(
            current_time - start_time)
            .count();

        if (elapsed > timeout_ms)
        {
            ROS_WARN("Timeout waiting for data after %d ms", timeout_ms);
            return false;
        }

        // 每2秒显示一次状态
        static int last_status_time = 0;
        if (elapsed - last_status_time > 2000)
        {
            ROS_INFO("Waiting for data... Map: %s, RefPath: %s, Vehicle: %s, Elapsed: %ld ms",
                TestData::map_received ? "YES" : "NO",
                TestData::reference_path_received ? "YES" : "NO",
                TestData::vehicle_state_received ? "YES" : "NO",
                elapsed);
            last_status_time = elapsed;
        }

        // 短暂休眠，避免占用过多CPU
        ros::Duration(0.01).sleep();
    }
}

/**
 * @brief 将TrajectoryPoint向量转换为nav_msgs::Path消息
 * @param trajectory TrajectoryPoint向量
 * @param frame_id 坐标系ID
 * @return nav_msgs::Path消息
 */
nav_msgs::Path ConvertTrajectoryToPathMsg(const std::vector<Path::TrajectoryPoint> & trajectory,
    const std::string & frame_id = "/map")
{
    nav_msgs::Path path_msg;
    path_msg.header.stamp = ros::Time::now();
    path_msg.header.frame_id = frame_id;

    for (const auto & point : trajectory)
    {
        geometry_msgs::PoseStamped pose_stamped;
        pose_stamped.header = path_msg.header;
        pose_stamped.pose.position.x = point.x;
        pose_stamped.pose.position.y = point.y;
        pose_stamped.pose.position.z = 0.0;

        // 使用默认朝向（可以后续根据轨迹点速度方向计算，目前使用单位四元数）
        pose_stamped.pose.orientation.w = 1.0;
        pose_stamped.pose.orientation.x = 0.0;
        pose_stamped.pose.orientation.y = 0.0;
        pose_stamped.pose.orientation.z = 0.0;

        path_msg.poses.push_back(pose_stamped);
    }

    return path_msg;
}

/**
 * @brief 发布轨迹到ROS话题
 * @param trajectory 轨迹点向量
 */
void PublishTrajectory(const std::vector<Path::TrajectoryPoint> & trajectory)
{
    if (trajectory.empty())
    {
        ROS_WARN("Trajectory is empty, nothing to publish");
        return;
    }

    // 转换轨迹为ROS消息
    nav_msgs::Path trajectory_msg = ConvertTrajectoryToPathMsg(trajectory);
    g_local_trajectory_pub.publish(trajectory_msg);

    // 输出轨迹信息
    ROS_INFO("Trajectory info: points=%lu, start=(%.2f, %.2f), end=(%.2f, %.2f)",
        trajectory.size(),
        trajectory.front().x, trajectory.front().y,
        trajectory.back().x, trajectory.back().y);
}

/**
 * @brief 初始化LocalPlanner
 */
void InitializePlanner()
{
    // 创建LocalPlanner参数
    LocalPlanner::LocalPlannerParams params;

    // 可以根据需要调整参数
    // params.common.PLANNING_CYCLE_TIME = 0.1;
    // params.reference_path.S_INTERVAL = 0.5;
    // 等等...

    // 创建并初始化规划器
    TestData::planner = std::make_unique<LocalPlanner>();
    TestData::planner->InitParams(params);

    // 如果已经有数据，设置到规划器中
    if (TestData::map_received && TestData::planner)
    {
        TestData::planner->SetMap(TestData::map);
    }
    if (TestData::reference_path_received && TestData::planner)
    {
        TestData::planner->SetReferencePath(TestData::reference_path);
    }
    if (TestData::vehicle_state_received && TestData::planner)
    {
        TestData::planner->SetVehicleState(TestData::vehicle_state);
    }

    ROS_INFO("LocalPlanner initialized");
}

/**
 * @brief 执行单次规划测试
 */
void ExecuteTest()
{
    if (TestData::test_executed)
    {
        ROS_WARN("Test already executed");
        return;
    }

    ROS_INFO("=======================================");
    ROS_INFO("Starting single-frame planning test...");
    ROS_INFO("=======================================");

    // 检查规划器是否初始化
    if (!TestData::planner)
    {
        TestData::last_error_msg = "LocalPlanner not initialized";
        TestData::test_success = false;
        ROS_ERROR("Test failed: %s", TestData::last_error_msg.c_str());
        return;
    }

    // 清空之前的轨迹结果
    TestData::trajectory_result.clear();

    // 执行规划
    std::string error_msg;
    auto start_time = std::chrono::high_resolution_clock::now();

    // 调用新的Plan函数，传入轨迹向量接收结果
    bool success = TestData::planner->Plan(TestData::trajectory_result, error_msg);

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    if (success)
    {
        TestData::test_success = true;
        ROS_INFO("Planning successful, time: %ld ms", duration.count());

        // 发布轨迹到ROS话题
        PublishTrajectory(TestData::trajectory_result);

        ROS_INFO("=======================================");
        ROS_INFO("Test completed successfully!");
        ROS_INFO("=======================================");
    }
    else
    {
        TestData::test_success = false;
        TestData::last_error_msg = error_msg;
        ROS_ERROR("Planning failed, time: %ld ms", duration.count());
        ROS_ERROR("Error: %s", error_msg.c_str());
    }

    TestData::test_executed = true;

    ROS_INFO("Node will continue running for RViz visualization.");
    ROS_INFO("Press Ctrl+C to exit.");
}


// ============================================================================
// 主函数 - 保持原有结构，但使用简化的WaitForData
// ============================================================================
/**
 * @brief 主函数
 */
int main(int argc, char ** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "test_path_planning_node");
    ros::NodeHandle nh("~");

    ROS_INFO("Test Path Planning Node started");
    ROS_INFO("Waiting for data from other nodes...");

    // 获取话题名称
    std::string ref_path_topic, costmap_topic, vehicle_state_topic;
    nh.param("input_global_ref_topic",    ref_path_topic,      std::string("/global_planning/path"));
    nh.param("input_costmap_topic",       costmap_topic,       std::string("/global_planning/costmap"));
    nh.param("input_vehicle_state_topic", vehicle_state_topic, std::string("/vehicle/odom"));

    // 创建发布者（用于可视化）
    g_local_trajectory_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);

    // 创建订阅者
    ros::Subscriber ref_sub     = nh.subscribe(ref_path_topic, 1, ReferencePathCallback);
    ros::Subscriber costmap_sub = nh.subscribe(costmap_topic, 1, CostmapCallback);
    ros::Subscriber vehicle_sub = nh.subscribe(vehicle_state_topic, 5, VehicleStateCallback);

    ROS_INFO("Subscribers created:");
    ROS_INFO("  - Reference path: %s", ref_path_topic.c_str());
    ROS_INFO("  - Costmap: %s", costmap_topic.c_str());
    ROS_INFO("  - Vehicle state: %s", vehicle_state_topic.c_str());

    // 初始化规划器
    LocalPlanner::LocalPlannerParams params;
    TestData::planner = std::make_unique<LocalPlanner>();
    TestData::planner->InitParams(params);
    ROS_INFO("LocalPlanner initialized");

    // 等待所有数据就绪 - 使用简化的等待函数
    if (WaitForData(600000))
    {
        ROS_INFO("All required data received");
        ExecuteTest();
    }
    else
    {
        ROS_ERROR("Failed to receive all required data");
        return 1;
    }

    // 保持节点运行以维持RViz可视化
    ROS_INFO("Entering spin() for visualization...");

    // 设置一个定时器，定期重新发布轨迹以保持RViz显示
    ros::Timer timer = nh.createTimer(ros::Duration(1.0), [](const ros::TimerEvent &) {
        if (!TestData::trajectory_result.empty())
        {
            PublishTrajectory(TestData::trajectory_result);
            ROS_INFO_THROTTLE(10, "Re-publishing trajectory for RViz visualization...");
        }
        });

    ros::spin();

    return 0;
}