#include <atomic>
#include <chrono>

#include <ros/ros.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
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

// LocalPlanner实例
std::unique_ptr<LocalPlanner> planner { nullptr };

// 规划结果
LocalPlanner::LocalPlannerResult planning_result;
std::string error_msg;
}

// ============================================================================
// ROS发布者全局变量
// ROS publishers global variables
// ============================================================================

ros::Publisher g_local_trajectory_pub;
ros::Publisher g_visualization_marker_array_pub;

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
        if (std::abs(TestData::vehicle_state->v) > 1e-3)
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
        pose_stamped.pose.position.z = 12.0;

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
 * @brief 创建颜色RGBA
 * @param r 红色分量 [0, 1]
 * @param g 绿色分量 [0, 1]
 * @param b 蓝色分量 [0, 1]
 * @param a 透明度 [0, 1]
 * @return std_msgs::ColorRGBA
 */
std_msgs::ColorRGBA CreateColor(double r, double g, double b, double a = 1.0)
{
    std_msgs::ColorRGBA color;
    color.r = r;
    color.g = g;
    color.b = b;
    color.a = a;
    return color;
}

/**
 * @brief 发布QP优化边界可视化信息
 * @param lower_bounds 下边界点数组
 * @param upper_bounds 上边界点数组
 * @param trajectory 轨迹点（用于参考位置）
 */
void PublishQPBoundaryVisualization(
    const std::vector<std::array<Path::PointXY, 3>> & lower_bounds,
    const std::vector<std::array<Path::PointXY, 3>> & upper_bounds,
    const std::vector<Path::TrajectoryPoint> & trajectory)
{
    if (lower_bounds.empty() || upper_bounds.empty())
    {
        ROS_WARN("QP boundaries are empty, nothing to visualize");
        return;
    }

    // 创建MarkerArray来包含所有可视化元素
    visualization_msgs::MarkerArray marker_array;

    // 1. 显示下边界点（红色系）
    for (size_t i = 0; i < lower_bounds.size(); ++i)
    {
        const auto & lb_points = lower_bounds[i];
        double g = Math::Lerp(0.2, 0.6, i / static_cast<double>(lower_bounds.size() - 1));

        // 为每个轨迹点的3个下边界点创建单独的Marker（便于区分）
        for (int j = 0; j < 3; ++j)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "qp_lower_bound";
            marker.id = i * 3 + j; // 唯一ID
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            // 设置位置
            marker.pose.position.x = lb_points[j].x;
            marker.pose.position.y = lb_points[j].y;
            marker.pose.position.z = 0.1; // 稍微抬高避免与地面重叠
            marker.pose.orientation.w = 1.0;

            // 设置大小
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;

            // 设置颜色 - 红色系，不同圆使用不同深浅
            marker.color = CreateColor(0.9, g, 0.2, 0.8);

            // 设置生命周期
            marker.lifetime = ros::Duration(0);

            marker_array.markers.push_back(marker);
        }
    }

    // 2. 显示上边界点（蓝色系）
    for (size_t i = 0; i < upper_bounds.size(); ++i)
    {
        const auto & ub_points = upper_bounds[i];
        double g = Math::Lerp(0.2, 0.6, i / static_cast<double>(upper_bounds.size() - 1));

        // 为每个轨迹点的3个上边界点创建单独的Marker
        for (int j = 0; j < 3; ++j)
        {
            visualization_msgs::Marker marker;
            marker.header.frame_id = "/map";
            marker.header.stamp = ros::Time::now();
            marker.ns = "qp_upper_bound";
            marker.id = i * 3 + j; // 唯一ID
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.action = visualization_msgs::Marker::ADD;

            // 设置位置
            marker.pose.position.x = ub_points[j].x;
            marker.pose.position.y = ub_points[j].y;
            marker.pose.position.z = 0.1; // 稍微抬高
            marker.pose.orientation.w = 1.0;

            // 设置大小
            marker.scale.x = 0.15;
            marker.scale.y = 0.15;
            marker.scale.z = 0.15;

            // 设置颜色 - 蓝色系，不同圆使用不同深浅
            marker.color = CreateColor(0.2, g, 0.9, 0.8);

            // 设置生命周期
            marker.lifetime = ros::Duration(0);

            marker_array.markers.push_back(marker);
        }
    }

    // 3. 连接同组边界点（显示边界框）
    // 3.1 连接下边界点（红色线框）
    for (size_t i = 0; i < lower_bounds.size(); ++i)
    {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "/map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "qp_lower_bound_lines";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;

        // 设置线宽
        line_marker.scale.x = 0.05;

        // 设置颜色 - 浅红色半透明
        line_marker.color = CreateColor(1.0, 0.5, 0.5, 0.6);

        // 添加点
        geometry_msgs::Point p1, p2, p3;
        p1.x = lower_bounds[i][0].x;
        p1.y = lower_bounds[i][0].y;
        p1.z = 0.05;

        p2.x = lower_bounds[i][1].x;
        p2.y = lower_bounds[i][1].y;
        p2.z = 0.05;

        p3.x = lower_bounds[i][2].x;
        p3.y = lower_bounds[i][2].y;
        p3.z = 0.05;

        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        line_marker.points.push_back(p3);

        line_marker.lifetime = ros::Duration(0);
        marker_array.markers.push_back(line_marker);
    }

    // 3.2 连接上边界点（蓝色线框）
    for (size_t i = 0; i < upper_bounds.size(); ++i)
    {
        visualization_msgs::Marker line_marker;
        line_marker.header.frame_id = "/map";
        line_marker.header.stamp = ros::Time::now();
        line_marker.ns = "qp_upper_bound_lines";
        line_marker.id = i;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.w = 1.0;

        // 设置线宽
        line_marker.scale.x = 0.05;

        // 设置颜色 - 浅蓝色半透明
        line_marker.color = CreateColor(0.5, 0.5, 1.0, 0.6);

        // 添加点
        geometry_msgs::Point p1, p2, p3;
        p1.x = upper_bounds[i][0].x;
        p1.y = upper_bounds[i][0].y;
        p1.z = 0.05;

        p2.x = upper_bounds[i][1].x;
        p2.y = upper_bounds[i][1].y;
        p2.z = 0.05;

        p3.x = upper_bounds[i][2].x;
        p3.y = upper_bounds[i][2].y;
        p3.z = 0.05;

        line_marker.points.push_back(p1);
        line_marker.points.push_back(p2);
        line_marker.points.push_back(p3);

        line_marker.lifetime = ros::Duration(0);
        marker_array.markers.push_back(line_marker);
    }

    // 4. 显示轨迹点与边界点的连接线（显示对应关系）
    if (!trajectory.empty() && trajectory.size() == lower_bounds.size())
    {
        for (size_t i = 0; i < trajectory.size(); i += 3) // 每3个点显示一次，避免过于密集
        {
            visualization_msgs::Marker connection_marker;
            connection_marker.header.frame_id = "/map";
            connection_marker.header.stamp = ros::Time::now();
            connection_marker.ns = "trajectory_boundary_connections";
            connection_marker.id = i;
            connection_marker.type = visualization_msgs::Marker::LINE_LIST;
            connection_marker.action = visualization_msgs::Marker::ADD;
            connection_marker.pose.orientation.w = 1.0;

            // 设置线宽
            connection_marker.scale.x = 0.02;

            // 设置颜色 - 灰色半透明
            connection_marker.color = CreateColor(0.7, 0.7, 0.7, 0.4);

            // 添加轨迹点到每个边界点的连线
            geometry_msgs::Point traj_point;
            traj_point.x = trajectory[i].x;
            traj_point.y = trajectory[i].y;
            traj_point.z = 0.0;

            // 连接到下边界点
            for (int j = 0; j < 3; ++j)
            {
                geometry_msgs::Point bound_point;
                bound_point.x = lower_bounds[i][j].x;
                bound_point.y = lower_bounds[i][j].y;
                bound_point.z = 0.05;

                connection_marker.points.push_back(traj_point);
                connection_marker.points.push_back(bound_point);
            }

            // 连接到上边界点
            for (int j = 0; j < 3; ++j)
            {
                geometry_msgs::Point bound_point;
                bound_point.x = upper_bounds[i][j].x;
                bound_point.y = upper_bounds[i][j].y;
                bound_point.z = 0.05;

                connection_marker.points.push_back(traj_point);
                connection_marker.points.push_back(bound_point);
            }

            connection_marker.lifetime = ros::Duration(0);
            marker_array.markers.push_back(connection_marker);
        }
    }

    // 5. 发布MarkerArray
    g_visualization_marker_array_pub.publish(marker_array);

    ROS_INFO("Published QP boundary visualization with %lu markers", marker_array.markers.size());
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
        TestData::test_success = false;
        ROS_ERROR("Test failed: LocalPlanner not initialized");
        return;
    }

    // 执行规划
    // 调用新的Plan函数，传入轨迹向量接收结果
    bool success = TestData::planner->Plan(TestData::planning_result, TestData::error_msg);
    if (success)
    {
        TestData::test_success = true;
        ROS_INFO("Planning successful, time: %ld ms", static_cast<long>(TestData::planning_result.planning_time));

        if (!TestData::planning_result.log.str().empty())
        {
            ROS_INFO("Planning log: \n%s", TestData::planning_result.log.str().c_str());
        }

        // 发布轨迹到ROS话题
        PublishTrajectory(TestData::planning_result.trajectory);

        // 发布QP边界可视化信息
        PublishQPBoundaryVisualization(
            TestData::planning_result.path_qp_lb,
            TestData::planning_result.path_qp_ub,
            TestData::planning_result.trajectory);

        ROS_INFO("=======================================");
        ROS_INFO("Test completed successfully!");
        ROS_INFO("=======================================");
    }
    else
    {
        TestData::test_success = false;
        ROS_ERROR("Planning failed, Error: %s", TestData::error_msg.c_str());
    }

    TestData::test_executed = true;

    ROS_INFO("Node will continue running for RViz visualization.");
    ROS_INFO("Press Ctrl+C to exit.");
}

// ============================================================================
// 主函数
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
    g_local_trajectory_pub           = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    g_visualization_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>("visualization_marker_array", 10, true);

    ROS_INFO("Publishers created:");
    ROS_INFO("  - Local trajectory: /trajectory");
    ROS_INFO("  - Visualization marker: /visualization_marker");
    ROS_INFO("  - Visualization marker array: /visualization_marker_array");

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
    ros::spin();
    return 0;
}