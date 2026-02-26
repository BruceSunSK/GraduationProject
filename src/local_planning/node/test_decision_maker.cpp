/**
 * @file test_decision_maker.cpp
 * @brief 决策模块单帧测试节点
 *        订阅参考线、代价地图、障碍物、车辆初始位姿，运行一次决策并可视化结果
 *        路径边界通过 RViz 和 matplotlib 显示，速度边界仅通过 matplotlib 显示
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>
#include <sys/types.h>
#include <errno.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/map/distance_map.h"
#include "global_planning/path/reference_path.h"
#include "global_planning/path/data_type.h"
#include "global_planning/path/utils.h"
#include "local_planning/decision/decision_maker.h"
#include "local_planning/obstacles/obstacle.h"
#include "local_planning/vehicle/data_type.h"
#include "perception/PredictedObstacles.h"

// ============================================================================
// 全局数据存储和标志
// ============================================================================
namespace TestData
{
// 数据接收标志
std::atomic<bool> map_received{false};
std::atomic<bool> reference_path_received{false};
std::atomic<bool> vehicle_state_received{false};
std::atomic<bool> obstacles_received{false};

// 数据存储
Map::MultiMap::Ptr map{nullptr};
Path::ReferencePath::Ptr reference_path{nullptr};
Vehicle::State::Ptr vehicle_state{nullptr};
Obstacle::Obstacle::List obstacles;

// 决策器
std::shared_ptr<Decision::DecisionMaker> decision_maker{nullptr};
Decision::DecisionMakerParams decision_params;

// 规划结果
Decision::PathBoundary path_boundary;
Decision::SpeedBoundary speed_boundary;

// 是否已执行测试
std::atomic<bool> test_executed{false};
}  // namespace TestData

// ============================================================================
// ROS 发布者
// ============================================================================
ros::Publisher g_visualization_marker_array_pub;

// ============================================================================
// 辅助函数声明
// ============================================================================
std_msgs::ColorRGBA CreateColor(double r, double g, double b, double a = 1.0);
bool CreateDirectoryRecursively(const std::string & path);
std::string GetCurrentTimestamp();
void SavePathBoundaryData(const Decision::PathBoundary & pb,
                          const Obstacle::Obstacle::List & obstacles,
                          const std::string & dir);
void SaveSpeedBoundaryData(const Decision::SpeedBoundary & sb,
                           const Obstacle::Obstacle::List & obstacles,
                           const std::string & dir);
void AddVehicleMarker(const Vehicle::State::Ptr & veh_state,
                      visualization_msgs::MarkerArray & marker_array,
                      double length = 3.8, double width = 2.0, double height = 1.5);
void PublishMarkers(const Decision::PathBoundary & pb,
                    const std::vector<Path::PathNode> & ref_points,
                    const Vehicle::State::Ptr & veh_state);
void RunTest();

// ============================================================================
// 回调函数
// ============================================================================

void CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
    if (TestData::map_received) return;

    try
    {
        TestData::map = std::make_shared<Map::MultiMap>();
        TestData::map->rows = msg->info.height;
        TestData::map->cols = msg->info.width;
        TestData::map->resolution = msg->info.resolution;
        TestData::map->origin_x = msg->info.origin.position.x;
        TestData::map->origin_y = msg->info.origin.position.y;

        cv::Mat cost_map = cv::Mat::zeros(msg->info.height, msg->info.width, CV_8UC1);
        for (size_t i = 0; i < msg->info.height; i++)
            for (size_t j = 0; j < msg->info.width; j++)
                cost_map.at<uchar>(i, j) = msg->data[i * msg->info.width + j];
        TestData::map->cost_map = std::move(cost_map);

        cv::Mat binary_map;
        cv::threshold(TestData::map->cost_map, binary_map, 99, 255, cv::THRESH_BINARY_INV);
        cv::Mat distance_map;
        cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32FC1);
        TestData::map->distance_map.SetMap(distance_map);

        TestData::map_received = true;
        ROS_INFO("Costmap received.");
        if (TestData::decision_maker)
        {
            TestData::decision_maker->SetCostMap(TestData::map);
        }
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("Error processing costmap: %s", e.what());
    }
}

void ReferencePathCallback(const nav_msgs::Path::ConstPtr & msg)
{
    try
    {
        std::vector<cv::Point2d> points;
        points.reserve(msg->poses.size());
        for (const auto & pose : msg->poses)
            points.emplace_back(pose.pose.position.x, pose.pose.position.y);

        TestData::reference_path = std::make_shared<Path::ReferencePath>(points, 4.0);
        TestData::reference_path_received = true;
        ROS_INFO("Reference path received: %zu points.", msg->poses.size());
        if (TestData::decision_maker)
        {
            TestData::decision_maker->SetReferencePath(TestData::reference_path);
        }
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("Error processing reference path: %s", e.what());
    }
}

void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
    if (TestData::vehicle_state_received)
    {
        ROS_DEBUG("Initial pose discarded (already have a pose).");
        return;
    }

    TestData::vehicle_state = std::make_shared<Vehicle::State>();
    TestData::vehicle_state->pos.x = msg->pose.pose.position.x;
    TestData::vehicle_state->pos.y = msg->pose.pose.position.y;
    TestData::vehicle_state->pos.theta = tf2::getYaw(msg->pose.pose.orientation);
    TestData::vehicle_state->v = 3.0;
    TestData::vehicle_state->w = 0.0;
    TestData::vehicle_state->pos.kappa = 0.0;

    TestData::vehicle_state_received = true;
    ROS_INFO("Initial pose received.");
}

void ObstaclesCallback(const perception::PredictedObstacles::ConstPtr & msg)
{
    TestData::obstacles.clear();
    for (const auto & obs_msg : msg->obstacles)
    {
        auto obstacle = std::make_shared<Obstacle::Obstacle>(obs_msg);
        TestData::obstacles.push_back(obstacle);
    }
    TestData::obstacles_received = true;
    // ROS_INFO("Obstacles received: %zu", TestData::obstacles.size());
}

// ============================================================================
// 辅助函数实现
// ============================================================================
std_msgs::ColorRGBA CreateColor(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

bool CreateDirectoryRecursively(const std::string & path)
{
    size_t pos = 0;
    std::string dir;
    int ret;
    while (pos < path.length())
    {
        pos = path.find('/', pos + 1);
        dir = path.substr(0, pos);
        if (dir.empty()) continue;
        ret = mkdir(dir.c_str(), 0755);
        if (ret != 0 && errno != EEXIST)
        {
            ROS_ERROR("Failed to create directory %s: %s", dir.c_str(), strerror(errno));
            return false;
        }
    }
    return true;
}

std::string GetCurrentTimestamp()
{
    auto now = std::chrono::system_clock::now();
    auto in_time_t = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::localtime(&in_time_t), "%Y%m%d_%H%M%S");
    return ss.str();
}

void SavePathBoundaryData(const Decision::PathBoundary & pb,
                          const Obstacle::Obstacle::List & obstacles,
                          const std::string & dir)
{
    std::string csv_path = dir + "/path_boundary.csv";
    std::ofstream file(csv_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open %s", csv_path.c_str());
        return;
    }

    // 写入障碍物注释（用于 SL 图）
    for (const auto & obs : obstacles)
    {
        auto proj = obs->GetProjection();
        file << "# obstacle: id=" << obs->GetId()
             << ", s=" << proj.s
             << ", l=" << proj.l
             << ", length=" << proj.length
             << ", width=" << proj.width
             << ", speed=" << obs->GetSpeed() << "\n";
    }

    // 表头：s, l_lower, l_upper (仅中间圆，下标1)
    file << "s,l_lower,l_upper\n";
    file << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < pb.s_coordinates.size(); ++i)
    {
        double s = pb.s_coordinates[i];
        double l_lower = pb.bounds[i][1].first;   // 中间圆下界（右侧）
        double l_upper = pb.bounds[i][1].second;  // 中间圆上界（左侧）
        file << s << "," << l_lower << "," << l_upper << "\n";
    }
    file.close();
    ROS_INFO("Path boundary saved to %s", csv_path.c_str());
}

void SaveSpeedBoundaryData(const Decision::SpeedBoundary & sb,
                           const Obstacle::Obstacle::List & obstacles,
                           const std::string & dir)
{
    std::string csv_path = dir + "/speed_boundary.csv";
    std::ofstream file(csv_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open %s", csv_path.c_str());
        return;
    }

    // 写入障碍物注释（用于 ST 图）
    for (const auto & obs : obstacles)
    {
        auto proj = obs->GetProjection();
        file << "# obstacle: id=" << obs->GetId()
             << ", s=" << proj.s
             << ", l=" << proj.l
             << ", length=" << proj.length
             << ", width=" << proj.width
             << ", speed=" << obs->GetSpeed() << "\n";
    }

    // 表头：t, s_lower, s_upper
    file << "t,s_lower,s_upper\n";
    file << std::fixed << std::setprecision(6);

    for (size_t i = 0; i < sb.time_points.size(); ++i)
    {
        double t = sb.time_points[i];
        double s_lower = sb.bounds[i].first;
        double s_upper = sb.bounds[i].second;
        file << t << "," << s_lower << "," << s_upper << "\n";
    }
    file.close();
    ROS_INFO("Speed boundary saved to %s", csv_path.c_str());
}

// 添加车辆轮廓 marker（长方体）
void AddVehicleMarker(const Vehicle::State::Ptr & veh_state,
                      visualization_msgs::MarkerArray & marker_array,
                      double length, double width, double height)
{
    if (!veh_state) return;

    visualization_msgs::Marker vehicle_marker;
    vehicle_marker.header.frame_id = "map";
    vehicle_marker.header.stamp = ros::Time::now();
    vehicle_marker.ns = "vehicle_footprint";
    vehicle_marker.id = 0;
    vehicle_marker.type = visualization_msgs::Marker::CUBE;
    vehicle_marker.action = visualization_msgs::Marker::ADD;

    // 设置位置和朝向
    vehicle_marker.pose.position.x = veh_state->pos.x;
    vehicle_marker.pose.position.y = veh_state->pos.y;
    vehicle_marker.pose.position.z = height / 2.0;  // 底面贴地，中心高度为一半

    tf2::Quaternion q;
    q.setRPY(0, 0, veh_state->pos.theta);
    vehicle_marker.pose.orientation = tf2::toMsg(q);

    // 尺寸
    vehicle_marker.scale.x = length;
    vehicle_marker.scale.y = width;
    vehicle_marker.scale.z = height;

    // 颜色：银白色半透明
    vehicle_marker.color = CreateColor(0.9, 0.9, 0.9, 0.8);

    vehicle_marker.lifetime = ros::Duration(0);
    marker_array.markers.push_back(vehicle_marker);
}

// 统一发布所有可视化 markers（车辆 + 路径边界）
void PublishMarkers(const Decision::PathBoundary & pb,
                    const std::vector<Path::PathNode> & ref_points,
                    const Vehicle::State::Ptr & veh_state)
{
    if (pb.bounds.empty() || pb.s_coordinates.empty()) return;

    visualization_msgs::MarkerArray marker_array;

    // 先添加车辆 marker
    AddVehicleMarker(veh_state, marker_array, 3.8, 2.0, 1.5);  // 车辆尺寸可调

    // 颜色定义
    std_msgs::ColorRGBA lower_color = CreateColor(1.0, 0.0, 0.0, 0.8); // 红
    std_msgs::ColorRGBA upper_color = CreateColor(0.0, 0.0, 1.0, 0.8); // 蓝

    // 下边界点（中间圆的下界）
    for (size_t i = 0; i < pb.bounds.size(); ++i)
    {
        double s = pb.s_coordinates[i];
        Path::PathNode node = TestData::reference_path->GetPathNode(s);
        double l_lower = pb.bounds[i][1].first;
        // 计算实际坐标：x = x_ref - l*sin(theta), y = y_ref + l*cos(theta)
        double x_lower = node.x - l_lower * std::sin(node.theta);
        double y_lower = node.y + l_lower * std::cos(node.theta);

        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "path_boundary_lower";
        m.id = i;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x_lower;
        m.pose.position.y = y_lower;
        m.pose.position.z = 0.5;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
        m.color = lower_color;
        m.lifetime = ros::Duration(0);
        marker_array.markers.push_back(m);
    }

    // 上边界点（中间圆的上界）
    for (size_t i = 0; i < pb.bounds.size(); ++i)
    {
        double s = pb.s_coordinates[i];
        Path::PathNode node = TestData::reference_path->GetPathNode(s);
        double l_upper = pb.bounds[i][1].second;
        double x_upper = node.x - l_upper * std::sin(node.theta);
        double y_upper = node.y + l_upper * std::cos(node.theta);

        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "path_boundary_upper";
        m.id = i;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = x_upper;
        m.pose.position.y = y_upper;
        m.pose.position.z = 0.5;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
        m.color = upper_color;
        m.lifetime = ros::Duration(0);
        marker_array.markers.push_back(m);
    }

    // 下边界折线
    visualization_msgs::Marker lower_line;
    lower_line.header.frame_id = "map";
    lower_line.header.stamp = ros::Time::now();
    lower_line.ns = "path_boundary_lower_line";
    lower_line.id = 0;
    lower_line.type = visualization_msgs::Marker::LINE_STRIP;
    lower_line.action = visualization_msgs::Marker::ADD;
    lower_line.pose.orientation.w = 1.0;
    lower_line.scale.x = 0.05;
    lower_line.color = lower_color;
    lower_line.lifetime = ros::Duration(0);

    for (size_t i = 0; i < pb.bounds.size(); ++i)
    {
        double s = pb.s_coordinates[i];
        Path::PathNode node = TestData::reference_path->GetPathNode(s);
        double l_lower = pb.bounds[i][1].first;
        double x = node.x - l_lower * std::sin(node.theta);
        double y = node.y + l_lower * std::cos(node.theta);
        geometry_msgs::Point p;
        p.x = x; p.y = y; p.z = 0.4;
        lower_line.points.push_back(p);
    }
    marker_array.markers.push_back(lower_line);

    // 上边界折线
    visualization_msgs::Marker upper_line;
    upper_line.header.frame_id = "map";
    upper_line.header.stamp = ros::Time::now();
    upper_line.ns = "path_boundary_upper_line";
    upper_line.id = 0;
    upper_line.type = visualization_msgs::Marker::LINE_STRIP;
    upper_line.action = visualization_msgs::Marker::ADD;
    upper_line.pose.orientation.w = 1.0;
    upper_line.scale.x = 0.05;
    upper_line.color = upper_color;
    upper_line.lifetime = ros::Duration(0);

    for (size_t i = 0; i < pb.bounds.size(); ++i)
    {
        double s = pb.s_coordinates[i];
        Path::PathNode node = TestData::reference_path->GetPathNode(s);
        double l_upper = pb.bounds[i][1].second;
        double x = node.x - l_upper * std::sin(node.theta);
        double y = node.y + l_upper * std::cos(node.theta);
        geometry_msgs::Point p;
        p.x = x; p.y = y; p.z = 0.4;
        upper_line.points.push_back(p);
    }
    marker_array.markers.push_back(upper_line);

    g_visualization_marker_array_pub.publish(marker_array);
    ROS_INFO("Published markers (vehicle + path boundary).");
}

void RunTest()
{
    if (!TestData::map_received || !TestData::reference_path_received ||
        !TestData::vehicle_state_received)
    {
        ROS_WARN("Data not ready, skipping test.");
        return;
    }

    ROS_INFO("=======================================");
    ROS_INFO("Running decision maker test...");
    ROS_INFO("=======================================");

    // 计算车辆在参考线上的投影（用于 ego_position）
    auto [veh_proj, idx] = TestData::reference_path->GetProjection(
        {TestData::vehicle_state->pos.x, TestData::vehicle_state->pos.y});
    Path::PointSL veh_sl = Path::Utils::XYtoSL(
        {TestData::vehicle_state->pos.x, TestData::vehicle_state->pos.y},
        {veh_proj.x, veh_proj.y}, veh_proj.s, veh_proj.theta);
    Path::PathNode ego_position = veh_proj;
    ego_position.l = veh_sl.l;
    ego_position.s = veh_sl.s;

    // 调用决策模块
    TestData::decision_maker->UpdateAndDecide(
        TestData::obstacles, ego_position, TestData::vehicle_state->v, 0.0);

    // 生成边界
    // 需要一组参考点（与路径规划一致，但这里只需边界，可用简单采样）
    double start_s = std::max(0.0, ego_position.s);
    double end_s = std::min(TestData::reference_path->GetLength(), ego_position.s + 25.0);
    double step = 0.5;
    std::vector<Path::PathNode> ref_points;
    for (double s = start_s; s <= end_s; s += step)
        ref_points.push_back(TestData::reference_path->GetPathNode(s));

    TestData::path_boundary = TestData::decision_maker->GeneratePathBoundary(ref_points);
    TestData::speed_boundary = TestData::decision_maker->GenerateSpeedBoundary(5.0, 0.1);

    // 保存数据
    std::string package_path = ros::package::getPath("local_planning");
    if (package_path.empty())
    {
        ROS_ERROR("Failed to get package path.");
        return;
    }
    std::string base_dir = package_path + "/result/test/decision_maker/";
    std::string timestamp = GetCurrentTimestamp();
    std::string target_dir = base_dir + timestamp + "/";
    if (!CreateDirectoryRecursively(target_dir))
    {
        ROS_ERROR("Failed to create directory %s", target_dir.c_str());
        return;
    }

    ROS_INFO("%s", TestData::decision_maker->GetDebugInfo().c_str());
    SavePathBoundaryData(TestData::path_boundary, TestData::obstacles, target_dir);
    SaveSpeedBoundaryData(TestData::speed_boundary, TestData::obstacles, target_dir);

    // RViz 可视化（统一发布车辆和路径边界）
    PublishMarkers(TestData::path_boundary, ref_points, TestData::vehicle_state);

    ROS_INFO("Test completed, data saved to %s", target_dir.c_str());
    TestData::test_executed = true;
}

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_decision_maker");
    ros::NodeHandle nh("~");

    // 获取话题参数
    std::string ref_topic, cost_topic, init_pose_topic, obstacles_topic;
    nh.param<std::string>("input_global_ref_topic",    ref_topic,       "/global_planning/path");
    nh.param<std::string>("input_costmap_topic",       cost_topic,      "/global_planning/costmap");
    nh.param<std::string>("input_initial_pose_topic",  init_pose_topic, "/initialpose");
    nh.param<std::string>("input_obstacles_topic",     obstacles_topic, "/perception/obstacles");

    // 创建发布者
    g_visualization_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker_array", 10, true);

    // 创建订阅者
    ros::Subscriber ref_sub       = nh.subscribe(ref_topic,       1, ReferencePathCallback);
    ros::Subscriber costmap_sub   = nh.subscribe(cost_topic,      1, CostmapCallback);
    ros::Subscriber init_pose_sub = nh.subscribe(init_pose_topic, 5, InitialPoseCallback);
    ros::Subscriber obstacles_sub = nh.subscribe(obstacles_topic, 5, ObstaclesCallback);

    // 初始化决策器
    TestData::decision_maker = std::make_shared<Decision::DecisionMaker>();
    TestData::decision_maker->Initialize(TestData::decision_params);

    ROS_INFO("Test decision maker node started.");
    ROS_INFO("Waiting for data... Test will run once when all data received.");

    // 手动设置车辆起点位置
    geometry_msgs::PoseWithCovarianceStamped msg;
    msg.header.frame_id = "/map";
    msg.header.stamp = ros::Time::now();
    msg.pose.pose.position.x = 53.5;
    msg.pose.pose.position.y = 183.5;
    msg.pose.pose.position.z = 0.0;
    msg.pose.pose.orientation.x = 0.0;
    msg.pose.pose.orientation.y = 0.0;
    msg.pose.pose.orientation.z = -0.0605622108305;
    msg.pose.pose.orientation.w = 0.998164424641;
    InitialPoseCallback(boost::make_shared<geometry_msgs::PoseWithCovarianceStamped>(msg));
    
    // 循环直到测试执行一次
    ros::Rate rate(10);
    while (ros::ok() && !TestData::test_executed)
    {
        ros::spinOnce();
        if (TestData::map_received && TestData::reference_path_received &&
            TestData::vehicle_state_received && TestData::obstacles_received)
        {
            RunTest();
        }
        rate.sleep();
    }

    ROS_INFO("Test finished, node exiting.");
    ros::spin();
    return 0;
}