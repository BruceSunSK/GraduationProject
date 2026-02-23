/**
 * @file test_path_planning.cpp
 * @brief 局部路径规划测试节点（仅路径规划，无决策/速度）
 *        触发方式：收到全局参考线消息时执行一次规划
 *        车辆位姿来自 RViz 的 /initialpose 话题（无速度信息）
 *        每次规划仅使用一次位姿，之后需重新通过 RViz 设置起点
 *        规划成功后，计算曲率和障碍物距离，保存为 CSV 文件
 */

#include <atomic>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <sstream>
#include <sys/stat.h>   // for mkdir
#include <sys/types.h>
#include <errno.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <std_msgs/ColorRGBA.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include "global_planning/map/distance_map.h"
#include "global_planning/path/reference_path.h"
#include "global_planning/path/data_type.h"
#include "global_planning/path/utils.h"
#include "global_planning/smoothers/piecewise_jerk_path_smoother2.h"

#include "local_planning/planners/local_planner.h"  // 用于 LocalPlannerParams
#include "local_planning/vehicle/data_type.h"
#include "local_planning/vehicle/collision.h"

// ============================================================================
// 前向声明
// ============================================================================
namespace PathPlanningOnly
{
bool Run(const Map::MultiMap::Ptr & map,
         const Path::ReferencePath::Ptr & ref_path,
         const Vehicle::State::Ptr & veh_state,
         const LocalPlanner::LocalPlannerParams & params,
         LocalPlanner::LocalPlannerResult & result,
         std::string & error_msg);
}

// ============================================================================
// 全局数据存储和标志
// ============================================================================
namespace TestData
{
// 数据接收标志
std::atomic<bool> map_received { false };           // 地图只接收一次
std::atomic<bool> vehicle_state_received { false }; // 是否有未使用的车辆位姿

// 数据存储
Map::MultiMap::Ptr map { nullptr };
Path::ReferencePath::Ptr reference_path { nullptr };
Vehicle::State::Ptr vehicle_state { nullptr };      // v/w 均置为 0

// 规划器参数（使用默认值，可从参数服务器加载）
LocalPlanner::LocalPlannerParams params;

// 规划结果
LocalPlanner::LocalPlannerResult planning_result;
std::string error_msg;
}  // namespace TestData

// ============================================================================
// ROS 发布者
// ============================================================================
ros::Publisher g_local_trajectory_pub;
ros::Publisher g_visualization_marker_array_pub;

// ============================================================================
// 辅助函数声明
// ============================================================================
std_msgs::ColorRGBA CreateColor(double r, double g, double b, double a = 1.0);
void PublishTrajectory(const std::vector<Path::TrajectoryPoint> & traj);
void PublishQPBoundaryVisualization(
    const std::vector<std::array<Path::PointXY, 3>> & lower_bounds,
    const std::vector<std::array<Path::PointXY, 3>> & upper_bounds,
    const std::vector<Path::TrajectoryPoint> & trajectory);
bool CreateDirectoryRecursively(const std::string & path);
std::string GetCurrentTimestamp();
void SavePlanningData(const std::vector<Path::TrajectoryPoint> & trajectory,
                      const Map::MultiMap::Ptr & map);

// ============================================================================
// 回调函数
// ============================================================================
void CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
    if (TestData::map_received)
        return;

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
        ROS_INFO("Costmap received: %dx%d, resolution=%.2f",
                 msg->info.width, msg->info.height, msg->info.resolution);
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
        ROS_INFO("Reference path received: %zu points, length=%.2f m",
                 msg->poses.size(), TestData::reference_path->GetLength());
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("Error processing reference path: %s", e.what());
        return;
    }

    if (!TestData::map_received || !TestData::vehicle_state_received)
    {
        ROS_WARN_THROTTLE(2.0, "Waiting for map and/or initial pose... (map: %d, pose: %d)",
                          TestData::map_received.load(), TestData::vehicle_state_received.load());
        return;
    }

    ROS_INFO("=======================================");
    ROS_INFO("Triggering path planning (reference path updated)");
    ROS_INFO("=======================================");

    bool success = PathPlanningOnly::Run(
        TestData::map,
        TestData::reference_path,
        TestData::vehicle_state,
        TestData::params,
        TestData::planning_result,
        TestData::error_msg);

    if (success)
    {
        PublishTrajectory(TestData::planning_result.trajectory);
        PublishQPBoundaryVisualization(
            TestData::planning_result.path_qp_lb,
            TestData::planning_result.path_qp_ub,
            TestData::planning_result.trajectory);

        // 保存数据到 CSV
        SavePlanningData(TestData::planning_result.trajectory, TestData::map);

        ROS_INFO("Planning succeeded.");
    }
    else
    {
        ROS_ERROR("Planning failed: %s", TestData::error_msg.c_str());
    }

    TestData::vehicle_state_received = false;
}

void InitialPoseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr & msg)
{
    if (TestData::vehicle_state_received)
    {
        ROS_DEBUG("Initial pose discarded (previous pose not used yet).");
        return;
    }

    TestData::vehicle_state = std::make_shared<Vehicle::State>();
    TestData::vehicle_state->pos.x = msg->pose.pose.position.x;
    TestData::vehicle_state->pos.y = msg->pose.pose.position.y;
    TestData::vehicle_state->pos.theta = tf2::getYaw(msg->pose.pose.orientation);
    TestData::vehicle_state->v = 0.0;
    TestData::vehicle_state->w = 0.0;
    TestData::vehicle_state->pos.kappa = 0.0;

    TestData::vehicle_state_received = true;
    ROS_INFO("Initial pose received: x=%.2f, y=%.2f, theta=%.2f",
             TestData::vehicle_state->pos.x,
             TestData::vehicle_state->pos.y,
             TestData::vehicle_state->pos.theta);
}

// ============================================================================
// 可视化辅助函数实现
// ============================================================================
std_msgs::ColorRGBA CreateColor(double r, double g, double b, double a)
{
    std_msgs::ColorRGBA c;
    c.r = r; c.g = g; c.b = b; c.a = a;
    return c;
}

nav_msgs::Path ConvertTrajectoryToPathMsg(const std::vector<Path::TrajectoryPoint> & traj,
                                          const std::string & frame_id = "/map")
{
    nav_msgs::Path msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = frame_id;
    for (const auto & pt : traj)
    {
        geometry_msgs::PoseStamped ps;
        ps.header = msg.header;
        ps.pose.position.x = pt.x;
        ps.pose.position.y = pt.y;
        ps.pose.position.z = 1.0;
        ps.pose.orientation.w = 1.0;
        msg.poses.push_back(ps);
    }
    return msg;
}

void PublishTrajectory(const std::vector<Path::TrajectoryPoint> & traj)
{
    if (traj.empty())
    {
        ROS_WARN("Trajectory empty, not publishing");
        return;
    }
    g_local_trajectory_pub.publish(ConvertTrajectoryToPathMsg(traj));
    ROS_INFO("Published %zu trajectory points", traj.size());
}

void PublishQPBoundaryVisualization(
    const std::vector<std::array<Path::PointXY, 3>> & lower_bounds,
    const std::vector<std::array<Path::PointXY, 3>> & upper_bounds,
    const std::vector<Path::TrajectoryPoint> & trajectory)
{
    if (lower_bounds.empty() || upper_bounds.empty()) return;

    visualization_msgs::MarkerArray marker_array;

    // 固定颜色
    std_msgs::ColorRGBA lower_color = CreateColor(1.0, 0.0, 0.0, 0.8); // 红色
    std_msgs::ColorRGBA upper_color = CreateColor(0.0, 0.0, 1.0, 0.8); // 蓝色

    // 1. 下边界点（中间圆，下标1）
    for (size_t i = 0; i < lower_bounds.size(); ++i)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "qp_lower_bound_points";
        m.id = i;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = lower_bounds[i][1].x;
        m.pose.position.y = lower_bounds[i][1].y;
        m.pose.position.z = 1.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
        m.color = lower_color;
        m.lifetime = ros::Duration(0);
        marker_array.markers.push_back(m);
    }

    // 2. 上边界点（中间圆，下标1）
    for (size_t i = 0; i < upper_bounds.size(); ++i)
    {
        visualization_msgs::Marker m;
        m.header.frame_id = "map";
        m.header.stamp = ros::Time::now();
        m.ns = "qp_upper_bound_points";
        m.id = i;
        m.type = visualization_msgs::Marker::SPHERE;
        m.action = visualization_msgs::Marker::ADD;
        m.pose.position.x = upper_bounds[i][1].x;
        m.pose.position.y = upper_bounds[i][1].y;
        m.pose.position.z = 1.0;
        m.pose.orientation.w = 1.0;
        m.scale.x = m.scale.y = m.scale.z = 0.15;
        m.color = upper_color;
        m.lifetime = ros::Duration(0);
        marker_array.markers.push_back(m);
    }

    // 3. 下边界折线
    visualization_msgs::Marker lower_line;
    lower_line.header.frame_id = "map";
    lower_line.header.stamp = ros::Time::now();
    lower_line.ns = "qp_lower_bound_line";
    lower_line.id = 0;
    lower_line.type = visualization_msgs::Marker::LINE_STRIP;
    lower_line.action = visualization_msgs::Marker::ADD;
    lower_line.pose.orientation.w = 1.0;
    lower_line.scale.x = 0.1; // 线宽
    lower_line.color = lower_color;
    lower_line.lifetime = ros::Duration(0);

    for (size_t i = 0; i < lower_bounds.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = lower_bounds[i][1].x;
        p.y = lower_bounds[i][1].y;
        p.z = 0.8; // 略低于球体
        lower_line.points.push_back(p);
    }
    marker_array.markers.push_back(lower_line);

    // 4. 上边界折线
    visualization_msgs::Marker upper_line;
    upper_line.header.frame_id = "map";
    upper_line.header.stamp = ros::Time::now();
    upper_line.ns = "qp_upper_bound_line";
    upper_line.id = 0;
    upper_line.type = visualization_msgs::Marker::LINE_STRIP;
    upper_line.action = visualization_msgs::Marker::ADD;
    upper_line.pose.orientation.w = 1.0;
    upper_line.scale.x = 0.1;
    upper_line.color = upper_color;
    upper_line.lifetime = ros::Duration(0);

    for (size_t i = 0; i < upper_bounds.size(); ++i)
    {
        geometry_msgs::Point p;
        p.x = upper_bounds[i][1].x;
        p.y = upper_bounds[i][1].y;
        p.z = 0.8;
        upper_line.points.push_back(p);
    }
    marker_array.markers.push_back(upper_line);

    // 5. 车辆轮廓显示间隔（每 step 个点显示一个）
    std_msgs::ColorRGBA vehicle_color = CreateColor(0.9, 0.9, 0.9, 0.8); // 银白色
    const int step = 3;
    double length = TestData::params.vehicle.LENGTH;
    double width = TestData::params.vehicle.WIDTH;

    for (size_t i = 0; i < trajectory.size(); i += step)
    {
        const auto & pt = trajectory[i];

        visualization_msgs::Marker rect;
        rect.header.frame_id = "map";
        rect.header.stamp = ros::Time::now();
        rect.ns = "vehicle_footprint";
        rect.id = i;
        rect.type = visualization_msgs::Marker::LINE_STRIP;
        rect.action = visualization_msgs::Marker::ADD;
        rect.pose.orientation.w = 1.0;  // 不设置方向，通过顶点坐标实现旋转
        rect.scale.x = 0.05;  // 线宽
        rect.color = vehicle_color;
        rect.lifetime = ros::Duration(0);

        // 矩形四个角相对于车辆中心的局部坐标（车辆前向为 x 轴正向，左为 y 轴正向）
        double half_l = length / 2.0;
        double half_w = width / 2.0;
        std::vector<std::pair<double, double>> local_corners = {
            { half_l,  half_w },  // 前左
            { half_l, -half_w },  // 前右
            { -half_l, -half_w }, // 后右
            { -half_l,  half_w }  // 后左
        };

        geometry_msgs::Point p;
        for (const auto & corner : local_corners)
        {
            double lx = corner.first;
            double ly = corner.second;
            // 旋转到全局坐标系
            p.x = pt.x + lx * std::cos(pt.theta) - ly * std::sin(pt.theta);
            p.y = pt.y + lx * std::sin(pt.theta) + ly * std::cos(pt.theta);
            p.z = 0.2;  // 抬高一点，避免与地面重叠
            rect.points.push_back(p);
        }
        // 闭合矩形：再次添加第一个点
        {
            double lx = local_corners[0].first;
            double ly = local_corners[0].second;
            p.x = pt.x + lx * std::cos(pt.theta) - ly * std::sin(pt.theta);
            p.y = pt.y + lx * std::sin(pt.theta) + ly * std::cos(pt.theta);
            p.z = 0.2;
            rect.points.push_back(p);
        }

        marker_array.markers.push_back(rect);
    }

    g_visualization_marker_array_pub.publish(marker_array);
}

// ============================================================================
// 数据保存辅助函数
// ============================================================================
bool CreateDirectoryRecursively(const std::string & path)
{
    // 使用 POSIX mkdir 递归创建目录
    size_t pos = 0;
    std::string dir;
    int ret;

    while (pos < path.length())
    {
        pos = path.find('/', pos + 1);
        dir = path.substr(0, pos);
        if (dir.length() == 0) continue;
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

void SavePlanningData(const std::vector<Path::TrajectoryPoint> & trajectory,
                      const Map::MultiMap::Ptr & map)
{
    if (trajectory.empty())
    {
        ROS_WARN("Trajectory empty, not saving data");
        return;
    }

    // 获取包路径
    std::string package_path = ros::package::getPath("local_planning");
    if (package_path.empty())
    {
        ROS_ERROR("Failed to get package path for local_planning");
        return;
    }

    // 构建目标文件夹： package_path/result/test/path_planning/timestamp/
    std::string base_dir = package_path + "/result/test/path_planning/";
    std::string timestamp = GetCurrentTimestamp();
    std::string target_dir = base_dir + timestamp + "/";

    if (!CreateDirectoryRecursively(target_dir))
    {
        ROS_ERROR("Failed to create directory %s", target_dir.c_str());
        return;
    }

    std::string csv_path = target_dir + "data.csv";
    std::ofstream file(csv_path);
    if (!file.is_open())
    {
        ROS_ERROR("Failed to open file %s", csv_path.c_str());
        return;
    }

    // 写入 CSV 表头
    file << "s,x,y,kappa,obstacle_distance\n";
    file << std::fixed << std::setprecision(6);

    for (const auto & pt : trajectory)
    {
        // 计算障碍物距离（地图坐标转换）
        double map_x = (pt.x - map->origin_x) / map->resolution;
        double map_y = (pt.y - map->origin_y) / map->resolution;
        double obstacle_dist = map->distance_map.GetDistance(map_x, map_y);

        file << pt.s << ","
             << pt.x << ","
             << pt.y << ","
             << pt.kappa << ","
             << obstacle_dist << "\n";
    }

    file.close();
    ROS_INFO("Saved planning data to %s", csv_path.c_str());
}

// ============================================================================
// 纯路径规划核心函数实现（新增曲率计算和 s 填充）
// ============================================================================
namespace PathPlanningOnly
{

static Path::PointXY GlobalToLocal(const Path::PointXY & global,
                                   const Path::PathNode & ref)
{
    double dx = global.x - ref.x;
    double dy = global.y - ref.y;
    double local_x = dx * std::cos(ref.theta) + dy * std::sin(ref.theta);
    double local_y = -dx * std::sin(ref.theta) + dy * std::cos(ref.theta);
    return { local_x, local_y };
}

static std::vector<Path::PathNode> SampleReferencePoints(
    const Path::PathNode & start,
    const Path::ReferencePath::Ptr & ref_path,
    const LocalPlanner::LocalPlannerParams & params,
    double & s_interval)
{
    double begin_s = std::max(0.0, start.s);
    double end_s = std::min(ref_path->GetLength(),
                             start.s + params.reference_path.TRUNCATED_FORWARD_S);

    int num = std::ceil((end_s - begin_s) / params.reference_path.S_INTERVAL);
    double step = (end_s - begin_s) / num;
    std::vector<Path::PathNode> points;
    for (int i = 0; i <= num; ++i)
    {
        double s = begin_s + i * step;
        points.push_back(ref_path->GetPathNode(s));
    }
    s_interval = step;
    ROS_INFO("Sampled %zu reference points, s_step=%.3f", points.size(), step);
    return points;
}

static std::vector<std::array<std::pair<double, double>, 3>> GetMapBounds(
    const std::vector<Path::PathNode> & ref_points,
    const Map::MultiMap::Ptr & map,
    const Path::ReferencePath::Ptr & ref_path,
    const LocalPlanner::LocalPlannerParams & params,
    LocalPlanner::LocalPlannerResult & result)
{
    std::vector<std::array<std::pair<double, double>, 3>> bounds;
    bounds.reserve(ref_points.size());

    auto GetApproxNode = [&](const Path::PathNode & original,
                              const Path::PathNode & actual,
                              double len) -> Path::PathNode
    {
        auto prj = ref_path->GetPathNode(original.s + len);
        double x = prj.x, y = prj.y;
        Path::PointXY v1 = { actual.x - original.x, actual.y - original.y };
        Path::PointXY v2 = { x - original.x, y - original.y };
        double proj = (v1.x * v2.x + v1.y * v2.y) /
                      std::max(0.001, std::sqrt(v1.x * v1.x + v1.y * v1.y));
        double move = std::fabs(len) - proj;
        Path::PathNode ret;
        int sign = (len >= 0) ? 1 : -1;
        ret.x = x + sign * move * std::cos(original.theta);
        ret.y = y + sign * move * std::sin(original.theta);
        ret.theta = original.theta;
        return ret;
    };

    for (const auto & pt : ref_points)
    {
        Path::PathNode c0, c1, c2;
        c0.x = pt.x + params.vehicle.CENTER_TO_COLLISION_CENTER * std::cos(pt.theta);
        c0.y = pt.y + params.vehicle.CENTER_TO_COLLISION_CENTER * std::sin(pt.theta);
        c0.theta = pt.theta;
        c1 = pt;
        c2.x = pt.x - params.vehicle.CENTER_TO_COLLISION_CENTER * std::cos(pt.theta);
        c2.y = pt.y - params.vehicle.CENTER_TO_COLLISION_CENTER * std::sin(pt.theta);
        c2.theta = pt.theta;

        Vehicle::CollisionCircle cc(*map, { c0, c1, c2 },
                                     params.vehicle.COLLISION_CIRCLE_RADIUS,
                                     params.vehicle.COLLISION_SAFETY_MARGIN);

        std::array<Path::PointXY, 3> lb_xy, ub_xy;
        auto bd = cc.GetCollisionBounds(
            params.map.BOUND_SEARCH_RANGE,
            params.map.BOUND_SEARCH_LARGE_RESOLUTION,
            params.map.BOUND_SEARCH_SMALL_RESOLUTION,
            &lb_xy, &ub_xy);

        Path::PathNode c0_new = GetApproxNode(pt, c0, params.vehicle.CENTER_TO_COLLISION_CENTER);
        Path::PathNode c2_new = GetApproxNode(pt, c2, -params.vehicle.CENTER_TO_COLLISION_CENTER);
        double offset_0 = GlobalToLocal({ c0.x, c0.y }, c0_new).y;
        double offset_2 = GlobalToLocal({ c2.x, c2.y }, c2_new).y;
        bd[0].first += offset_0;  bd[0].second += offset_0;
        bd[2].first += offset_2;  bd[2].second += offset_2;

        double dis = params.vehicle.CENTER_TO_COLLISION_CENTER * std::sin(pt.theta);
        bd[0].first -= dis;  bd[0].second -= dis;
        bd[2].first += dis;  bd[2].second += dis;

        result.path_qp_lb.push_back(lb_xy);
        result.path_qp_ub.push_back(ub_xy);
        bounds.push_back(bd);
    }
    return bounds;
}

// 计算轨迹点的累积距离 s ，航向角 theta 和曲率 kappa
static void ComputePathProperties(const std::vector<Path::PointXY> & points,
    std::vector<double> & s,
    std::vector<double> & kappa,
    std::vector<double> & theta)  // 新增输出参数
{
    size_t n = points.size();
    if (n < 2)
    {
        s.clear();
        kappa.clear();
        theta.clear();
        return;
    }

    // 计算累积距离 s
    s.resize(n);
    s[0] = 0.0;
    for (size_t i = 1; i < n; ++i)
    {
        double dx = points[i].x - points[i - 1].x;
        double dy = points[i].y - points[i - 1].y;
        s[i] = s[i - 1] + std::sqrt(dx * dx + dy * dy);
    }

    // 计算每个点的航向角 theta（基于前后点差分）
    theta.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        if (i == 0)
        {
            double dx = points[1].x - points[0].x;
            double dy = points[1].y - points[0].y;
            theta[i] = std::atan2(dy, dx);
        }
        else if (i == n - 1)
        {
            double dx = points[n - 1].x - points[n - 2].x;
            double dy = points[n - 1].y - points[n - 2].y;
            theta[i] = std::atan2(dy, dx);
        }
        else
        {
            double dx = points[i + 1].x - points[i - 1].x;
            double dy = points[i + 1].y - points[i - 1].y;
            theta[i] = std::atan2(dy, dx);
        }
    }

    // 对 theta 进行 unwrap，避免跳变
    for (size_t i = 1; i < n; ++i)
    {
        double diff = theta[i] - theta[i - 1];
        if (diff > M_PI)
            theta[i] -= 2.0 * M_PI;
        else if (diff < -M_PI)
            theta[i] += 2.0 * M_PI;
    }

    // 计算曲率 kappa = dθ/ds
    kappa.resize(n);
    for (size_t i = 0; i < n; ++i)
    {
        if (i == 0)
        {
            kappa[i] = (theta[1] - theta[0]) / (s[1] - s[0]);
        }
        else if (i == n - 1)
        {
            kappa[i] = (theta[n - 1] - theta[n - 2]) / (s[n - 1] - s[n - 2]);
        }
        else
        {
            kappa[i] = (theta[i + 1] - theta[i - 1]) / (s[i + 1] - s[i - 1]);
        }
    }
}

bool Run(const Map::MultiMap::Ptr & map,
         const Path::ReferencePath::Ptr & ref_path,
         const Vehicle::State::Ptr & veh_state,
         const LocalPlanner::LocalPlannerParams & params,
         LocalPlanner::LocalPlannerResult & result,
         std::string & error_msg)
{
    result.Clear();
    error_msg.clear();

    auto [veh_proj, idx] = ref_path->GetProjection({ veh_state->pos.x, veh_state->pos.y });

    Path::PathNode start = veh_state->pos;
    Path::PointSL start_sl = Path::Utils::XYtoSL(
        { start.x, start.y }, { veh_proj.x, veh_proj.y }, veh_proj.s, veh_proj.theta);
    start.s = start_sl.s;
    start.l = start_sl.l;

    ROS_INFO("Planning start: s=%.3f, l=%.3f", start.s, start.l);

    double s_interval = 0.0;
    auto ref_points = SampleReferencePoints(start, ref_path, params, s_interval);
    if (ref_points.empty())
    {
        error_msg = "No reference points sampled";
        return false;
    }

    auto map_bounds = GetMapBounds(ref_points, map, ref_path, params, result);
    if (map_bounds.empty())
    {
        error_msg = "Failed to compute map bounds";
        return false;
    }

    const auto & final_bounds = map_bounds;

    Path::PointSLWithDerivatives start_state = Path::Utils::XYtoSL(
        { start.x, start.y },
        start.theta,
        start.kappa,
        { veh_proj.x, veh_proj.y },
        veh_proj.s,
        veh_proj.theta,
        veh_proj.kappa,
        veh_proj.dkappa);

    std::array<double, 3> init_state = { start_state.l,
                                         start_state.l_prime,
                                         start_state.l_double_prime };
    std::array<double, 3> end_state_ref = { 0.0, 0.0, 0.0 };

    const Smoother::PiecewiseJerkPathSmoother2::Weights weights {
        params.path_qp.WEIGHT_L,
        params.path_qp.WEIGHT_DL,
        params.path_qp.WEIGHT_DDL,
        params.path_qp.WEIGHT_DDDL,
        params.path_qp.WEIGHT_END_STATE_L,
        params.path_qp.WEIGHT_END_STATE_DL,
        params.path_qp.WEIGHT_END_STATE_DDL
    };
    const Smoother::PiecewiseJerkPathSmoother2::Params opt_params {
        params.path_qp.DL_LIMIT,
        params.vehicle.MAX_KAPPA
    };
    Smoother::PiecewiseJerkPathSmoother2 smoother(weights, opt_params);

    std::vector<Path::PointXY> optimized_path;
    if (!smoother.Solve(s_interval, ref_points, final_bounds,
                        init_state, end_state_ref, optimized_path))
    {
        error_msg = "Path QP optimization failed";
        return false;
    }

    ROS_INFO("Optimized path generated %zu points", optimized_path.size());

    // 计算路径点的累积距离 s、曲率 kappa 和航向角 theta
    std::vector<double> s_vals, kappa_vals, theta_vals;
    ComputePathProperties(optimized_path, s_vals, kappa_vals, theta_vals);

    result.trajectory.clear();
    result.trajectory.reserve(optimized_path.size());
    for (size_t i = 0; i < optimized_path.size(); ++i)
    {
        Path::TrajectoryPoint pt;
        pt.x = optimized_path[i].x;
        pt.y = optimized_path[i].y;
        pt.s = s_vals[i];
        pt.kappa = kappa_vals[i];
        pt.theta = theta_vals[i];  // 填充航向角
        // 其他字段保持默认值
        result.trajectory.push_back(pt);
    }

    return true;
}

}  // namespace PathPlanningOnly

// ============================================================================
// 主函数
// ============================================================================
int main(int argc, char ** argv)
{
    ros::init(argc, argv, "test_path_planning");
    ros::NodeHandle nh("~");

    std::string ref_topic, cost_topic, init_pose_topic;
    nh.param<std::string>("input_global_ref_topic",    ref_topic,       "/global_planning/path");
    nh.param<std::string>("input_costmap_topic",       cost_topic,      "/global_planning/costmap");
    nh.param<std::string>("input_initial_pose_topic",  init_pose_topic, "/initialpose");

    nh.param<double>("w_l",       TestData::params.path_qp.WEIGHT_L, 1.0);
    nh.param<double>("w_dl",      TestData::params.path_qp.WEIGHT_DL, 50.0);
    nh.param<double>("w_ddl",     TestData::params.path_qp.WEIGHT_DDL, 1000.0);
    nh.param<double>("w_dddl",    TestData::params.path_qp.WEIGHT_DDDL, 1000.0);
    nh.param<double>("w_end_l",   TestData::params.path_qp.WEIGHT_END_STATE_L, 1.0);
    nh.param<double>("w_end_dl",  TestData::params.path_qp.WEIGHT_END_STATE_DL, 5.0);
    nh.param<double>("w_end_ddl", TestData::params.path_qp.WEIGHT_END_STATE_DDL, 50.0);
    TestData::params.reference_path.TRUNCATED_FORWARD_S = 10000.0;

    g_local_trajectory_pub = nh.advertise<nav_msgs::Path>("trajectory", 1, true);
    g_visualization_marker_array_pub = nh.advertise<visualization_msgs::MarkerArray>(
        "visualization_marker_array", 10, true);

    ros::Subscriber ref_sub       = nh.subscribe(ref_topic,       1, ReferencePathCallback);
    ros::Subscriber costmap_sub   = nh.subscribe(cost_topic,      1, CostmapCallback);
    ros::Subscriber init_pose_sub = nh.subscribe(init_pose_topic, 5, InitialPoseCallback);

    ROS_INFO("Path planning test node started.");
    ROS_INFO("Subscribed topics:");
    ROS_INFO("  - Reference path: %s", ref_topic.c_str());
    ROS_INFO("  - Costmap: %s", cost_topic.c_str());
    ROS_INFO("  - Initial pose: %s", init_pose_topic.c_str());

    ROS_INFO("Waiting for data... Planning will be triggered upon receiving a new reference path.");
    ROS_INFO("Each planning uses one initial pose; set a new pose via RViz for next planning.");
    ROS_INFO("Planning data will be saved to result/test/path_planning/<timestamp>/data.csv");

    ros::spin();
    return 0;
}