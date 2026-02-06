#include "local_planning/local_planning.h"


LocalPlanning::LocalPlanning(ros::NodeHandle & nh, const ros::Rate & loop_rate)
    : nh_(nh), loop_rate_(loop_rate)
    , tf_listener_(tf_buffer_)
    , planner_(nullptr)
    , is_initialized_(false)
{
    Initialize();
}

LocalPlanning::~LocalPlanning()
{
    ROS_INFO("[LocalPlanning]: LocalPlanning destroyed");
}

void LocalPlanning::Run()
{
    ROS_INFO("[LocalPlanning]: LocalPlanning is running...");
    while (ros::ok())
    {
        // todo
        loop_rate_.sleep();
        ros::spinOnce();
    }    
}


void LocalPlanning::Initialize()
{
    if (is_initialized_)
    {
        ROS_WARN("[LocalPlanning]: LocalPlanning already initialized");
        return;
    }

    // 1. 加载参数
    LoadParameters();

    // 2. 初始化订阅者
    InitializeSubscribers();

    // 3. 初始化发布者
    InitializePublishers();

    // 4. 初始化planner
    InitializePlanner();

    is_initialized_ = true;
    ROS_INFO("[LocalPlanning]: LocalPlanning initialized successfully");
}

void LocalPlanning::LoadParameters()
{
    // 加载订阅话题名称
    nh_.param("input_global_ref_topic",    input_ref_path_topic_,      std::string("/global_planning/path"));
    nh_.param("input_costmap_topic",       input_costmap_topic_,       std::string("/global_planning/costmap"));
    nh_.param("input_vehicle_state_topic", input_vehicle_state_topic_, std::string("/vehicle/odom"));
    nh_.param("input_obstacles_topic",     input_obstacles_topic_,     std::string("/perception/obstacles"));

    // 加载发布话题名称
    nh_.param("output_local_trajectory_topic", output_local_trajectory_topic_, std::string("trajectory"));
    nh_.param("output_debug_path_topic",       output_debug_path_topic_,       std::string("debug/path"));
    nh_.param("output_debug_speed_topic",      output_debug_speed_topic_,      std::string("debug/speed_profile"));


    ROS_INFO("[LocalPlanning]: Parameters loaded successfully:");
    ROS_INFO("[LocalPlanning]:   Input topics:");
    ROS_INFO("[LocalPlanning]:     - Global reference: %s", input_ref_path_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Costmap: %s", input_costmap_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Vehicle state: %s", input_vehicle_state_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Obstacles: %s", input_obstacles_topic_.c_str());
    ROS_INFO("[LocalPlanning]:   Output topics:");
    ROS_INFO("[LocalPlanning]:     - Local trajectory: %s", output_local_trajectory_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Debug path: %s", output_debug_path_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Debug speed: %s", output_debug_speed_topic_.c_str());
}

void LocalPlanning::InitializeSubscribers()
{
    sub_ref_path_      = nh_.subscribe(input_ref_path_topic_,      1, &LocalPlanning::ReferencePathCallback, this);
    sub_costmap_       = nh_.subscribe(input_costmap_topic_,       1, &LocalPlanning::CostmapCallback, this);
    sub_vehicle_state_ = nh_.subscribe(input_vehicle_state_topic_, 5, &LocalPlanning::VehicleStateCallback, this);

    // sub_obstacles_ = nh_.subscribe(input_obstacles_topic_, 1,
    //                               &LocalPlanning::PredictedObstaclesCallback<perception_prediction::PredictedObstacles>, this);

    ROS_INFO("[LocalPlanning]: All subscribers initialized");
}

void LocalPlanning::InitializePublishers()
{
    pub_local_trajectory_ = nh_.advertise<nav_msgs::Path>(output_local_trajectory_topic_, 1, true);
    pub_debug_path_       = nh_.advertise<nav_msgs::Path>(output_debug_path_topic_,       1, true);
    pub_debug_speed_      = nh_.advertise<nav_msgs::Path>(output_debug_speed_topic_,      1, true);

    ROS_INFO("[LocalPlanning]: All publishers initialized");
}

void LocalPlanning::InitializePlanner()
{
    LocalPlanner::LocalPlannerParams params;
    // todo ros参数服务器
    planner_ = std::make_unique<LocalPlanner>();
    planner_->InitParams(params);
    
    ROS_INFO("[LocalPlanning]: LocalPlanner initialized");
}

bool LocalPlanning::IsDataReady() const
{
    // todo: chuli
    // 检查是否收到全局参考线（必须有）
    // if (reference_line_.poses.empty())
    // {
    //     ROS_INFO_THROTTLE(1.0, "[LocalPlanning]: Waiting for global reference");
    //     return false;
    // }

    // 检查是否收到车辆位姿（必须有）
    // if (vehicle_pose_.header.frame_id.empty())
    // {
    //     ROS_INFO_THROTTLE(1.0, "[LocalPlanning]: Waiting for vehicle pose");
    //     return false;
    // }


    return true;
}

void LocalPlanning::ReferencePathCallback(const nav_msgs::Path::ConstPtr & msg)
{
    std::vector<cv::Point2d> points;
    points.reserve(msg->poses.size());
    for (const auto & pose : msg->poses)
    {
        points.emplace_back(pose.pose.position.x, pose.pose.position.y);
    }
    // 全局目前间隔是4.0m，因此保存备份也按照4.0m保存，后续截断全局参考线时再使用更小的间隔。
    Path::ReferencePath::Ptr reference_path = std::make_shared<Path::ReferencePath>(points, 4.0);
    planner_->SetReferencePath(reference_path);

    ROS_INFO("[LocalPlanning]: Received reference path with %lu points and %.2fm",
        msg->poses.size(), reference_path->GetLength());
}

void LocalPlanning::CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
    // 这部分直接是从全局规划算法内部抄过来并修改的代码。理论上这部分应该也用话题通讯传输数据，懒了。。。。
    Map::MultiMap::Ptr multi_map = std::make_shared<Map::MultiMap>();
    
    // 1. 离散代价值的障碍物地图，正常范围在[0, 100]，也包括255未探明的情况
    multi_map->rows = msg->info.height;
    multi_map->cols = msg->info.width;
    multi_map->resolution = msg->info.resolution;
    multi_map->origin_x = msg->info.origin.position.x;
    multi_map->origin_y = msg->info.origin.position.y;
    cv::Mat map = cv::Mat::zeros(multi_map->cols, multi_map->cols, CV_8UC1);
    for (size_t i = 0; i < multi_map->rows; i++)
    {
        for (size_t j = 0; j < multi_map->cols; j++)
        {
            map.at<uchar>(i, j) = msg->data[i * multi_map->cols + j];
        }
    }
    multi_map->cost_map = std::move(map);
    // 2. 代价值二值化后的障碍物地图，未探明情况视为障碍物。即[0, OBSTACLE_THRESHOLD)值为255，白色，前景；[OBSTACLE_THRESHOLD, 255]值为0，黑色，背景。
    cv::Mat binary_map;
    cv::threshold(multi_map->cost_map, binary_map, 99, 255, cv::ThresholdTypes::THRESH_BINARY_INV);    // 代价值>=99的视为障碍物，该值目前直接从全局部分copy
    // 3. 在二值化地图中进行distanceTransform变换的结果，用于描述每个栅格到障碍物的距离
    cv::Mat distance_map;
    cv::distanceTransform(binary_map, distance_map, cv::DIST_L2, cv::DIST_MASK_PRECISE, CV_32FC1);
    // 4. 设置距离地图，用于描述每个栅格到最近不可通过障碍物的距离
    multi_map->distance_map.SetMap(distance_map);

    planner_->SetMap(multi_map);
    ROS_INFO("[LocalPlanning]: Received costmap with resolution: %.2f, size: %dx%d",
        msg->info.resolution, msg->info.width, msg->info.height);
}

void LocalPlanning::VehicleStateCallback(const nav_msgs::Odometry::ConstPtr & msg)
{
    Vehicle::State::Ptr vehicle_state = std::make_shared<Vehicle::State>();
    vehicle_state->pos.x = msg->pose.pose.position.x;
    vehicle_state->pos.y = msg->pose.pose.position.y;
    vehicle_state->pos.theta = tf2::getYaw(msg->pose.pose.orientation);
    vehicle_state->v = msg->twist.twist.linear.x;
    vehicle_state->w = msg->twist.twist.angular.z;
    vehicle_state->pos.kappa = vehicle_state->w / (vehicle_state->v + 1e-6);
    planner_->SetVehicleState(vehicle_state);
}

// 模板函数实现（预留，待感知模块完成后实现）todo
template<typename T>
void LocalPlanning::PredictedObstaclesCallback(const typename T::ConstPtr & msg)
{
    // 预留实现，待感知预测功能包完成后补充
}