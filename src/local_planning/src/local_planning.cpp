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
    // 清理资源
    if (planner_ != nullptr)
    {
        delete planner_;
        planner_ = nullptr;
    }

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
    nh_.param("input_global_ref_topic",   input_global_ref_topic_,   std::string("/global_planning/reference_path"));
    nh_.param("input_costmap_topic",      input_costmap_topic_,      std::string("/global_planning/costmap"));
    nh_.param("input_vehicle_pose_topic", input_vehicle_pose_topic_, std::string("/simulation/vehicle_pose"));
    nh_.param("input_vehicle_vel_topic",  input_vehicle_vel_topic_,  std::string("/simulation/vehicle_velocity"));
    nh_.param("input_obstacles_topic",    input_obstacles_topic_,    std::string("/perception_prediction/predicted_obstacles"));

    // 加载发布话题名称
    nh_.param("output_local_trajectory_topic", output_local_trajectory_topic_, std::string("trajectory"));
    nh_.param("output_debug_path_topic",       output_debug_path_topic_,       std::string("debug/path"));
    nh_.param("output_debug_speed_topic",      output_debug_speed_topic_,      std::string("debug/speed_profile"));


    ROS_INFO("[LocalPlanning]: Parameters loaded successfully:");
    ROS_INFO("[LocalPlanning]:   Input topics:");
    ROS_INFO("[LocalPlanning]:     - Global reference: %s", input_global_ref_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Costmap: %s", input_costmap_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Vehicle pose: %s", input_vehicle_pose_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Vehicle velocity: %s", input_vehicle_vel_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Obstacles: %s", input_obstacles_topic_.c_str());
    ROS_INFO("[LocalPlanning]:   Output topics:");
    ROS_INFO("[LocalPlanning]:     - Local trajectory: %s", output_local_trajectory_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Debug path: %s", output_debug_path_topic_.c_str());
    ROS_INFO("[LocalPlanning]:     - Debug speed: %s", output_debug_speed_topic_.c_str());
}

void LocalPlanning::InitializeSubscribers()
{
    sub_global_ref_   = nh_.subscribe(input_global_ref_topic_,   1, &LocalPlanning::GlobalReferenceCallback, this);
    sub_costmap_      = nh_.subscribe(input_costmap_topic_,      1, &LocalPlanning::CostmapCallback, this);
    sub_vehicle_pose_ = nh_.subscribe(input_vehicle_pose_topic_, 5, &LocalPlanning::VehiclePoseCallback, this);
    sub_vehicle_vel_  = nh_.subscribe(input_vehicle_vel_topic_,  5, &LocalPlanning::VehicleVelocityCallback, this);

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
    // todo
    planner_ = new LocalPlanner();
    planner_->InitParams(params);
    
    ROS_INFO("[LocalPlanning]: LocalPlanner initialized");
}

bool LocalPlanning::IsDataReady() const
{
    // todo: chuli
    // 检查是否收到全局参考线（必须有）
    if (global_reference_.poses.empty())
    {
        ROS_INFO_THROTTLE(1.0, "[LocalPlanning]: Waiting for global reference");
        return false;
    }

    // 检查是否收到车辆位姿（必须有）
    if (vehicle_pose_.header.frame_id.empty())
    {
        ROS_INFO_THROTTLE(1.0, "[LocalPlanning]: Waiting for vehicle pose");
        return false;
    }

    // 检查是否收到车辆速度（最好有，但可以先使用默认值）
    if (vehicle_velocity_.header.frame_id.empty())
    {
        ROS_INFO_THROTTLE(1.0, "[LocalPlanning]: Vehicle velocity not received yet");
        // 这里不返回false，因为速度可以有默认值
    }

    return true;
}

void LocalPlanning::GlobalReferenceCallback(const nav_msgs::Path::ConstPtr & msg)
{
    global_reference_ = *msg;

    ROS_INFO("[LocalPlanning]: Received global reference with %lu points", msg->poses.size());
}

void LocalPlanning::CostmapCallback(const nav_msgs::OccupancyGrid::ConstPtr & msg)
{
    costmap_ = *msg;

    ROS_INFO("[LocalPlanning]: Received costmap with resolution: %.2f, size: %dx%d",
        msg->info.resolution, msg->info.width, msg->info.height);
}

void LocalPlanning::VehiclePoseCallback(const geometry_msgs::PoseStamped::ConstPtr & msg)
{
    vehicle_pose_ = *msg;
}

void LocalPlanning::VehicleVelocityCallback(const geometry_msgs::TwistStamped::ConstPtr & msg)
{
    vehicle_velocity_ = *msg;
}

// 模板函数实现（预留，待感知模块完成后实现）todo
template<typename T>
void LocalPlanning::PredictedObstaclesCallback(const typename T::ConstPtr & msg)
{
    // 预留实现，待感知预测功能包完成后补充
}