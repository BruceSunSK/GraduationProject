#include "perception/perception_simulator.h"


namespace Perception
{

PerceptionSimulator::PerceptionSimulator()
    : private_nh_("~")
    , simulation_rate_(10.0)
    , prediction_time_step_(0.5)
    , prediction_points_(10)
    , marker_lifetime_(0.2)
    , enable_visualization_(true)
{
    Initialize();
}

void PerceptionSimulator::Initialize()
{
    ROS_INFO("[PerceptionSimulator]: Initializing...");

    // 加载参数
    LoadParameters();

    // 初始化颜色映射
    InitializeColorMap();

    // 加载障碍物配置
    LoadObstacles(config_file_);

    // 初始化发布者
    InitializePublishers();

    // 初始化定时器
    InitializeTimer();

    ROS_INFO("[PerceptionSimulator]: Initialized with %zu obstacles", obstacles_state_.size());
}

void PerceptionSimulator::LoadParameters()
{
    // 从参数服务器加载参数
    private_nh_.param<std::string>("config_file", config_file_, "obstacles.yaml");
    private_nh_.param<double>("simulation_rate", simulation_rate_, 10.0);
    private_nh_.param<double>("prediction_time_step", prediction_time_step_, 0.5);
    private_nh_.param<int>("prediction_points", prediction_points_, 10);
    private_nh_.param<double>("marker_lifetime", marker_lifetime_, 0.2);
    private_nh_.param<bool>("enable_visualization", enable_visualization_, true);

    ROS_INFO("[PerceptionSimulator]: Parameters loaded:");
    ROS_INFO("[PerceptionSimulator]:   Config file: %s", config_file_.c_str());
    ROS_INFO("[PerceptionSimulator]:   Simulation rate: %.1f Hz", simulation_rate_);
    ROS_INFO("[PerceptionSimulator]:   Prediction time step: %.1f s", prediction_time_step_);
    ROS_INFO("[PerceptionSimulator]:   Prediction points: %d", prediction_points_);
}

void PerceptionSimulator::InitializeColorMap()
{
    // 设置不同类型障碍物的颜色
    // VEHICLE: 红色
    color_map_[1] = { 1.0f, 0.0f, 0.0f, 0.8f };
    // PEDESTRIAN: 绿色
    color_map_[2] = { 0.0f, 1.0f, 0.0f, 0.8f };
    // BICYCLE: 蓝色
    color_map_[3] = { 0.0f, 0.0f, 1.0f, 0.8f };
    // UNKNOWN: 灰色
    color_map_[0] = { 0.5f, 0.5f, 0.5f, 0.8f };
}

void PerceptionSimulator::LoadObstacles(const std::string & config_file)
{
    try
    {
        // 构建完整路径
        std::string full_path;
        if (config_file[0] == '/')
        {
            // 绝对路径
            full_path = config_file;
        }
        else
        {
            // 相对路径，假设在包的config目录下
            std::string package_path = ros::package::getPath("perception");
            if (package_path.empty())
            {
                ROS_ERROR("[PerceptionSimulator]: Could not find perception package path");
                return;
            }
            full_path = package_path + "/config/" + config_file;
        }
        ROS_INFO("[PerceptionSimulator]: Loading obstacles from: %s", full_path.c_str());

        YAML::Node config = YAML::LoadFile(full_path);
        if (!config["obstacles"])
        {
            ROS_WARN("[PerceptionSimulator]: No obstacles found in config file");
            return;
        }

        const YAML::Node & obstacles_node = config["obstacles"];
        for (size_t i = 0; i < obstacles_node.size(); ++i)
        {
            ObstacleConfig cfg;
            cfg.id = obstacles_node[i]["id"].as<int>();
            cfg.type = obstacles_node[i]["type"].as<int>();
            cfg.init_x = obstacles_node[i]["init_x"].as<double>();
            cfg.init_y = obstacles_node[i]["init_y"].as<double>();
            cfg.init_yaw = obstacles_node[i]["init_yaw"].as<double>();
            cfg.length = obstacles_node[i]["length"].as<double>();
            cfg.width = obstacles_node[i]["width"].as<double>();
            cfg.height = obstacles_node[i]["height"].as<double>();
            cfg.speed = obstacles_node[i]["speed"].as<double>();
            cfg.use_noise = obstacles_node[i]["use_noise"].as<bool>(false);
            cfg.noise_stddev = obstacles_node[i]["noise_stddev"].as<double>(0.1);
            cfg.name = obstacles_node[i]["name"].as<std::string>("");

            obstacles_state_.emplace_back(cfg);

            ROS_INFO("[PerceptionSimulator]: Loaded obstacle %d: %s at (%.2f, %.2f), speed: %.2f m/s",
                cfg.id, cfg.name.c_str(), cfg.init_x, cfg.init_y, cfg.speed);
        }
    }
    catch (const YAML::Exception & e)
    {
        ROS_ERROR("[PerceptionSimulator]: Failed to load config file %s: %s",
            config_file.c_str(), e.what());
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("[PerceptionSimulator]: Error loading obstacles: %s", e.what());
    }
}

void PerceptionSimulator::InitializePublishers()
{
    // 发布障碍物数据
    obstacles_pub_ = nh_.advertise<perception::PredictedObstacles>("/perception/obstacles", 1);

    // 发布可视化消息
    if (enable_visualization_)
    {
        visualization_pub_ = nh_.advertise<visualization_msgs::MarkerArray>("/perception/visualization", 1);
    }

    ROS_INFO("[PerceptionSimulator]: Publishers initialized");
}

void PerceptionSimulator::InitializeTimer()
{
    double period = 1.0 / simulation_rate_;
    timer_ = nh_.createTimer(ros::Duration(period),
        &PerceptionSimulator::TimerCallback, this);
    last_update_time_ = ros::Time::now();
}

void PerceptionSimulator::TimerCallback(const ros::TimerEvent & event)
{
    ros::Time current_time = ros::Time::now();
    double dt = (current_time - last_update_time_).toSec();
    last_update_time_ = current_time;

    // 更新障碍物状态
    UpdateObstacles(dt);

    // 发布障碍物数据
    PublishObstacles();

    // 发布可视化
    if (enable_visualization_)
    {
        PublishVisualization();
    }
}

void PerceptionSimulator::UpdateObstacles(double dt)
{
    for (auto & obstacle : obstacles_state_)
    {
        obstacle.Update(dt);
    }
}

void PerceptionSimulator::PublishObstacles()
{
    perception::PredictedObstacles msg;
    msg.header.stamp = ros::Time::now();
    msg.header.frame_id = "/map";

    for (const auto & obstacle_state : obstacles_state_)
    {
        perception::Obstacle obstacle_msg;
        obstacle_msg.header.stamp = ros::Time::now();
        obstacle_msg.header.frame_id = "/map";
        obstacle_msg.id = obstacle_state.config.id;
        obstacle_msg.type = obstacle_state.config.type;
        obstacle_msg.speed = obstacle_state.current_speed;

        // 当前位姿
        geometry_msgs::Pose pose;
        pose.position.x = obstacle_state.current_x;
        pose.position.y = obstacle_state.current_y;
        pose.position.z = obstacle_state.config.height / 2.0;  // 高度的一半

        tf2::Quaternion q;
        q.setRPY(0, 0, obstacle_state.current_yaw);
        pose.orientation = tf2::toMsg(q);
        obstacle_msg.pose = pose;

        // 尺寸
        geometry_msgs::Vector3 dimension;
        dimension.x = obstacle_state.config.length;
        dimension.y = obstacle_state.config.width;
        dimension.z = obstacle_state.config.height;
        obstacle_msg.dimension = dimension;

        // 预测轨迹
        for (int i = 0; i < prediction_points_; ++i)
        {
            double t = (i + 1) * prediction_time_step_;
            perception::PredictedTrajectory pred;
            pred.time_relative = t;

            // 预测位置（恒定速度模型）
            geometry_msgs::Pose pred_pose;
            pred_pose.position.x = obstacle_state.current_x +
                obstacle_state.current_speed * cos(obstacle_state.current_yaw) * t;
            pred_pose.position.y = obstacle_state.current_y +
                obstacle_state.current_speed * sin(obstacle_state.current_yaw) * t;
            pred_pose.position.z = obstacle_state.config.height / 2.0;
            pred_pose.orientation = pose.orientation;  // 朝向不变

            pred.pose = pred_pose;
            pred.speed = obstacle_state.current_speed;
            pred.confidence = 0.8;  // 置信度

            obstacle_msg.predicted_trajectory.push_back(pred);
        }

        msg.obstacles.push_back(obstacle_msg);
    }

    obstacles_pub_.publish(msg);
    ROS_DEBUG_THROTTLE(1.0, "[PerceptionSimulator]: Published %zu obstacles",
        obstacles_state_.size());
}

void PerceptionSimulator::PublishVisualization()
{
    visualization_msgs::MarkerArray marker_array;
    int marker_id = 0;

    for (const auto & obstacle : obstacles_state_)
    {
        // 障碍物立方体
        marker_array.markers.push_back(CreateObstacleMarker(obstacle, marker_id++));

        // 速度方向箭头
        marker_array.markers.push_back(CreateVelocityMarker(obstacle, marker_id++));

        // 预测轨迹
        marker_array.markers.push_back(CreateTrajectoryMarker(obstacle, marker_id++));

        // 文本标签
        marker_array.markers.push_back(CreateTextMarker(obstacle, marker_id++));
    }

    visualization_pub_.publish(marker_array);
}

visualization_msgs::Marker PerceptionSimulator::CreateObstacleMarker(const ObstacleState & obstacle, int id) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "obstacles";
    marker.id = id;
    marker.type = visualization_msgs::Marker::CUBE;
    marker.action = visualization_msgs::Marker::ADD;

    // 位置和方向
    marker.pose.position.x = obstacle.current_x;
    marker.pose.position.y = obstacle.current_y;
    marker.pose.position.z = obstacle.config.height / 2.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, obstacle.current_yaw);
    marker.pose.orientation = tf2::toMsg(q);

    // 尺寸
    marker.scale.x = obstacle.config.length;
    marker.scale.y = obstacle.config.width;
    marker.scale.z = obstacle.config.height;

    // 颜色
    auto color_it = color_map_.find(obstacle.config.type);
    if (color_it != color_map_.end())
    {
        marker.color.r = color_it->second.r;
        marker.color.g = color_it->second.g;
        marker.color.b = color_it->second.b;
        marker.color.a = color_it->second.a;
    }
    else
    {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
        marker.color.a = 0.8;
    }

    marker.lifetime = ros::Duration(marker_lifetime_);

    return marker;
}

visualization_msgs::Marker PerceptionSimulator::CreateVelocityMarker(const ObstacleState & obstacle, int id) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "velocities";
    marker.id = id;
    marker.type = visualization_msgs::Marker::ARROW;
    marker.action = visualization_msgs::Marker::ADD;

    // 起点：障碍物中心
    geometry_msgs::Point start;
    start.x = obstacle.current_x;
    start.y = obstacle.current_y;
    start.z = obstacle.config.height;

    // 终点：根据速度方向计算
    geometry_msgs::Point end;
    double arrow_length = obstacle.current_speed * 0.5;  // 缩放因子，1秒的距离
    end.x = start.x + arrow_length * cos(obstacle.current_yaw);
    end.y = start.y + arrow_length * sin(obstacle.current_yaw);
    end.z = start.z;

    marker.points.push_back(start);
    marker.points.push_back(end);

    // 箭头尺寸
    marker.scale.x = 0.1;  // 箭头杆直径
    marker.scale.y = 0.2;  // 箭头头直径
    marker.scale.z = 0.3;  // 箭头头长度

    // 颜色：黄色
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.color.a = 0.8;

    marker.lifetime = ros::Duration(marker_lifetime_);

    return marker;
}

visualization_msgs::Marker PerceptionSimulator::CreateTrajectoryMarker(const ObstacleState & obstacle, int id) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "trajectories";
    marker.id = id;
    marker.type = visualization_msgs::Marker::LINE_STRIP;
    marker.action = visualization_msgs::Marker::ADD;

    // 轨迹点：从当前位置到预测位置
    marker.points.push_back(geometry_msgs::Point());  // 当前位置
    marker.points[0].x = obstacle.current_x;
    marker.points[0].y = obstacle.current_y;
    marker.points[0].z = obstacle.config.height / 2.0;

    // 添加预测点
    for (int i = 1; i <= prediction_points_; ++i)
    {
        double t = i * prediction_time_step_;
        marker.points.push_back(CalculateTrajectoryPoint(obstacle, t));
    }

    // 线宽
    marker.scale.x = 0.1;

    // 颜色：根据障碍物类型
    auto color_it = color_map_.find(obstacle.config.type);
    if (color_it != color_map_.end())
    {
        marker.color.r = color_it->second.r;
        marker.color.g = color_it->second.g;
        marker.color.b = color_it->second.b;
    }
    else
    {
        marker.color.r = 0.5;
        marker.color.g = 0.5;
        marker.color.b = 0.5;
    }
    marker.color.a = 0.5;  // 半透明

    marker.lifetime = ros::Duration(marker_lifetime_);

    return marker;
}

visualization_msgs::Marker PerceptionSimulator::CreateTextMarker(const ObstacleState & obstacle, int id) const
{
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";
    marker.header.stamp = ros::Time::now();
    marker.ns = "labels";
    marker.id = id;
    marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::Marker::ADD;

    // 位置：障碍物上方
    marker.pose.position.x = obstacle.current_x;
    marker.pose.position.y = obstacle.current_y;
    marker.pose.position.z = obstacle.config.height + 0.5;
    marker.pose.orientation.w = 1.0;

    // 文本内容
    std::stringstream ss;
    ss << "ID: " << obstacle.config.id;
    if (!obstacle.config.name.empty())
    {
        ss << " (" << obstacle.config.name << ")";
    }
    ss << "\nSpeed: " << std::fixed << std::setprecision(1) << obstacle.current_speed << " m/s";

    marker.text = ss.str();

    // 文本大小
    marker.scale.z = 0.5;  // 文字高度

    // 颜色：白色
    marker.color.r = 1.0;
    marker.color.g = 1.0;
    marker.color.b = 1.0;
    marker.color.a = 1.0;

    marker.lifetime = ros::Duration(marker_lifetime_);

    return marker;
}

geometry_msgs::Point PerceptionSimulator::CalculateTrajectoryPoint(const ObstacleState & obstacle, double t) const
{
    geometry_msgs::Point point;

    // 恒定速度模型
    point.x = obstacle.current_x + obstacle.current_speed * cos(obstacle.current_yaw) * t;
    point.y = obstacle.current_y + obstacle.current_speed * sin(obstacle.current_yaw) * t;
    point.z = obstacle.config.height / 2.0;

    return point;
}

void PerceptionSimulator::Run()
{
    ROS_INFO("[PerceptionSimulator]: Starting perception simulator...");
    ros::spin();
}

// ObstacleState实现
PerceptionSimulator::ObstacleState::ObstacleState(const ObstacleConfig & cfg)
    : config(cfg)
{
    current_x = config.init_x;
    current_y = config.init_y;
    current_yaw = config.init_yaw;
    current_speed = config.speed;

    if (config.use_noise)
    {
        distribution = std::normal_distribution<double>(0.0, config.noise_stddev);
        generator.seed(std::random_device()());
    }
}

void PerceptionSimulator::ObstacleState::Update(double dt)
{
    // 恒定速度模型
    current_x += current_speed * cos(current_yaw) * dt;
    current_y += current_speed * sin(current_yaw) * dt;

    // 添加噪声
    if (config.use_noise)
    {
        current_x += distribution(generator);
        current_y += distribution(generator);
        // 稍微添加一些航向噪声
        current_yaw += distribution(generator) * 0.1;
    }
}

} // namespace Perception