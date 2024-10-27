#include "global_planning/global_planning.h"


/// @brief 完成全局规划的初始化。设定ros相关的话题内容；设定选用的规划器以及对应的参数
/// @param nh ros节点句柄
GlobalPlanning::GlobalPlanning(ros::NodeHandle & nh) : nh_(nh), listener_(buffer_)
{
    // 话题参数初始化
    nh_.param<std::string>("input_map_topic",  input_map_topic_,  "/grid_cost_map/global_occupancy_grid_map");
    nh_.param<std::string>("input_goal_topic", input_goal_topic_, "/move_base_simple/goal");
    nh_.param<std::string>("output_processed_map_topic", output_processed_map_topic_, "processed_map");
    nh_.param<std::string>("output_path_topic",          output_path_topic_,          "path");
    nh_.param<std::string>("output_smooth_path_topic",   output_smooth_path_topic_, "smooth_path");
    nh_.param<bool>("info_flag", info_flag_, true);
    nh_.param<bool>("save_flag", save_flag_, false);
    nh_.param<std::string>("save_dir_path", save_dir_path_, "");

    sub_map_  = nh_.subscribe(input_map_topic_ , 1, &GlobalPlanning::set_map, this);
    sub_goal_ = nh_.subscribe(input_goal_topic_, 1, &GlobalPlanning::set_goal, this);
    pub_processed_map_ = nh.advertise<nav_msgs::OccupancyGrid>(output_processed_map_topic_, 1, true);
    pub_path_          = nh.advertise<nav_msgs::Path>(output_path_topic_, 10);
    pub_smooth_path_   = nh.advertise<nav_msgs::Path>(output_smooth_path_topic_, 10);

    // 规划器初始化
    const std::string planner_name = nh_.param<std::string>("planner_name", "MCAstar");
    if (planner_name == "MCAstar")
    {
        planner_ = new MCAstar;
        MCAstar::MCAstarParams p;
        p.map_params.EXPANDED_K                         = nh_.param<double>("MCAstar/map_params/EXPANDED_K", 1.3);
        p.map_params.EXPANDED_MIN_THRESHOLD             = nh_.param<int>("MCAstar/map_params/EXPANDED_MIN_THRESHOLD", 0);
        p.map_params.EXPANDED_MAX_THRESHOLD             = nh_.param<int>("MCAstar/map_params/EXPANDED_MAX_THRESHOLD", 100);
        p.map_params.COST_THRESHOLD                     = nh_.param<int>("MCAstar/map_params/COST_THRESHOLD", 10);
        p.map_params.OBSTACLE_THRESHOLD                 = nh_.param<int>("MCAstar/map_params/OBSTACLE_THRESHOLD", 100);
        p.cost_function_params.NEIGHBOR_TYPE            = static_cast<MCAstar::NeighborType>(
                                                          nh_.param<int>("MCAstar/cost_function_params/NEIGHBOR_TYPE", 1));
        p.cost_function_params.HEURISTICS_TYPE          = static_cast<MCAstar::HeuristicsType>(
                                                          nh_.param<int>("MCAstar/cost_function_params/HEURISTICS_TYPE", 2));
        p.cost_function_params.TRAV_COST_K              = nh_.param<double>("MCAstar/cost_function_params/TRAV_COST_K", 2.0);
        p.cost_function_params.TURN_COST_STRAIGHT       = nh_.param<double>("MCAstar/cost_function_params/TURN_COST_STRAIGHT", 1.0);
        p.cost_function_params.TURN_COST_SLANT          = nh_.param<double>("MCAstar/cost_function_params/TURN_COST_SLANT", 1.4);
        p.cost_function_params.TURN_COST_VERTICAL       = nh_.param<double>("MCAstar/cost_function_params/TURN_COST_VERTICAL", 2.0);
        p.cost_function_params.TURN_COST_REVERSE_SLANT  = nh_.param<double>("MCAstar/cost_function_params/TURN_COST_REVERSE_SLANT", 3.0);
        p.path_simplification_params.PATH_SIMPLIFICATION_TYPE = static_cast<MCAstar::PathSimplificationType>(
                                                          nh_.param<int>("MCAstar/path_simplification_params/PATH_SIMPLIFICATION_TYPE", 3));
        p.path_simplification_params.DISTANCE_THRESHOLD = nh_.param<double>("MCAstar/path_simplification_params/DISTANCE_THRESHOLD", 1.8);
        p.path_simplification_params.ANGLE_THRESHOLD    = nh_.param<double>("MCAstar/path_simplification_params/ANGLE_THRESHOLD", 0.17);
        p.path_simplification_params.OBSTACLE_THRESHOLD = nh_.param<int>("MCAstar/path_simplification_params/OBSTACLE_THRESHOLD", 70);
        p.path_simplification_params.LINE_WIDTH         = nh_.param<double>("MCAstar/path_simplification_params/LINE_WIDTH", 1.5);
        p.path_smooth_params.T_STEP                     = nh_.param<double>("MCAstar/path_smooth_params/T_STEP", 0.01);
        p.downsampling_params.INTERVAL                  = nh_.param<double>("MCAstar/downsampling_params/INTERVAL", 0.3);
        planner_->initParams(p);
    }
    else if (planner_name == "Astar")
    {
        planner_ = new Astar;
        Astar::AstarParams p;
        p.map_params.OBSTACLE_THRESHOLD                 = nh_.param<int>("Astar/map_params/OBSTACLE_THRESHOLD", 50);
        p.cost_function_params.HEURISTICS_TYPE          = static_cast<Astar::HeuristicsType>(
                                                          nh_.param<int>("Astar/cost_function_params/HEURISTICS_TYPE", 2));
        planner_->initParams(p);
    }
    else
    {
        ROS_FATAL("[GlobalPlanning]: Wrong planner name! exit!");
        ros::shutdown();
    }
}

/// @brief 释放资源，delete规划器
GlobalPlanning::~GlobalPlanning()
{
    if (planner_)
    {
        delete planner_;
    }
}

/// @brief 设置地图的回调函数，完成栅格地图的设置、地图属性的设置，得到算法内部处理后的地图并发布
void GlobalPlanning::set_map(const nav_msgs::OccupancyGrid::Ptr msg)
{
    const int rows = msg->info.height;
    const int cols = msg->info.width;
    res_ = msg->info.resolution;
    ori_x_ = msg->info.origin.position.x;
    ori_y_ = msg->info.origin.position.y;
    planner_->setMapInfo(res_, ori_x_, ori_y_);

    cv::Mat map = cv::Mat::zeros(rows, cols, CV_8UC1);
    for (size_t i = 0; i < rows; i++)
    {
        for (size_t j = 0; j < cols; j++)
        {
            map.at<uchar>(i, j) = msg->data[i * cols + j];
        }
    }
    map_flag = planner_->setMap(map);

    if (map_flag)
    {
        cv::Mat processed_map;
        planner_->getProcessedMap(processed_map);

        nav_msgs::OccupancyGrid new_msg;
        new_msg.header.frame_id = msg->header.frame_id;
        new_msg.header.stamp = ros::Time::now();
        new_msg.info = msg->info;
        new_msg.data.resize(msg->data.size());
        for (size_t i = 0; i < rows; i++)
        {
            for (size_t j = 0; j < cols; j++)
            {
                new_msg.data[i * cols + j] = processed_map.at<uchar>(i, j);
            }
        }
        pub_processed_map_.publish(new_msg);
    }
}

/// @brief 订阅到终点时捕捉车辆当前位置并设置为起点；话题终点设置为重点；发布原始路径和平滑后的路径。
void GlobalPlanning::set_goal(const geometry_msgs::PoseStamped::Ptr msg)
{
    // 起点
    geometry_msgs::TransformStamped tfs;
    try
    {
        tfs = buffer_.lookupTransform("map", "veh", ros::Time(0));
        goal_flag = planner_->setStartPoint(static_cast<int>((tfs.transform.translation.x - ori_x_) / res_), 
                                            static_cast<int>((tfs.transform.translation.y - ori_y_) / res_));
        if (goal_flag == false)
        {
            return;
        }   
    }
    catch(const tf2::TransformException & e)
    {
        ROS_ERROR_THROTTLE(1, "%s", e.what());
        goal_flag = false;
        return;
    }

    // 终点
    goal_flag = planner_->setEndPoint(static_cast<int>((msg->pose.position.x - ori_x_) / res_), 
                                      static_cast<int>((msg->pose.position.y - ori_y_) / res_));

    // 发布路径
    if (goal_flag && map_flag)
    {
        std::vector<cv::Point2i> path;
        std::vector<cv::Point2d> smooth_path;
        planner_->getRawPath(path);
        planner_->getSmoothPath(smooth_path);

        nav_msgs::Path msg;
        msg.header.frame_id = "map";
        msg.header.stamp = ros::Time::now();
        for (cv::Point2i & p : path)
        {
            geometry_msgs::PoseStamped m;
            m.header = msg.header;
            m.pose.position.x = (p.x + 0.5) * res_ + ori_x_;
            m.pose.position.y = (p.y + 0.5) * res_ + ori_y_;
            m.pose.position.z = 0;
            msg.poses.push_back(m);
        }
        nav_msgs::Path msg2;
        msg2.header = msg.header;
        for (cv::Point2d & p : smooth_path)
        {
            geometry_msgs::PoseStamped m;
            m.header = msg2.header;
            m.pose.position.x = p.x;
            m.pose.position.y = p.y;
            m.pose.position.z = 0;
            msg2.poses.push_back(m);
        }
        
        pub_path_.publish(msg);
        pub_smooth_path_.publish(msg2);

        if (info_flag_)
        {
            planner_->showAllInfo(save_flag_, save_dir_path_);
        }
    }
}
