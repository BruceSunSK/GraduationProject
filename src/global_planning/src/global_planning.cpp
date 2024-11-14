#include "global_planning/global_planning.h"


/// @brief 完成全局规划的初始化。设定ros相关的话题内容；设定选用的规划器以及对应的参数
/// @param nh ros节点句柄
GlobalPlanning::GlobalPlanning(ros::NodeHandle & nh) : nh_(nh), listener_(buffer_)
{
    // 话题参数初始化
    nh_.param<std::string>("input_map_topic",  input_map_topic_,  "/grid_cost_map/global_occupancy_grid_map");
    nh_.param<std::string>("input_goal_topic", input_goal_topic_, "/move_base_simple/goal");
    nh_.param<std::string>("output_processed_map_topic",  output_processed_map_topic_, "processed_map");
    nh_.param<std::string>("output_path_topic",           output_path_topic_,          "path");
    nh_.param<std::string>("output_auxiliary_info_topic", output_auxiliary_info_topic_, "auxiliary_info");
    nh_.param<bool>("info_flag", info_flag_, true);
    nh_.param<bool>("save_flag", save_flag_, false);
    nh_.param<std::string>("save_dir_path", save_dir_path_, "");

    sub_map_  = nh_.subscribe(input_map_topic_ , 1, &GlobalPlanning::set_map, this);
    sub_goal_ = nh_.subscribe(input_goal_topic_, 1, &GlobalPlanning::set_goal, this);
    pub_processed_map_ = nh.advertise<nav_msgs::OccupancyGrid>(output_processed_map_topic_, 1, true);
    pub_path_          = nh.advertise<nav_msgs::Path>(output_path_topic_, 10);
    pub_auxiliary_     = nh.advertise<visualization_msgs::MarkerArray>(output_auxiliary_info_topic_, 1, true);

    // 规划器初始化
    planner_name_ = nh_.param<std::string>("planner_name", "MCAstar");
    if (planner_name_ == "MCAstar")
    {
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
        p.path_simplification_params.DISTANCE_THRESHOLD = nh_.param<double>("MCAstar/path_simplification_params/DISTANCE_THRESHOLD", 1.5);
        p.path_simplification_params.ANGLE_THRESHOLD    = nh_.param<double>("MCAstar/path_simplification_params/ANGLE_THRESHOLD", 0.17);
        p.path_simplification_params.OBSTACLE_THRESHOLD = nh_.param<int>("MCAstar/path_simplification_params/OBSTACLE_THRESHOLD", 70);
        p.path_simplification_params.LINE_WIDTH         = nh_.param<double>("MCAstar/path_simplification_params/LINE_WIDTH", 1.0);
        p.path_simplification_params.MAX_INTAVAL        = nh_.param<double>("MCAstar/path_simplification_params/MAX_INTAVAL", 8.0);
        p.path_smooth_params.PATH_SMOOTH_TYPE           = static_cast<MCAstar::PathSmoothType>(
                                                          nh_.param<int>("MCAstar/path_smooth_params/PATH_SMOOTH_TYPE", 1));
        p.path_smooth_params.T_STEP                     = nh_.param<double>("MCAstar/path_smooth_params/T_STEP", 0.0005);
        p.downsampling_params.INTERVAL                  = nh_.param<double>("MCAstar/downsampling_params/INTERVAL", 0.4);
        planner_ = new MCAstar;
        planner_->initParams(p);
    }
    else if (planner_name_ == "Astar")
    {
        Astar::AstarParams p;
        p.map_params.OBSTACLE_THRESHOLD                 = nh_.param<int>("Astar/map_params/OBSTACLE_THRESHOLD", 50);
        p.cost_function_params.HEURISTICS_TYPE          = static_cast<Astar::HeuristicsType>(
                                                          nh_.param<int>("Astar/cost_function_params/HEURISTICS_TYPE", 2));
        planner_ = new Astar;
        planner_->initParams(p);
    }
    else if (planner_name_ == "RRT")
    {
        RRT::RRTParams p;
        p.map_params.OBSTACLE_THRESHOLD                 = nh_.param<int>("RRT/map_params/OBSTACLE_THRESHOLD", 50);
        p.sample_params.ITERATOR_TIMES                  = nh_.param<int>("RRT/sample_params/ITERATOR_TIMES", 100000);
        p.sample_params.GOAL_SAMPLE_RATE                = nh_.param<double>("RRT/sample_params/GOAL_SAMPLE_RATE", 0.1);
        p.sample_params.GOAL_DIS_TOLERANCE              = nh_.param<double>("RRT/sample_params/GOAL_DIS_TOLERANCE", 2.0);
        p.sample_params.STEP_SIZE                       = nh_.param<double>("RRT/sample_params/STEP_SIZE", 3.0);
        planner_ = new RRT;
        planner_->initParams(p);
    }
    else if (planner_name_ == "RRTstar")
    {
        RRTstar::RRTstarParams p;
        p.map_params.OBSTACLE_THRESHOLD                 = nh_.param<int>("RRTstar/map_params/OBSTACLE_THRESHOLD", 50);
        p.sample_params.ITERATOR_TIMES                  = nh_.param<int>("RRTstar/sample_params/ITERATOR_TIMES", 100000);
        p.sample_params.GOAL_SAMPLE_RATE                = nh_.param<double>("RRTstar/sample_params/GOAL_SAMPLE_RATE", 0.1);
        p.sample_params.GOAL_DIS_TOLERANCE              = nh_.param<double>("RRTstar/sample_params/GOAL_DIS_TOLERANCE", 2.0);
        p.sample_params.STEP_SIZE                       = nh_.param<double>("RRTstar/sample_params/STEP_SIZE", 3.0);
        p.sample_params.NEAR_DIS                        = nh_.param<double>("RRTstar/sample_params/NEAR_DIS", 10.0);
        planner_ = new RRTstar;
        planner_->initParams(p);
    }
    else if (planner_name_ == "GA")
    {
        GA::GAParams p;
        p.map_params.OBSTACLE_THRESHOLD                 = nh_.param<int>("GA/map_params/OBSTACLE_THRESHOLD", 50);
        p.optimization_params.GENERATION_SIZE           = nh_.param<int>("GA/optimization_params/GENERATION_SIZE", 200);
        p.optimization_params.POPULATION_SIZE           = nh_.param<int>("GA/optimization_params/POPULATION_SIZE", 50);
        p.optimization_params.CHROMOSOME_SIZE           = nh_.param<int>("GA/optimization_params/CHROMOSOME_SIZE", 10);
        p.optimization_params.CROSSOVER_RATE            = nh_.param<double>("GA/optimization_params/CROSSOVER_RATE", 0.7);
        p.optimization_params.MUTATION_RATE             = nh_.param<double>("GA/optimization_params/MUTATION_RATE", 0.01);
        planner_ = new GA;
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
        goal_flag = planner_->setStartPoint(tfs.transform.translation.x, tfs.transform.translation.y);
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
    goal_flag = planner_->setEndPoint(msg->pose.position.x, msg->pose.position.y);


    // 发布路径
    if ((goal_flag && map_flag) == false)
    {
        return;
    }
    std::vector<cv::Point2d> path;
    std::vector<std::vector<cv::Point2d>> auxiliary_info;
    planner_->getPath(path, auxiliary_info);

    // 规划路径信息
    nav_msgs::Path path_msg;
    path_msg.header.frame_id = "map";
    path_msg.header.stamp = ros::Time::now();
    for (cv::Point2d & p : path)
    {
        geometry_msgs::PoseStamped m;
        m.header = path_msg.header;
        m.pose.position.x = p.x;
        m.pose.position.y = p.y;
        m.pose.position.z = 10;
        path_msg.poses.push_back(m);
    }

    
    // 辅助信息
    visualization_msgs::MarkerArray marker_array;
    visualization_msgs::Marker clean_marker;
    clean_marker.header = path_msg.header;
    clean_marker.action = visualization_msgs::Marker::DELETEALL;
    marker_array.markers.push_back(std::move(clean_marker));
    if (planner_name_ == "MCAstar")
    {
        // 用于显示
        visualization_msgs::Marker marker;
        marker.header = path_msg.header;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;


        // 0. 规划原始节点
        marker.ns = "raw_nodes";
        marker.id = 0;
        marker.scale.x = 0.35;
        marker.scale.y = 0.35;
        marker.scale.z = 0.35;
        marker.color.a = 0.9;
        marker.color.r = 0.0 / 255.0;
        marker.color.g = 0.0 / 255.0;
        marker.color.b = 255.0 / 255.0;
        marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[0][i].x;
            p.y = auxiliary_info[0][i].y;
            p.z = 1;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);


        // 1. 规划扩展节点
        marker.ns = "expanded_nodes";
        marker.id = 1;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 1.0;
        marker.color.r = 65.0 / 255.0;
        marker.color.g = 105.0 / 255.0;
        marker.color.b = 255.0 / 255.0;
        marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[1].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[1][i].x;
            p.y = auxiliary_info[1][i].y;
            p.z = 0;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);


        // 2. 去除冗余点后的关键节点
        marker.ns = "key_nodes";
        marker.id = 2;
        marker.scale.x = 0.7;
        marker.scale.y = 0.7;
        marker.scale.z = 0.7;
        marker.color.a = 1.0;
        marker.color.r = 255.0 / 255.0;
        marker.color.g = 215.0 / 255.0;
        marker.color.b = 0.0 / 255.0;
        marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[2].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[2][i].x;
            p.y = auxiliary_info[2][i].y;
            p.z = 2;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);
    }
    else if (planner_name_ == "Astar")
    {
        // 用于显示
        visualization_msgs::Marker marker;
        marker.header = path_msg.header;
        marker.type = visualization_msgs::Marker::CUBE_LIST;
        marker.action = visualization_msgs::Marker::ADD;
        marker.pose.orientation.x = 0.0;
        marker.pose.orientation.y = 0.0;
        marker.pose.orientation.z = 0.0;
        marker.pose.orientation.w = 1.0;


        // 0. 规划扩展节点
        marker.ns = "expanded_nodes";
        marker.id = 0;
        marker.scale.x = 0.2;
        marker.scale.y = 0.2;
        marker.scale.z = 0.2;
        marker.color.a = 0.7;
        marker.color.r = 65.0 / 255.0;
        marker.color.g = 105.0 / 255.0;
        marker.color.b = 255.0 / 255.0;
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[0][i].x;
            p.y = auxiliary_info[0][i].y;
            p.z = 0;
            marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(marker);
    }
    else if (planner_name_ == "RRT" || planner_name_ == "RRTstar")
    {
        // 用于显示
        visualization_msgs::Marker line_marker;
        line_marker.header = path_msg.header;
        line_marker.type = visualization_msgs::Marker::LINE_LIST;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        visualization_msgs::Marker point_marker;
        point_marker.header = path_msg.header;
        point_marker.type = visualization_msgs::Marker::CUBE_LIST;
        point_marker.action = visualization_msgs::Marker::ADD;
        point_marker.pose.orientation.x = 0.0;
        point_marker.pose.orientation.y = 0.0;
        point_marker.pose.orientation.z = 0.0;
        point_marker.pose.orientation.w = 1.0;


        // 0. rrt树上所有节点
        line_marker.ns = "tree_lines";
        line_marker.id = 0;
        line_marker.scale.x = 0.2;
        line_marker.scale.y = 0.2;
        line_marker.scale.z = 0.2;
        line_marker.color.a = 0.7;
        line_marker.color.r = 255.0 / 255.0;
        line_marker.color.g = 215.0 / 255.0;
        line_marker.color.b = 0.0 / 255.0;

        point_marker.ns = "tree_nodes";
        point_marker.id = 1;
        point_marker.scale.x = 0.4;
        point_marker.scale.y = 0.4;
        point_marker.scale.z = 0.4;
        point_marker.color.a = 1.0;
        point_marker.color.r = 139.0 / 255.0;
        point_marker.color.g = 105.0 / 255.0;
        point_marker.color.b = 20.0 / 255.0;
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p1;
            p1.x = auxiliary_info[0][i].x;
            p1.y = auxiliary_info[0][i].y;
            p1.z = 0;
            point_marker.points.push_back(p1);
            p1.z = 5;
            line_marker.points.push_back(std::move(p1));
            geometry_msgs::Point p2;
            p2.x = auxiliary_info[1][i].x;
            p2.y = auxiliary_info[1][i].y;
            p2.z = 0;
            line_marker.points.push_back(std::move(p2));
        }
        marker_array.markers.push_back(line_marker);
        marker_array.markers.push_back(point_marker);
    }
    else if (planner_name_ == "GA")
    {
    }

    pub_path_.publish(path_msg);
    pub_auxiliary_.publish(marker_array);

    if (info_flag_)
    {
        planner_->showAllInfo(save_flag_, save_dir_path_);
    }
}
