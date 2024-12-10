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
    planner_name_ = nh_.param<std::string>("planner_name", "TSHAstar");
    if (planner_name_ == "TSHAstar")
    {
        TSHAstar::TSHAstarParams p;
        p.map.KERNEL_SIZE                               = nh_.param<int>("TSHAstar/map/KERNEL_SIZE", 15);
        p.map.EXPANDED_K                                = nh_.param<double>("TSHAstar/map/EXPANDED_K", 1.3);
        p.map.EXPANDED_MIN_THRESHOLD                    = nh_.param<int>("TSHAstar/map/EXPANDED_MIN_THRESHOLD", 0);
        p.map.EXPANDED_MAX_THRESHOLD                    = nh_.param<int>("TSHAstar/map/EXPANDED_MAX_THRESHOLD", 100);
        p.map.COST_THRESHOLD                            = nh_.param<int>("TSHAstar/map/COST_THRESHOLD", 10);
        p.map.OBSTACLE_THRESHOLD                        = nh_.param<int>("TSHAstar/map/OBSTACLE_THRESHOLD", 100);
        p.search.path_search.NEIGHBOR_TYPE              = static_cast<TSHAstar::NeighborType>(
                                                          nh_.param<int>("TSHAstar/search/path_search/NEIGHBOR_TYPE", 1));
        p.search.path_search.HEURISTICS_TYPE            = static_cast<TSHAstar::HeuristicsType>(
                                                          nh_.param<int>("TSHAstar/search/path_search/HEURISTICS_TYPE", 2));
        p.search.path_search.TRAV_COST_K                = nh_.param<double>("TSHAstar/search/path_search/TRAV_COST_K", 2.0);
        p.search.path_search.TURN_COST_STRAIGHT         = nh_.param<double>("TSHAstar/search/path_search/TURN_COST_STRAIGHT", 1.0);
        p.search.path_search.TURN_COST_SLANT            = nh_.param<double>("TSHAstar/search/path_search/TURN_COST_SLANT", 1.4);
        p.search.path_search.TURN_COST_VERTICAL         = nh_.param<double>("TSHAstar/search/path_search/TURN_COST_VERTICAL", 2.0);
        p.search.path_search.TURN_COST_REVERSE_SLANT    = nh_.param<double>("TSHAstar/search/path_search/TURN_COST_REVERSE_SLANT", 3.0);
        p.search.path_simplification.PATH_SIMPLIFICATION_TYPE = static_cast<TSHAstar::PathSimplificationType>(
                                                          nh_.param<int>("TSHAstar/search/path_simplification/PATH_SIMPLIFICATION_TYPE", 3));
        p.search.path_simplification.DISTANCE_THRESHOLD = nh_.param<double>("TSHAstar/search/path_simplification/DISTANCE_THRESHOLD", 1.5);
        p.search.path_simplification.ANGLE_THRESHOLD    = nh_.param<double>("TSHAstar/search/path_simplification/ANGLE_THRESHOLD", 0.17);
        p.search.path_simplification.OBSTACLE_THRESHOLD = nh_.param<int>("TSHAstar/search/path_simplification/OBSTACLE_THRESHOLD", 70);
        p.search.path_simplification.LINE_WIDTH         = nh_.param<double>("TSHAstar/search/path_simplification/LINE_WIDTH", 1.0);
        p.search.path_simplification.MAX_INTAVAL        = nh_.param<double>("TSHAstar/search/path_simplification/MAX_INTAVAL", 8.0);
        p.search.path_smooth.PATH_SMOOTH_TYPE           = static_cast<TSHAstar::PathSmoothType>(
                                                          nh_.param<int>("TSHAstar/search/path_smooth/PATH_SMOOTH_TYPE", 1));
        p.search.path_smooth.T_STEP                     = nh_.param<double>("TSHAstar/search/path_smooth/T_STEP", 0.0005);
        p.search.path_optimization.S_INTERVAL           = nh_.param<double>("TSHAstar/search/path_optimization/S_INTERVAL", 5.0);
        p.search.path_optimization.REF_WEIGTH_SMOOTH    = nh_.param<double>("TSHAstar/search/path_optimization/REF_WEIGTH_SMOOTH", 100.0);   
        p.search.path_optimization.REF_WEIGTH_LENGTH    = nh_.param<double>("TSHAstar/search/path_optimization/REF_WEIGTH_LENGTH", 1.0);   
        p.search.path_optimization.REF_WEIGTH_DEVIATION = nh_.param<double>("TSHAstar/search/path_optimization/REF_WEIGTH_DEVIATION", 50.0);
        p.search.path_optimization.REF_BUFFER_DISTANCE  = nh_.param<double>("TSHAstar/search/path_optimization/REF_BUFFER_DISTANCE", 1.0);
        p.sample.path_sample.LONGITUDIAL_SAMPLE_SPACING = nh_.param<double>("TSHAstar/sample/path_sample/LONGITUDIAL_SAMPLE_SPACING", 0.5); 
        p.sample.path_sample.LATERAL_SAMPLE_SPACING     = nh_.param<double>("TSHAstar/sample/path_sample/LATERAL_SAMPLE_SPACING", 0.5); 
        p.sample.path_sample.LATERAL_SAMPLE_RANGE       = nh_.param<double>("TSHAstar/sample/path_sample/LATERAL_SAMPLE_RANGE", 10.0); 
        p.sample.path_dp.COLLISION_DISTANCE             = nh_.param<double>("TSHAstar/sample/path_dp/COLLISION_DISTANCE", 1.2); 
        p.sample.path_dp.WARNING_DISTANCE               = nh_.param<double>("TSHAstar/sample/path_dp/WARNING_DISTANCE", 5.0); 
        p.sample.path_dp.BOUND_CHECK_INTERVAL           = nh_.param<double>("TSHAstar/sample/path_dp/BOUND_CHECK_INTERVAL", 0.3); 
        p.sample.path_dp.WEIGHT_OFFSET                  = nh_.param<double>("TSHAstar/sample/path_dp/WEIGHT_OFFSET", 50.0); 
        p.sample.path_dp.WEIGHT_OBSTACLE                = nh_.param<double>("TSHAstar/sample/path_dp/WEIGHT_OBSTACLE", 100.0); 
        p.sample.path_dp.WEIGHT_ANGLE_CHANGE            = nh_.param<double>("TSHAstar/sample/path_dp/WEIGHT_ANGLE_CHANGE", 1000.0); 
        p.sample.path_dp.WEIGHT_ANGLE_DIFF              = nh_.param<double>("TSHAstar/sample/path_dp/WEIGHT_ANGLE_DIFF", 1.0); 
        p.sample.path_qp.WEIGHT_L                       = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_L", 1.0);
        p.sample.path_qp.WEIGHT_DL                      = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_DL", 100.0);
        p.sample.path_qp.WEIGHT_DDL                     = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_DDL", 1000.0);
        p.sample.path_qp.WEIGHT_DDDL                    = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_DDDL", 7000.0);
        p.sample.path_qp.WEIGHT_CENTER                  = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_CENTER", 0.6);
        p.sample.path_qp.WEIGHT_END_STATE_L             = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_END_STATE_L", 10.0);
        p.sample.path_qp.WEIGHT_END_STATE_DL            = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_END_STATE_DL", 50.0);
        p.sample.path_qp.WEIGHT_END_STATE_DDL           = nh_.param<double>("TSHAstar/sample/path_qp/WEIGHT_END_STATE_DDL", 500.0);
        p.sample.path_qp.DL_LIMIT                       = nh_.param<double>("TSHAstar/sample/path_qp/DL_LIMIT", 2.0);
        p.sample.path_qp.VEHICLE_KAPPA_MAX              = nh_.param<double>("TSHAstar/sample/path_qp/VEHICLE_KAPPA_MAX", 0.5);
        p.sample.path_qp.CENTER_DEVIATION_THRESHOLD     = nh_.param<double>("TSHAstar/sample/path_qp/CENTER_DEVIATION_THRESHOLD", 2.2);
        p.sample.path_qp.CENTER_BOUNDS_THRESHOLD        = nh_.param<double>("TSHAstar/sample/path_qp/CENTER_BOUNDS_THRESHOLD", 3.2);
        p.sample.path_qp.CENTER_OBS_COEFFICIENT         = nh_.param<double>("TSHAstar/sample/path_qp/CENTER_OBS_COEFFICIENT", 10.0);
        planner_ = new TSHAstar;
        planner_->initParams(p);
    }
    else if (planner_name_ == "Astar")
    {
        Astar::AstarParams p;
        p.map.OBSTACLE_THRESHOLD                 = nh_.param<int>("Astar/map/OBSTACLE_THRESHOLD", 50);
        p.cost_function.HEURISTICS_TYPE          = static_cast<Astar::HeuristicsType>(
                                                   nh_.param<int>("Astar/cost_function/HEURISTICS_TYPE", 2));
        planner_ = new Astar;
        planner_->initParams(p);
    }
    else if (planner_name_ == "RRT")
    {
        RRT::RRTParams p;
        p.map.OBSTACLE_THRESHOLD                 = nh_.param<int>("RRT/map/OBSTACLE_THRESHOLD", 50);
        p.sample.ITERATOR_TIMES                  = nh_.param<int>("RRT/sample/ITERATOR_TIMES", 100000);
        p.sample.GOAL_SAMPLE_RATE                = nh_.param<double>("RRT/sample/GOAL_SAMPLE_RATE", 0.1);
        p.sample.GOAL_DIS_TOLERANCE              = nh_.param<double>("RRT/sample/GOAL_DIS_TOLERANCE", 2.0);
        p.sample.STEP_SIZE                       = nh_.param<double>("RRT/sample/STEP_SIZE", 3.0);
        planner_ = new RRT;
        planner_->initParams(p);
    }
    else if (planner_name_ == "RRTstar")
    {
        RRTstar::RRTstarParams p;
        p.map.OBSTACLE_THRESHOLD                 = nh_.param<int>("RRTstar/map/OBSTACLE_THRESHOLD", 50);
        p.sample.ITERATOR_TIMES                  = nh_.param<int>("RRTstar/sample/ITERATOR_TIMES", 100000);
        p.sample.GOAL_SAMPLE_RATE                = nh_.param<double>("RRTstar/sample/GOAL_SAMPLE_RATE", 0.1);
        p.sample.GOAL_DIS_TOLERANCE              = nh_.param<double>("RRTstar/sample/GOAL_DIS_TOLERANCE", 2.0);
        p.sample.STEP_SIZE                       = nh_.param<double>("RRTstar/sample/STEP_SIZE", 3.0);
        p.sample.NEAR_DIS                        = nh_.param<double>("RRTstar/sample/NEAR_DIS", 10.0);
        planner_ = new RRTstar;
        planner_->initParams(p);
    }
    else if (planner_name_ == "GA")
    {
        GA::GAParams p;
        p.map.OBSTACLE_THRESHOLD                 = nh_.param<int>("GA/map/OBSTACLE_THRESHOLD", 50);
        p.optimization.GENERATION_SIZE           = nh_.param<int>("GA/optimization/GENERATION_SIZE", 200);
        p.optimization.POPULATION_SIZE           = nh_.param<int>("GA/optimization/POPULATION_SIZE", 50);
        p.optimization.CHROMOSOME_SIZE           = nh_.param<int>("GA/optimization/CHROMOSOME_SIZE", 10);
        p.optimization.CROSSOVER_RATE            = nh_.param<double>("GA/optimization/CROSSOVER_RATE", 0.7);
        p.optimization.MUTATION_RATE             = nh_.param<double>("GA/optimization/MUTATION_RATE", 0.01);
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
        goal_flag = planner_->setStartPoint(tfs.transform.translation.x, tfs.transform.translation.y) &&
                    planner_->setStartPointYaw(tf2::getYaw(tfs.transform.rotation));
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
    goal_flag = planner_->setEndPoint(msg->pose.position.x, msg->pose.position.y) &&
                planner_->setEndPointYaw(tf2::getYaw(msg->pose.orientation));

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
    if (planner_name_ == "TSHAstar")
    {
        // 用于显示
        visualization_msgs::Marker line_marker;
        line_marker.header = path_msg.header;
        line_marker.type = visualization_msgs::Marker::LINE_STRIP;
        line_marker.action = visualization_msgs::Marker::ADD;
        line_marker.pose.orientation.x = 0.0;
        line_marker.pose.orientation.y = 0.0;
        line_marker.pose.orientation.z = 0.0;
        line_marker.pose.orientation.w = 1.0;
        visualization_msgs::Marker points_marker;
        points_marker.header = path_msg.header;
        points_marker.type = visualization_msgs::Marker::CUBE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.pose.orientation.x = 0.0;
        points_marker.pose.orientation.y = 0.0;
        points_marker.pose.orientation.z = 0.0;
        points_marker.pose.orientation.w = 1.0;


        // 0. 规划原始节点
        points_marker.ns = "search_raw_nodes";
        points_marker.id = 0;
        points_marker.scale.x = 0.35;
        points_marker.scale.y = 0.35;
        points_marker.scale.z = 0.35;
        points_marker.color.a = 0.9;
        points_marker.color.r = 0.0 / 255.0;
        points_marker.color.g = 0.0 / 255.0;
        points_marker.color.b = 255.0 / 255.0;
        points_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[0][i].x;
            p.y = auxiliary_info[0][i].y;
            p.z = 1;
            points_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(points_marker);


        // 1. 规划扩展节点
        points_marker.ns = "search_expanded_nodes";
        points_marker.id = 1;
        points_marker.scale.x = 0.2;
        points_marker.scale.y = 0.2;
        points_marker.scale.z = 0.2;
        points_marker.color.a = 1.0;
        points_marker.color.r = 65.0 / 255.0;
        points_marker.color.g = 105.0 / 255.0;
        points_marker.color.b = 255.0 / 255.0;
        points_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[1].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[1][i].x;
            p.y = auxiliary_info[1][i].y;
            p.z = 0;
            points_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(points_marker);


        // 2. 去除冗余点后的关键节点
        points_marker.ns = "search_key_nodes";
        points_marker.id = 2;
        points_marker.scale.x = 0.6;
        points_marker.scale.y = 0.6;
        points_marker.scale.z = 0.6;
        points_marker.color.a = 1.0;
        points_marker.color.r = 255.0 / 255.0;
        points_marker.color.g = 215.0 / 255.0;
        points_marker.color.b = 0.0 / 255.0;
        points_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[2].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[2][i].x;
            p.y = auxiliary_info[2][i].y;
            p.z = 2;
            points_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(points_marker);


        // 3. 进行曲线平滑后的结果
        line_marker.ns = "search_bspline_smooth_path";
        line_marker.id = 3;
        line_marker.scale.x = 0.3;
        line_marker.scale.y = 0.3;
        line_marker.scale.z = 0.3;
        line_marker.color.a = 0.8;
        line_marker.color.r = 255.0 / 255.0;
        line_marker.color.g = 165.0 / 255.0;
        line_marker.color.b = 0.0 / 255.0;
        line_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[3].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[3][i].x;
            p.y = auxiliary_info[3][i].y;
            p.z = 3;
            line_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(line_marker);


        // 4. 平滑后曲线均匀采样点
        points_marker.ns = "search_s_sample_points";
        points_marker.id = 4;
        points_marker.scale.x = 0.3;
        points_marker.scale.y = 0.3;
        points_marker.scale.z = 0.3;
        points_marker.color.a = 1.0;
        points_marker.color.r = 255.0 / 255.0;
        points_marker.color.g = 255.0 / 255.0;
        points_marker.color.b = 0.0 / 255.0;
        points_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[4].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[4][i].x;
            p.y = auxiliary_info[4][i].y;
            p.z = 4;
            points_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(points_marker);


        // 5. 进行数值优化平滑后的结果
        line_marker.ns = "search_optimized_path";
        line_marker.id = 5;
        line_marker.scale.x = 0.3;
        line_marker.scale.y = 0.3;
        line_marker.scale.z = 0.3;
        line_marker.color.a = 0.8;
        line_marker.color.r = 238.0 / 255.0;
        line_marker.color.g = 44.0 / 255.0;
        line_marker.color.b = 44.0 / 255.0;
        line_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[5].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[5][i].x;
            p.y = auxiliary_info[5][i].y;
            p.z = 5;
            line_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(line_marker);


        // 6. dp后的路径和上下边界
        // 6.1 dp路径
        line_marker.ns = "sample_dp_path";
        line_marker.id = 6;
        line_marker.scale.x = 0.3;
        line_marker.scale.y = 0.3;
        line_marker.scale.z = 0.3;
        line_marker.color.a = 0.8;
        line_marker.color.r = 54.0 / 255.0;
        line_marker.color.g = 54.0 / 255.0;
        line_marker.color.b = 54.0 / 255.0;
        line_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[6].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[6][i].x;
            p.y = auxiliary_info[6][i].y;
            p.z = 6;
            line_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(line_marker);
        // 6.2 dp得到的下边界
        line_marker.ns = "sample_dp_lower_bound";
        line_marker.id = 7;
        line_marker.scale.x = 0.4;
        line_marker.scale.y = 0.4;
        line_marker.scale.z = 0.4;
        line_marker.color.a = 0.9;
        line_marker.color.r = 47.0 / 255.0;
        line_marker.color.g = 79.0 / 255.0;
        line_marker.color.b = 79.0 / 255.0;
        line_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[7].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[7][i].x;
            p.y = auxiliary_info[7][i].y;
            p.z = 7;
            line_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(line_marker);
        // 6.3 dp得到的上边界
        line_marker.ns = "sample_dp_upper_bound";
        line_marker.id = 8;
        line_marker.scale.x = 0.4;
        line_marker.scale.y = 0.4;
        line_marker.scale.z = 0.4;
        line_marker.color.a = 0.9;
        line_marker.color.r = 47.0 / 255.0;
        line_marker.color.g = 79.0 / 255.0;
        line_marker.color.b = 79.0 / 255.0;
        line_marker.points.clear();
        for (size_t i = 0; i < auxiliary_info[8].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[8][i].x;
            p.y = auxiliary_info[8][i].y;
            p.z = 7;
            line_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(line_marker);
    }
    else if (planner_name_ == "Astar")
    {
        // 用于显示
        visualization_msgs::Marker points_marker;
        points_marker.header = path_msg.header;
        points_marker.type = visualization_msgs::Marker::CUBE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.pose.orientation.x = 0.0;
        points_marker.pose.orientation.y = 0.0;
        points_marker.pose.orientation.z = 0.0;
        points_marker.pose.orientation.w = 1.0;


        // 0. 规划扩展节点
        points_marker.ns = "expanded_nodes";
        points_marker.id = 0;
        points_marker.scale.x = 0.2;
        points_marker.scale.y = 0.2;
        points_marker.scale.z = 0.2;
        points_marker.color.a = 0.7;
        points_marker.color.r = 65.0 / 255.0;
        points_marker.color.g = 105.0 / 255.0;
        points_marker.color.b = 255.0 / 255.0;
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p;
            p.x = auxiliary_info[0][i].x;
            p.y = auxiliary_info[0][i].y;
            p.z = 0;
            points_marker.points.push_back(std::move(p));
        }
        marker_array.markers.push_back(points_marker);
    }
    else if (planner_name_ == "RRT" || planner_name_ == "RRTstar")
    {
        // 用于显示
        visualization_msgs::Marker lines_marker;
        lines_marker.header = path_msg.header;
        lines_marker.type = visualization_msgs::Marker::LINE_LIST;
        lines_marker.action = visualization_msgs::Marker::ADD;
        lines_marker.pose.orientation.x = 0.0;
        lines_marker.pose.orientation.y = 0.0;
        lines_marker.pose.orientation.z = 0.0;
        lines_marker.pose.orientation.w = 1.0;
        visualization_msgs::Marker points_marker;
        points_marker.header = path_msg.header;
        points_marker.type = visualization_msgs::Marker::CUBE_LIST;
        points_marker.action = visualization_msgs::Marker::ADD;
        points_marker.pose.orientation.x = 0.0;
        points_marker.pose.orientation.y = 0.0;
        points_marker.pose.orientation.z = 0.0;
        points_marker.pose.orientation.w = 1.0;


        // 0. rrt树上所有节点
        lines_marker.ns = "tree_lines";
        lines_marker.id = 0;
        lines_marker.scale.x = 0.2;
        lines_marker.scale.y = 0.2;
        lines_marker.scale.z = 0.2;
        lines_marker.color.a = 0.7;
        lines_marker.color.r = 255.0 / 255.0;
        lines_marker.color.g = 215.0 / 255.0;
        lines_marker.color.b = 0.0 / 255.0;

        points_marker.ns = "tree_nodes";
        points_marker.id = 1;
        points_marker.scale.x = 0.4;
        points_marker.scale.y = 0.4;
        points_marker.scale.z = 0.4;
        points_marker.color.a = 1.0;
        points_marker.color.r = 139.0 / 255.0;
        points_marker.color.g = 105.0 / 255.0;
        points_marker.color.b = 20.0 / 255.0;
        for (size_t i = 0; i < auxiliary_info[0].size(); i++)
        {
            geometry_msgs::Point p1;
            p1.x = auxiliary_info[0][i].x;
            p1.y = auxiliary_info[0][i].y;
            p1.z = 0;
            points_marker.points.push_back(p1);
            p1.z = 5;
            lines_marker.points.push_back(std::move(p1));
            geometry_msgs::Point p2;
            p2.x = auxiliary_info[1][i].x;
            p2.y = auxiliary_info[1][i].y;
            p2.z = 0;
            lines_marker.points.push_back(std::move(p2));
        }
        marker_array.markers.push_back(lines_marker);
        marker_array.markers.push_back(points_marker);
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
