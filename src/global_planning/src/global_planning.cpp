#include "global_planning/global_planning.h"

GlobalPlanning::GlobalPlanning(ros::NodeHandle & nh) : nh_(nh), listener_(buffer_)
{
    nh_.param("input_map_topic",  input_map_topic_,  std::string("/grid_cost_map/global_occupancy_grid_map"));
    nh_.param("input_goal_topic", input_goal_topic_, std::string("/move_base_simple/goal"));
    nh_.param("output_extended_map_topic", output_extended_map_topic_, std::string("extended_map"));
    nh_.param("output_path_topic",         output_path_topic_,         std::string("path"));
    nh_.param("output_smooth_path_topic",  output_smooth_path_topic_,  std::string("smooth_path"));

    sub_map_  = nh_.subscribe(input_map_topic_ , 1, &GlobalPlanning::set_map, this);
    sub_goal_ = nh_.subscribe(input_goal_topic_, 1, &GlobalPlanning::set_goal, this);
    pub_extended_map_ = nh.advertise<nav_msgs::OccupancyGrid>(output_extended_map_topic_, 1, true);
    pub_path_         = nh.advertise<nav_msgs::Path>(output_path_topic_, 10);
    pub_smooth_path_  = nh.advertise<nav_msgs::Path>(output_smooth_path_topic_, 10);
}

GlobalPlanning::~GlobalPlanning()
{
}

void GlobalPlanning::set_map(const nav_msgs::OccupancyGrid::Ptr msg)
{
    int rows = msg->info.height;
    int cols = msg->info.width;
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

    // 暂时先取消输出扩展后的map
    // nav_msgs::OccupancyGrid new_msg;
    // new_msg.header.frame_id = msg->header.frame_id;
    // new_msg.header.stamp = ros::Time::now();
    // new_msg.info = msg->info;
    // new_msg.data.resize(msg->data.size());
    // for (size_t i = 0; i < rows; i++)
    // {
    //     for (size_t j = 0; j < cols; j++)
    //     {
    //         new_msg.data[i * cols + j] = map.at<uchar>(i, j);
    //     }
    // }
    // pub_extended_map_.publish(new_msg);
}

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
            m.pose.position.x = (p.x + 0.5) * res_ + ori_x_;
            m.pose.position.y = (p.y + 0.5) * res_ + ori_y_;
            m.pose.position.z = 0;
            msg2.poses.push_back(m);
        }
        
        pub_path_.publish(msg);
        pub_smooth_path_.publish(msg2);
    }
}
