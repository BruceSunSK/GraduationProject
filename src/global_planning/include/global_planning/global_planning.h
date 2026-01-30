#pragma once
#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>  
#include <geometry_msgs/PoseStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "global_planning/planners/TSHAstar.h"
#include "global_planning/planners/astar.h"
#include "global_planning/planners/rrt.h"
#include "global_planning/planners/rrtstar.h"
#include "global_planning/planners/genetic_algorithm.h"


class GlobalPlanning
{
public:
    GlobalPlanning() = delete;
    GlobalPlanning(const ros::NodeHandle & nh);
    GlobalPlanning(const GlobalPlanning & other) = delete;
    GlobalPlanning(GlobalPlanning && other) = delete;
    GlobalPlanning & operator=(const GlobalPlanning & other) = delete;
    GlobalPlanning & operator=(GlobalPlanning && other) = delete;
    ~GlobalPlanning();

private:
    ros::NodeHandle nh_;

    std::string input_map_topic_;
    std::string input_goal_topic_;
    ros::Subscriber sub_map_;
    ros::Subscriber sub_goal_;
    bool map_flag = false;
    bool goal_flag = false;
    void set_map(const nav_msgs::OccupancyGrid::Ptr msg);
    void set_goal(const geometry_msgs::PoseStamped::Ptr msg);

    std::string output_processed_map_topic_;
    std::string output_path_topic_;
    std::string output_ref_path_topic_;
    std::string output_auxiliary_info_topic_;
    ros::Publisher pub_processed_map_;
    ros::Publisher pub_path_;
    ros::Publisher pub_auxiliary_;

    bool info_flag_ = false;
    bool save_flag_ = false;
    std::string save_dir_path_;
    std::string planner_name_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    GlobalPlannerInterface * planner_ = nullptr;
    double res_ = 0;
    double ori_x_ = 0;
    double ori_y_ = 0;
};

