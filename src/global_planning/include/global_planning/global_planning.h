#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include "global_planning/MC_astar.h"

class GlobalPlanning
{
public:
    GlobalPlanning(ros::NodeHandle & nh);
    ~GlobalPlanning();

    void set_planner(GlobalPlannerInterface * planner) { planner_ = planner;}

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

    std::string output_extended_map_topic_;
    std::string output_path_topic_;
    std::string output_smooth_path_topic_;
    ros::Publisher pub_extended_map_;
    ros::Publisher pub_path_;
    ros::Publisher pub_smooth_path_;

    tf2_ros::Buffer buffer_;
    tf2_ros::TransformListener listener_;

    GlobalPlannerInterface * planner_ = nullptr;
    double res_ = 0;
    double ori_x_ = 0;
    double ori_y_ = 0;
};

