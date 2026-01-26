#pragma once
#include <ros/ros.h>

class LocalPlanning
{
public:
    LocalPlanning() = delete;
    LocalPlanning(const ros::NodeHandle & nh);
    LocalPlanning(const LocalPlanning & other) = delete;
    LocalPlanning(LocalPlanning && other) = delete;
    LocalPlanning & operator=(const LocalPlanning & other) = delete;
    LocalPlanning & operator=(LocalPlanning && other) = delete;
    ~LocalPlanning();

private:
    ros::NodeHandle nh_;


};