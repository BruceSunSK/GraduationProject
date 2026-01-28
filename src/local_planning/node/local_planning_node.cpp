#include "local_planning/local_planning.h"


int main(int argc, char ** argv)
{
    // 初始化ROS节点
    ros::init(argc, argv, "local_planning");
    ros::NodeHandle nh("~");
    ros::Rate rate(10);

    LocalPlanning local_planning(nh, rate);
    local_planning.Run();

    return 0;
}