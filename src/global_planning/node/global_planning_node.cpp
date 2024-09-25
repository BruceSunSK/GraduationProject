#include "global_planning/global_planning.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "global_planning");
    ros::NodeHandle nh("~");

    GlobalPlanning GP(nh);

    ros::spin();
    return 0;
}
