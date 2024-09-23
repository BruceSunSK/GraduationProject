#include "global_planning/global_planning.h"

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "global_planning");
    ros::NodeHandle nh("~");

    GlobalPlanning GP(nh);
    MCAstar MCAstar;
    GP.set_planner(&MCAstar);

    ros::spin();
    return 0;
}
