#include "control/control.h"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "control");
    ros::NodeHandle nh;
    ros::NodeHandle private_nh("~");

    Control control(nh, private_nh);
    control.run();

    return 0;
}