#include "perception/perception_simulator.h"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "perception_simulator");
    
    try
    {
        Perception::PerceptionSimulator simulator;
        simulator.Run();
    }
    catch (const std::exception& e)
    {
        ROS_ERROR("[PerceptionSimulator]: Exception: %s", e.what());
        return 1;
    }
    
    return 0;
}