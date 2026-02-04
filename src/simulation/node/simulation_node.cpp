#include "simulation/simulation.h"


int main(int argc, char ** argv)
{
    ros::init(argc, argv, "simulation");
    ros::NodeHandle private_nh("~");
    
    Simulation simulation(private_nh);
    simulation.run();
    
    return 0;
}