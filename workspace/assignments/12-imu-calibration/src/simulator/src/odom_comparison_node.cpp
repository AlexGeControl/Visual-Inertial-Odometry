#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include "activity.h"

int main(int argc, char** argv) {
    std::string node_name{"imu_simulation_node"};
    ros::init(argc, argv, node_name);
    
    imu::simulator::Activity simulator_activity;

    simulator_activity.Init();
    
    // 50 Hz:
    ros::Rate loop_rate(400);
    while (ros::ok())
    {
        ros::spinOnce();
        simulator_activity.PublishMeasurement();
        loop_rate.sleep();
    } 

    return EXIT_SUCCESS;
}