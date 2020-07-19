#include <fstream>

#include <ros/ros.h>
#include <rosbag/bag.h>
#include "activity.h"

int main(int argc, char** argv) {
    std::string node_name{"imu_simulation_node"};
    ros::init(argc, argv, node_name);
    
    imu::simulator::Activity simulator_activity;

    simulator_activity.Init();
    
    const std::string bag_path = "/workspace/imu-calibration/imu.bag";
    rosbag::Bag bag;
    bag.open(bag_path, rosbag::bagmode::Write);

    ros::Time::init();
    double t0 =ros::Time::now().toSec();
    std::cout << "Start generate data, please waiting..."<< std::endl;

    const char symbol[4] = {'|','/','-','\\'};
    const double imu_frequency = 50;
    for (double t = 0.0; t < 5 * 3600.0; ) {
        // to ros msg
        ros::Time timestamp(t0 + t);
        sensor_msgs::Imu message = simulator_activity.PublishMeasurement(timestamp);

        bag.write("/imu", timestamp, message);

        t += 1.0 / imu_frequency;
    }

    bag.close();
    std::cout << "Done, save to " << bag_path <<std::endl;

    return 0;
}