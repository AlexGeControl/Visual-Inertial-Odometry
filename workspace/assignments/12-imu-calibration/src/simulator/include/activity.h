#ifndef IMU_SIMULATOR_ACTIVITY_H
#define IMU_SIMULATOR_ACTIVITY_H

#include <random>
#include <string>

#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>


namespace imu {

namespace simulator {

struct Config {
    // device info:
    std::string device_name;
    std::string topic_name;
    std::string frame_id;

    // mode:
    bool calibration_mode;

    // angular velocity noises:
    double gyro_bias_stddev;
    double gyro_noise_stddev;

    // linear acceleration noises:
    double acc_bias_stddev;
    double acc_noise_stddev;

    // pose:
    std::string pose_frame_id;
    std::string pose_topic_name_ground_truth;
};

class Activity {
public:
    Activity();

    void Init(void);
    sensor_msgs::Imu PublishMeasurement(ros::Time timestamp);
    void PublishMeasurement(void);
private:
    // get groud truth from motion equation:
    void GetGroundTruth(void);
    // random walk & measurement noise generation:
    Eigen::Vector3d GetGaussianNoise(double stddev);
    void AddNoise(double delta_t);
    // pose from integration:
    void UpdatePose(void);

    // convert to ROS messages:
    void ToIMUMessage(sensor_msgs::Imu &message);
    void ToGroundTruthPose(nav_msgs::Odometry &message);

    ros::NodeHandle private_nh_;

    ros::Publisher pub_;
    ros::Publisher pub_pose_ground_truth_;

    // config:
    Config config_;

    // noise generator:
    std::default_random_engine normal_generator_;
    std::normal_distribution<double> normal_distribution_;

    // measurements:
    ros::Time timestamp_;
    // a. gravity constant:
    Eigen::Vector3d G_;
    // b. pose:
    Eigen::Matrix3d R_;
    Eigen::Vector3d t_;
    // c. angular velocity:
    Eigen::Vector3d angular_vel_;
    Eigen::Vector3d angular_vel_bias_;
    // d. linear acceleration:
    Eigen::Vector3d linear_acc_;
    Eigen::Vector3d linear_acc_bias_;
    // ROS IMU message:
    sensor_msgs::Imu message_;
    nav_msgs::Odometry message_groud_truth_pose_;
    nav_msgs::Odometry message_integrated_pose_;
};

}  // namespace simulator

}  // namespace imu

#endif  // IMU_SIMULATOR_ACTIVITY_H