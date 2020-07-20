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

struct IMUConfig {
    // general info:
    std::string device_name;
    std::string frame_id;
    std::string topic_name;

    // mode:
    bool calibration_mode;

    // angular velocity noises:
    double gyro_bias_stddev;
    double gyro_noise_stddev;

    // linear acceleration noises:
    double acc_bias_stddev;
    double acc_noise_stddev;
};

struct OdomConfig {
    OdomConfig() : initialized(false) {}

    // general info:
    std::string frame_id;
    std::string topic_name_ground_truth;
    std::string topic_name_integrated;

    // set the initial pose of integrated odometry using ground truth:
    bool initialized;
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
    // update pose with integration:
    void UpdatePoseIntegration(double delta_t);

    // convert to ROS messages:
    void UpdateIMUMessage(void);
    void UpdateOdometryMessage(
        const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
        nav_msgs::Odometry &message
    );

    ros::NodeHandle private_nh_;

    ros::Publisher pub_;
    // TODO: separate odometry estimation from IMU device
    ros::Publisher pub_odom_ground_truth_;
    ros::Publisher pub_odom_integrated_;

    // config:
    IMUConfig imu_config_;
    OdomConfig odom_config_;

    // noise generator:
    std::default_random_engine normal_generator_;
    std::normal_distribution<double> normal_distribution_;

    // measurements:
    ros::Time timestamp_;
    // a. gravity constant:
    Eigen::Vector3d G_;
    // b. pose:
    Eigen::Matrix3d R_gt_, R_integrated_;
    Eigen::Vector3d t_gt_, t_integrated_;
    Eigen::Vector3d v_gt_, v_integrated_;
    // c. angular velocity:
    Eigen::Vector3d angular_vel_;
    Eigen::Vector3d angular_vel_bias_;
    // d. linear acceleration:
    Eigen::Vector3d linear_acc_;
    Eigen::Vector3d linear_acc_bias_;
    // ROS IMU message:
    sensor_msgs::Imu message_measurement_;

    nav_msgs::Odometry message_groud_truth_pose_;
    nav_msgs::Odometry message_integrated_pose_;
};

}  // namespace simulator

}  // namespace imu

#endif  // IMU_SIMULATOR_ACTIVITY_H