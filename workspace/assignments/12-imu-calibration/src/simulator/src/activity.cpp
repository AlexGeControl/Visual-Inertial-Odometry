#include "node_constants.h"
#include "activity.h"

#include <eigen3/Eigen/src/Geometry/Quaternion.h>
#include <math.h>

namespace imu {

namespace simulator {

Eigen::Matrix3d EulerAnglesToRotation(Eigen::Vector3d euler_angles) {
    // parse Euler angles:
    double roll = euler_angles(0);
    double pitch = euler_angles(1);
    double yaw = euler_angles(2);

    double cr =  cos(roll); double sr =  sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);
    double cy =   cos(yaw); double sy =   sin(yaw);

    Eigen::Matrix3d R_ib;
    R_ib << 
        cy*cp,  cy*sp*sr - sy*cr,   sy*sr + cy* cr*sp,
        sy*cp, cy *cr + sy*sr*sp,    sp*sy*cr - cy*sr,
          -sp,             cp*sr,               cp*cr;

    return R_ib;
}

Eigen::Vector3d EulerAngleRatesToBodyAngleRates(const Eigen::Vector3d &euler_angles, const Eigen::Vector3d &euler_angle_rates) {
    // parse euler angles:
    double roll = euler_angles(0);
    double pitch = euler_angles(1);

    double cr =  cos(roll); double sr =  sin(roll);
    double cp = cos(pitch); double sp = sin(pitch);

    Eigen::Matrix3d R;

    R <<  
        1,     0,    - sp,
        0,    cr,   sr*cp,
        0,   -sr,   cr*cp;

    return R * euler_angle_rates;
}

Activity::Activity() 
    : private_nh_("~"), 
    // standard normal distribution:
    normal_distribution_(0.0, 1.0),
    // gravity acceleration:
    G_(0, 0, -9.81),
    // angular velocity bias:
    angular_vel_bias_(0.0, 0.0, 0.0),
    // linear acceleration bias:
    linear_acc_bias_(0.0, 0.0, 0.0)
{}

void Activity::Init(void) {
    // parse IMU config:
    private_nh_.param("imu/device_name", imu_config_.device_name, std::string("VIO_IMU"));
    private_nh_.param("imu/frame_id", imu_config_.frame_id, std::string("ENU"));
    private_nh_.param("imu/topic_name", imu_config_.topic_name, std::string("/imu"));

    private_nh_.param("imu/calibration_mode", imu_config_.calibration_mode, true);

    private_nh_.param("imu/gyro/sigma_bias", imu_config_.gyro_bias_stddev, 5e-5);
    private_nh_.param("imu/gyro/sigma_noise", imu_config_.gyro_noise_stddev, 0.015);

    private_nh_.param("imu/acc/sigma_bias", imu_config_.acc_bias_stddev, 5e-4);
    private_nh_.param("imu/acc/sigma_noise", imu_config_.acc_noise_stddev, 0.019);

    // parse odom config:
    private_nh_.param("pose/frame_id", odom_config_.frame_id, std::string("inertial"));
    private_nh_.param("pose/topic_name/ground_truth", odom_config_.topic_name_ground_truth, std::string("/pose/ground_truth"));
    private_nh_.param("pose/topic_name/integrated", odom_config_.topic_name_integrated, std::string("/pose/imu_integrated"));

    // init publishers:
    pub_ = private_nh_.advertise<sensor_msgs::Imu>(imu_config_.topic_name, 500);

    // TODO: separate odometry estimation from IMU device
    pub_odom_ground_truth_ = private_nh_.advertise<nav_msgs::Odometry>(odom_config_.topic_name_ground_truth, 500);
    pub_odom_integrated_ = private_nh_.advertise<nav_msgs::Odometry>(odom_config_.topic_name_integrated, 500);

    // init timestamp:
    timestamp_ = ros::Time::now();
}

sensor_msgs::Imu Activity::PublishMeasurement(ros::Time timestamp) {
    // get delta t:
    double delta_t = timestamp.toSec() - timestamp_.toSec();
    timestamp_ = timestamp;

    // get ground truth from motion equation:
    GetGroundTruth();
    // update bias & add measurement noises:
    AddNoise(delta_t);
    // to ROS IMU message:
    UpdateIMUMessage();

    return message_measurement_;
}

void Activity::PublishMeasurement(void) {
    // get delta t:
    ros::Time timestamp = ros::Time::now();
    double delta_t = timestamp.toSec() - timestamp_.toSec();
    timestamp_ = timestamp;

    // get ground truth from motion equation:
    GetGroundTruth();
    // update bias & add measurement noises:
    AddNoise(delta_t);
    // update pose integration:
    UpdatePoseIntegration(delta_t);

    // to ROS IMU message:
    UpdateIMUMessage();
    pub_.publish(message_measurement_);

    // TODO: separate odometry estimation from IMU device
    UpdateOdometryMessage(R_gt_, t_gt_, message_groud_truth_pose_);
    UpdateOdometryMessage(R_integrated_, t_integrated_, message_integrated_pose_);
    pub_odom_ground_truth_.publish(message_groud_truth_pose_);
    pub_odom_integrated_.publish(message_integrated_pose_);
}

void Activity::GetGroundTruth(void) {
    // acceleration:
    double timestamp_in_sec = timestamp_.toSec();
    double sin_w_xy_t = sin(kOmegaXY*timestamp_in_sec);
    double cos_w_xy_t = cos(kOmegaXY*timestamp_in_sec);
    double sin_w_z_t = sin(kOmegaZ*timestamp_in_sec);
    double cos_w_z_t = cos(kOmegaZ*timestamp_in_sec);
    double rho_x_w_xy = kRhoX*kOmegaXY;
    double rho_y_w_xy = kRhoY*kOmegaXY;
    double rho_z_w_z = kRhoZ*kOmegaZ;

    Eigen::Vector3d p(
        kRhoX*cos_w_xy_t, 
        kRhoY*sin_w_xy_t, 
        kRhoZ*sin_w_z_t
    );
    Eigen::Vector3d v(
        -rho_x_w_xy*sin_w_xy_t, 
         rho_y_w_xy*cos_w_xy_t, 
         rho_z_w_z*cos_w_z_t
    );
    Eigen::Vector3d a(
        -rho_x_w_xy*kOmegaXY*cos_w_xy_t, 
        -rho_y_w_xy*kOmegaXY*sin_w_xy_t, 
        -rho_z_w_z*kOmegaZ*sin_w_z_t
    );

    // angular velocity:
    double sin_t = sin(timestamp_in_sec);
    double cos_t = cos(timestamp_in_sec);

    Eigen::Vector3d euler_angles(
        kRoll*cos_t,
        kPitch*sin_t,
        kYaw*timestamp_in_sec
    );

    Eigen::Vector3d euler_angle_rates(
        -kRoll*sin_t,
        kPitch*cos_t,
        kYaw
    );

    if (!imu_config_.calibration_mode) {
        // transform to body frame:
        R_gt_ = EulerAnglesToRotation(euler_angles);
        t_gt_ = p;
        v_gt_ = v;
        // a. angular velocity:
        angular_vel_ = EulerAngleRatesToBodyAngleRates(euler_angles, euler_angle_rates);
        // b. linear acceleration:
        linear_acc_ = R_gt_.transpose() * (a + G_);
    } else {
        R_gt_ = Eigen::Matrix3d::Identity();
        t_gt_ = Eigen::Vector3d::Zero();
        v_gt_ = Eigen::Vector3d::Zero();
        angular_vel_ = Eigen::Vector3d::Zero();
        linear_acc_ = R_gt_.transpose() * G_;
    }
}

Eigen::Vector3d Activity::GetGaussianNoise(double stddev) {
    return stddev * Eigen::Vector3d(
        normal_distribution_(normal_generator_),
        normal_distribution_(normal_generator_),
        normal_distribution_(normal_generator_)
    );
}

void Activity::AddNoise(double delta_t) {
    // TODO: add params to class attributes:
    double sqrt_delta_t = sqrt(delta_t);

    // a. update bias:
    angular_vel_bias_ += GetGaussianNoise(imu_config_.gyro_bias_stddev * sqrt_delta_t);
    linear_acc_bias_ += GetGaussianNoise(imu_config_.acc_bias_stddev * sqrt_delta_t);

    // b. get measurement noise:
    Eigen::Vector3d angular_vel_noise = GetGaussianNoise(imu_config_.gyro_noise_stddev / sqrt_delta_t);
    Eigen::Vector3d linear_acc_noise = GetGaussianNoise(imu_config_.acc_noise_stddev / sqrt_delta_t);

    // apply to measurement:
    angular_vel_ += angular_vel_bias_ + angular_vel_noise;
    linear_acc_ += linear_acc_bias_ + linear_acc_noise;
}

void Activity::UpdatePoseIntegration(double delta_t) {
    static Eigen::Vector3d angular_vel_prev;
    static Eigen::Vector3d linear_acc_prev;

    if (!odom_config_.initialized) {
        R_integrated_ = R_gt_;
        t_integrated_ = t_gt_;
        v_integrated_ = v_gt_;

        angular_vel_prev = angular_vel_ - angular_vel_bias_;
        linear_acc_prev = R_integrated_*(linear_acc_ - linear_acc_bias_) - G_;

        odom_config_.initialized = true;
    } else {
        // update position:
        Eigen::Vector3d linear_acc_curr = R_integrated_*(linear_acc_ - linear_acc_bias_) - G_;
        Eigen::Vector3d linear_acc_mid_value = 0.5*(linear_acc_prev + linear_acc_curr);

        t_integrated_ = t_integrated_ + delta_t*v_integrated_ + 0.5*delta_t*delta_t*linear_acc_mid_value;
        v_integrated_ = v_integrated_ + delta_t*linear_acc_mid_value;

        // update orientation:
        Eigen::Vector3d angular_vel_curr = angular_vel_ - angular_vel_bias_;
        Eigen::Vector3d angular_vel_mid_value = 0.5*(angular_vel_prev + angular_vel_curr);

        Eigen::Vector3d da = 0.5*delta_t*angular_vel_mid_value;
        Eigen::Quaterniond dq(1.0, da.x(), da.y(), da.z());
        Eigen::Quaterniond q(R_integrated_);

        q = q*dq;
        R_integrated_ = q.normalized().toRotationMatrix();

        // move forward:
        angular_vel_prev = angular_vel_curr;
        linear_acc_prev = linear_acc_curr;
    }
}

void Activity::UpdateIMUMessage(void) {
    // a. set header:
    message_measurement_.header.stamp = timestamp_;
    message_measurement_.header.frame_id = imu_config_.frame_id;

    // b. set orientation:
    Eigen::Quaterniond q(R_gt_);
    message_measurement_.orientation.x = q.x();
    message_measurement_.orientation.y = q.y();
    message_measurement_.orientation.z = q.z();
    message_measurement_.orientation.w = q.w();
    // c. set angular velocity:
    message_measurement_.angular_velocity.x = angular_vel_(0); 
    message_measurement_.angular_velocity.y = angular_vel_(1); 
    message_measurement_.angular_velocity.z = angular_vel_(2);
    // d. set linear acceleration:
    message_measurement_.linear_acceleration.x = linear_acc_(0); 
    message_measurement_.linear_acceleration.y = linear_acc_(1);
    message_measurement_.linear_acceleration.z = linear_acc_(2);
}

void Activity::UpdateOdometryMessage(
    const Eigen::Matrix3d &R, const Eigen::Vector3d &t,
    nav_msgs::Odometry &message
) {
    // a. set header:
    message.header.stamp = timestamp_;
    message.header.frame_id = odom_config_.frame_id;
    
    // b. set child frame id:
    message.child_frame_id = imu_config_.frame_id;

    // b. set pose:
    Eigen::Quaterniond q(R);
    message.pose.pose.orientation.x = q.x();
    message.pose.pose.orientation.y = q.y();
    message.pose.pose.orientation.z = q.z();
    message.pose.pose.orientation.w = q.w();

    // c. set position:
    message.pose.pose.position.x = t.x();
    message.pose.pose.position.y = t.y();
    message.pose.pose.position.z = t.z();  
}

}  // namespace simulator

}  // namespace imu