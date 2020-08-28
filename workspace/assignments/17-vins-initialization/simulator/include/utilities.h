#ifndef IMUSIMWITHPOINTLINE_UTILITIES_H
#define IMUSIMWITHPOINTLINE_UTILITIES_H

#include "imu.h"
#include <eigen3/Eigen/Core>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include <vector>
#include <fstream>

// save 3D points in homogeneous coordinates as CSV:
void SavePoints(
    const std::string &filename, 
    const std::vector<Eigen::Vector4d,Eigen::aligned_allocator<Eigen::Vector4d>> &points
);

// save 3d points and corresponding observation(in normalized plane):
struct Feature {
    double timestamp;
    size_t id;
    Eigen::Vector4d P_world;
    Eigen::Vector2d p_normalized; 
    Eigen::Vector2d p_image;
};
void SaveFeatures(
    const std::string &filename,
    const std::vector<Feature> &features
);

// save line observations(in normalized plane):
void SaveLines(
    const std::string &filename,
    const std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d>> &features
);


void LoadPose(
    const std::string &filename, 
    std::vector<MotionData>& poses
);

// save poses with IMU measurements:
void SavePose(
    const std::string &filename, 
    const std::vector<MotionData> &poses
);

// save pose in TUM format:
void SavePoseTUM(
    const std::string &filename, 
    const std::vector<MotionData> &poses
);

#endif //IMUSIMWITHPOINTLINE_UTILITIES_H


