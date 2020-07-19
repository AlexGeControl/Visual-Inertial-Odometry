#ifndef IMU_SIMULATOR_CONSTANTS_H
#define IMU_SIMULATOR_CONSTANTS_H

#include <cmath>
#include <ros/ros.h>

namespace imu {

namespace simulator {

constexpr double kRhoX = 3.0;
constexpr double kRhoY = 4.0;
constexpr double kRhoZ = 1.0;

constexpr double kOmegaXY = M_PI / 10.0;
constexpr double kOmegaZ = 10.0 * kOmegaXY;

constexpr double kYaw = M_PI / 10.0;
constexpr double kPitch = 0.20;
constexpr double kRoll = 0.10; 

}  // namespace simulator

}  // namespace imu

#endif  // IMU_SIMULATOR_CONSTANTS_H