cmake_minimum_required(VERSION 3.10)

project(vio_introduction)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

################################################
## dependencies ##
################################################
FIND_PACKAGE(Boost REQUIRED)
FIND_PACKAGE(Eigen3 REQUIRED)
FIND_PACKAGE(Sophus REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES(
    ${Boost_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
)

##
## 1. Rotation Update
##
ADD_EXECUTABLE(rotation_update 01-rotation-update/rotation_update.cpp)
TARGET_LINK_LIBRARIES(
    rotation_update
    ${BOOST_LIBS}
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
)