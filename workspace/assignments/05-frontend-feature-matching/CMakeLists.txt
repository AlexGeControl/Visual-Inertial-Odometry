cmake_minimum_required(VERSION 3.10)

project(frontend_feature_matching)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

################################################
## dependencies ##
################################################
# a. OpenCV3:
FIND_PACKAGE(OpenCV 3.4 REQUIRED)
# b. Eigen 3:
FIND_PACKAGE(Eigen3 REQUIRED)
# c. Ceres:
FIND_PACKAGE(Ceres REQUIRED)
# d. g2o:
SET(g2o REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES(
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${CERES_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
)

##
## Output executable directory
##
## set(OUTPUT_EXECUTABLE_DIR ./bin)
## SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_EXECUTABLE_DIR})

##
## 1. ORB feature matching
##
ADD_EXECUTABLE(orb_feature_matching 01-orb-feature-matching/compute_orb.cpp)
TARGET_LINK_LIBRARIES(
    orb_feature_matching
    ${OpenCV_LIBS}
)