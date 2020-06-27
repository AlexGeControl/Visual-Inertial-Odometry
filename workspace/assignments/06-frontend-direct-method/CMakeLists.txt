cmake_minimum_required(VERSION 3.10)

project(frontend_direct_method)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

################################################
## dependencies ##
################################################
FIND_PACKAGE(Boost REQUIRED)
# a. OpenCV3:
FIND_PACKAGE(OpenCV 3.4 REQUIRED)
# b. Eigen 3:
FIND_PACKAGE(Eigen3 REQUIRED)
# c. Sophus:
FIND_PACKAGE(Sophus REQUIRED)
# d. Ceres:
FIND_PACKAGE(Ceres REQUIRED)
# e. g2o:
FIND_PACKAGE(g2o REQUIRED)
# f. Pangolin:
FIND_PACKAGE(Pangolin REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES(
    ${BOOST_INCLUDE_DIR}
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${CERES_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    ${PANGOLIN_INCLUDE_DIR}
)

##
## Output executable directory
##
## set(OUTPUT_EXECUTABLE_DIR ./bin)
## SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_EXECUTABLE_DIR})

##
## 1. optical flow
##
ADD_EXECUTABLE(optical_flow 01-optical-flow/optical_flow.cpp)
TARGET_LINK_LIBRARIES(
    optical_flow
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
)

##
## 2. direct method
##
ADD_EXECUTABLE(direct_method 02-direct-method/direct_method.cpp)
TARGET_LINK_LIBRARIES(
    direct_method
    ${BOOST_LIBS}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
    ${Pangolin_LIBRARIES}
)