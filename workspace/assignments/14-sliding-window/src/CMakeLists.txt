cmake_minimum_required(VERSION 3.10)

project(hessian_nullspace_test)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

################################################
## dependencies ##
################################################
FIND_PACKAGE(Eigen3 REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES(
    ${EIGEN3_INCLUDE_DIR}
)

##
## Hessian nullspace test
##
ADD_EXECUTABLE(${PROJECT_NAME} hessian_nullspace_test.cpp)