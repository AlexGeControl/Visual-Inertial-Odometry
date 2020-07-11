cmake_minimum_required(VERSION 3.10)

project(close_loop_detection)

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

################################################
## dependencies ##
################################################
FIND_PACKAGE(Boost REQUIRED COMPONENTS 
    system 
    program_options
    thread 
    filesystem
)
# a. OpenCV3:
FIND_PACKAGE(OpenCV 3.4 REQUIRED)
FIND_PACKAGE(DBoW3 REQUIRED)
# b. Eigen 3:
FIND_PACKAGE(Eigen3 REQUIRED)
# c. Sophus:
FIND_PACKAGE(Sophus REQUIRED)
# d. Ceres:
FIND_PACKAGE(Ceres REQUIRED)
# e. g2o:
FIND_PACKAGE(CSparse REQUIRED)
FIND_PACKAGE(g2o REQUIRED)
# f. Pangolin:
FIND_PACKAGE(Pangolin REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
INCLUDE_DIRECTORIES(
    ${Boost_INCLUDE_DIRS}
    ${OPENCV_INCLUDE_DIR}
    ${DBoW3_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${CERES_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    ${PANGOLIN_INCLUDE_DIR}
)

##
## Output executable directory
##
## set(OUTPUT_EXECUTABLE_DIR ./bin)
## SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${OUTPUT_EXECUTABLE_DIR})

SET(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core g2o_types_sba cxsparse)

##
## 1. Build DBoW
##
ADD_EXECUTABLE(dbow_demo 01-dbow-demo/dbow_demo.cpp)
TARGET_LINK_LIBRARIES(
    dbow_demo
    ${Boost_LIBRARIES}
    ${OpenCV_LIBS}
    ${DBoW3_LIBRARIES}
)