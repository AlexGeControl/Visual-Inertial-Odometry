cmake_minimum_required(VERSION 3.10)

project(backend_optimization)

LIST(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

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
    ./01-bal-g2o/include
    ./02-direct-ba/include
    ${OPENCV_INCLUDE_DIR}
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
## 1. BAL G2O
##
SET(LIB_BAL_PROBLEM bal_problem)
SET(LIB_BAL_PROBLEM_SOURCES 01-bal-g2o/src/bal.cpp)
ADD_LIBRARY(${LIB_BAL_PROBLEM} SHARED ${LIB_BAL_PROBLEM_SOURCES})
TARGET_LINK_LIBRARIES(
    ${LIB_BAL_PROBLEM}
    ${EIGEN3_LIBS}
)

ADD_EXECUTABLE(bal_g2o 01-bal-g2o/src/bal_g2o.cpp)
TARGET_LINK_LIBRARIES(
    bal_g2o
    ${BOOST_LIBS}
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
    ${G2O_LIBS}
    ${LIB_BAL_PROBLEM}
)

##
## 2. Direct BA
##
ADD_EXECUTABLE(direct_ba 02-direct-ba/direct_ba.cpp)
TARGET_LINK_LIBRARIES(
    direct_ba
    ${BOOST_LIBS}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
    ${G2O_LIBS}
    ${Pangolin_LIBRARIES}
)

##
## 2. Direct BA
##
ADD_EXECUTABLE(direct_ba_ref 03-ref/direct_ba.cpp)
TARGET_LINK_LIBRARIES(
    direct_ba_ref
    ${BOOST_LIBS}
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
    ${G2O_LIBS}
    ${Pangolin_LIBRARIES}
)