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
## 1. ORB feature matching
##
ADD_EXECUTABLE(orb_feature_matching 01-orb-feature-matching/compute_orb.cpp)
TARGET_LINK_LIBRARIES(
    orb_feature_matching
    ${OpenCV_LIBS}
)

##
## 2. essential matrix to relative pose
##
ADD_EXECUTABLE(essential_matrix_to_relative_pose 02-essential-matrix-to-pose/E2Rt.cpp)
TARGET_LINK_LIBRARIES(
    essential_matrix_to_relative_pose
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
)

##
## 3. PnP using Gaussian-Newton on se(3)
##
ADD_EXECUTABLE(pnp_gaussian_newton 03-pnp/GN-BA.cpp)
TARGET_LINK_LIBRARIES(
    pnp_gaussian_newton
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
)

##
## 4.a ICP using SVD
##
ADD_EXECUTABLE(icp_svd 04-icp/SVD.cpp)
TARGET_LINK_LIBRARIES(
    icp_svd
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
    ${Pangolin_LIBRARIES}
)

##
## 4.a ICP using Gaussian-Newton on se(3)
##
ADD_EXECUTABLE(icp_gaussian_newton 04-icp/G-N.cpp)
TARGET_LINK_LIBRARIES(
    icp_gaussian_newton
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
    ${Pangolin_LIBRARIES}
)
