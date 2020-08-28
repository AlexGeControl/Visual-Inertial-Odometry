cmake_minimum_required(VERSION 3.10)

project(vins_estimator)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++11")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

################################################
## dependencies ##
################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

find_package(Boost REQUIRED COMPONENTS filesystem program_options system regex)
find_package(Threads REQUIRED)

# a. OpenCV3:
find_package(OpenCV 3.4 REQUIRED)
# b. Eigen 3:
find_package(Eigen3 REQUIRED)
# c. Sophus:
find_package(Sophus REQUIRED)
# d. Ceres:
find_package(Ceres REQUIRED)
# e. g2o:
find_package(CSparse REQUIRED)
find_package(g2o REQUIRED)
# f. Pangolin:
find_package(Pangolin REQUIRED)

message(WARNING "OpenCV_VERSION: ${OpenCV_VERSION}")

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${OPENCV_INCLUDE_DIR}
  ${EIGEN3_INCLUDE_DIR}
  ${CERES_INCLUDE_DIRS}
  ${PANGOLIN_INCLUDE_DIR}

  ${PROJECT_SOURCE_DIR}/include
)

## Config G2O components:
set(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core g2o_types_sba cxsparse)

## Declare a C++ library
add_library(camera_model SHARED
  src/camera_models/chessboard/Chessboard.cc
  src/camera_models/calib/CameraCalibration.cc
  src/camera_models/camera_models/Camera.cc
  src/camera_models/camera_models/CameraFactory.cc
  src/camera_models/camera_models/CostFunctionFactory.cc
  src/camera_models/camera_models/PinholeCamera.cc
  src/camera_models/camera_models/CataCamera.cc
  src/camera_models/camera_models/EquidistantCamera.cc
  src/camera_models/camera_models/ScaramuzzaCamera.cc
  src/camera_models/sparse_graph/Transform.cc
  src/camera_models/gpl/gpl.cc
  src/camera_models/gpl/EigenQuaternionParameterization.cc
)
target_link_libraries(camera_model 
  ${Boost_LIBRARIES} 
  ${OpenCV_LIBS} 
  ${CERES_LIBRARIES}
)

## Declare a C++ library
ADD_LIBRARY(MyVio SHARED
  src/System.cpp
  src/parameters.cpp
  src/estimator.cpp
  src/feature_manager.cpp
  src/feature_tracker.cpp

  src/utility/utility.cpp
  src/initial/solve_5pts.cpp
  src/initial/initial_aligment.cpp
  src/initial/initial_sfm.cpp
  src/initial/initial_ex_rotation.cpp

  src/backend/vertex.cc
  src/backend/edge.cc
  src/backend/problem.cc
  src/backend/vertex_pose.cc
  src/backend/edge_reprojection.cc
  src/backend/edge_imu.cc
  src/backend/edge_prior.cpp
  src/backend/loss_function.cc
  src/backend/imu_integration.cc
)
target_link_libraries(MyVio  
  ${OpenCV_LIBS}  
  ${CERES_LIBRARIES}
	${Pangolin_LIBRARIES}
  camera_model
)

## Declare a C++ executable
add_executable(testCurveFitting test/CurveFitting.cpp)
target_link_libraries(testCurveFitting 
  MyVio
)

## Declare a C++ executable
add_executable(run_euroc test/run_euroc.cpp)
target_link_libraries(run_euroc 
  MyVio  
  Threads::Threads
) 

## Declare a C++ executable
add_executable(run_vio_simulation test/run_vio_simulation.cpp)
target_link_libraries(run_vio_simulation 
  MyVio  
  Threads::Threads
)

## Specify artifacts output path:
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)