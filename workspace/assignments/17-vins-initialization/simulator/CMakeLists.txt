cmake_minimum_required(VERSION 3.10)

project(vio_data_simulation)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)
add_definitions(-std=c++11)

################################################
## dependencies ##
################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

find_package(Boost REQUIRED)
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

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
    ${OPENCV_INCLUDE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${SOPHUS_INCLUDE_DIR}
    ${CERES_INCLUDE_DIR}
    ${CSPARSE_INCLUDE_DIR}
    ${G2O_INCLUDE_DIR}
    ${PANGOLIN_INCLUDE_DIR}

    ${PROJECT_SOURCE_DIR}/include
)

## Config G2O components:
set(G2O_LIBS g2o_csparse_extension g2o_stuff g2o_core g2o_types_sba cxsparse)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
    src/imu.cpp
    src/param.cpp
    src/utilities.cpp
)
target_link_libraries(${PROJECT_NAME}_lib
    ${EIGEN3_LIBS}
    ${SOPHUS_LIBS}
    ${OpenCV_LIBS}
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} 
    src/main.cpp
)
target_link_libraries(${PROJECT_NAME}
    ${EIGEN3_LIBS}
    ${PROJECT_NAME}_lib
)

## Specify artifacts output path:
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)
