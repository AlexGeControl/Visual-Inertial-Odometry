cmake_minimum_required(VERSION 3.10.2)
project(backend_solver)

## Build options:
option(BUILD_APPS "Build APPs for slam course" YES)
option(BUILD_TESTS "Build test for slam course" No)

## Compile as C++11, supported in ROS Kinetic and newer
set(CMAKE_CXX_FLAGS "-std=c++11 -g -Wall")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS} -Wno-reorder" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS} -DNDEBUG -Wno-reorder -O2" CACHE STRING "" FORCE)

## System dependencies are found with CMake's conventions
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake_modules")

# OpenMP
find_package( OpenMP REQUIRED)
if(OPENMP_FOUND)
    message("OPENMP FOUND")
    ADD_DEFINITIONS(-DUSE_OPENMP)
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
endif()

# Eigen:
find_package(Eigen3 REQUIRED)

# OpenCV
find_package(OpenCV REQUIRED)

# Glog
find_package(Glog REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${PROJECT_SOURCE_DIR}
    ${EIGEN3_INCLUDE_DIR}
    ${OpenCV_INCLUDE_DIRS}
    ${GLOG_INCLUDE_DIRS}
    ${PROJECT_SOURCE_DIR}/thirdparty/Sophus
)

## Specify source files
add_subdirectory(frontend)
add_subdirectory(backend)
add_subdirectory(utils)

## Build release
if (BUILD_APPS)
    add_subdirectory(app)
endif ()

## Build tests
if (BUILD_TESTS)
    enable_testing()
    add_subdirectory(test)
endif ()
