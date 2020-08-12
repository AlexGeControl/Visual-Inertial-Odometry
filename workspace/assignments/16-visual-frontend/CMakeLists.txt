cmake_minimum_required(VERSION 3.10.2)

project(visual_frontend)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

################################################
## dependencies ##
################################################
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)

find_package(Boost REQUIRED COMPONENTS 
    program_options
)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
    ${Boost_INCLUDE_DIRS}
    ${EIGEN3_INCLUDE_DIR}
    ${PROJECT_SOURCE_DIR}/include
)

add_executable(triangulate src/triangulate.cpp)
target_link_libraries(
    triangulate
    ${Boost_LIBRARIES}
)