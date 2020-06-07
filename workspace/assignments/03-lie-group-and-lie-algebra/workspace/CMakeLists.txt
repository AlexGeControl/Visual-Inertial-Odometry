CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

PROJECT(TRAJECTORY_ANALYSIS)

# Eigen3:
FIND_PACKAGE(Eigen3 REQUIRED)
# Pangolin:
FIND_PACKAGE(Pangolin REQUIRED)
# Sophus:
SET(SOPHUS_INCLUDE_DIR "/home/alexge/svslocal/include")
INCLUDE_DIRECTORIES(
    ${SOPHUS_INCLUDE_DIR}
)
LINK_DIRECTORIES(/home/alexge/svslocal/lib)

# runnable:
ADD_COMPILE_OPTIONS(-std=c++11)

# 01: draw trajectory:
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/01-draw-trajectory)
ADD_EXECUTABLE(draw-trajectory 01-draw-trajectory/draw_trajectory.cpp)
TARGET_LINK_LIBRARIES(
    draw-trajectory 
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    Sophus
)

# 02: error analysis
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/02-error-analysis)
ADD_EXECUTABLE(error-analysis 02-error-analysis/error-analysis.cpp)
TARGET_LINK_LIBRARIES(
    error-analysis 
    ${EIGEN3_LIBS}
    ${Pangolin_LIBRARIES}
    Sophus
)