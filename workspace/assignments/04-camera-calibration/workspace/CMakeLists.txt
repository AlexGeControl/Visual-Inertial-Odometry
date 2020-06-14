CMAKE_MINIMUM_REQUIRED(VERSION 3.5)

PROJECT(OBSERVATION_MODEL)

# OpenCV3:
FIND_PACKAGE(OpenCV 3.0 QUIET)
IF(NOT OpenCV_FOUND)
   FIND_PACKAGE(OpenCV 2.4.3 QUIET)
   IF(NOT OpenCV_FOUND)
      MESSAGE(FATAL_ERROR "OpenCV > 2.4.3 not found.")
   ENDIF()
ENDIF()
# Eigen 3:
FIND_PACKAGE(Eigen3 REQUIRED)
# Ceres:
FIND_PACKAGE(Ceres REQUIRED)
# g2o:
SET(G2O_INCLUDE_DIR "/usr/local/include/g2o")
# Pangolin:
FIND_PACKAGE(Pangolin REQUIRED)

# include directories:
INCLUDE_DIRECTORIES(
    ${EIGEN3_INCLUDE_DIR}
    ${Pangolin_INCLUDE_DIRS}
    ${G2O_INCLUDE_DIR}
)

# runnable:
ADD_COMPILE_OPTIONS(-std=c++11)

# 01: Image Undistortion:
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/01-undistort-image)
ADD_EXECUTABLE(undistort-image 01-undistort-image/undistort_image.cpp)
TARGET_LINK_LIBRARIES(
    undistort-image
    ${OpenCV_LIBS}
)

# 02: Stereo Vision:
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/02-stereo-vision)
ADD_EXECUTABLE(stereo-vision 02-stereo-vision/disparity.cpp)
TARGET_LINK_LIBRARIES(
    stereo-vision
    ${OpenCV_LIBS}
    ${Pangolin_LIBRARIES}
)

# 03: Gaussian-Newton estimation:
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/03-gaussian-newton-estimation)
ADD_EXECUTABLE(gaussian-newton 03-gaussian-newton-estimation/gaussnewton.cpp)
TARGET_LINK_LIBRARIES(
    gaussian-newton
    ${OpenCV_LIBS}
)

# 04: Optimization through Ceres:
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/04-intro-to-ceres)
ADD_EXECUTABLE(intro-to-ceres 04-intro-to-ceres/intro-to-ceres.cpp)
TARGET_LINK_LIBRARIES(
    intro-to-ceres
    ${OpenCV_LIBS}
    ${CERES_LIBRARIES}
)

# 05: Optimization with g2o:
SET(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/05-intro-to-g2o)
ADD_EXECUTABLE(intro-to-g2o 05-intro-to-g2o/intro-to-g2o.cpp)
TARGET_LINK_LIBRARIES(
    intro-to-g2o
    ${OpenCV_LIBS}
    ${EIGEN3_LIBS}
    g2o_core 
    g2o_stuff
)