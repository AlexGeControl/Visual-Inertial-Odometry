cmake_minimum_required(VERSION 3.10.2)

project(msfcomp)

## Compile as C++17
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++17")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g")

################################################
## dependencies ##
################################################
list(APPEND CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake_modules)

## Specify system libs:
find_package(Boost REQUIRED COMPONENTS 
  system 
  program_options 
  filesystem 
  regex
)
find_package(Threads REQUIRED)

###########
## Build ##
###########

## Specify additional locations of header files
include_directories(
  ${Boost_INCLUDE_DIRS}
  ${PROJECT_SOURCE_DIR}/include
)

## Declare a C++ library
add_library(${PROJECT_NAME}_lib
    src/record.cpp
)
target_link_libraries(${PROJECT_NAME}_lib 
  ${Boost_LIBRARIES}
  Threads::Threads
)

## Declare a C++ executable
add_executable(${PROJECT_NAME} src/main.cpp)
target_link_libraries(${PROJECT_NAME} 
  ${PROJECT_NAME}_lib
  ${Boost_LIBRARIES}
  Threads::Threads
)

###############
## Artifacts ##
###############

## Specify artifacts output path:
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)