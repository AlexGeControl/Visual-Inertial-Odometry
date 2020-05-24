PROJECT (INTRODUCTION)

SET(SRC_LIST main.cpp)

MESSAGE(STATUS "This is binary directory: " ${PROJECT_BINARY_DIR})
MESSAGE(STATUS "This is source directory: " ${PROJECT_SOURCE_DIR})

ADD_EXECUTABLE(introduction ${SRC_LIST})
