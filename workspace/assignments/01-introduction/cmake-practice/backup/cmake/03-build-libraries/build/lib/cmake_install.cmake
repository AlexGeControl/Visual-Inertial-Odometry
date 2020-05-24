# Install script for directory: /home/alexge/Workspace/Distributed-System/Spark/01-ML-theory/03-robotics/05-self-driving-car/05-visual-slam/01-introduction/cmake-practice/backup/cmake/03-build-libraries/lib

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/tmp")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhello.so.1.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhello.so.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhello.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      file(RPATH_CHECK
           FILE "${file}"
           RPATH "")
    endif()
  endforeach()
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE SHARED_LIBRARY FILES
    "/home/alexge/Workspace/Distributed-System/Spark/01-ML-theory/03-robotics/05-self-driving-car/05-visual-slam/01-introduction/cmake-practice/backup/cmake/03-build-libraries/build/lib/libhello.so.1.0"
    "/home/alexge/Workspace/Distributed-System/Spark/01-ML-theory/03-robotics/05-self-driving-car/05-visual-slam/01-introduction/cmake-practice/backup/cmake/03-build-libraries/build/lib/libhello.so.1"
    "/home/alexge/Workspace/Distributed-System/Spark/01-ML-theory/03-robotics/05-self-driving-car/05-visual-slam/01-introduction/cmake-practice/backup/cmake/03-build-libraries/build/lib/libhello.so"
    )
  foreach(file
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhello.so.1.0"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhello.so.1"
      "$ENV{DESTDIR}${CMAKE_INSTALL_PREFIX}/lib/libhello.so"
      )
    if(EXISTS "${file}" AND
       NOT IS_SYMLINK "${file}")
      if(CMAKE_INSTALL_DO_STRIP)
        execute_process(COMMAND "/usr/bin/strip" "${file}")
      endif()
    endif()
  endforeach()
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/lib" TYPE STATIC_LIBRARY FILES "/home/alexge/Workspace/Distributed-System/Spark/01-ML-theory/03-robotics/05-self-driving-car/05-visual-slam/01-introduction/cmake-practice/backup/cmake/03-build-libraries/build/lib/libhello.a")
endif()

if(NOT CMAKE_INSTALL_COMPONENT OR "${CMAKE_INSTALL_COMPONENT}" STREQUAL "Unspecified")
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include/hello" TYPE FILE FILES "/home/alexge/Workspace/Distributed-System/Spark/01-ML-theory/03-robotics/05-self-driving-car/05-visual-slam/01-introduction/cmake-practice/backup/cmake/03-build-libraries/lib/hello.h")
endif()

