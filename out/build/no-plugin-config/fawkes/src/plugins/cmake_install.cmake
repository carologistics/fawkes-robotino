# Install script for directory: /home/adas/fawkes-robotino/fawkes/src/plugins

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
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
  set(CMAKE_INSTALL_SO_NO_EXE "0")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/amcl/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/bblogger/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/bbsync/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips-agent/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips-executive/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips-navgraph/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips-pddl-parser/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips-protobuf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips-robot-memory/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/clips-tf/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/execution-time-estimator/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/execution-time-estimator-lookup/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/execution-time-estimator-navgraph/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/flite/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/gazebo/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/imu/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/joystick/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/laser/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/laser-cluster/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/laser-filter/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/laser-lines/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/laser-pointclouds/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/mongodb/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/mongodb_log/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/navgraph/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/navgraph-generator/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/pddl-planner/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/pddl-robot-memory/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/perception/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/realsense2/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/robot-memory/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/robotino/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/ros/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/ros2/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/rrd/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/skiller/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/skiller-simulator/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/static_transforms/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/webview/cmake_install.cmake")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for the subdirectory.
  include("/home/adas/fawkes-robotino/out/build/no-plugin-config/fawkes/src/plugins/pos3d-publisher/cmake_install.cmake")
endif()

