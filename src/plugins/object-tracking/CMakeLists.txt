# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann and Daniel Swoboda
#
# *****************************************************************************
#
# This program is free software: you can redistribute it and/or modify it under
# the terms of the GNU General Public License as published by the Free Software
# Foundation, either version 3 of the License, or (at your option) any later
# version.
#
# This program is distributed in the hope that it will be useful, but WITHOUT
# ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
# FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
# details.
#
# You should have received a copy of the GNU General Public License along with
# this program.  If not, see <http://www.gnu.org/licenses/>.
#
# *****************************************************************************

set(PLUGIN_object-tracking
    ON
    CACHE BOOL "Build object-tracking plugin")

if(PLUGIN_object-tracking)
  # Check for OpenCL support
  find_package(OpenCL QUIET)
  if(OpenCL_FOUND)
    message(STATUS "OpenCL found - enabling OpenCL acceleration for object-tracking")
    include_directories(${OpenCL_INCLUDE_DIRS})
    add_definitions(-DHAVE_OPENCL)
  endif()

  # Check for OpenMP support for multi-core processing
  find_package(OpenMP QUIET)
  if(OpenMP_CXX_FOUND)
    message(STATUS "OpenMP found - enabling multi-core processing for object-tracking")
    add_definitions(-D_OPENMP)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
  endif()

  # Raspberry Pi specific optimizations
  if(${CMAKE_SYSTEM_PROCESSOR} MATCHES "arm")
    message(STATUS "ARM processor detected - enabling Raspberry Pi optimizations")
    add_definitions(-DRPI_OPTIMIZATIONS)
    
    # Enable NEON instructions for ARM
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mfpu=neon -ftree-vectorize")
    
    # Optimize specifically for Raspberry Pi 5 Cortex-A76 cores
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -mcpu=cortex-a76 -mtune=cortex-a76")
    
    # Add more aggressive optimizations for image processing
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fomit-frame-pointer -ffast-math -funroll-loops")
    
    # Target ARMv8-A architecture with Advanced SIMD (NEON)
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -march=armv8-a+simd")
  endif()

  # Set optimization level
  set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3")

  add_library(object-tracking MODULE object_tracking_plugin.cpp
                                     object_tracking_thread.cpp)
  target_link_libraries(
    object-tracking
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    ObjectTrackingInterface
    fvutils
    fawkestf
    LaserLineInterface)

  # Add OpenCL libraries if found
  if(OpenCL_FOUND)
    target_link_libraries(object-tracking ${OpenCL_LIBRARIES})
  endif()

  # Add OpenMP libraries if found
  if(OpenMP_CXX_FOUND)
    target_link_libraries(object-tracking ${OpenMP_CXX_LIBRARIES})
  endif()

else()
  plugin_disabled_message("object-tracking")
endif()
