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

set(PLUGIN_ros2-rto-data-in
    ON
    CACHE BOOL "Build ros2-rto-data-in")

set(PLUGIN_ros2-motor-switch
    ON
    CACHE BOOL "Build ros2-motor-switch")

if(PLUGIN_ros2-motor-switch)
  add_library(ros2-motor-switch MODULE motor_switch_plugin.cpp
                                         motor_switch_thread.cpp)
  depend_on_ros2(ros2-motor-switch)
  depend_on_ros2_libs(
    ros2-motor-switch
    "rto_msgs"
  )
  target_link_libraries(
    ros2-motor-switch
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    SwitchInterface
    RTODataInterface)
else()
  plugin_disabled_message(ros2-motor-switch)
endif()

if(PLUGIN_ros2-rto-data-in)
  add_library(ros2-rto-data-in MODULE rto_data_in_plugin.cpp
                                         rto_data_in_thread.cpp)
  depend_on_ros2(ros2-rto-data-in)
  depend_on_ros2_libs(
    ros2-rto-data-in
    "rmw;rosidl_typesupport_interface;rcl_interfaces;rto_msgs"
  )
  target_link_libraries(
    ros2-rto-data-in
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    fawkesros2aspect
    RTODataInterface)
else()
  plugin_disabled_message(ros2-rto-data-in)
endif()
add_subdirectory(interfaces)
