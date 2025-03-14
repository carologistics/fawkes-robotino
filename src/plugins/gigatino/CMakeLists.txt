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

set(PLUGIN_gigatino
    ON
    CACHE BOOL "Build gigatino plugin")

if(PLUGIN_gigatino)
  add_library(gigatino MODULE gigatino_plugin.cpp com_thread.cpp)
  depend_on_ros2(gigatino)
set(GIGATINO_ROS_2_DEPS rclcpp_action gigatino_msgs)

  depend_on_ros2_libs(gigatino "${GIGATINO_ROS_2_DEPS}")

  target_link_libraries(
    gigatino
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesros2aspect
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    ArduinoInterface)
else()
  plugin_disabled_message("gigatino")
endif()
