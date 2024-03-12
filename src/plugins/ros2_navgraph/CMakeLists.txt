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

set(PLUGIN_ros2-cx-navgraph
  ON
  CACHE BOOL "Build ros2 cx-navgraph plugin")

if(PLUGIN_ros2-cx-navgraph)
  add_library(ros2-cx-navgraph MODULE navgraph_plugin.cpp
    navgraph_thread.cpp navgraph_node.cpp)
  depend_on_ros2(ros2-cx-navgraph)
  depend_on_ros2_libs(
    ros2-cx-navgraph
    "rmw;rosidl_typesupport_interface;std_srvs::std_srvs__rosidl_generator_cpp;rcl_interfaces;cx_msgs;lifecycle_msgs;rclcpp_lifecycle"
  )
  target_link_libraries(
    ros2-cx-navgraph
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesblackboard
    fawkesinterface
    NavGraphGeneratorInterface
    NavGraphWithMPSGeneratorInterface
    fawkesros2aspect)
else()
  plugin_disabled_message(ros2-cx-navgraph)
endif()
