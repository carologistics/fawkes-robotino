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

set(PLUGIN_conveyor-pose
    ON
    CACHE BOOL "Build conveyor-pose")

add_subdirectory(interfaces)

if(PLUGIN_conveyor-pose)
  # pkg_check_modules(BULLET bullet>=2.82)
  add_library(
    conveyor-pose MODULE conveyor_pose_plugin.cpp conveyor_pose_thread.cpp
                         recognition_thread.cpp)

  depend_on_pcl(conveyor-pose)
  depend_on_pcl_extra_libs(conveyor-pose "filters;kdtree;recognition")
  depend_on_ros(conveyor-pose)
  depend_on_tf(conveyor-pose)

  target_link_libraries(
    conveyor-pose
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkestf
    fawkesblackboard
    fawkesinterface
    fawkespcl_utils
    fawkesrosaspect
    SwitchInterface
    ConveyorPoseInterface
    LaserLineInterface)
else()
  plugin_disabled_message("conveyor-pose")
endif()
