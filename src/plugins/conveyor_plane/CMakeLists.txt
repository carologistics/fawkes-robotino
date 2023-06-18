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

set(PLUGIN_conveyor-plane
    ON
    CACHE BOOL "Build conveyor-plane")

add_subdirectory(interfaces)

if(PLUGIN_conveyor-plane)
  # pkg_check_modules(BULLET bullet>=2.82)
  add_library(conveyor-plane MODULE conveyor_plane_plugin.cpp
                                    conveyor_plane_thread.cpp)

  depend_on_pcl(conveyor-plane)
  depend_on_pcl_extra_libs(
    conveyor-plane "sample_consensus;segmentation;filters;surface;features")
  depend_on_ros(conveyor-plane)
  depend_on_tf(conveyor-plane)

  target_link_libraries(
    conveyor-plane
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
    Position3DInterface
    LaserLineInterface)
else()
  plugin_disabled_message("conveyor-plane")
endif()
