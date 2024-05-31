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

set(PLUGIN_box_detect
    ON
    CACHE BOOL "Build box_detect")

add_subdirectory(interfaces)

if(PLUGIN_box_detect)
  add_library(box_detect MODULE box_detect_plugin.cpp
                                box_detect_thread.cpp)
  depend_on_tf(box_detect)
  depend_on_ros2(box_detect)
  depend_on_ros2_libs(box_detect "std_srvs")
  target_link_libraries(
      box_detect
      fawkescore
      fawkesutils
      fawkesaspects
      fawkestf
      fawkesinterface
      fawkesblackboard
      fawkesconfig
      BoxInterface)
else()
  plugin_disabled_message("box_detect")
en()
