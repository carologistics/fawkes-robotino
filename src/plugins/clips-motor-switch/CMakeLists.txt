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

set(PLUGIN_clips-motor-switch
    ON
    CACHE BOOL "Build clips-motor-switch")

if(PLUGIN_clips-motor-switch)
  add_library(clips-motor-switch MODULE clips-motor-switch-plugin.cpp
                                        clips-motor-switch-thread.cpp)
  depend_on_clipsmm(clips-motor-switch)
  target_link_libraries(
    clips-motor-switch
    fawkescore
    fawkesutils
    fawkesconfig
    fawkesaspects
    fawkesclipsaspect
    fawkesinterface)
  depend_on_ros2(clips-motor-switch)
  depend_on_ros2_libs(
    clips-motor-switch
    "rto_msgs"
  )
else()
  plugin_disabled_message("clips-motor-switch")
endif()
