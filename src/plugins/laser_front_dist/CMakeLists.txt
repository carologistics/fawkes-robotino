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

set(PLUGIN_laser_front_dist
    ON
    CACHE BOOL "Build laser_front_dist plugin")

if(PLUGIN_laser_front_dist)
  add_library(laser_front_dist MODULE laser_front_dist_plugin.cpp
                                      laser_front_dist_thread.cpp)
  depend_on_tf(laser_front_dist)
  target_link_libraries(
    laser_front_dist
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    Laser360Interface
    Position3DInterface
    fawkestf)

else()
  plugin_disabled_message("laser_front_dist")
endif()
