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

set(PLUGIN_mill
    ON
    CACHE BOOL "Build mill plugin")

add_subdirectory(interfaces)

if(PLUGIN_mill)
  add_library(mill MODULE mill_plugin.cpp mill_thread.cpp)
  depend_on_pkgconfig_libs(mill libudev)
  depend_on_pcl(mill)
  depend_on_tf(mill)
  depend_on_boost_libs(mill "thread;chrono")
  target_link_libraries(
    mill
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    ArduinoInterface
    RTODataInterface)
else()
  plugin_disabled_message("mill")
endif()
