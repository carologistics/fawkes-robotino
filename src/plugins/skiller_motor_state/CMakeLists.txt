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

set(PLUGIN_skiller_motor_state
    ON
    CACHE BOOL "Build skiller_motor_state plugin")

if(PLUGIN_skiller_motor_state)
  add_library(skiller_motor_state MODULE skiller_motor_state_plugin.cpp
                                         skiller_motor_state_thread.cpp)
  target_link_libraries(
    skiller_motor_state
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    SkillerInterface
    RobotinoSensorInterface
    MotorInterface)

  target_compile_features(skiller_motor_state PUBLIC cxx_std_11)
  depend_on_cpp_version(skiller_motor_state 11)
else()
  plugin_disabled_message("skiller_motor_state")
endif()
