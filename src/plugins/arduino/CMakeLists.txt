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

set(PLUGIN_arduino
    ON
    CACHE BOOL "Build arduino plugin")
set(DEV_ARDUINO
    /dev/arduino
    CACHE STRING Arduino_Location)

add_subdirectory(interfaces)

if(PLUGIN_arduino)
  add_library(arduino MODULE arduino_plugin.cpp com_thread.cpp tf_thread.cpp
                             com_message.cpp serialport.cpp)
  depend_on_pkgconfig_libs(arduino libudev)
  depend_on_pcl(arduino)
  depend_on_tf(arduino)
  depend_on_boost_libs(arduino "thread;chrono")
  target_link_libraries(
    arduino
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    ArduinoInterface
    JoystickInterface)

  add_custom_target(
    flash_arduino ALL
    /bin/sh ${PROJECT_SOURCE_DIR}/etc/scripts/flash_arduino.sh
    ${CMAKE_CURRENT_SOURCE_DIR} ${DEV_ARDUINO}
    COMMENT "flash the arduino")
  add_dependencies(arduino flash_arduino)
  # target_compile_options(arduino PRIVATE -ggdb3 -O0) set(CMAKE_CXX_FLAGS
  # "${CMAKE_CXX_FLAGS} -g -O0") Uncomment when using gdb and having trouble to
  # connect to the serial port
else()
  plugin_disabled_message("arduino")
endif()
