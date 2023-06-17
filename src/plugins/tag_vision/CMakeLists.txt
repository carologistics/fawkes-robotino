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

set(PLUGIN_tag_vision
    ON
    CACHE BOOL "Build tag_vision plugin")

add_subdirectory(interfaces)

if(PLUGIN_tag_vision)
  pkg_check_modules(ALVAR alvar)
  pkg_check_modules(ALVAR-ROS ar_track_alvar)

  add_library(
    tag_vision MODULE tag_vision_plugin.cpp tag_vision_thread.cpp
                      tag_position_list.cpp tag_position_interface_helper.cpp)
  depend_on_fvconf(tag_vision)
  depend_on_tf(tag_vision)
  optional_depend_on_pkgconfig_libs(tag_vision alvar BUILD_WITH_ALVAR)
  if(BUILD_WITH_ALVAR)
    target_compile_options(tag_vision PUBLIC -DHAVE_ALVAR)
  else()
    optional_depend_on_pkgconfig_libs(tag_vision ar_track_alvar
                                      BUILD_WITH_ROS_ALVAR)
    if(BUILD_WITH_ROS_ALVAR)
      target_compile_options(tag_vision PUBLIC -DHAVE_AR_TRACK_ALVAR)
    endif()
  endif()
  target_link_libraries(
    tag_vision
    fawkescore
    fawkesutils
    fawkesaspects
    fvutils
    fawkestf
    fawkesinterface
    fawkesblackboard
    Position3DInterface
    LaserLineInterface
    fvcams
    fvmodels
    fvclassifiers
    fvfilters
    fawkesconfig
    TagVisionInterface)
else()
  plugin_disabled_message("tag_vision")
endif()
