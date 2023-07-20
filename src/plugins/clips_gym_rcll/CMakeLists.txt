# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2023 by Tarik Viehmann
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
set(PLUGIN_clips_gym_rcll
    ON
    CACHE BOOL "Build clips_gym_rcll")

if(PLUGIN_clips_gym_rcll)
  add_library(clips_gym_rcll MODULE clips_gym_rcll_plugin.cpp
                                    clips_gym_rcll_thread.cpp)

  depend_on_clipsmm(clips_gym_rcll)
  depend_on_cpp_version(clips_gym_rcll 17)
  target_link_libraries(clips_gym_rcll pybind11::headers)
  target_link_libraries(
    clips_gym_rcll
    fawkes_clips_gym
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    fawkesclipsaspect
    fawkeslogging
    Python3::Module
    Python3::Python)
else()
  plugin_disabled_message("clips_gym_rcll")
endif()
