# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2024 by Daniel Swoboda
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

set(PLUGIN_picam-client
    ON
    CACHE BOOL "Build picam-client plugin")
add_subdirectory(interfaces)
if(PLUGIN_picam-client)
  add_library(picam-client MODULE picam_client_plugin.cpp
                                     picam_client_thread.cpp
                                     base64.cpp)
  target_link_libraries(
    picam-client
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    ObjectTrackingInterface
    PiCamPluginInterface
    fvutils)

else()
  plugin_disabled_message("picam-client")
endif()
