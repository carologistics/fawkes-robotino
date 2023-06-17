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

set(PLUGIN_navgraph-generator-mps
    ON
    CACHE BOOL "Build navgraph-generator-mps plugin")
add_subdirectory(interfaces)
if(PLUGIN_navgraph-generator-mps)
  add_library(navgraph-generator-mps MODULE navgraph_generator_mps_plugin.cpp
                                            navgraph_generator_mps_thread.cpp)
  depend_on_eigen3(navgraph-generator-mps)
  depend_on_cgal_libs(navgraph-generator-mps)
  depend_on_tf(navgraph-generator-mps)
  depend_on_navgraph(navgraph-generator-mps)
  depend_on_clipsmm(navgraph-generator-mps)
  target_link_libraries(
    navgraph-generator-mps
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkestf
    fawkesnavgraph
    Position3DInterface
    NavGraphGeneratorInterface
    NavGraphWithMPSGeneratorInterface)
else()
  plugin_disabled_message("navgraph-generator-mps")
endif()
