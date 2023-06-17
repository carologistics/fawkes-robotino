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

cmake_minimum_required(VERSION 3.21)
project(fawkes-robotino)
set(FAWKES_DIR 1)
set(FAWKES_CORE_DIR ${CMAKE_CURRENT_SOURCE_DIR}/fawkes)
# make local cmake modules available
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${FAWKES_CORE_DIR}/cmake")
include(base)

include_directories(${FAWKES_CORE_DIR}/src)
add_subdirectory(fawkes)

include_directories(${FAWKES_CORE_DIR}/src/libs)
add_subdirectory(src)
add_subdirectory(etc)

set_common_properties_of_targets(all_targets)
