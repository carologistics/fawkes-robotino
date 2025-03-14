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

set_output_directory_plugins()

set(CMAKE_SHARED_MODULE_PREFIX "")
add_subdirectory(agent-task-skiller-bridge)
add_subdirectory(clips-motor-switch)
add_subdirectory(navgraph-generator-mps)
add_subdirectory(skiller_motor_state)
add_subdirectory(mps-laser-gen)
add_subdirectory(laser_front_dist)
add_subdirectory(tag_vision)
add_subdirectory(picam-client)
add_subdirectory(object-tracking)
add_subdirectory(conveyor_pose)
add_subdirectory(conveyor_plane)
add_subdirectory(arduino)
add_subdirectory(box_detect)
add_subdirectory(ros2)
add_subdirectory(gigatino)
