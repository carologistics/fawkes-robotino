# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2024 by Tarik Viehmann
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

set(PLUGIN_agent-task-skiller-bridge
    ON
    CACHE BOOL "Build agent-task-skiller-bridge")

if(PLUGIN_clips-motor-switch)
  add_library(agent-task-skiller-bridge MODULE bridge-plugin.cpp bridge-thread.cpp)
  depend_on_protobuf(agent-task-skiller-bridge)
  depend_on_boost_libs(agent-task-skiller-bridge system)
  target_link_libraries(
    agent-task-skiller-bridge
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    fawkesconfig
    SkillerInterface
    rcll-protobuf-msgs
    protobuf_comm
    protobuf::libprotobuf )
else()
  plugin_disabled_message("agent-task-skiller-bridge")
endif()
