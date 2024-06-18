# *****************************************************************************
# CMake Build System for Fawkes
# -------------------
# Copyright (C) 2024 by Carologistics
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
cmake_minimum_required(VERSION 3.0)
project(MyProject)

# Function to check and update submodules
function(update_submodules)
  # Read the .gitmodules file
  file(READ "${CMAKE_SOURCE_DIR}/.gitmodules" gitmodules_content)

  # Initialize an empty list to store submodule paths
  set(submodule_paths "")

  # Use a regex to extract the paths of the submodules
  string(REGEX MATCHALL "path = ([^\n]+)" matches "${gitmodules_content}")

  # Iterate over the matches to extract the submodule paths
  foreach(match ${matches})
    string(REGEX REPLACE "path = " "" submodule_path "${match}")
    list(APPEND submodule_paths "${CMAKE_SOURCE_DIR}/${submodule_path}")
  endforeach()

  # Display the submodule paths for debugging purposes
  message(STATUS "Submodule paths: ${submodule_paths}")

  # Update each submodule
  foreach(submodule_path ${submodule_paths})
    if( NOT EXISTS "${submodule_path}/.git" )
      execute_process(
        COMMAND git submodule update --init --recursive ${submodule_path}
      )

      if (result)
        message(FATAL_ERROR "Error at submodule ${submodule_path}: ${error}")
      else()
        message(STATUS "Submodule ${submodule_path} updated successfully")
      endif()
    endif()
  endforeach()
endfunction()
