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
