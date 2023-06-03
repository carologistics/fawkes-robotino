set(PLUGIN_conveyor-plane
    ON
    CACHE BOOL Build conveyor-plane)

add_subdirectory(interfaces)

if(PLUGIN_conveyor-plane)
  # pkg_check_modules(BULLET bullet>=2.82)
  add_library(conveyor-plane MODULE conveyor_plane_plugin.cpp
                                    conveyor_plane_thread.cpp)

  depend_on_pcl(conveyor-plane)
  depend_on_pcl_extra_libs(
    conveyor-plane "sample_consensus;segmentation;filters;surface;features")
  depend_on_ros(conveyor-plane)
  depend_on_tf(conveyor-plane)

  target_link_libraries(
    conveyor-plane
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkestf
    fawkesblackboard
    fawkesinterface
    fawkespcl_utils
    fawkesrosaspect
    SwitchInterface
    Position3DInterface
    LaserLineInterface)
else()
  plugin_disabled_message("conveyor-plane")
endif()
