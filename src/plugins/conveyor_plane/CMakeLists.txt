set(PLUGIN_conveyor-plane
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_conveyor-plane)
  pkg_check_modules(BULLET bullet>=2.82)
  check_deps_pcl_extra_libs(sample_consensus segmentation filters surface features)
  if(ROS_FOUND
     AND BULLET_FOUND
     AND NOT PCL_WARNING)

    add_library(
      conveyor-plane MODULE conveyor_plane_plugin.cpp conveyor_plane_thread.cpp)

    depend_on_pcl(conveyor-plane)
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
    if(NOT ROS_FOUND)
      build_skipped_message("conveyor-plane" "ROS dependencies")
    endif()
    if(NOT BULLET_FOUND)
      build_skipped_message("conveyor-plane" "bullet dependencies")
    endif()
    if(PCL_WARNING)
      foreach(pcl_warn ${PCL_WARNING})
        message(WARNING "${pcl_warn}")
      endforeach()
    endif()
  endif()
else()
  plugin_disabled_message("conveyor-plane")
endif()
