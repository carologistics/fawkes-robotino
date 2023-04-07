set(PLUGIN_conveyor-pose
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_conveyor-pose)
  pkg_check_modules(BULLET bullet>=2.82)
  check_deps_pcl_extra_libs(filters kdtree recognition)
  if(ROS_FOUND
     AND BULLET_FOUND
     AND NOT PCL_WARNING)

    add_library(
      conveyor-pose MODULE conveyor_pose_plugin.cpp conveyor_pose_thread.cpp
                           recognition_thread.cpp)

    depend_on_pcl(conveyor-pose)
    depend_on_ros(conveyor-pose)
    depend_on_tf(conveyor-pose)

    target_link_libraries(
      conveyor-pose
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
      ConveyorPoseInterface
      LaserLineInterface)
  else()
    if(NOT ROS_FOUND)
      build_skipped_message("conveyor-pose" "ROS dependencies")
    endif()
    if(NOT BULLET_FOUND)
      build_skipped_message("conveyor-pose" "bullet dependencies")
    endif()
    if(PCL_WARNING)
      foreach(pcl_warn ${PCL_WARNING})
        message(WARNING "${pcl_warn}")
      endforeach()
    endif()
  endif()
else()
  plugin_disabled_message("conveyor-pose")
endif()
