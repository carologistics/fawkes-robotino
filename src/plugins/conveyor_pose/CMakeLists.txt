set(PLUGIN_conveyor-pose
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_conveyor-pose)
  pkg_check_modules(BULLET bullet>=2.82)
  add_library(
    conveyor-pose MODULE conveyor_pose_plugin.cpp conveyor_pose_thread.cpp
                         recognition_thread.cpp)

  depend_on_pcl(conveyor-pose)
  depend_on_pcl_extra_libs(conveyor-pose "filters;kdtree;recognition")
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
  plugin_disabled_message("conveyor-pose")
endif()
