set(PLUGIN_mps-laser-gen
    ON
    CACHE BOOL ON)

if(PLUGIN_mps-laser-gen)
  if(EIGEN3_FOUND
     AND ROS_FOUND
     AND NAVGRAPH_FOUND
     AND TF_FOUND)
    add_library(mps-laser-gen MODULE mps-laser-gen_plugin.cpp
                                     mps-laser-gen_thread.cpp)
    depend_on_eigen3(mps-laser-gen)
    depend_on_ros(mps-laser-gen)
    depend_on_tf(mps-laser-gen)
    depend_on_navgraph(mps-laser-gen)

    target_link_libraries(
      mps-laser-gen
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesbaseapp
      fawkesblackboard
      fawkesinterface
      fawkesrosaspect
      LaserBoxFilterInterface)
  else()
    if(NOT EIGEN3_FOUND)
      build_skipped_message("mps-laser-gen" "eigen3 dependencies")
    endif()
    if(NOT TF_FOUND)
      build_skipped_message("mps-laser-gen" "TF dependencies")
    endif()
    if(NOT NAVGRAPH_FOUND)
      build_skipped_message("mps-laser-gen" "navgraph dependencies")
    endif()
    if(NOT ROS_FOUND)
      build_skipped_message("mps-laser-gen" "ROS dependencies")
    endif()
  endif()
else()
  plugin_disabled_message("mps-laser-gen")
endif()
