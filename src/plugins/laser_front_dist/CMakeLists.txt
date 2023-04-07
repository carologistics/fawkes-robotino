set(PLUGIN_laser_front_dist
    ON
    CACHE BOOL ON)

if(PLUGIN_laser_front_dist)
  if(TF_FOUND)
    add_library(laser_front_dist MODULE laser_front_dist_plugin.cpp
                                        laser_front_dist_thread.cpp)
    depend_on_tf(laser_front_dist)
    target_link_libraries(
      laser_front_dist
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesbaseapp
      fawkesblackboard
      fawkesinterface
      Laser360Interface
      Position3DInterface
      fawkestf)

  else()
    build_skipped_message("laser_front_dist" "TF dependencies")
  endif()

else()
  plugin_disabled_message("laser_front_dist")
endif()
