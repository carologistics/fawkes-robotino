set(PLUGIN_mps-laser-gen
    ON
    CACHE BOOL ON)

if(PLUGIN_mps-laser-gen)
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
  plugin_disabled_message("mps-laser-gen")
endif()
