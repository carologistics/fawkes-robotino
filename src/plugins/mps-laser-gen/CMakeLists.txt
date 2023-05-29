set(PLUGIN_mps-laser-gen
    ON
    CACHE BOOL ON)
set(PLUGIN_mps-laser-gen-ros2
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

if(PLUGIN_mps-laser-gen-ros2)
  add_library(mps-laser-gen-ros2 MODULE mps-laser-gen-ros2_plugin.cpp
                                   mps-laser-gen_thread.cpp)
  depend_on_eigen3(mps-laser-gen-ros2)
  depend_on_ros2(mps-laser-gen-ros2)
	#find_package(visualization_msgs QUIET)
  #remember_dependency(visualization_msgs)
  depend_on_tf(mps-laser-gen-ros2)
  depend_on_navgraph(mps-laser-gen-ros2)

  target_link_libraries(
    mps-laser-gen-ros2
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    LaserBoxFilterInterface
    fawkesros2aspect)
	depend_on_ros2_libs(mps-laser-gen-ros2 "visualization_msgs")
else()
  plugin_disabled_message("mps-laser-gen-ros2")
endif()
