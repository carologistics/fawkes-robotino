set(PLUGIN_object-tracking
    ON
    CACHE BOOL ON)

if(PLUGIN_object-tracking)
  add_library(object-tracking MODULE object_tracking_plugin.cpp
                                     object_tracking_thread.cpp)
  target_link_libraries(
    object-tracking
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesinterface
    fawkesblackboard
    ObjectTrackingInterface
    fvutils
    fawkestf
    LaserLineInterface)

else()
  plugin_disabled_message("object-tracking")
endif()
