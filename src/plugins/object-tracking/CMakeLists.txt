set(PLUGIN_object-tracking
    ON
    CACHE BOOL ON)

if(PLUGIN_object-tracking)
  if(TF_FOUND
     AND NAVGRAPH_FOUND
     AND OPENCV_FOUND)
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
      fawkestf)

  else()
    if(NOT TF_FOUND)
      build_skipped_message("object-tracking" "TF dependencies")
    endif()
    if(NOT NAVGRAPH_FOUND)
      build_skipped_message("object-tracking" "navgraph dependencies")
    endif()
    if(NOT OPENCV_FOUND)
      build_skipped_message("object-tracking" "OpenCV dependencies")
    endif()
  endif()
else()
  plugin_disabled_message("object-tracking")
endif()
