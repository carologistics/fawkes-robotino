set(PLUGIN_tag_vision
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_tag_vision)
  pkg_check_modules(ALVAR alvar)
  pkg_check_modules(ALVAR-ROS ar_track_alvar)

  if(BOOST_FOUND
     AND TF_FOUND
     AND OPENCV_FOUND
     AND ROS_FOUND
     AND ALVAR_FOUND
     OR ALVAR-ROS_FOUND)
    add_library(
      tag_vision MODULE tag_vision_plugin.cpp tag_vision_thread.cpp
                        tag_position_list.cpp tag_position_interface_helper.cpp)
    depend_on_fvconf(tag_vision)
    depend_on_ros(tag_vision)
    depend_on_tf(tag_vision)
    if(ALVAR_FOUND)
      target_compile_options(tag_vision PUBLIC -DHAVE_ALVAR ${ALVAR_CFLAGS})
      target_link_libraries(tag_vision ${ALVAR_LDFLAGS})
    else()
      target_compile_options(tag_vision PUBLIC -DHAVE_AR_TRACK_ALVAR
                                               ${ALVAR-ROS_CFLAGS})
      target_link_libraries(tag_vision ${ALVAR-ROS_LDFLAGS})
    endif()
    target_link_libraries(
      tag_vision
      fawkescore
      fawkesutils
      fawkesaspects
      fvutils
      fawkestf
      fawkesinterface
      fawkesblackboard
      Position3DInterface
      LaserLineInterface
      fvcams
      fvmodels
      fvclassifiers
      fvfilters
      fawkesconfig
      TagVisionInterface)
  else()
    if(NOT BOOST_FOUND)
      build_skipped_message("tag_vision" "BOOST dependencies")
    endif()
    if(NOT TF_FOUND)
      build_skipped_message("tag_vision" "TF dependencies")
    endif()
    if(NOT OPENCV_FOUND)
      build_skipped_message("tag_vision" "OpenCV dependencies")
    endif()
    if(NOT ROS_FOUND)
      build_skipped_message("tag_vision" "ROS dependencies")
    endif()
    if(NOT ALVAR_FOUND)
      build_skipped_message("tag_vision" "ALVAR dependencies")
    endif()
  endif()
else()
  plugin_disabled_message("tag_vision")
endif()
