set(PLUGIN_tag_vision
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_tag_vision)
  pkg_check_modules(ALVAR alvar)
  pkg_check_modules(ALVAR-ROS ar_track_alvar)

  add_library(
    tag_vision MODULE tag_vision_plugin.cpp tag_vision_thread.cpp
                      tag_position_list.cpp tag_position_interface_helper.cpp)
  depend_on_fvconf(tag_vision)
  depend_on_tf(tag_vision)
  if(ALVAR_FOUND)
    target_compile_options(tag_vision PUBLIC -DHAVE_ALVAR ${ALVAR_CFLAGS})
    target_link_libraries(tag_vision ${ALVAR_LDFLAGS})
  else()
    if(ALVAR-ROS_FOUND)
      target_compile_options(tag_vision PUBLIC -DHAVE_AR_TRACK_ALVAR
                                               ${ALVAR-ROS_CFLAGS})
      target_link_libraries(tag_vision ${ALVAR-ROS_LDFLAGS})
    endif()
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
  plugin_disabled_message("tag_vision")
endif()
