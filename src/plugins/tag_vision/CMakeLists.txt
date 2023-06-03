set(PLUGIN_tag_vision
    ON
    CACHE BOOL Build tag_vision plugin)

add_subdirectory(interfaces)

if(PLUGIN_tag_vision)
  pkg_check_modules(ALVAR alvar)
  pkg_check_modules(ALVAR-ROS ar_track_alvar)

  add_library(
    tag_vision MODULE tag_vision_plugin.cpp tag_vision_thread.cpp
                      tag_position_list.cpp tag_position_interface_helper.cpp)
  depend_on_fvconf(tag_vision)
  depend_on_tf(tag_vision)
  optional_depend_on_pkgconfig_libs(tag_vision alvar BUILD_WITH_ALVAR)
  if(BUILD_WITH_ALVAR)
    target_compile_options(tag_vision PUBLIC -DHAVE_ALVAR)
  else()
    optional_depend_on_pkgconfig_libs(tag_vision ar_track_alvar
                                      BUILD_WITH_ROS_ALVAR)
    if(BUILD_WITH_ROS_ALVAR)
      target_compile_options(tag_vision PUBLIC -DHAVE_AR_TRACK_ALVAR)
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
