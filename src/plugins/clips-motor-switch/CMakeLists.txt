set(PLUGIN_clips-motor-switch
    ON
    CACHE BOOL Build clips-motor-switch)

if(PLUGIN_clips-motor-switch)
  add_library(clips-motor-switch MODULE clips-motor-switch-plugin.cpp
                                        clips-motor-switch-thread.cpp)
  depend_on_clipsmm(clips-motor-switch)
  target_link_libraries(
    clips-motor-switch
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesclipsaspect
    fawkesblackboard
    fawkesinterface
    MotorInterface)
  depend_on_pkgconfig_libs(clips-motor-switch pddl_parser)
else()
  plugin_disabled_message("clips-motor-switch")
endif()
