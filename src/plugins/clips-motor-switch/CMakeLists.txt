set(PLUGIN_clips-motor-switch
    ON
    CACHE BOOL ON)

if(PLUGIN_clips-motor-switch)
  pkg_check_modules(pddl_parser pddl_parser)
  if(pddl_parser_FOUND)
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
  endif()
else()
  plugin_disabled_message("clips-motor-switch")
endif()
