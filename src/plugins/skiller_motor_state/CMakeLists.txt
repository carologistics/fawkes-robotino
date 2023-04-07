set(PLUGIN_skiller_motor_state
    ON
    CACHE BOOL ON)

if(PLUGIN_skiller_motor_state)
  if(CPP_11_FOUND)
    add_library(skiller_motor_state MODULE skiller_motor_state_plugin.cpp
                                           skiller_motor_state_thread.cpp)
    target_link_libraries(
      skiller_motor_state
      m
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesbaseapp
      fawkesblackboard
      fawkesinterface)

    target_compile_features(skiller_motor_state PUBLIC cxx_std_11)
  else()
    message("Omitting skiller_motor_state (C++11 required")
  endif()
else()
  plugin_disabled_message("skiller_motor_state")
endif()
