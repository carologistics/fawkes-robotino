set(PLUGIN_skiller_motor_state
    ON
    CACHE BOOL ON)

if(PLUGIN_skiller_motor_state)
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
    fawkesinterface
    SkillerInterface
    RobotinoSensorInterface
    MotorInterface)

  target_compile_features(skiller_motor_state PUBLIC cxx_std_11)
  depend_on_cpp_version(skiller_motor_state 11)
else()
  plugin_disabled_message("skiller_motor_state")
endif()
