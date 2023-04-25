set(PLUGIN_arduino
    ON
    CACHE BOOL ON)
set(DEV_ARDUINO
    /dev/arduino
    CACHE STRING Arduino_Location)

add_subdirectory(interfaces)

if(PLUGIN_arduino)
  pkg_check_modules(LIBUDEV libudev)

  add_library(arduino MODULE arduino_plugin.cpp com_thread.cpp tf_thread.cpp
                             com_message.cpp)
  depend_on_pcl(arduino)
  depend_on_tf(arduino)
  target_compile_options(arduino PUBLIC ${LIBUDEV_CFLAGS})
  target_link_libraries(arduino ${LIBUDEV_LDFLAGS})
  target_link_libraries(
    arduino
    m
    fawkescore
    fawkesutils
    fawkesaspects
    fawkesbaseapp
    fawkesblackboard
    fawkesinterface
    ArduinoInterface
    JoystickInterface)

  add_custom_target(
    flash_arduino ALL echo "ARDUINO FLASHING IS DISABLED!")
    #/bin/sh ${PROJECT_SOURCE_DIR}/etc/scripts/flash_arduino.sh
    #${CMAKE_CURRENT_SOURCE_DIR} ${DEV_ARDUINO})
  add_dependencies(arduino flash_arduino)
else()
  plugin_disabled_message("arduino")
endif()
