set(PLUGIN_arduino
    ON
    CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_arduino)
  pkg_check_modules(LIBUDEV libudev)

  if(ROS_FOUND
     AND TF_FOUND
     AND PCL_FOUND
     AND LIBUDEV_FOUND)

    add_library(arduino MODULE arduino_plugin.cpp com_thread.cpp tf_thread.cpp
                               com_message.cpp)
    depend_on_ros(arduino)
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

    if(EXISTS /dev/arduino)
      add_custom_target(
        flash_arduino bash ${PROJECT_SOURCE_DIR}/etc/scripts/flash_arduino.bash
                      ${CMAKE_CURRENT_SOURCE_DIR})
    else()
      add_custom_target(flash_arduino echo "No arduino connected, can't flash")
    endif()
  else()
    if(NOT TF_FOUND)
      build_skipped_message("arduino" "TF dependencies")
    endif()
    if(NOT ROS_FOUND)
      build_skipped_message("arduino" "ROS dependencies")
    endif()
    if(NOT PCL_FOUND)
      build_skipped_message("arduino" "PCL dependencies")
    endif()
    if(NOT LIBUDEV_FOUND)
      build_skipped_message("arduino" "LIBUDEV dependencies")
    endif()
  endif()
else()
  plugin_disabled_message("arduino")
endif()
