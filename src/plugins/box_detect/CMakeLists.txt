set(PLUGIN_box_detect
	ON
	CACHE BOOL ON)

add_subdirectory(interfaces)

if(PLUGIN_box_detect)
	add_library(box_detect MODULE box_detect_plugin.cpp 
								  box_detect_thread.cpp)
	depend_on_tf(box_detect)
	depend_on_ros2(box_detect)
	depend_on_ros2_libs(box_detect "std_srvs") 
	target_link_libraries(
		box_detect
		fawkescore
		fawkesutils
		fawkesaspects
		fawkestf
		fawkesinterface
		fawkesblackboard
		fawkesconfig
		BoxInterface)
else()
	plugin_disabled_message("box_detect")
endif()
