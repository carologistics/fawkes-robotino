set(PLUGIN_clips-simulation-result
    ON
    CACHE BOOL "Build clips-simulation-result plugin")

if(PLUGIN_clips-simulation-result)
  add_library(clips-simulation-result MODULE clips-simulation-result-plugin.cpp
                                             clips-simulation-result-thread.cpp)
  target_compile_features(clips-simulation-result PUBLIC cxx_std_11)
  depend_on_clipsmm(clips-simulation-result)
  target_link_libraries(clips-simulation-result fawkescore fawkesutils fawkesaspects
                        fawkesclipsaspect)

else()
  plugin_disabled_message("clips-simulation-result")
endif()