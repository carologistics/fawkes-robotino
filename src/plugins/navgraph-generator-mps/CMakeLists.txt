set(PLUGIN_navgraph-generator-mps
    ON
    CACHE BOOL ON)
add_subdirectory(interfaces)
if(PLUGIN_navgraph-generator-mps)
  pkg_check_modules(pddl_parser pddl_parser)
  if(EIGEN3_FOUND
     AND TF_FOUND
     AND NAVGRAPH_FOUND
     AND CGAL_LIBS_FOUND)
    add_library(navgraph-generator-mps MODULE navgraph_generator_mps_plugin.cpp
                                              navgraph_generator_mps_thread.cpp)
    depend_on_eigen3(navgraph-generator-mps)
    depend_on_cgal_libs(navgraph-generator-mps)
    depend_on_tf(navgraph-generator-mps)
    depend_on_navgraph(navgraph-generator-mps)
    depend_on_clipsmm(navgraph-generator-mps)
    target_link_libraries(
      navgraph-generator-mps
      fawkescore
      fawkesutils
      fawkesaspects
      fawkesinterface
      fawkesblackboard
      fawkestf
      fawkesnavgraph
      Position3DInterface
      NavGraphGeneratorInterface
      NavGraphWithMPSGeneratorInterface)

  else()
    if(NOT EIGEN3_FOUND)
      build_skipped_message("navgraph-generator-mps" "eigen3 dependencies")
    endif()
    if(NOT TF_FOUND)
      build_skipped_message("navgraph-generator-mps" "TF dependencies")
    endif()
    if(NOT NAVGRAPH_FOUND)
      build_skipped_message("navgraph-generator-mps" "navgraph dependencies")
    endif()
    if(NOT CGAL_LIBS_FOUND)
      build_skipped_message("navgraph-generator-mps" "CGAL dependencies")
    endif()
  endif()
else()
  plugin_disabled_message("navgraph-generator-mps")
endif()
