set(PLUGIN_navgraph-generator-mps
    ON
    CACHE BOOL ON)
add_subdirectory(interfaces)
if(PLUGIN_navgraph-generator-mps)
  pkg_check_modules(pddl_parser pddl_parser)
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
  plugin_disabled_message("navgraph-generator-mps")
endif()
