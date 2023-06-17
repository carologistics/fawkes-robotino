create_symlink_custom_target(
  fawkes_robotino_symlink_clips-executive
  ${FAWKES_CORE_DIR}/src/plugins/clips-executive/clips
  ${CMAKE_CURRENT_SOURCE_DIR}/clips-executive)
create_symlink_custom_target(
  fawkes_robotino_symlink_clips ${FAWKES_CORE_DIR}/src/plugins/clips/clips
  ${CMAKE_CURRENT_SOURCE_DIR}/clips)
create_symlink_custom_target(
  fawkes_robotino_symlink_clips-navgraph
  ${FAWKES_CORE_DIR}/src/plugins/clips-navgraph/clips
  ${CMAKE_CURRENT_SOURCE_DIR}/clips-navgraph)
create_symlink_custom_target(
  fawkes_robotino_symlink_clips-protobuf
  ${FAWKES_CORE_DIR}/src/plugins/clips-protobuf/clips
  ${CMAKE_CURRENT_SOURCE_DIR}/clips-protobuf)
