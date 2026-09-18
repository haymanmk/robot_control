add_library(robot_control_telemetry
  src/provenance.cpp
  src/file_sink.cpp
)
add_library(robot_control::telemetry ALIAS robot_control_telemetry)
target_include_directories(robot_control_telemetry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robot_control_telemetry PUBLIC robot_control::realtime robot_control::bridge)
target_compile_definitions(robot_control_telemetry PRIVATE
  ROBOT_CONTROL_GIT_SHA="${ROBOT_CONTROL_GIT_SHA}"
  ROBOT_CONTROL_GIT_DIRTY="${ROBOT_CONTROL_GIT_DIRTY}"
  ROBOT_CONTROL_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
