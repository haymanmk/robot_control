add_library(robot_control_can
  src/socketcan.cpp
  src/statistics.cpp
)
add_library(robot_control::can ALIAS robot_control_can)
target_include_directories(robot_control_can PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
# Layers depend downward only (AGENTS.md): can may use realtime and telemetry,
# nothing above. Today it needs only realtime; the telemetry record's
# can_*_nanoseconds fields are filled by the drive layer, one level up.
target_link_libraries(robot_control_can PUBLIC robot_control::realtime)
