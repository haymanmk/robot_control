add_library(robot_control_realtime
  src/clock.cpp
  src/latency_histogram.cpp
  src/realtime_setup.cpp
  src/cyclic_task.cpp
)
add_library(robot_control::realtime ALIAS robot_control_realtime)

target_include_directories(robot_control_realtime PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(robot_control_realtime PUBLIC pthread)
