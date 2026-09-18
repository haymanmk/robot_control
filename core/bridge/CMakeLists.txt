add_library(robot_control_bridge
  src/shared_region.cpp
  src/server.cpp
  src/client.cpp
)
add_library(robot_control::bridge ALIAS robot_control_bridge)
target_include_directories(robot_control_bridge PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/core/telemetry/include   # record.hpp only: the wire format
)
target_link_libraries(robot_control_bridge PUBLIC robot_control::realtime)
# shm_open lives in librt on older glibc; harmless where it does not.
target_link_libraries(robot_control_bridge PUBLIC rt)
