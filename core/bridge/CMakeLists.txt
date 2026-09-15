add_library(rc_bridge
  src/shared_region.cpp
  src/server.cpp
  src/client.cpp
)
add_library(rc::bridge ALIAS rc_bridge)
target_include_directories(rc_bridge PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${CMAKE_SOURCE_DIR}/core/telemetry/include   # record.hpp only: the wire format
)
target_link_libraries(rc_bridge PUBLIC rc::rt)
# shm_open lives in librt on older glibc; harmless where it does not.
target_link_libraries(rc_bridge PUBLIC rt)
