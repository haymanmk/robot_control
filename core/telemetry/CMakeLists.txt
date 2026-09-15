add_library(rc_telemetry
  src/provenance.cpp
  src/file_sink.cpp
)
add_library(rc::telemetry ALIAS rc_telemetry)
target_include_directories(rc_telemetry PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rc_telemetry PUBLIC rc::rt rc::bridge)
target_compile_definitions(rc_telemetry PRIVATE
  RC_GIT_SHA="${RC_GIT_SHA}"
  RC_GIT_DIRTY="${RC_GIT_DIRTY}"
  RC_BUILD_TYPE="${CMAKE_BUILD_TYPE}"
)
