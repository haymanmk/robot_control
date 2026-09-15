add_library(rc_rt
  src/clock.cpp
  src/latency_histogram.cpp
  src/rt_setup.cpp
  src/cyclic_task.cpp
)
add_library(rc::rt ALIAS rc_rt)

target_include_directories(rc_rt PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_link_libraries(rc_rt PUBLIC pthread)
