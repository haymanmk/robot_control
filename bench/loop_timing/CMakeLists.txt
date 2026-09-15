add_executable(bench_loop_timing loop_timing.cpp)
target_link_libraries(bench_loop_timing PRIVATE rc::rt)
