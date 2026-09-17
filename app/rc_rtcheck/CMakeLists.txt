add_executable(rc_rtcheck main.cpp)
target_link_libraries(rc_rtcheck PRIVATE rc::rt rc::telemetry)
