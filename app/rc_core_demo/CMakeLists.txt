add_executable(rc_core_demo main.cpp)
target_link_libraries(rc_core_demo PRIVATE rc::rt rc::bridge rc::telemetry)
