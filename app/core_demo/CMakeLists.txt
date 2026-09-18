add_executable(core_demo main.cpp)
target_link_libraries(core_demo PRIVATE robot_control::realtime robot_control::bridge robot_control::telemetry)
