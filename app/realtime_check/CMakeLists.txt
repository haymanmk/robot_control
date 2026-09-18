add_executable(realtime_check main.cpp)
target_link_libraries(realtime_check PRIVATE robot_control::realtime robot_control::telemetry)
