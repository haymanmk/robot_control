add_executable(bench_can_capture can_capture.cpp)
target_link_libraries(bench_can_capture PRIVATE robot_control::can robot_control::telemetry)

# The analysis script checks itself: every candump notation it parses and the
# bit arithmetic, which must agree with robot_control::can::bits_on_wire.
find_package(Python3 QUIET COMPONENTS Interpreter)
if(Python3_Interpreter_FOUND)
  add_test(NAME can_bus_load_selftest
           COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/can_bus_load.py selftest)
endif()
