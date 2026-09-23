add_executable(test_rings test_rings.cpp)
target_include_directories(test_rings PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_rings PRIVATE robot_control::realtime)
add_test(NAME rings COMMAND test_rings)

add_executable(test_bridge test_bridge.cpp)
target_include_directories(test_bridge PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_bridge PRIVATE robot_control::bridge robot_control::telemetry)
add_test(NAME bridge COMMAND test_bridge)
set_tests_properties(bridge PROPERTIES TIMEOUT 120)

add_executable(test_can test_can.cpp)
target_include_directories(test_can PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_can PRIVATE robot_control::can)
add_test(NAME can COMMAND test_can)

# Needs vcan0 (see the file comment). Exit code 77 tells CTest "skipped", which
# it reports separately from passed; a machine without CAN must not look green.
add_executable(test_can_loopback test_can_loopback.cpp)
target_include_directories(test_can_loopback PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_can_loopback PRIVATE robot_control::can)
add_test(NAME can_loopback COMMAND test_can_loopback)
set_tests_properties(can_loopback PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)

# Needs seccomp filter mode; skips (exit 77) where the kernel lacks it.
add_executable(test_cyclic_guard test_cyclic_guard.cpp)
target_include_directories(test_cyclic_guard PRIVATE ${CMAKE_CURRENT_SOURCE_DIR})
target_link_libraries(test_cyclic_guard PRIVATE robot_control::realtime)
add_test(NAME cyclic_guard COMMAND test_cyclic_guard)
set_tests_properties(cyclic_guard PROPERTIES SKIP_RETURN_CODE 77 TIMEOUT 60)
