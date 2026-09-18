cmake_minimum_required(VERSION 3.20)
project(robot_control LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  # RelWithDebInfo, not Release: timing numbers from a -O0 build are fiction,
  # and a control system you cannot get a stack trace out of is not one you
  # want to debug with a 2.5 kg arm swinging.
  set(CMAKE_BUILD_TYPE RelWithDebInfo CACHE STRING "" FORCE)
endif()

add_compile_options(-Wall -Wextra -Wpedantic -Wshadow -Wconversion)

# Opt-in sanitizers. ThreadSanitizer is the one that matters here: the bridge is
# lock-free, and wrong memory ordering fails rarely, unreproducibly, and never
# under a debugger.  cmake -B build-tsan -DRC_SANITIZE=thread
set(ROBOT_CONTROL_SANITIZE "" CACHE STRING "Sanitizer to enable: thread, address, undefined")
if(ROBOT_CONTROL_SANITIZE)
  add_compile_options(-fsanitize=${ROBOT_CONTROL_SANITIZE} -fno-omit-frame-pointer -g)
  add_link_options(-fsanitize=${ROBOT_CONTROL_SANITIZE})
endif()

include(cmake/GitVersion.cmake)
include(cmake/Docs.cmake)
enable_testing()

add_subdirectory(core/realtime)
add_subdirectory(core/bridge)
add_subdirectory(core/telemetry)
add_subdirectory(bench/loop_timing)
add_subdirectory(app/core_demo)
add_subdirectory(app/realtime_check)
add_subdirectory(tests)
