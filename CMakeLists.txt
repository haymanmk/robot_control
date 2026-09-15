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

add_subdirectory(core/rt)
add_subdirectory(labs/01_loop_timing/cpp)
