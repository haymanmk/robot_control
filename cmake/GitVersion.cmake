# Provenance is only useful if it is accurate, so the SHA and the dirty flag are
# baked in at configure time rather than guessed at runtime.
find_package(Git QUIET)
set(ROBOT_CONTROL_GIT_SHA "unknown")
set(ROBOT_CONTROL_GIT_DIRTY "unknown")

if(GIT_FOUND AND EXISTS "${CMAKE_SOURCE_DIR}/.git")
  execute_process(
    COMMAND ${GIT_EXECUTABLE} rev-parse --short=12 HEAD
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE ROBOT_CONTROL_GIT_SHA
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  execute_process(
    COMMAND ${GIT_EXECUTABLE} status --porcelain --untracked-files=no
    WORKING_DIRECTORY ${CMAKE_SOURCE_DIR}
    OUTPUT_VARIABLE _robot_control_git_status
    OUTPUT_STRIP_TRAILING_WHITESPACE
    ERROR_QUIET)
  if(_robot_control_git_status STREQUAL "")
    set(ROBOT_CONTROL_GIT_DIRTY "clean")
  else()
    set(ROBOT_CONTROL_GIT_DIRTY "dirty")
  endif()
endif()

message(STATUS "Build provenance: ${ROBOT_CONTROL_GIT_SHA} (${ROBOT_CONTROL_GIT_DIRTY})")
