# Optional API documentation. Doxygen is never required to build, test or
# measure: if it is absent this file does nothing except say so.
#
#   cmake --build build --target docs        -> build/docs/html/index.html
#   cmake -B build -DRC_DOCS_WARN_AS_ERROR=ON  -> an undocumented public
#                                                 symbol fails the target (CI)
find_package(Doxygen QUIET OPTIONAL_COMPONENTS dot)
find_package(Python3 QUIET COMPONENTS Interpreter)

option(RC_DOCS_WARN_AS_ERROR "Fail the docs target on any Doxygen warning" OFF)

if(NOT DOXYGEN_FOUND OR NOT Python3_Interpreter_FOUND)
  message(STATUS "Doxygen or Python 3 not found; the 'docs' target is unavailable")
  return()
endif()

set(DOXYGEN_OUTPUT_DIR "${CMAKE_BINARY_DIR}/docs")
if(RC_DOCS_WARN_AS_ERROR)
  set(DOXYGEN_WARN_AS_ERROR YES)
else()
  set(DOXYGEN_WARN_AS_ERROR NO)
endif()

if(TARGET Doxygen::dot)
  set(DOXYGEN_HAVE_DOT YES)
else()
  set(DOXYGEN_HAVE_DOT NO)
endif()

configure_file("${CMAKE_SOURCE_DIR}/docs/Doxyfile.in"
               "${CMAKE_BINARY_DIR}/Doxyfile" @ONLY)

add_custom_target(docs
  COMMAND ${DOXYGEN_EXECUTABLE} "${CMAKE_BINARY_DIR}/Doxyfile"
  WORKING_DIRECTORY "${CMAKE_SOURCE_DIR}"
  COMMENT "Generating API documentation with Doxygen -> ${DOXYGEN_OUTPUT_DIR}/html"
  VERBATIM)

message(STATUS "Doxygen ${DOXYGEN_VERSION} found; 'docs' target enabled")
