#pragma once

/// Minimal assertion harness. No gtest, deliberately: ADR-0003 says core/ must
/// build and test with no dependency beyond libc, and that constraint is what
/// keeps the RT boundary honest. This is ~40 lines and does the job.

#include <cstdio>
#include <cstdlib>
#include <string>

namespace rc::test {

inline int g_failures = 0;
inline int g_checks = 0;

inline void report(bool ok, const char* expression, const char* file, int line,
                   const std::string& detail = {}) {
  ++g_checks;
  if (!ok) {
    ++g_failures;
    std::fprintf(stderr, "FAIL %s:%d  %s%s%s\n", file, line, expression,
                 detail.empty() ? "" : "  -- ", detail.c_str());
  }
}

inline int finish(const char* suite) {
  if (g_failures == 0) {
    std::printf("PASS %s (%d checks)\n", suite, g_checks);
    return 0;
  }
  std::fprintf(stderr, "FAILED %s: %d of %d checks failed\n", suite, g_failures, g_checks);
  return 1;
}

}  // namespace rc::test

#define CHECK(expression) ::rc::test::report((expression), #expression, __FILE__, __LINE__)
#define CHECK_EQ(left, right)                                                     \
  ::rc::test::report((left) == (right), #left " == " #right, __FILE__, __LINE__,  \
                     std::to_string(left) + " vs " + std::to_string(right))
#define CHECK_MSG(expression, message) ::rc::test::report((expression), #expression, __FILE__, __LINE__, (message))
