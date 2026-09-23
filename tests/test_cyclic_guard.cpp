/// Tests for the cyclic guard (ADR-0008).
///
/// The guard must (1) stay silent on a loop that obeys the rules, (2) count
/// and refuse a stray syscall, naming it, (3) count page faults, and (4) in
/// kill mode, end the thread with SIGSYS. Each violating case runs on its own
/// thread or in a forked child, because a seccomp filter cannot be removed.
///
/// Skips with exit code 77 (CTest "skipped") where seccomp filter mode is not
/// available, so a kernel without it does not look green.

#include <errno.h>
#include <signal.h>
#include <sys/mman.h>
#include <sys/syscall.h>
#include <sys/wait.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <string>

#include "robot_control/realtime/cyclic_guard.hpp"
#include "robot_control/realtime/cyclic_task.hpp"
#include "test_support.hpp"

using robot_control::realtime::CyclicConfig;
using robot_control::realtime::CyclicReport;
using robot_control::realtime::CyclicTask;
using robot_control::realtime::nanoseconds;

namespace {

constexpr int skip_exit_code = 77;

CyclicConfig guarded_config() {
  CyclicConfig config;
  config.period = nanoseconds{1'000'000};  // 1 kHz keeps the tests quick
  config.realtime.priority = 0;            // unprivileged test
  config.realtime.lock_memory = false;
  config.realtime.prefault_bytes = 64 * 1024;
  config.guard = true;
  return config;
}

/// A loop that obeys every rule must produce a clean report. This is the case
/// that matters most: a guard that cries wolf would be switched off.
void test_clean_loop_is_clean() {
  CyclicTask task(guarded_config());
  std::atomic<std::uint64_t> sum{0};
  const CyclicReport report = task.run_in_thread(200, [&](std::uint64_t cycle, nanoseconds) {
    sum.fetch_add(cycle, std::memory_order_relaxed);  // memory only
  });
  CHECK_EQ(report.cycles, 200u);
  CHECK_MSG(report.guard.filter_installed, "filter should install on the loop thread");
  CHECK_MSG(report.guard.syscall_violations == 0,
            "clean loop reported violations: " + report.guard.format());
  CHECK_MSG(report.guard.major_faults == 0, "clean loop took major faults");
  // Minor faults: the first cycles may touch histogram buckets for the first
  // time. Allow a handful; a real loop after mlockall+prefault expects zero.
  CHECK_MSG(report.guard.minor_faults < 16,
            "clean loop took " + std::to_string(report.guard.minor_faults) + " minor faults");
}

/// A write() from the cycle body is the canonical violation: a stray printf.
/// It must be counted, named, and refused -- the write must not happen.
void test_stray_write_is_refused_and_named() {
  CyclicTask task(guarded_config());
  std::atomic<long> write_result{12345};
  std::atomic<int> write_errno{0};
  const CyclicReport report = task.run_in_thread(50, [&](std::uint64_t cycle, nanoseconds) {
    if (cycle == 10) {
      const long result = ::write(STDOUT_FILENO, "this must not appear\n", 21);
      write_result.store(result, std::memory_order_relaxed);
      write_errno.store(errno, std::memory_order_relaxed);
    }
  });
  CHECK_EQ(report.guard.syscall_violations, 1u);
  CHECK_EQ(report.guard.first_violation_syscall, static_cast<int>(SYS_write));
  CHECK_MSG(write_result.load() == -1, "the refused write must return -1, got " +
                                            std::to_string(write_result.load()));
  CHECK_EQ(write_errno.load(), ENOSYS);
  CHECK_MSG(report.cycles == 50, "the loop must continue after a refused call");
}

/// Touching memory that was mapped but never written faults once per page.
/// The guard must see every one of them.
void test_page_faults_are_counted() {
  constexpr std::size_t pages = 32;
  void* region = ::mmap(nullptr, pages * 4096, PROT_READ | PROT_WRITE,
                        MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
  CHECK_MSG(region != MAP_FAILED, "mmap failed");
  auto* bytes = static_cast<volatile unsigned char*>(region);

  CyclicTask task(guarded_config());
  const CyclicReport report = task.run_in_thread(pages + 5, [&](std::uint64_t cycle, nanoseconds) {
    if (cycle < pages) {
      bytes[cycle * 4096] = 1;  // first touch of a fresh page: one minor fault
    }
  });
  ::munmap(region, pages * 4096);
  CHECK_MSG(report.guard.minor_faults >= pages,
            "expected at least " + std::to_string(pages) + " minor faults, saw " +
                std::to_string(report.guard.minor_faults));
  CHECK_MSG(report.guard.syscall_violations == 0, "touching memory is not a syscall");
}

/// In kill mode the first violation ends the thread with SIGSYS. Proved in a
/// child process so the test binary survives to report it.
void test_kill_mode_is_fatal() {
  const pid_t child = ::fork();
  if (child == 0) {
    CyclicConfig config = guarded_config();
    config.guard_options.kill_on_violation = true;
    CyclicTask task(config);
    (void)task.run_in_thread(20, [](std::uint64_t cycle, nanoseconds) {
      if (cycle == 3) {
        (void)!::write(STDOUT_FILENO, "x", 1);
      }
    });
    ::_exit(0);  // reached only if the kill did not happen
  }
  int status = 0;
  ::waitpid(child, &status, 0);
  CHECK_MSG(WIFSIGNALED(status), "child should have died from a signal, status=" +
                                     std::to_string(status));
  if (WIFSIGNALED(status)) {
    CHECK_EQ(WTERMSIG(status), SIGSYS);
  }
}

/// The filter applies to the loop thread only: the caller must still be able
/// to do ordinary things afterwards, which is the point of run_in_thread().
void test_caller_thread_is_not_filtered() {
  CyclicTask task(guarded_config());
  (void)task.run_in_thread(5, [](std::uint64_t, nanoseconds) {});
  const long result = ::write(STDOUT_FILENO, "", 0);
  CHECK_MSG(result == 0, "caller's write should succeed after a guarded run");
}

}  // namespace

int main() {
  if (!robot_control::realtime::cyclic_guard_supported()) {
    std::printf("SKIP cyclic_guard: seccomp filter mode is not available on this kernel\n");
    return skip_exit_code;
  }
  test_clean_loop_is_clean();
  test_stray_write_is_refused_and_named();
  test_page_faults_are_counted();
  test_kill_mode_is_fatal();
  test_caller_thread_is_not_filtered();
  return robot_control::test::finish("cyclic_guard");
}
