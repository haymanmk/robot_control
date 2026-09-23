#pragma once

/// @file cyclic_guard.hpp
/// Mechanical enforcement of two cyclic-path rules that were, until now,
/// enforced only by code review: "no syscalls except the fieldbus and the
/// clock" and "no page faults".
///
/// The idea is borrowed from the dual-kernel world. EVL's `EVL_T_WOSS` reports
/// every demotion of an out-of-band thread with a diagnostic naming the cause
/// (`EVL_HMDIAG_SYSDEMOTE` for a stray in-band syscall, `EVL_HMDIAG_EXDEMOTE`
/// for a fault). There is no stage to switch on PREEMPT_RT, but the rule is the
/// same and the kernel offers the same two hooks (ADR-0008):
///
///   * **A seccomp-BPF filter on the cyclic thread.** Only the syscalls the
///     loop is meant to make are allowed; anything else is refused and reported
///     with its syscall number, and the offending call is *not performed* -- it
///     returns -ENOSYS to whoever made it. That is the right failure mode: the
///     forbidden thing does not happen, the loop keeps its deadline, and the
///     report names the culprit.
///   * **Page-fault counters** from `getrusage(RUSAGE_THREAD)` around the run.
///     After `mlockall` and a stack prefault, a correctly written loop faults
///     zero times. Any fault is a defect.
///
/// One property of seccomp shapes the API: **a filter cannot be removed** from
/// a thread. So a guarded run must be the last thing its thread does before
/// exiting, and CyclicTask::run_in_thread() exists for exactly that. Arming
/// the guard on a thread that will go on to print reports or close files means
/// those calls are refused too.
///
/// x86-64 only for the "set the return value" part of the trap handler; on
/// other architectures the violation is still counted and the call still
/// refused, but the caller sees whatever the kernel left in the result register.

#include <sys/types.h>

#include <cstdint>
#include <string>
#include <vector>

namespace robot_control::realtime {

/// What the guard permits and how it reacts.
struct GuardOptions {
  /// Install the syscall filter. Requires seccomp filter mode in the kernel.
  bool filter_syscalls = true;
  /// Count minor and major page faults over the run.
  bool count_page_faults = true;
  /// Kill the thread with SIGSYS on the first refused syscall instead of
  /// counting it and continuing. For tests that must prove a violation is
  /// fatal, and for a production loop where "continue with the call refused"
  /// is not an acceptable state.
  bool kill_on_violation = false;
  /// Syscall numbers to allow beyond the built-in set (see cyclic_guard.cpp
  /// for that set: the clock, the sleep, the socket send and receive calls,
  /// and the calls a thread needs to report and exit).
  std::vector<int> extra_allowed_syscalls;
};

/// What the guard observed. Counters are per thread; a guard on one thread
/// says nothing about another.
struct GuardReport {
  /// The filter is installed on the thread that ran the loop.
  bool filter_installed = false;
  /// getrusage(RUSAGE_THREAD) was read before and after.
  bool faults_counted = false;
  /// Page faults served from memory already resident (no disk I/O).
  std::uint64_t minor_faults = 0;
  /// Page faults that needed I/O. Any of these is a multi-millisecond stall.
  std::uint64_t major_faults = 0;
  /// Refused syscalls. Zero is the only acceptable value.
  std::uint64_t syscall_violations = 0;
  /// Syscall number of the first refused call, or -1.
  int first_violation_syscall = -1;
  /// Why a step could not be applied, if it could not.
  std::vector<std::string> notes;

  /// True only if nothing was observed that the rules forbid.
  [[nodiscard]] bool clean() const noexcept {
    return syscall_violations == 0 && minor_faults == 0 && major_faults == 0;
  }
  /// Lines indented to match RealtimeStatus::format().
  [[nodiscard]] std::string format() const;
};

/// Arm on the calling thread. Non-cyclic: allocates, installs a signal handler,
/// and makes several syscalls. Call after apply_realtime() and immediately
/// before the loop.
[[nodiscard]] GuardReport arm_cyclic_guard(const GuardOptions& options);

/// Read the counters after the loop into @p report. Cyclic-safe in the sense
/// that it makes only an allowed syscall (getrusage); intended to be called
/// once, right after the last cycle.
void read_cyclic_guard(GuardReport& report) noexcept;

/// Is seccomp filter mode available on this kernel? Lets a test skip honestly
/// instead of failing or passing for the wrong reason.
[[nodiscard]] bool cyclic_guard_supported() noexcept;

/// Best-effort name for a syscall number, for reports. "nr 231" if unknown.
[[nodiscard]] std::string syscall_name(int number);

}  // namespace robot_control::realtime
