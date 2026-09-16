#pragma once

/// @file rt_setup.hpp
/// The four things a process must do before it is allowed to call itself
/// real-time on Linux, and honest reporting of which ones it was permitted.
///
///   1. SCHED_FIFO  -- leave the fair scheduler, which is tuned for throughput
///                     and will happily delay you a few milliseconds to be
///                     fair to a browser tab.
///   2. mlockall    -- a page fault in the cyclic path is a multi-millisecond
///                     stall. Lock the whole address space resident up front.
///   3. prefault    -- touch the stack and heap now, so the first write in the
///                     loop is not the one that triggers the fault.
///   4. affinity    -- pin to a CPU, ideally one the kernel has been told to
///                     leave alone (isolcpus / nohz_full / rcu_nocbs).
///
/// Every one of these can be refused. A control process that silently runs
/// without them is worse than one that never asked, because you will believe
/// its timing numbers. So this reports what it got.

#include <cstddef>
#include <string>
#include <vector>

namespace rc::rt {

/// What to request from the kernel: scheduling class, memory locking, CPU
/// affinity and pre-faulting. Each can be refused; see RtStatus.
struct RtOptions {
  /// SCHED_FIFO priority, 1..99. 0 means "leave the scheduling class alone".
  /// Stay below the kernel's own threads (typically 50) unless you know why.
  int priority = 0;
  /// mlockall() the whole address space so the cyclic path never page-faults.
  bool lock_memory = true;
  /// CPU to pin to; -1 means no affinity change.
  int cpu = -1;
  /// Stack bytes to touch up front so the first write in the loop does not fault.
  std::size_t prefault_bytes = 512 * 1024;
};

/// Which of the requested real-time steps were actually granted. A process
/// that silently runs without them produces timing numbers you will wrongly
/// believe, so always read this.
struct RtStatus {
  /// SCHED_FIFO was applied at the requested priority.
  bool scheduler_applied = false;
  /// mlockall() succeeded.
  bool memory_locked = false;
  /// The thread is pinned to the requested CPU.
  bool affinity_applied = false;
  /// Human-readable detail on every step, including why one was refused.
  std::vector<std::string> notes;

  /// True only if every requested step succeeded.
  [[nodiscard]] bool fully_applied(const RtOptions& opts) const noexcept;
  [[nodiscard]] std::string format() const;
};

/// Apply what is permitted; never throws, never exits. Read the result.
RtStatus apply_realtime(const RtOptions& opts) noexcept;

/// Grow and dirty the stack so later calls do not fault. Called by
/// apply_realtime(); exposed because every thread needs its own.
void prefault_stack(std::size_t bytes) noexcept;

}  // namespace rc::rt
