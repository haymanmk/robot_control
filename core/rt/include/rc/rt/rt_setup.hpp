#pragma once

/// @file rt_setup.hpp
/// What a process must do before it may call itself real-time on Linux, and
/// honest reporting of which of those things it was actually allowed to do.
///
///   1. Shrink the footprint   -- see prepare_process(). Without this, locking
///                                memory means locking ~80 MiB for 1 MiB of work.
///   2. SCHED_FIFO             -- leave the fair scheduler, which is tuned for
///                                throughput and will delay you by milliseconds
///                                to be fair to a browser tab.
///   3. mlockall               -- a page fault in the cyclic path is a stall of
///                                several milliseconds. Lock everything resident.
///   4. prefault the stack     -- touch it now, so the first deep call in the
///                                loop is not the one that faults.
///   5. CPU affinity           -- pin to a core the kernel has been told to
///                                leave alone (isolcpus / nohz_full / rcu_nocbs).
///
/// Every one of these can be refused, usually by a resource limit. A control
/// process that silently runs without them is worse than one that never asked,
/// because you will believe its timing numbers. So this reports what it got,
/// and when it is refused it reports the numbers you need to fix it
/// (docs/rt-setup.md).
///
/// One refusal is deliberate rather than imposed. Once mlockall(MCL_FUTURE)
/// has succeeded, exceeding RLIMIT_MEMLOCK later -- by growing the stack or
/// the heap -- does not fail with an error. The page fault fails, and the
/// kernel delivers SIGSEGV. So apply_realtime() locks only when the limit
/// leaves room to grow (RtOptions::headroom_bytes), and otherwise explains
/// what to raise. A lock that succeeds and kills you a millisecond later is
/// worse than no lock.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rc::rt {

/// Resource limits and memory figures that decide whether mlockall() can
/// succeed, all in bytes. Read them before changing a limit, not after.
struct MemoryFigures {
  std::uint64_t memlock_soft = 0;   ///< RLIMIT_MEMLOCK soft limit (UINT64_MAX = unlimited)
  std::uint64_t memlock_hard = 0;   ///< RLIMIT_MEMLOCK hard limit; only root can raise it
  std::uint64_t rtprio_hard = 0;    ///< RLIMIT_RTPRIO hard limit: the highest SCHED_FIFO priority allowed
  std::uint64_t vm_size = 0;        ///< VmSize: everything mapped -- what mlockall(MCL_CURRENT) locks
  std::uint64_t vm_rss = 0;         ///< VmRSS: actually resident right now
  std::uint64_t vm_locked = 0;      ///< VmLck: currently locked
  bool has_cap_ipc_lock = false;    ///< root or CAP_IPC_LOCK: the memlock limit does not apply

  /// Read the current values. Does file I/O; never call from the cyclic path.
  [[nodiscard]] static MemoryFigures read();
  /// Three lines: limits, memory, rtprio -- indented to match RtStatus::format().
  [[nodiscard]] std::string format() const;
};

/// Process-wide tuning that must happen **before any thread is created**.
///
/// glibc gives every non-main thread an 8 MiB stack and, on first allocation,
/// a private 64 MiB malloc arena. With mlockall(MCL_FUTURE) both are faulted in
/// and locked in full: one background thread costs 72 MiB of locked memory.
/// This caps the arena count, stops malloc handing memory back to the kernel
/// (which would let later allocations fault again), and sets a small default
/// thread stack. Call prepare_process() first thing in main().
struct ProcessTuning {
  bool single_malloc_arena = true;        ///< mallopt(M_ARENA_MAX, 1)
  bool keep_freed_memory = true;          ///< M_TRIM_THRESHOLD=-1, M_MMAP_MAX=0
  std::size_t default_thread_stack = 1024 * 1024;  ///< bytes for every later thread; 0 = glibc default
};

/// Apply ProcessTuning. Returns one human-readable note per step taken.
std::vector<std::string> prepare_process(const ProcessTuning& tuning = {}) noexcept;

/// What to request from the kernel: scheduling class, memory locking, CPU
/// affinity and pre-faulting. Each can be refused; see RtStatus.
struct RtOptions {
  /// SCHED_FIFO priority, 1..99. 0 means "leave the scheduling class alone".
  /// Stay below the kernel's own threads (typically 50) unless you know why.
  int priority = 0;
  /// mlockall() the whole address space so the cyclic path never page-faults.
  bool lock_memory = true;
  /// If the soft RLIMIT_MEMLOCK is below the hard limit, raise it to the hard
  /// limit before locking. A process may do this for itself; only raising the
  /// hard limit needs privilege.
  bool raise_soft_memlock = true;
  /// Locked memory that must remain *unused* under RLIMIT_MEMLOCK after
  /// locking. With MCL_FUTURE, a later page fault that would exceed the limit
  /// is not an error return: the kernel delivers SIGSEGV. So if
  /// VmSize + prefault_bytes + headroom_bytes exceeds the limit, apply_realtime()
  /// refuses to lock and says why, rather than letting the process die at the
  /// next stack growth or allocation. Ignored when CAP_IPC_LOCK is held.
  std::size_t headroom_bytes = 4 * 1024 * 1024;
  /// CPU to pin to; -1 means no affinity change.
  int cpu = -1;
  /// Stack bytes to touch up front so the first write in the loop does not
  /// fault. Touched *before* locking, so they are counted and never grow the
  /// stack afterwards.
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
  /// Limits and memory figures as read after the attempt.
  MemoryFigures memory;
  /// Human-readable detail on every step, including why one was refused and
  /// the numbers needed to fix it.
  std::vector<std::string> notes;

  /// True only if every requested step succeeded.
  [[nodiscard]] bool fully_applied(const RtOptions& opts) const noexcept;
  /// The notes, one per line, indented to line up with CyclicReport::format().
  [[nodiscard]] std::string format() const;
};

/// Apply what is permitted; never throws, never exits. Read the result.
RtStatus apply_realtime(const RtOptions& opts) noexcept;

/// Grow and dirty the stack so later calls do not fault. Called by
/// apply_realtime(); exposed because every thread needs its own.
void prefault_stack(std::size_t bytes) noexcept;

}  // namespace rc::rt
