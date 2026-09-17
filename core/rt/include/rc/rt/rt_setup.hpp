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
/// and when it is refused it reports the numbers you need to fix it.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rc::rt {

/// Resource limits and memory figures relevant to locking, all in bytes.
struct MemoryFigures {
  std::uint64_t memlock_soft = 0;   ///< RLIMIT_MEMLOCK soft limit (UINT64_MAX = unlimited)
  std::uint64_t memlock_hard = 0;
  std::uint64_t rtprio_hard = 0;    ///< RLIMIT_RTPRIO hard limit (max SCHED_FIFO priority)
  std::uint64_t vm_size = 0;        ///< VmSize: everything mapped -- what mlockall(MCL_CURRENT) locks
  std::uint64_t vm_rss = 0;         ///< VmRSS: actually resident right now
  std::uint64_t vm_locked = 0;      ///< VmLck: currently locked
  bool has_cap_ipc_lock = false;    ///< root or CAP_IPC_LOCK: the memlock limit does not apply

  /// Read the current values. Does file I/O; never call from the cyclic path.
  [[nodiscard]] static MemoryFigures read();
  [[nodiscard]] std::string format() const;
};

/// Process-wide tuning that must happen **before any thread is created**.
///
/// glibc gives every non-main thread an 8 MiB stack and, on first allocation,
/// a private 64 MiB malloc arena. With mlockall(MCL_FUTURE) both are faulted in
/// and locked in full: one background thread costs 72 MiB of locked memory.
/// This caps the arena count, stops malloc handing memory back to the kernel
/// (which would let later allocations fault again), and sets a small default
/// thread stack. Call it first thing in main().
struct ProcessTuning {
  bool single_malloc_arena = true;
  bool keep_freed_memory = true;          ///< M_TRIM_THRESHOLD=-1, M_MMAP_MAX=0
  std::size_t default_thread_stack = 1024 * 1024;  ///< 0 = leave glibc's default
};
std::vector<std::string> prepare_process(const ProcessTuning& tuning = {}) noexcept;

struct RtOptions {
  /// SCHED_FIFO priority, 1..99. 0 means "leave the scheduling class alone".
  /// Stay below the kernel's own threads (typically 50) unless you know why.
  int priority = 0;
  bool lock_memory = true;
  /// If the soft RLIMIT_MEMLOCK is below the hard limit, raise it to the hard
  /// limit before locking. A process may do this for itself; only raising the
  /// hard limit needs privilege.
  bool raise_soft_memlock = true;
  /// CPU to pin to; -1 means no affinity change.
  int cpu = -1;
  std::size_t prefault_bytes = 512 * 1024;
};

struct RtStatus {
  bool scheduler_applied = false;
  bool memory_locked = false;
  bool affinity_applied = false;
  MemoryFigures memory;                 ///< as read after the attempt
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
