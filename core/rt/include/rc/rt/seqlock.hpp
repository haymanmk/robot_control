#pragma once

/// @file seqlock.hpp
/// Single-writer / many-reader snapshot of one value. Wait-free for the writer.
///
/// The queue in spsc_ring.hpp answers "give me every record in order". This
/// answers a different question: **"what is the arm doing right now?"** A live
/// plot, a UI, or a `ros2` state publisher wants the newest value and does not
/// care what it missed. Handing that job to a queue means a slow reader either
/// stalls the writer or accumulates a backlog it then has to skip -- both worse
/// than just reading the latest.
///
/// ## How it works
///
/// A sequence counter brackets every write. Odd means "a write is in progress";
/// even means "stable". A reader takes the counter, copies the data, takes the
/// counter again, and retries if either it was odd or it changed. The writer
/// never waits for readers, and readers never block the writer -- which is
/// exactly the asymmetry the RT thread needs.
///
/// ## The caveat, and what was actually measured
///
/// The reader may copy bytes while the writer is modifying them. Under a strict
/// reading of the C++ memory model that is a data race, because `storage_` is
/// touched non-atomically on both sides. The sequence check means such a torn
/// copy is always *detected and discarded*, never returned, which is why this
/// is the shape Linux's own seqlock has had for twenty years.
///
/// The usual next sentence is "and ThreadSanitizer will complain about it."
/// On this toolchain it does not: `tests/test_rings.cpp` runs a 300k-iteration
/// reader against a saturating writer under `-fsanitize=thread` and TSan reports
/// nothing (verified against a positive control, so the silence is meaningful).
/// TSan models `atomic_thread_fence`, and the fences below are what make the
/// pattern well-formed enough for it. Do not read that as a proof of
/// portability: a different compiler or architecture may judge it differently,
/// and the strictly-conforming alternative is word-wise `std::atomic_ref` with
/// relaxed ordering, which costs nothing on x86-64 and ARM.
///
/// What must NOT happen is someone "fixing" this with a mutex. A mutex here
/// would let a reader that was killed mid-critical-section block the RT thread
/// forever -- the single failure this entire architecture exists to prevent
/// (ADR-0006).

#include <atomic>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "rc/rt/spsc_ring.hpp"  // kCacheLine

namespace rc::rt {

template <typename T>
class Seqlock {
  static_assert(std::is_trivially_copyable_v<T>,
                "shared-memory snapshots must be trivially copyable");

 public:
  /// Writer side. Wait-free: bounded steps, no blocking, safe in the cyclic path.
  void store(const T& value) noexcept {
    const std::uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);        // odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);  // odd is visible first
    std::memcpy(&storage_, &value, sizeof(T));
    seq_.store(s + 2, std::memory_order_release);         // even: stable again
  }

  /// Reader side. Retries until it gets a torn-free copy.
  /// @param max_attempts bound on retries; 0 means retry forever. A reader that
  ///        cannot win in a few attempts is being starved by a writer running at
  ///        a much higher rate, and should be told rather than spin.
  [[nodiscard]] bool load(T& out, unsigned max_attempts = 16) const noexcept {
    for (unsigned attempt = 0; max_attempts == 0 || attempt < max_attempts; ++attempt) {
      const std::uint64_t before = seq_.load(std::memory_order_acquire);
      if (before & 1u) {
        continue;  // writer mid-update
      }
      std::memcpy(&out, &storage_, sizeof(T));
      std::atomic_thread_fence(std::memory_order_acquire);
      if (seq_.load(std::memory_order_relaxed) == before) {
        return true;
      }
      // Sequence moved: the copy may be torn. Discard it and try again.
    }
    return false;
  }

  /// Number of completed writes. Lets a reader tell "nothing new" from "stale".
  [[nodiscard]] std::uint64_t generation() const noexcept {
    return seq_.load(std::memory_order_acquire) / 2;
  }

  /// Only safe before either side is running.
  void reset() noexcept {
    seq_.store(0, std::memory_order_relaxed);
    std::memset(&storage_, 0, sizeof(T));
  }

 private:
  alignas(kCacheLine) std::atomic<std::uint64_t> seq_{0};
  alignas(kCacheLine) T storage_{};
};

}  // namespace rc::rt
