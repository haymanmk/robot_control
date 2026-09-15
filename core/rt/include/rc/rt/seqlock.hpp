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
/// ## Why the data is stored as atomic words
///
/// The classic seqlock copies the payload with plain loads and stores while the
/// other side may be writing it. That is a data race in the C++ memory model:
/// the *protocol* discards the torn copy, but the *language* says the torn read
/// was undefined behaviour before it was ever compared. Two things follow from
/// taking that seriously rather than waving at Linux:
///
///   1. The payload is accessed word-by-word through `std::atomic_ref` with
///      relaxed ordering. Relaxed atomics compile to ordinary loads and stores
///      on x86-64 and AArch64, so this costs nothing; what it buys is that
///      every access is an atomic access and there is no data race left for
///      the compiler to exploit.
///   2. The *ordering* between the sequence counter and the payload still
///      comes from fences, in the construction from Boehm, "Can Seqlocks Get
///      Along with Programming Language Memory Models?" (MSPC 2012): a release
///      fence after the odd store, an acquire fence before the closing load.
///
/// ## What ThreadSanitizer can and cannot tell you here
///
/// GCC prints `atomic_thread_fence is not supported with -fsanitize=thread`
/// when compiling this file. It means what it says: **TSan does not model
/// fences.** An earlier version of this comment claimed the opposite, based on
/// TSan being silent on the fence-based seqlock; that silence was not evidence
/// of anything. With atomic_ref, TSan's silence *does* mean "no data race",
/// because there is none by construction -- but it still says nothing about
/// whether the fence ordering is right. That argument is Boehm's, and the
/// tearing oracle in `tests/test_rings.cpp` is the empirical check.
///
/// What must NOT happen is someone "fixing" any of this with a mutex. A mutex
/// here would let a reader that was killed mid-critical-section block the RT
/// thread forever -- the single failure this entire architecture exists to
/// prevent (ADR-0006).

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

#include "rc/rt/spsc_ring.hpp"  // kCacheLine

namespace rc::rt {

template <typename T>
class Seqlock {
  static_assert(std::is_trivially_copyable_v<T>,
                "shared-memory snapshots must be trivially copyable");
  static_assert(std::atomic_ref<std::uint64_t>::is_always_lock_free,
                "word-wise atomic access must be lock-free or this is not wait-free");

  using Word = std::uint64_t;
  static constexpr std::size_t kWords = (sizeof(T) + sizeof(Word) - 1) / sizeof(Word);

 public:
  /// Writer side. Wait-free: bounded steps, no blocking, safe in the cyclic path.
  void store(const T& value) noexcept {
    Word tmp[kWords] = {};
    std::memcpy(tmp, &value, sizeof(T));

    const std::uint64_t s = seq_.load(std::memory_order_relaxed);
    seq_.store(s + 1, std::memory_order_relaxed);        // odd: write in progress
    std::atomic_thread_fence(std::memory_order_release);  // odd is visible before payload
    for (std::size_t i = 0; i < kWords; ++i) {
      std::atomic_ref<Word>(words_[i]).store(tmp[i], std::memory_order_relaxed);
    }
    seq_.store(s + 2, std::memory_order_release);         // even: stable again
  }

  /// Reader side. Retries until it gets a torn-free copy.
  /// @param max_attempts bound on retries; 0 means retry forever. A reader that
  ///        cannot win in a few attempts is being starved by a writer running at
  ///        a much higher rate, and should be told rather than spin.
  [[nodiscard]] bool load(T& out, unsigned max_attempts = 16) const noexcept {
    Word tmp[kWords];
    for (unsigned attempt = 0; max_attempts == 0 || attempt < max_attempts; ++attempt) {
      const std::uint64_t before = seq_.load(std::memory_order_acquire);
      if (before & 1u) {
        continue;  // writer mid-update
      }
      for (std::size_t i = 0; i < kWords; ++i) {
        tmp[i] = std::atomic_ref<Word>(words_[i]).load(std::memory_order_relaxed);
      }
      std::atomic_thread_fence(std::memory_order_acquire);  // payload before the re-check
      if (seq_.load(std::memory_order_relaxed) == before) {
        std::memcpy(&out, tmp, sizeof(T));
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
    for (std::size_t i = 0; i < kWords; ++i) {
      words_[i] = 0;
    }
  }

 private:
  alignas(kCacheLine) std::atomic<std::uint64_t> seq_{0};
  // mutable so a const load() can form a non-const atomic_ref; the accesses are
  // reads, the qualifier is an artefact of atomic_ref's interface.
  alignas(kCacheLine) mutable Word words_[kWords]{};
};

}  // namespace rc::rt
