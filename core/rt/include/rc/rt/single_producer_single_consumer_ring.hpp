#pragma once

/// @file single_producer_single_consumer_ring.hpp
/// Single-producer / single-consumer bounded queue. Lock-free, wait-free,
/// allocation-free, and usable across processes in shared memory.
///
/// This is the primitive the whole RT/non-RT boundary rests on
/// ([ADR-0006](../../../../docs/adr/0006-process-topology-and-rt-client-transport.md)).
/// Two properties make it safe to call from the cyclic path:
///
///   * **Wait-free.** push() and pop() complete in a bounded number of steps.
///     They never block, never spin on another thread, never syscall. A
///     consumer that is descheduled, swapped out, or *killed* cannot stall the
///     producer -- it just fills the ring, and push() starts returning false.
///   * **No pointers inside.** Everything is indices into an inline array, so
///     the same object works when two processes map it at different virtual
///     addresses. A single pointer member here would be a crash in the client.
///
/// ## How it works
///
/// `head` (consumer) and `tail` (producer) are monotonically increasing
/// counters, *not* wrapped indices. The slot is `counter & mask`. Using
/// free-running counters is what makes "empty" (`head == tail`) and "full"
/// (`tail - head == Capacity`) unambiguous; with wrapped indices those two
/// states look identical and you need a wasted slot to tell them apart.
/// 64-bit counters at 500 Hz overflow in about 1.2 billion years.
///
/// ## The memory ordering, and why each one
///
/// Producer: reads its own `tail` **relaxed** (nobody else writes it), reads
/// `head` **acquire** (to see the consumer's progress), writes the slot, then
/// publishes with a **release** store to `tail`. The release is what
/// guarantees the consumer's acquire-load of `tail` also sees the slot data.
/// Get this wrong and it works on x86 and fails on ARM -- rarely, and never
/// under a debugger.
///
/// Consumer: mirror image.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <type_traits>

namespace rc::rt {

/// Cache line size. Head and tail live on separate lines so that the producer
/// writing `tail` does not invalidate the line the consumer is reading
/// `head` from -- false sharing turns a wait-free queue into a cache-line
/// ping-pong and can cost an order of magnitude.
inline constexpr std::size_t cache_line_bytes = 64;

/// Single-producer / single-consumer bounded queue. Wait-free on both sides,
/// allocation-free, and free of pointers so it works in shared memory mapped
/// at different addresses. push() drops when full rather than blocking.
template <typename T, std::size_t Capacity>
class SingleProducerSingleConsumerRing {
  static_assert(std::is_trivially_copyable_v<T>,
                "shared-memory records must be trivially copyable: no pointers, "
                "no std::string, no vtables");
  static_assert(Capacity >= 2, "capacity must be at least 2");
  static_assert((Capacity & (Capacity - 1)) == 0,
                "capacity must be a power of two so that masking replaces modulo");

 public:
  using value_type = T;                             ///< element type
  static constexpr std::size_t capacity = Capacity;  ///< slots; the ring holds at most this many

  /// Producer side. Returns false if the ring is full; the caller decides the
  /// policy. The cyclic path must never retry in a loop -- it drops, counts,
  /// and moves on.
  [[nodiscard]] bool push(const T& value) noexcept {
    const std::uint64_t write_index = tail.load(std::memory_order_relaxed);
    const std::uint64_t read_index = head.load(std::memory_order_acquire);
    if (write_index - read_index >= Capacity) {
      return false;  // full
    }
    slots[write_index & mask] = value;
    tail.store(write_index + 1, std::memory_order_release);
    return true;
  }

  /// Consumer side. Returns false if empty.
  [[nodiscard]] bool pop(T& out) noexcept {
    const std::uint64_t read_index = head.load(std::memory_order_relaxed);
    const std::uint64_t write_index = tail.load(std::memory_order_acquire);
    if (read_index == write_index) {
      return false;  // empty
    }
    out = slots[read_index & mask];
    head.store(read_index + 1, std::memory_order_release);
    return true;
  }

  /// Approximate occupancy. Both counters are sampled independently, so this is
  /// a snapshot that may already be stale -- fine for diagnostics, never for
  /// control flow.
  [[nodiscard]] std::size_t approximate_size() const noexcept {
    const std::uint64_t write_index = tail.load(std::memory_order_acquire);
    const std::uint64_t read_index = head.load(std::memory_order_acquire);
    return static_cast<std::size_t>(write_index - read_index);
  }

  /// approximate_size() == 0, with the same caveat.
  [[nodiscard]] bool approximately_empty() const noexcept { return approximate_size() == 0; }

  /// Total items ever published / consumed. Useful for detecting a stalled
  /// consumer without inferring it from occupancy.
  [[nodiscard]] std::uint64_t produced() const noexcept {
    return tail.load(std::memory_order_acquire);
  }
  /// @copydoc produced()
  [[nodiscard]] std::uint64_t consumed() const noexcept {
    return head.load(std::memory_order_acquire);
  }

  /// Reset to empty. Only safe when neither side is running -- i.e. at
  /// construction time in shared memory, by whoever creates the region.
  void reset() noexcept {
    head.store(0, std::memory_order_relaxed);
    tail.store(0, std::memory_order_relaxed);
  }

 private:
  static constexpr std::uint64_t mask = Capacity - 1;

  alignas(cache_line_bytes) std::atomic<std::uint64_t> head{0};
  alignas(cache_line_bytes) std::atomic<std::uint64_t> tail{0};
  alignas(cache_line_bytes) T slots[Capacity]{};
};

static_assert(std::atomic<std::uint64_t>::is_always_lock_free,
              "a std::atomic<uint64_t> that takes a lock would deadlock across "
              "processes if the lock-holder is killed");

}  // namespace rc::rt
