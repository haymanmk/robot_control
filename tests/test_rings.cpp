/// Unit tests for the lock-free primitives, including the concurrent cases that
/// are the only ones that matter. Run under ThreadSanitizer too:
///   cmake -B build-tsan -DRC_SANITIZE=thread && ctest --test-dir build-tsan

#include <atomic>
#include <thread>
#include <vector>

#include "rc/rt/seqlock.hpp"
#include "rc/rt/spsc_ring.hpp"
#include "test_support.hpp"

using rc::rt::Seqlock;
using rc::rt::SpscRing;

namespace {

void test_spsc_basic() {
  SpscRing<int, 4> ring;
  int out = 0;
  CHECK(!ring.pop(out));               // empty
  CHECK(ring.push(1));
  CHECK(ring.push(2));
  CHECK(ring.push(3));
  CHECK(ring.push(4));
  CHECK_MSG(!ring.push(5), "must report full rather than overwrite");
  CHECK_EQ(ring.size_approx(), 4u);

  CHECK(ring.pop(out));
  CHECK_EQ(out, 1);
  CHECK(ring.push(5));                 // space freed by the pop
  for (int expected : {2, 3, 4, 5}) {
    CHECK(ring.pop(out));
    CHECK_EQ(out, expected);
  }
  CHECK(!ring.pop(out));
}

/// Wrap-around is where an implementation using wrapped indices instead of
/// free-running counters breaks. Push/pop far more than capacity.
void test_spsc_wraparound() {
  SpscRing<std::uint64_t, 8> ring;
  std::uint64_t out = 0;
  for (std::uint64_t value = 0; value < 10'000; ++value) {
    CHECK(ring.push(value));
    CHECK(ring.pop(out));
    if (out != value) {
      CHECK_EQ(out, value);
      return;  // one failure message is enough
    }
  }
  CHECK_EQ(ring.produced(), 10'000u);
  CHECK_EQ(ring.consumed(), 10'000u);
}

/// The real test: a producer and a consumer on different threads. Every value
/// must arrive exactly once, in order, with nothing torn or duplicated.
void test_spsc_concurrent() {
  constexpr std::uint64_t kCount = 200'000;
  SpscRing<std::uint64_t, 1024> ring;
  std::atomic<bool> producer_done{false};

  std::thread producer([&] {
    for (std::uint64_t value = 0; value < kCount; ++value) {
      while (!ring.push(value)) {
        std::this_thread::yield();  // test code may spin; the RT path must not
      }
    }
    producer_done.store(true, std::memory_order_release);
  });

  std::uint64_t expected = 0;
  std::uint64_t out = 0;
  bool ordered = true;
  while (expected < kCount) {
    if (ring.pop(out)) {
      if (out != expected) {
        ordered = false;
        break;
      }
      ++expected;
    } else if (producer_done.load(std::memory_order_acquire) && ring.empty_approx()) {
      break;
    }
  }
  producer.join();

  CHECK_MSG(ordered, "values must arrive in order with none lost or duplicated");
  CHECK_EQ(expected, kCount);
}

struct Wide {
  std::uint64_t word0, word1, word2, word3, word4, word5, word6, word7;
  [[nodiscard]] bool consistent() const noexcept {
    return word1 == word0 + 1 && word2 == word0 + 2 && word3 == word0 + 3 && word4 == word0 + 4 &&
           word5 == word0 + 5 && word6 == word0 + 6 && word7 == word0 + 7;
  }
};

/// A seqlock's whole job is that a reader never returns a half-written value.
/// The struct is wide enough that a torn read is likely if the protocol is wrong.
void test_seqlock_concurrent() {
  Seqlock<Wide> lock;
  lock.reset();
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> torn{0};
  std::atomic<std::uint64_t> reads{0};

  // Start at 1: Wide{0,0,...} is what reset() leaves behind, and it fails
  // consistent() trivially. An earlier version of this test counted those
  // pre-first-write reads as tearing and blamed the seqlock -- a reminder that
  // a failing concurrency test is as likely to be a bad oracle as a bad lock.
  std::thread writer([&] {
    for (std::uint64_t stamp = 1; !stop.load(std::memory_order_acquire); ++stamp) {
      lock.store(Wide{stamp, stamp + 1, stamp + 2, stamp + 3, stamp + 4, stamp + 5, stamp + 6,
                      stamp + 7});
    }
  });

  while (lock.generation() == 0) {  // nothing to validate until the first write lands
    std::this_thread::yield();
  }

  std::uint64_t first_seen = 0;
  std::uint64_t last_seen = 0;
  for (int attempt = 0; attempt < 300'000; ++attempt) {
    Wide value{};
    if (lock.load(value)) {
      reads.fetch_add(1, std::memory_order_relaxed);
      if (!value.consistent()) {
        torn.fetch_add(1, std::memory_order_relaxed);
      }
      if (first_seen == 0) {
        first_seen = value.word0;
      }
      last_seen = value.word0;
    }
  }
  stop.store(true, std::memory_order_release);
  writer.join();

  CHECK_MSG(torn.load() == 0, "seqlock returned a torn value: " + std::to_string(torn.load()));
  CHECK_MSG(reads.load() > 0, "reader never won a race, so the test proved nothing");
  // Without this, a reader that saw one stable value 300k times would "pass"
  // while proving nothing at all about concurrent access.
  CHECK_MSG(last_seen > first_seen + 1000,
            "reader did not observe the writer advancing, so no race was exercised: " +
                std::to_string(first_seen) + " -> " + std::to_string(last_seen));
}

void test_seqlock_generation() {
  Seqlock<Wide> lock;
  lock.reset();
  CHECK_EQ(lock.generation(), 0u);
  lock.store(Wide{1, 2, 3, 4, 5, 6, 7, 8});
  CHECK_EQ(lock.generation(), 1u);
  Wide out{};
  CHECK(lock.load(out));
  CHECK_EQ(out.word0, 1u);
  CHECK(out.consistent());
}

}  // namespace

int main() {
  test_spsc_basic();
  test_spsc_wraparound();
  test_spsc_concurrent();
  test_seqlock_concurrent();
  test_seqlock_generation();
  return rc::test::finish("rings");
}
