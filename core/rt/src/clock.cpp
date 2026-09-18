#include "rc/rt/clock.hpp"

#include <cerrno>
#include <ctime>

namespace rc::rt {
namespace {

constexpr std::int64_t kNsPerSec = 1'000'000'000;

timespec to_timespec(Nanos duration) noexcept {
  timespec spec{};
  spec.tv_sec = static_cast<time_t>(duration.count() / kNsPerSec);
  spec.tv_nsec = static_cast<long>(duration.count() % kNsPerSec);
  return spec;
}

}  // namespace

Nanos monotonic_now() noexcept {
  timespec spec{};
  ::clock_gettime(CLOCK_MONOTONIC, &spec);
  return Nanos{static_cast<std::int64_t>(spec.tv_sec) * kNsPerSec + spec.tv_nsec};
}

void sleep_until(Nanos deadline) noexcept {
  const timespec spec = to_timespec(deadline);
  // clock_nanosleep is unusual: it returns the error number directly instead of
  // returning -1 and setting errno. Checking errno here would read stale state.
  // With TIMER_ABSTIME the same absolute deadline is correct to retry on EINTR.
  while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &spec, nullptr) == EINTR) {
  }
}

void sleep_for(Nanos duration) noexcept {
  if (duration <= Nanos::zero()) {
    return;
  }
  timespec spec = to_timespec(duration);
  timespec remaining{};
  // Relative sleep must resume with the *remaining* time after a signal, which
  // is already one hint that this is the more fragile of the two calls.
  while (::clock_nanosleep(CLOCK_MONOTONIC, 0, &spec, &remaining) == EINTR) {
    spec = remaining;
  }
}

}  // namespace rc::rt
