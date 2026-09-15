#include "rc/rt/clock.hpp"

#include <cerrno>
#include <ctime>

namespace rc::rt {
namespace {

constexpr std::int64_t kNsPerSec = 1'000'000'000;

timespec to_timespec(Nanos ns) noexcept {
  timespec ts{};
  ts.tv_sec = static_cast<time_t>(ns.count() / kNsPerSec);
  ts.tv_nsec = static_cast<long>(ns.count() % kNsPerSec);
  return ts;
}

}  // namespace

Nanos monotonic_now() noexcept {
  timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return Nanos{static_cast<std::int64_t>(ts.tv_sec) * kNsPerSec + ts.tv_nsec};
}

void sleep_until(Nanos deadline) noexcept {
  const timespec ts = to_timespec(deadline);
  // clock_nanosleep is unusual: it returns the error number directly instead of
  // returning -1 and setting errno. Checking errno here would read stale state.
  // With TIMER_ABSTIME the same absolute deadline is correct to retry on EINTR.
  while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &ts, nullptr) == EINTR) {
  }
}

void sleep_for(Nanos duration) noexcept {
  if (duration <= Nanos::zero()) {
    return;
  }
  timespec ts = to_timespec(duration);
  timespec remaining{};
  // Relative sleep must resume with the *remaining* time after a signal, which
  // is already one hint that this is the more fragile of the two calls.
  while (::clock_nanosleep(CLOCK_MONOTONIC, 0, &ts, &remaining) == EINTR) {
    ts = remaining;
  }
}

}  // namespace rc::rt
