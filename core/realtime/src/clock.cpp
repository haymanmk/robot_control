#include "robot_control/realtime/clock.hpp"

#include <cerrno>
#include <ctime>

namespace robot_control::realtime {
namespace {

constexpr std::int64_t nanoseconds_per_second = 1'000'000'000;

timespec to_timespec(nanoseconds duration) noexcept {
  timespec spec{};
  spec.tv_sec = static_cast<time_t>(duration.count() / nanoseconds_per_second);
  spec.tv_nsec = static_cast<long>(duration.count() % nanoseconds_per_second);
  return spec;
}

}  // namespace

nanoseconds monotonic_now() noexcept {
  timespec spec{};
  ::clock_gettime(CLOCK_MONOTONIC, &spec);
  return nanoseconds{static_cast<std::int64_t>(spec.tv_sec) * nanoseconds_per_second + spec.tv_nsec};
}

void sleep_until(nanoseconds deadline) noexcept {
  const timespec spec = to_timespec(deadline);
  // clock_nanosleep is unusual: it returns the error number directly instead of
  // returning -1 and setting errno. Checking errno here would read stale state.
  // With TIMER_ABSTIME the same absolute deadline is correct to retry on EINTR.
  while (::clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &spec, nullptr) == EINTR) {
  }
}

void sleep_for(nanoseconds duration) noexcept {
  if (duration <= nanoseconds::zero()) {
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

}  // namespace robot_control::realtime
