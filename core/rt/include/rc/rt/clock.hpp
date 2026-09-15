#pragma once

/// @file clock.hpp
/// The one clock the control system is allowed to use.
///
/// Everything cyclic in this project is timed against CLOCK_MONOTONIC, because
/// it is the only clock that cannot jump. CLOCK_REALTIME can step backwards
/// when NTP corrects it; a loop timed against it will, one day, compute a
/// negative sleep and either spin or block forever. That bug appears once a
/// month, at 3am, and takes a week to find.
///
/// The deadline functions here take *absolute* times, not durations. See
/// labs/01_loop_timing for the measurement of why that distinction is worth
/// 188 ms of drift every 4 seconds.

#include <chrono>

namespace rc::rt {

/// Nanosecond-resolution duration, and also our absolute-time representation
/// (time since the monotonic epoch). Deliberately one type: mixing a duration
/// and a time point is exactly the mistake this header exists to prevent, and
/// a single explicit unit makes the arithmetic auditable.
using Nanos = std::chrono::nanoseconds;

/// Current value of CLOCK_MONOTONIC.
[[nodiscard]] Nanos monotonic_now() noexcept;

/// Block until CLOCK_MONOTONIC reaches @p deadline.
///
/// Absolute: the kernel's wake-up latency is absorbed into the wait rather than
/// added on top of it, so repeated use does not accumulate drift. Restarts
/// itself on EINTR. Returns immediately if the deadline has already passed --
/// callers that care must check for that (see CyclicTask's missed_deadlines).
void sleep_until(Nanos deadline) noexcept;

/// Block for @p duration.
///
/// Provided so that labs can measure how much worse this is than sleep_until().
/// Do not use it to pace a control loop.
void sleep_for(Nanos duration) noexcept;

}  // namespace rc::rt
