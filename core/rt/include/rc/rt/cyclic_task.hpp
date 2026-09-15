#pragma once

/// @file cyclic_task.hpp
/// The phase-locked cyclic executive that every real-time loop in this project
/// runs on: the 500 Hz CAN cycle, the trajectory interpolator, the watchdog.
///
/// The shape is always the same, in any language:
///
///     origin = now()
///     for n in 0..:
///         sleep_until(origin + n * period)     <-- deadline from a FIXED origin
///         do_work()
///
/// Computing the deadline from a fixed origin rather than from "now" is the
/// whole trick. A late wake-up shortens the *next* sleep instead of pushing the
/// schedule back, so error is corrected every cycle instead of accumulating.
/// The vendor's `sleep(dt - elapsed)` does the opposite and loses 4.7% of
/// wall-clock time permanently (bench/loop_timing).
///
/// Four different numbers get confused with each other constantly, so this
/// class measures all four separately:
///
///   wake latency   wake - deadline          the scheduler's fault
///   period error   wake[n] - wake[n-1] - T  what the motors actually see
///   execution time how long your body() ran your fault
///   drift          wake[N] - (origin + N*T) does the loop keep long-run time?
///
/// You can have zero drift and terrible jitter, or tiny jitter on a loop that
/// is steadily 5% slow. They are different failures with different fixes.

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>

#include "rc/rt/clock.hpp"
#include "rc/rt/latency_histogram.hpp"
#include "rc/rt/rt_setup.hpp"

namespace rc::rt {

struct CyclicConfig {
  Nanos period{std::chrono::milliseconds(2)};  // 500 Hz: the B601-RS nominal
  RtOptions rt{};
  /// A cycle whose period error exceeds this counts as an overrun.
  /// Zero means "10% of the period".
  Nanos overrun_threshold{Nanos::zero()};
  /// Histogram range. Samples outside it are counted but lose bucket
  /// resolution; LatencyHistogram::out_of_range() reports how many.
  Nanos histogram_span{std::chrono::milliseconds(2)};
};

struct CyclicReport {
  std::uint64_t cycles = 0;
  /// Cycles whose period error exceeded the overrun threshold.
  std::uint64_t overruns = 0;
  /// Cycles where body() itself ran longer than the period -- the loop cannot
  /// keep up and no amount of scheduler tuning will save it.
  std::uint64_t missed_deadlines = 0;
  /// End-to-end schedule error: where we finished versus where we should have.
  Nanos drift{Nanos::zero()};
  Nanos period{Nanos::zero()};
  RtStatus rt_status{};

  LatencyHistogram wake_latency;
  LatencyHistogram period_error;
  LatencyHistogram exec_time;

  CyclicReport(Nanos span, Nanos period_ns);
  [[nodiscard]] std::string format() const;
};

class CyclicTask {
 public:
  /// Cycle body. Receives the cycle index and the nominal period -- use the
  /// nominal dt for integration, not the measured one, unless you have thought
  /// hard about what a 200 us jitter spike does to your derivative term.
  using Body = std::function<void(std::uint64_t cycle, Nanos dt)>;

  explicit CyclicTask(CyclicConfig cfg);

  /// Applies RT options, then runs exactly @p cycles iterations in the calling
  /// thread. Deliberately synchronous: the RT loop owns its thread, it is not
  /// something spawned and forgotten.
  CyclicReport run(std::uint64_t cycles, const Body& body);

  /// Runs until @p stop becomes true. Checked once per cycle.
  CyclicReport run_until(const std::atomic<bool>& stop, const Body& body);

 private:
  CyclicReport run_impl(std::uint64_t max_cycles, const std::atomic<bool>* stop,
                        const Body& body);

  CyclicConfig cfg_;
};

}  // namespace rc::rt
