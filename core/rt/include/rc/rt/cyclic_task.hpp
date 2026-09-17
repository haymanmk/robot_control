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

/// Everything CyclicTask needs to know before it starts: the period, the
/// real-time options to apply, and the thresholds for counting and measuring.
struct CyclicConfig {
  /// Nominal cycle period. 2 ms is 500 Hz, the B601-RS nominal.
  Nanos period{std::chrono::milliseconds(2)};
  /// Scheduler, memory-lock and affinity settings applied at run() start.
  RtOptions rt{};
  /// A cycle whose period error exceeds this counts as an overrun.
  /// Zero means "10% of the period".
  Nanos overrun_threshold{Nanos::zero()};
  /// Histogram range. Samples outside it are counted but lose bucket
  /// resolution; LatencyHistogram::out_of_range() reports how many.
  Nanos histogram_span{std::chrono::milliseconds(2)};
};

/// What a run produced: cycle counts, the three latency histograms, and the
/// long-run drift. Each number answers a different question; see the file
/// comment for which one is whose fault.
struct CyclicReport {
  /// Cycles completed.
  std::uint64_t cycles = 0;
  /// Cycles whose period error exceeded the overrun threshold.
  std::uint64_t overruns = 0;
  /// Cycles where body() itself ran longer than the period -- the loop cannot
  /// keep up and no amount of scheduler tuning will save it.
  std::uint64_t missed_deadlines = 0;
  /// End-to-end schedule error: where we finished versus where we should have.
  Nanos drift{Nanos::zero()};
  /// Nominal period the run was configured with.
  Nanos period{Nanos::zero()};
  /// Which real-time setup steps were actually granted.
  RtStatus rt_status{};

  /// wake - deadline: the scheduler's fault.
  LatencyHistogram wake_latency;
  /// wake[n] - wake[n-1] - T: what the motors actually see.
  LatencyHistogram period_error;
  /// How long body() ran: your fault.
  LatencyHistogram exec_time;

  /// Sizes the three histograms to @p span; @p period_ns is recorded as-is.
  CyclicReport(Nanos span, Nanos period_ns);
  /// Multi-line human-readable summary: counts, drift, and one row per histogram.
  [[nodiscard]] std::string format() const;
};

/// Phase-locked cyclic executive. Sleeps to deadlines computed from a fixed
/// origin, so a late wake-up shortens the next sleep instead of shifting the
/// schedule, and measures wake latency, period error, execution time and
/// drift separately.
class CyclicTask {
 public:
  /// Cycle body. Receives the cycle index and the nominal period -- use the
  /// nominal dt for integration, not the measured one, unless you have thought
  /// hard about what a 200 us jitter spike does to your derivative term.
  using Body = std::function<void(std::uint64_t cycle, Nanos dt)>;

  /// Stores the config. Nothing runs and no RT option is applied until run().
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
