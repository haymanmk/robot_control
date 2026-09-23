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

#include "robot_control/realtime/clock.hpp"
#include "robot_control/realtime/cyclic_guard.hpp"
#include "robot_control/realtime/latency_histogram.hpp"
#include "robot_control/realtime/realtime_setup.hpp"

namespace robot_control::realtime {

/// Everything CyclicTask needs to know before it starts: the period, the
/// real-time options to apply, and the thresholds for counting and measuring.
struct CyclicConfig {
  /// Nominal cycle period. 2 ms is 500 Hz, the B601-RS nominal.
  nanoseconds period{std::chrono::milliseconds(2)};
  /// Scheduler, memory-lock and affinity settings applied at run() start.
  RealtimeOptions realtime{};
  /// A cycle whose period error exceeds this counts as an overrun.
  /// Zero means "10% of the period".
  nanoseconds overrun_threshold{nanoseconds::zero()};
  /// Histogram range. Samples outside it are counted but lose bucket
  /// resolution; LatencyHistogram::out_of_range() reports how many.
  nanoseconds histogram_span{std::chrono::milliseconds(2)};
  /// Arm the cyclic guard (syscall filter and fault counters) on the loop's
  /// thread after the real-time options are applied. Because a seccomp filter
  /// stays on the thread for life, a guarded loop should run through
  /// run_in_thread() or run_until_in_thread(), not on a thread that has work
  /// to do afterwards.
  bool guard = false;
  /// What the guard permits, when enabled.
  GuardOptions guard_options{};
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
  nanoseconds drift{nanoseconds::zero()};
  /// Nominal period the run was configured with.
  nanoseconds period{nanoseconds::zero()};
  /// Which real-time setup steps were actually granted.
  RealtimeStatus realtime_status{};
  /// What the cyclic guard observed, if CyclicConfig::guard was set.
  GuardReport guard{};

  /// wake - deadline: the scheduler's fault.
  LatencyHistogram wake_latency;
  /// wake[n] - wake[n-1] - T: what the motors actually see.
  LatencyHistogram period_error;
  /// How long body() ran: your fault.
  LatencyHistogram execution_time;

  /// Sizes the three histograms to @p span; @p nominal_period is recorded as-is.
  CyclicReport(nanoseconds span, nanoseconds nominal_period);
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
  /// nominal period for integration, not the measured one, unless you have thought
  /// hard about what a 200 us jitter spike does to your derivative term.
  using cycle_body = std::function<void(std::uint64_t cycle, nanoseconds period)>;

  /// Stores the config. Nothing runs and no RT option is applied until run().
  explicit CyclicTask(CyclicConfig config_);

  /// Applies RT options, then runs exactly @p cycles iterations in the calling
  /// thread. Deliberately synchronous: the RT loop owns its thread, it is not
  /// something spawned and forgotten.
  CyclicReport run(std::uint64_t cycles, const cycle_body& body);

  /// Runs until @p stop becomes true. Checked once per cycle.
  CyclicReport run_until(const std::atomic<bool>& stop, const cycle_body& body);

  /// Same as run(), on a fresh thread that exits as soon as the loop ends; the
  /// caller blocks until then. This is the form to use with the guard, whose
  /// syscall filter cannot be removed from a thread once installed. The thread
  /// gets the process default stack (see prepare_process()).
  CyclicReport run_in_thread(std::uint64_t cycles, const cycle_body& body);

  /// Same as run_until(), on a fresh thread. See run_in_thread().
  CyclicReport run_until_in_thread(const std::atomic<bool>& stop, const cycle_body& body);

 private:
  CyclicReport run_loop(std::uint64_t max_cycles, const std::atomic<bool>* stop,
                        const cycle_body& body);

  CyclicConfig config;
};

}  // namespace robot_control::realtime
