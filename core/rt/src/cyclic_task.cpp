#include "rc/rt/cyclic_task.hpp"

#include <cstdio>
#include <limits>
#include <sstream>

namespace rc::rt {

CyclicReport::CyclicReport(nanoseconds span, nanoseconds nominal_period)
    : period(nominal_period),
      // Wake latency is one-sided: you cannot wake before your deadline.
      wake_latency("wake-latency", 0, span.count(), 64),
      // Period error is two-sided: a cycle that ran late is followed by one
      // that runs short as the phase lock pulls the schedule back.
      period_error("period-error", -span.count(), span.count(), 64),
      execution_time("exec-time", 0, span.count(), 64) {}

std::string CyclicReport::format() const {
  std::ostringstream out;
  char line[256];

  out << realtime_status.format();
  std::snprintf(line, sizeof(line), "%-18s%10s%10s%10s%10s\n", "metric", "mean", "p99", "p99.9",
                "max");
  out << line << std::string(58, '-') << '\n';
  out << wake_latency.format_row() << '\n'
     << period_error.format_row() << '\n'
     << execution_time.format_row() << '\n';

  std::snprintf(line, sizeof(line),
                "\n  cycles %lu   overruns %lu   missed deadlines %lu\n"
                "  drift %.3f ms over %.2f s nominal\n",
                static_cast<unsigned long>(cycles), static_cast<unsigned long>(overruns),
                static_cast<unsigned long>(missed_deadlines),
                static_cast<double>(drift.count()) / 1e6,
                static_cast<double>(cycles) * static_cast<double>(period.count()) / 1e9);
  out << line;

  const auto outside_range = wake_latency.out_of_range() + period_error.out_of_range();
  if (outside_range > 0) {
    std::snprintf(line, sizeof(line),
                  "  note: %lu samples fell outside the histogram range; "
                  "percentiles are clipped (raise histogram_span)\n",
                  static_cast<unsigned long>(outside_range));
    out << line;
  }
  return out.str();
}

CyclicTask::CyclicTask(CyclicConfig config_) : config(config_) {
  if (config.overrun_threshold == nanoseconds::zero()) {
    config.overrun_threshold = config.period / 10;
  }
}

CyclicReport CyclicTask::run(std::uint64_t cycles, const cycle_body& body) {
  return run_impl(cycles, nullptr, body);
}

CyclicReport CyclicTask::run_until(const std::atomic<bool>& stop, const cycle_body& body) {
  return run_impl(std::numeric_limits<std::uint64_t>::max(), &stop, body);
}

CyclicReport CyclicTask::run_impl(std::uint64_t max_cycles, const std::atomic<bool>* stop,
                                  const cycle_body& body) {
  CyclicReport report(config.histogram_span, config.period);
  report.realtime_status = apply_realtime(config.realtime);

  const nanoseconds period = config.period;
  const nanoseconds origin = monotonic_now();
  nanoseconds previous_wake{nanoseconds::zero()};

  for (std::uint64_t cycle = 0; cycle < max_cycles; ++cycle) {
    if (stop != nullptr && stop->load(std::memory_order_relaxed)) {
      break;
    }

    // The phase lock: the deadline for a cycle depends only on the origin and
    // the cycle index, never on when the previous cycle happened to finish.
    const nanoseconds deadline = origin + period * static_cast<std::int64_t>(cycle);
    if (cycle > 0) {
      sleep_until(deadline);
    }

    const nanoseconds wake = monotonic_now();
    report.wake_latency.record((wake - deadline).count());

    if (cycle > 0) {
      const std::int64_t error = (wake - previous_wake - period).count();
      report.period_error.record(error);
      if (error > config.overrun_threshold.count() || error < -config.overrun_threshold.count()) {
        ++report.overruns;
      }
    }
    previous_wake = wake;

    body(cycle, period);

    const nanoseconds execution = monotonic_now() - wake;
    report.execution_time.record(execution.count());
    if (execution > period) {
      // We are not skipping cycles to catch up: the next deadline is already in
      // the past, sleep_until() returns immediately, and the loop runs flat out
      // until it recovers. That is a deliberate choice -- a control loop that
      // silently drops setpoints is worse than one that visibly falls behind --
      // but it means a sustained overrun shows up here, not as a stall.
      ++report.missed_deadlines;
    }
    ++report.cycles;
  }

  if (report.cycles > 0) {
    const nanoseconds ideal = origin + period * static_cast<std::int64_t>(report.cycles - 1);
    report.drift = previous_wake - ideal;
  }
  return report;
}

}  // namespace rc::rt
