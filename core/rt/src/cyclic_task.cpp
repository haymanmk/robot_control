#include "rc/rt/cyclic_task.hpp"

#include <cstdio>
#include <limits>
#include <sstream>

namespace rc::rt {

CyclicReport::CyclicReport(Nanos span, Nanos period_ns)
    : period(period_ns),
      // Wake latency is one-sided: you cannot wake before your deadline.
      wake_latency("wake-latency", 0, span.count(), 64),
      // Period error is two-sided: a cycle that ran late is followed by one
      // that runs short as the phase lock pulls the schedule back.
      period_error("period-error", -span.count(), span.count(), 64),
      exec_time("exec-time", 0, span.count(), 64) {}

std::string CyclicReport::format() const {
  std::ostringstream os;
  char buf[256];

  os << rt_status.format();
  std::snprintf(buf, sizeof(buf), "%-18s%10s%10s%10s%10s\n", "metric", "mean", "p99", "p99.9",
                "max");
  os << buf << std::string(58, '-') << '\n';
  os << wake_latency.format_row() << '\n'
     << period_error.format_row() << '\n'
     << exec_time.format_row() << '\n';

  std::snprintf(buf, sizeof(buf),
                "\n  cycles %lu   overruns %lu   missed deadlines %lu\n"
                "  drift %.3f ms over %.2f s nominal\n",
                static_cast<unsigned long>(cycles), static_cast<unsigned long>(overruns),
                static_cast<unsigned long>(missed_deadlines),
                static_cast<double>(drift.count()) / 1e6,
                static_cast<double>(cycles) * static_cast<double>(period.count()) / 1e9);
  os << buf;

  const auto oor = wake_latency.out_of_range() + period_error.out_of_range();
  if (oor > 0) {
    std::snprintf(buf, sizeof(buf),
                  "  note: %lu samples fell outside the histogram range; "
                  "percentiles are clipped (raise histogram_span)\n",
                  static_cast<unsigned long>(oor));
    os << buf;
  }
  return os.str();
}

CyclicTask::CyclicTask(CyclicConfig cfg) : cfg_(cfg) {
  if (cfg_.overrun_threshold == Nanos::zero()) {
    cfg_.overrun_threshold = cfg_.period / 10;
  }
}

CyclicReport CyclicTask::run(std::uint64_t cycles, const Body& body) {
  return run_impl(cycles, nullptr, body);
}

CyclicReport CyclicTask::run_until(const std::atomic<bool>& stop, const Body& body) {
  return run_impl(std::numeric_limits<std::uint64_t>::max(), &stop, body);
}

CyclicReport CyclicTask::run_impl(std::uint64_t max_cycles, const std::atomic<bool>* stop,
                                  const Body& body) {
  CyclicReport report(cfg_.histogram_span, cfg_.period);
  report.rt_status = apply_realtime(cfg_.rt);

  const Nanos period = cfg_.period;
  const Nanos origin = monotonic_now();
  Nanos previous_wake{Nanos::zero()};

  for (std::uint64_t n = 0; n < max_cycles; ++n) {
    if (stop != nullptr && stop->load(std::memory_order_relaxed)) {
      break;
    }

    // The phase lock: the deadline for cycle n depends only on the origin and
    // n, never on when the previous cycle happened to finish.
    const Nanos deadline = origin + period * static_cast<std::int64_t>(n);
    if (n > 0) {
      sleep_until(deadline);
    }

    const Nanos wake = monotonic_now();
    report.wake_latency.record((wake - deadline).count());

    if (n > 0) {
      const std::int64_t err = (wake - previous_wake - period).count();
      report.period_error.record(err);
      if (err > cfg_.overrun_threshold.count() || err < -cfg_.overrun_threshold.count()) {
        ++report.overruns;
      }
    }
    previous_wake = wake;

    body(n, period);

    const Nanos exec = monotonic_now() - wake;
    report.exec_time.record(exec.count());
    if (exec > period) {
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
    const Nanos ideal = origin + period * static_cast<std::int64_t>(report.cycles - 1);
    report.drift = previous_wake - ideal;
  }
  return report;
}

}  // namespace rc::rt
