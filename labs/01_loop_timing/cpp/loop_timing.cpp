/// Lab 01, C++ edition -- the same measurement as loop_timing.py, in the
/// language the control core is actually written in.
///
/// Running both on your own machine answers a question you will be asked
/// repeatedly: how much of the jitter is Python's fault, and how much is the
/// kernel's? (Spoiler, and the reason the C++ decision is about more than
/// speed: at 500 Hz most of it is the kernel's. C++ buys you the *ability* to
/// fix the rest -- no GC, no GIL, no surprise allocation -- not an automatic win.)
///
/// Build:  cmake -B build && cmake --build build -j
/// Run:    ./build/labs/01_loop_timing/cpp/lab01_loop_timing
///         sudo ./build/labs/01_loop_timing/cpp/lab01_loop_timing --rt 80 --cpu 3

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>
#include <vector>

#include "rc/rt/clock.hpp"
#include "rc/rt/cyclic_task.hpp"
#include "rc/rt/latency_histogram.hpp"

using rc::rt::CyclicConfig;
using rc::rt::CyclicTask;
using rc::rt::LatencyHistogram;
using rc::rt::Nanos;
using rc::rt::monotonic_now;

namespace {

/// Stand-in for a real cycle body (Pinocchio RNEA + CAN frame packing).
/// A busy-spin, not a sleep: work that sleeps hides the very scheduling
/// behaviour we are trying to measure.
void busy_for(Nanos duration) {
  const Nanos end = monotonic_now() + duration;
  while (monotonic_now() < end) {
  }
}

struct Sample {
  Nanos wake;
};

/// Strategy A -- the vendor pattern: sleep for (period - elapsed).
/// Relative sleep, so the kernel's wake-up latency is added to every period and
/// never repaid. Expect a mean period error of tens of microseconds and drift
/// that grows without bound.
std::vector<Sample> run_relative(std::uint64_t cycles, Nanos period, Nanos work) {
  std::vector<Sample> out;
  out.reserve(cycles);
  for (std::uint64_t n = 0; n < cycles; ++n) {
    const Nanos t0 = monotonic_now();
    out.push_back({t0});
    busy_for(work);
    const Nanos remaining = period - (monotonic_now() - t0);
    if (remaining > Nanos::zero()) {
      rc::rt::sleep_for(remaining);
    }
  }
  return out;
}

/// Strategy B -- absolute deadline, but still computed then slept relatively.
/// Phase-locked, so drift is corrected; the residual jitter is the scheduler's.
std::vector<Sample> run_absolute_relative_sleep(std::uint64_t cycles, Nanos period, Nanos work) {
  std::vector<Sample> out;
  out.reserve(cycles);
  const Nanos origin = monotonic_now();
  for (std::uint64_t n = 0; n < cycles; ++n) {
    out.push_back({monotonic_now()});
    busy_for(work);
    const Nanos deadline = origin + period * static_cast<std::int64_t>(n + 1);
    rc::rt::sleep_for(deadline - monotonic_now());
  }
  return out;
}

/// Strategy C -- clock_nanosleep(TIMER_ABSTIME), via CyclicTask.
/// The real-time idiom, and the one the rest of this project builds on.
std::vector<Sample> run_clock_nanosleep(std::uint64_t cycles, Nanos period, Nanos work) {
  std::vector<Sample> out;
  out.reserve(cycles);
  CyclicConfig cfg;
  cfg.period = period;
  cfg.rt.priority = 0;       // measured separately by the caller's --rt
  cfg.rt.lock_memory = false;
  CyclicTask task(cfg);
  task.run(cycles, [&](std::uint64_t, Nanos) {
    out.push_back({monotonic_now()});
    busy_for(work);
  });
  return out;
}

struct Analysis {
  std::string name;
  LatencyHistogram period_error;
  double drift_ms = 0.0;
  std::uint64_t overruns = 0;
};

Analysis analyse(const std::string& name, const std::vector<Sample>& s, Nanos period) {
  // +/- half a period with 100 buckets gives ~20 us resolution at 500 Hz.
  // Outliers beyond that are counted as out-of-range and reported below; max
  // is tracked exactly either way.
  Analysis a{name, LatencyHistogram(name, -period.count() / 2, period.count() / 2, 100), 0.0, 0};
  const std::int64_t threshold = period.count() / 10;
  for (std::size_t i = 1; i < s.size(); ++i) {
    const std::int64_t err = (s[i].wake - s[i - 1].wake - period).count();
    a.period_error.record(err);
    if (err > threshold || err < -threshold) {
      ++a.overruns;
    }
  }
  if (s.size() > 1) {
    const std::int64_t ideal = period.count() * static_cast<std::int64_t>(s.size() - 1);
    a.drift_ms = static_cast<double>((s.back().wake - s.front().wake).count() - ideal) / 1e6;
  }
  return a;
}

[[noreturn]] void usage(const char* argv0, int code) {
  std::printf(
      "usage: %s [--rate HZ] [--seconds S] [--work-us US] [--rt PRIO] [--cpu N] [--only NAME]\n"
      "\n"
      "  --rate     loop rate in Hz (default 500, the B601-RS nominal)\n"
      "  --seconds  duration per strategy (default 10)\n"
      "  --work-us  simulated cycle body cost in microseconds (default 400)\n"
      "  --rt PRIO  request SCHED_FIFO at PRIO and mlockall (needs privileges)\n"
      "  --cpu N    pin to CPU N\n"
      "  --only     run one of: relative-sleep, absolute-sleep, clock-nanosleep\n",
      argv0);
  std::exit(code);
}

}  // namespace

int main(int argc, char** argv) {
  double rate_hz = 500.0;
  double seconds = 10.0;
  double work_us = 400.0;
  int rt_priority = 0;
  int cpu = -1;
  std::string only;

  for (int i = 1; i < argc; ++i) {
    const std::string arg = argv[i];
    auto next = [&]() -> const char* {
      if (i + 1 >= argc) usage(argv[0], 2);
      return argv[++i];
    };
    if (arg == "--rate") rate_hz = std::atof(next());
    else if (arg == "--seconds") seconds = std::atof(next());
    else if (arg == "--work-us") work_us = std::atof(next());
    else if (arg == "--rt") rt_priority = std::atoi(next());
    else if (arg == "--cpu") cpu = std::atoi(next());
    else if (arg == "--only") only = next();
    else if (arg == "-h" || arg == "--help") usage(argv[0], 0);
    else usage(argv[0], 2);
  }

  const auto period = Nanos{static_cast<std::int64_t>(1e9 / rate_hz)};
  const auto work = Nanos{static_cast<std::int64_t>(work_us * 1000.0)};
  const auto cycles = static_cast<std::uint64_t>(seconds * rate_hz);

  std::printf("Lab 01 (C++) -- control loop timing\n");
  std::printf("  rate      %g Hz  (period %.1f us)\n", rate_hz,
              static_cast<double>(period.count()) / 1000.0);
  std::printf("  cycles    %lu per strategy (%g s)\n", static_cast<unsigned long>(cycles), seconds);
  std::printf("  work      %g us busy-spin per cycle\n", work_us);

  if (rt_priority > 0 || cpu >= 0) {
    rc::rt::RtOptions opts;
    opts.priority = rt_priority;
    opts.lock_memory = true;
    opts.cpu = cpu;
    const auto status = rc::rt::apply_realtime(opts);
    std::printf("%s", status.format().c_str());
  }
  std::printf("\n");

  struct Strategy {
    const char* name;
    std::vector<Sample> (*fn)(std::uint64_t, Nanos, Nanos);
  };
  const Strategy strategies[] = {
      {"relative-sleep", run_relative},
      {"absolute-sleep", run_absolute_relative_sleep},
      {"clock-nanosleep", run_clock_nanosleep},
  };

  std::vector<Analysis> results;
  for (const auto& s : strategies) {
    if (!only.empty() && only != s.name) continue;
    std::printf("running %s ... ", s.name);
    std::fflush(stdout);
    results.push_back(analyse(s.name, s.fn(cycles, period, work), period));
    std::printf("done\n");
  }
  if (results.empty()) usage(argv[0], 2);

  std::printf("\n%-18s%10s%10s%10s%10s%12s%10s\n", "strategy", "mean", "p99", "p99.9", "max",
              "drift", "overruns");
  std::printf("%s\n", std::string(80, '-').c_str());
  for (const auto& r : results) {
    std::printf("%s%11.2fm%10lu\n", r.period_error.format_row().c_str(), r.drift_ms,
                static_cast<unsigned long>(r.overruns));
  }
  std::printf(
      "\n  mean/p99/max are per-cycle period error in microseconds (us).\n"
      "  drift is total schedule error in milliseconds (ms) after the whole run.\n"
      "  overruns are cycles off nominal by more than 10%% (%.0f us).\n\n",
      static_cast<double>(period.count()) / 10000.0);

  for (const auto& r : results) {
    std::printf("  %s -- per-cycle period error distribution\n", r.name.c_str());
    if (r.period_error.out_of_range() > 0) {
      std::printf("  (%lu samples outside the charted range; p99.9 may be clipped, max is exact)\n",
                  static_cast<unsigned long>(r.period_error.out_of_range()));
    }
    std::printf("%s\n", r.period_error.format_chart().c_str());
  }
  return 0;
}
