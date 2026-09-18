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
/// Run:    ./build/bench/loop_timing/bench_loop_timing
///         sudo ./build/bench/loop_timing/bench_loop_timing --rt 80 --cpu 3

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
  for (std::uint64_t cycle = 0; cycle < cycles; ++cycle) {
    const Nanos start = monotonic_now();
    out.push_back({start});
    busy_for(work);
    const Nanos remaining = period - (monotonic_now() - start);
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
  for (std::uint64_t cycle = 0; cycle < cycles; ++cycle) {
    out.push_back({monotonic_now()});
    busy_for(work);
    const Nanos deadline = origin + period * static_cast<std::int64_t>(cycle + 1);
    rc::rt::sleep_for(deadline - monotonic_now());
  }
  return out;
}

/// Strategy C -- clock_nanosleep(TIMER_ABSTIME), via CyclicTask.
/// The real-time idiom, and the one the rest of this project builds on.
std::vector<Sample> run_clock_nanosleep(std::uint64_t cycles, Nanos period, Nanos work) {
  std::vector<Sample> out;
  out.reserve(cycles);
  CyclicConfig config;
  config.period = period;
  config.rt.priority = 0;       // measured separately by the caller's --rt
  config.rt.lock_memory = false;
  CyclicTask task(config);
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

Analysis analyse(const std::string& name, const std::vector<Sample>& samples, Nanos period) {
  // +/- half a period with 100 buckets gives ~20 us resolution at 500 Hz.
  // Outliers beyond that are counted as out-of-range and reported below; max
  // is tracked exactly either way.
  Analysis analysis{name, LatencyHistogram(name, -period.count() / 2, period.count() / 2, 100), 0.0, 0};
  const std::int64_t threshold = period.count() / 10;
  for (std::size_t index = 1; index < samples.size(); ++index) {
    const std::int64_t error = (samples[index].wake - samples[index - 1].wake - period).count();
    analysis.period_error.record(error);
    if (error > threshold || error < -threshold) {
      ++analysis.overruns;
    }
  }
  if (samples.size() > 1) {
    const std::int64_t ideal = period.count() * static_cast<std::int64_t>(samples.size() - 1);
    analysis.drift_ms = static_cast<double>((samples.back().wake - samples.front().wake).count() - ideal) / 1e6;
  }
  return analysis;
}

[[noreturn]] void usage(const char* program, int code) {
  std::printf(
      "usage: %s [--rate HZ] [--seconds S] [--work-us US] [--rt PRIO] [--cpu N] [--only NAME]\n"
      "\n"
      "  --rate     loop rate in Hz (default 500, the B601-RS nominal)\n"
      "  --seconds  duration per strategy (default 10)\n"
      "  --work-us  simulated cycle body cost in microseconds (default 400)\n"
      "  --rt PRIO  request SCHED_FIFO at PRIO and mlockall (needs privileges)\n"
      "  --cpu N    pin to CPU N\n"
      "  --only     run one of: relative-sleep, absolute-sleep, clock-nanosleep\n",
      program);
  std::exit(code);
}

}  // namespace

int main(int argc, char** argv) {
  double rate_hertz = 500.0;
  double seconds = 10.0;
  double work_microseconds = 400.0;
  int realtime_priority = 0;
  int cpu = -1;
  std::string only;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto next = [&]() -> const char* {
      if (index + 1 >= argc) usage(argv[0], 2);
      return argv[++index];
    };
    if (argument == "--rate") rate_hertz = std::atof(next());
    else if (argument == "--seconds") seconds = std::atof(next());
    else if (argument == "--work-us") work_microseconds = std::atof(next());
    else if (argument == "--rt") realtime_priority = std::atoi(next());
    else if (argument == "--cpu") cpu = std::atoi(next());
    else if (argument == "--only") only = next();
    else if (argument == "-h" || argument == "--help") usage(argv[0], 0);
    else usage(argv[0], 2);
  }

  const auto period = Nanos{static_cast<std::int64_t>(1e9 / rate_hertz)};
  const auto work = Nanos{static_cast<std::int64_t>(work_microseconds * 1000.0)};
  const auto cycles = static_cast<std::uint64_t>(seconds * rate_hertz);

  std::printf("Lab 01 (C++) -- control loop timing\n");
  std::printf("  rate      %g Hz  (period %.1f us)\n", rate_hertz,
              static_cast<double>(period.count()) / 1000.0);
  std::printf("  cycles    %lu per strategy (%g s)\n", static_cast<unsigned long>(cycles), seconds);
  std::printf("  work      %g us busy-spin per cycle\n", work_microseconds);

  if (realtime_priority > 0 || cpu >= 0) {
    rc::rt::RtOptions options;
    options.priority = realtime_priority;
    options.lock_memory = true;
    options.cpu = cpu;
    const auto status = rc::rt::apply_realtime(options);
    std::printf("%s", status.format().c_str());
  }
  std::printf("\n");

  struct Strategy {
    const char* name;
    std::vector<Sample> (*run)(std::uint64_t, Nanos, Nanos);
  };
  const Strategy strategies[] = {
      {"relative-sleep", run_relative},
      {"absolute-sleep", run_absolute_relative_sleep},
      {"clock-nanosleep", run_clock_nanosleep},
  };

  std::vector<Analysis> results;
  for (const auto& strategy : strategies) {
    if (!only.empty() && only != strategy.name) continue;
    std::printf("running %s ... ", strategy.name);
    std::fflush(stdout);
    results.push_back(analyse(strategy.name, strategy.run(cycles, period, work), period));
    std::printf("done\n");
  }
  if (results.empty()) usage(argv[0], 2);

  std::printf("\n%-18s%10s%10s%10s%10s%12s%10s\n", "strategy", "mean", "p99", "p99.9", "max",
              "drift", "overruns");
  std::printf("%s\n", std::string(80, '-').c_str());
  for (const auto& result : results) {
    std::printf("%s%11.2fm%10lu\n", result.period_error.format_row().c_str(), result.drift_ms,
                static_cast<unsigned long>(result.overruns));
  }
  std::printf(
      "\n  mean/p99/max are per-cycle period error in microseconds (us).\n"
      "  drift is total schedule error in milliseconds (ms) after the whole run.\n"
      "  overruns are cycles off nominal by more than 10%% (%.0f us).\n\n",
      static_cast<double>(period.count()) / 10000.0);

  for (const auto& result : results) {
    std::printf("  %s -- per-cycle period error distribution\n", result.name.c_str());
    if (result.period_error.out_of_range() > 0) {
      std::printf("  (%lu samples outside the charted range; p99.9 may be clipped, max is exact)\n",
                  static_cast<unsigned long>(result.period_error.out_of_range()));
    }
    std::printf("%s\n", result.period_error.format_chart().c_str());
  }
  return 0;
}
