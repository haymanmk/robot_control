#pragma once

/// @file provenance.hpp
/// What machine, what kernel, what build, under what load.
///
/// [ADR-0004](../../../../docs/adr/0004-system-decomposition.md) §3: a number
/// without its machine is not a measurement. Six months from now "p99.9 was
/// 40 us" is worthless unless you can say which kernel, which governor, whether
/// isolation was on, and — per
/// [ADR-0007](../../../../docs/adr/0007-rt-platform-on-a-cuda-laptop.md) —
/// whether CUDA inference was running while you measured.
///
/// Comparison across time is the entire point of a regression suite, and
/// comparison needs provenance.

#include <string>

namespace rc::telemetry {

/// Everything about the build, machine and kernel that can change a timing
/// result. Recorded with every run so a number can be reproduced, or
/// explained when it cannot be.
struct Provenance {
  // Build
  std::string git_sha;       ///< commit the binary was built from
  std::string git_dirty;     ///< "clean" or "dirty" — a dirty build is not reproducible
  std::string build_type;    ///< CMAKE_BUILD_TYPE, e.g. "Release"
  std::string compiler;      ///< compiler id and version
  std::string build_time;    ///< when the binary was built

  // Kernel and machine
  std::string hostname;        ///< uname -n
  std::string kernel_release;  ///< uname -r
  std::string kernel_version;  ///< uname -v
  std::string preempt_model;  ///< "PREEMPT_RT", "PREEMPT_DYNAMIC", "none", ...
  bool realtime_kernel = false;  ///< /sys/kernel/realtime says so, or preempt_model is PREEMPT_RT
  std::string cpu_model;      ///< from /proc/cpuinfo
  unsigned cpu_count = 0;     ///< online CPUs
  std::string cpu_governor;   ///< cpufreq governor; anything but "performance" adds jitter
  std::string isolated_cpus;  ///< /sys/devices/system/cpu/isolated
  std::string nohz_full;      ///< /sys/devices/system/cpu/nohz_full
  std::string memlock_limit;  ///< RLIMIT_MEMLOCK "soft/hard" in MiB, or "unlimited"
  std::string realtime_priority_limit;  ///< RLIMIT_RTPRIO hard limit: highest SCHED_FIFO priority allowed

  // GPU (ADR-0007: the driver is mandatory and it is a latency source)
  std::string nvidia_driver;  ///< empty if not present
  bool gpu_workload_running = false;  ///< caller asserts; we cannot infer it honestly

  // Run
  std::string wall_clock;     ///< ISO-8601 UTC, for correlating with external logs
  std::string realtime_notes;  ///< what apply_realtime() actually granted
  std::string can_interface;  ///< e.g. "can0"; empty if the run did not use the bus
  unsigned can_bitrate = 0;   ///< bit/s
  double control_rate_hertz = 0.0;  ///< nominal loop rate
  std::string label;          ///< free-text: what this run was testing

  /// Collect everything discoverable. Does file I/O — never call from the
  /// cyclic path.
  [[nodiscard]] static Provenance collect();

  /// JSON sidecar, written next to the telemetry file.
  [[nodiscard]] std::string to_json() const;

  /// Short human summary for a terminal header.
  [[nodiscard]] std::string to_summary() const;

  /// True if this run is fit to be used as a regression baseline. A dirty tree
  /// or an unknown kernel makes a baseline that cannot be reproduced, which is
  /// worse than no baseline at all because it looks authoritative.
  [[nodiscard]] bool suitable_as_baseline(std::string& why_not) const;
};

}  // namespace rc::telemetry
