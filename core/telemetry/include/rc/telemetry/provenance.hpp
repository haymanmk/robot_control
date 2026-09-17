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

struct Provenance {
  // Build
  std::string git_sha;
  std::string git_dirty;     ///< "clean" or "dirty" — a dirty build is not reproducible
  std::string build_type;
  std::string compiler;
  std::string build_time;

  // Kernel and machine
  std::string hostname;
  std::string kernel_release;
  std::string kernel_version;
  std::string preempt_model;  ///< "PREEMPT_RT", "PREEMPT_DYNAMIC", "none", ...
  bool realtime_kernel = false;
  std::string cpu_model;
  unsigned cpu_count = 0;
  std::string cpu_governor;
  std::string isolated_cpus;  ///< /sys/devices/system/cpu/isolated
  std::string nohz_full;
  std::string memlock_limit;  ///< "soft/hard" in MiB, or "unlimited"
  std::string rtprio_limit;   ///< RLIMIT_RTPRIO hard limit

  // GPU (ADR-0007: the driver is mandatory and it is a latency source)
  std::string nvidia_driver;  ///< empty if not present
  bool gpu_workload_running = false;  ///< caller asserts; we cannot infer it honestly

  // Run
  std::string wall_clock;     ///< ISO-8601 UTC, for correlating with external logs
  std::string rt_notes;       ///< what apply_realtime() actually granted
  std::string can_interface;
  unsigned can_bitrate = 0;
  double control_rate_hz = 0.0;
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
