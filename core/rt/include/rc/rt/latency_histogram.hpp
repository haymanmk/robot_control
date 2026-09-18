#pragma once

/// @file latency_histogram.hpp
/// Allocation-free distribution recorder for use inside the cyclic path.
///
/// Why a histogram and not a std::vector<sample>: a 500 Hz loop running for an
/// hour produces 1.8 M samples. Pushing them into a growing vector means a
/// realloc -- an unbounded, unpredictable pause -- at an arbitrary cycle. The
/// first rule of real-time code is that the cyclic path does not allocate, and
/// "but it only reallocs occasionally" is precisely the kind of occasional that
/// shows up as your worst-case number.
///
/// So: buckets are allocated once at construction, record() is O(1) and
/// lock-free, and percentiles are reconstructed from the buckets afterwards.
/// The cost is resolution -- a reported percentile is a bucket edge, not an
/// exact sample. min and max are tracked exactly, because those are the two
/// numbers you will actually be asked to defend.

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace rc::rt {

/// Allocation-free distribution recorder for the cyclic path. Buckets are
/// allocated once at construction, record() is O(1) and lock-free, and
/// percentiles are reconstructed from the buckets afterwards. min and max are
/// tracked exactly; percentiles have bucket resolution.
class LatencyHistogram {
 public:
  /// Linear buckets spanning [low_nanoseconds, high_nanoseconds]. Samples
  /// outside that range land in dedicated underflow/overflow counters and still
  /// update min/max exactly.
  LatencyHistogram(std::string name, std::int64_t low_nanoseconds_,
                   std::int64_t high_nanoseconds_, std::size_t bucket_count = 64);

  /// Hot path. No allocation, no locks, no syscalls.
  void record(std::int64_t value_nanoseconds) noexcept;

  /// Label given at construction; appears in format_row() and format_chart().
  [[nodiscard]] const std::string& name() const noexcept { return label; }
  /// Samples recorded, including out-of-range ones.
  [[nodiscard]] std::uint64_t count() const noexcept { return sample_count; }
  /// Exact minimum sample; 0 if nothing was recorded.
  [[nodiscard]] std::int64_t min_ns() const noexcept;
  /// Exact maximum sample; 0 if nothing was recorded.
  [[nodiscard]] std::int64_t max_ns() const noexcept;
  /// Exact arithmetic mean; 0 if nothing was recorded.
  [[nodiscard]] double mean_ns() const noexcept;

  /// Bucket-resolution percentile, e.g. percentile_ns(0.999).
  /// Quote p99.9 and max, never the mean: a control loop is not harmed by its
  /// average cycle, it is harmed by the worst one.
  [[nodiscard]] std::int64_t percentile_ns(double fraction) const noexcept;

  /// Number of samples that fell outside [low_nanoseconds, high_nanoseconds].
  /// Non-zero means the histogram range was chosen badly and percentiles are
  /// clipped -- always check this before believing the output.
  [[nodiscard]] std::uint64_t out_of_range() const noexcept { return underflow + overflow; }

  /// One aligned line: name, mean, p99, p99.9, max (microseconds).
  [[nodiscard]] std::string format_row() const;

  /// ASCII bar chart of the distribution.
  [[nodiscard]] std::string format_chart(int width = 52) const;

 private:
  std::string label;
  std::int64_t low_nanoseconds;
  std::int64_t high_nanoseconds;
  std::int64_t span_nanoseconds;
  std::vector<std::uint64_t> buckets;
  std::uint64_t underflow = 0;
  std::uint64_t overflow = 0;
  std::uint64_t sample_count = 0;
  std::int64_t minimum = INT64_MAX;
  std::int64_t maximum = INT64_MIN;
  double sum = 0.0;
};

}  // namespace rc::rt
