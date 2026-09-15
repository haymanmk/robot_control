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

class LatencyHistogram {
 public:
  /// Linear buckets spanning [lo_ns, hi_ns]. Samples outside that range land in
  /// dedicated underflow/overflow counters and still update min/max exactly.
  LatencyHistogram(std::string name, std::int64_t lo_ns, std::int64_t hi_ns,
                   std::size_t bucket_count = 64);

  /// Hot path. No allocation, no locks, no syscalls.
  void record(std::int64_t value_ns) noexcept;

  [[nodiscard]] const std::string& name() const noexcept { return name_; }
  [[nodiscard]] std::uint64_t count() const noexcept { return count_; }
  [[nodiscard]] std::int64_t min_ns() const noexcept;
  [[nodiscard]] std::int64_t max_ns() const noexcept;
  [[nodiscard]] double mean_ns() const noexcept;

  /// Bucket-resolution percentile, e.g. percentile_ns(0.999).
  /// Quote p99.9 and max, never the mean: a control loop is not harmed by its
  /// average cycle, it is harmed by the worst one.
  [[nodiscard]] std::int64_t percentile_ns(double p) const noexcept;

  /// Number of samples that fell outside [lo_ns, hi_ns]. Non-zero means the
  /// histogram range was chosen badly and percentiles are clipped -- always
  /// check this before believing the output.
  [[nodiscard]] std::uint64_t out_of_range() const noexcept { return underflow_ + overflow_; }

  /// One aligned line: name, mean, p99, p99.9, max (microseconds).
  [[nodiscard]] std::string format_row() const;

  /// ASCII bar chart of the distribution.
  [[nodiscard]] std::string format_chart(int width = 52) const;

 private:
  std::string name_;
  std::int64_t lo_ns_;
  std::int64_t hi_ns_;
  std::int64_t span_ns_;
  std::vector<std::uint64_t> buckets_;
  std::uint64_t underflow_ = 0;
  std::uint64_t overflow_ = 0;
  std::uint64_t count_ = 0;
  std::int64_t min_ = INT64_MAX;
  std::int64_t max_ = INT64_MIN;
  double sum_ = 0.0;
};

}  // namespace rc::rt
