#include "rc/rt/latency_histogram.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <sstream>
#include <utility>

namespace rc::rt {

LatencyHistogram::LatencyHistogram(std::string name, std::int64_t low_nanoseconds,
                                   std::int64_t high_nanoseconds, std::size_t bucket_count)
    : name_(std::move(name)),
      lo_ns_(low_nanoseconds),
      hi_ns_(high_nanoseconds),
      span_ns_(std::max<std::int64_t>(1, high_nanoseconds - low_nanoseconds)),
      buckets_(std::max<std::size_t>(1, bucket_count), 0) {}

void LatencyHistogram::record(std::int64_t value_nanoseconds) noexcept {
  ++count_;
  sum_ += static_cast<double>(value_nanoseconds);
  min_ = std::min(min_, value_nanoseconds);
  max_ = std::max(max_, value_nanoseconds);

  if (value_nanoseconds < lo_ns_) {
    ++underflow_;
    return;
  }
  if (value_nanoseconds > hi_ns_) {
    ++overflow_;
    return;
  }
  const auto bucket_count = static_cast<std::int64_t>(buckets_.size());
  auto bucket_index = (value_nanoseconds - lo_ns_) * bucket_count / span_ns_;
  bucket_index = std::clamp<std::int64_t>(bucket_index, 0, bucket_count - 1);
  ++buckets_[static_cast<std::size_t>(bucket_index)];
}

std::int64_t LatencyHistogram::min_ns() const noexcept { return count_ ? min_ : 0; }
std::int64_t LatencyHistogram::max_ns() const noexcept { return count_ ? max_ : 0; }

double LatencyHistogram::mean_ns() const noexcept {
  return count_ ? sum_ / static_cast<double>(count_) : 0.0;
}

std::int64_t LatencyHistogram::percentile_ns(double fraction) const noexcept {
  if (count_ == 0) {
    return 0;
  }
  const auto target = static_cast<std::uint64_t>(std::ceil(fraction * static_cast<double>(count_)));
  std::uint64_t seen = underflow_;
  if (seen >= target) {
    return min_;  // percentile lies below the histogram range
  }
  const auto bucket_count = static_cast<std::int64_t>(buckets_.size());
  for (std::int64_t bucket = 0; bucket < bucket_count; ++bucket) {
    seen += buckets_[static_cast<std::size_t>(bucket)];
    if (seen >= target) {
      // Report the bucket's upper edge -- pessimistic, which is the right bias
      // for a latency number you will have to defend. But clamp to the observed
      // extremes: an instrument that reports p99.9 above max is not one anyone
      // should trust, however defensible the arithmetic.
      return std::clamp(lo_ns_ + (bucket + 1) * span_ns_ / bucket_count, min_, max_);
    }
  }
  return max_;
}

std::string LatencyHistogram::format_row() const {
  // Columns: name, mean, p99, p99.9, max -- all in microseconds.
  char row[160];
  std::snprintf(row, sizeof(row), "%-18s%9.1fu%9.1fu%9.1fu%9.1fu",
                name_.c_str(),
                mean_ns() / 1000.0,
                static_cast<double>(percentile_ns(0.99)) / 1000.0,
                static_cast<double>(percentile_ns(0.999)) / 1000.0,
                static_cast<double>(max_ns()) / 1000.0);
  return row;
}

std::string LatencyHistogram::format_chart(int width) const {
  std::ostringstream out;
  if (count_ == 0) {
    return "    (no samples)\n";
  }
  const std::uint64_t peak = std::max<std::uint64_t>(
      1, *std::max_element(buckets_.begin(), buckets_.end()));
  const auto bucket_count = static_cast<std::int64_t>(buckets_.size());
  out.setf(std::ios::fixed);
  out.precision(1);
  if (underflow_) {
    out << "    <   below range | " << underflow_ << " samples\n";
  }
  for (std::int64_t bucket = 0; bucket < bucket_count; ++bucket) {
    const std::uint64_t samples = buckets_[static_cast<std::size_t>(bucket)];
    if (samples == 0) {
      continue;  // linear buckets over a wide range are mostly empty; skip them
    }
    const double edge_microseconds = static_cast<double>(lo_ns_ + bucket * span_ns_ / bucket_count) / 1000.0;
    const int bar = static_cast<int>(static_cast<double>(width) *
                                     static_cast<double>(samples) / static_cast<double>(peak));
    out << "    ";
    out.width(10);
    out << edge_microseconds << " us | " << std::string(static_cast<std::size_t>(std::max(0, bar)), '#')
       << std::string(static_cast<std::size_t>(std::max(0, width - bar)), ' ') << ' ' << samples << '\n';
  }
  if (overflow_) {
    out << "    >   above range | " << overflow_ << " samples\n";
  }
  return out.str();
}

}  // namespace rc::rt
