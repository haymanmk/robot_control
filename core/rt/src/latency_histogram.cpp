#include "rc/rt/latency_histogram.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <sstream>
#include <utility>

namespace rc::rt {

LatencyHistogram::LatencyHistogram(std::string name, std::int64_t lo_ns,
                                   std::int64_t hi_ns, std::size_t bucket_count)
    : name_(std::move(name)),
      lo_ns_(lo_ns),
      hi_ns_(hi_ns),
      span_ns_(std::max<std::int64_t>(1, hi_ns - lo_ns)),
      buckets_(std::max<std::size_t>(1, bucket_count), 0) {}

void LatencyHistogram::record(std::int64_t value_ns) noexcept {
  ++count_;
  sum_ += static_cast<double>(value_ns);
  min_ = std::min(min_, value_ns);
  max_ = std::max(max_, value_ns);

  if (value_ns < lo_ns_) {
    ++underflow_;
    return;
  }
  if (value_ns > hi_ns_) {
    ++overflow_;
    return;
  }
  const auto n = static_cast<std::int64_t>(buckets_.size());
  auto idx = (value_ns - lo_ns_) * n / span_ns_;
  idx = std::clamp<std::int64_t>(idx, 0, n - 1);
  ++buckets_[static_cast<std::size_t>(idx)];
}

std::int64_t LatencyHistogram::min_ns() const noexcept { return count_ ? min_ : 0; }
std::int64_t LatencyHistogram::max_ns() const noexcept { return count_ ? max_ : 0; }

double LatencyHistogram::mean_ns() const noexcept {
  return count_ ? sum_ / static_cast<double>(count_) : 0.0;
}

std::int64_t LatencyHistogram::percentile_ns(double p) const noexcept {
  if (count_ == 0) {
    return 0;
  }
  const auto target = static_cast<std::uint64_t>(std::ceil(p * static_cast<double>(count_)));
  std::uint64_t seen = underflow_;
  if (seen >= target) {
    return min_;  // percentile lies below the histogram range
  }
  const auto n = static_cast<std::int64_t>(buckets_.size());
  for (std::int64_t i = 0; i < n; ++i) {
    seen += buckets_[static_cast<std::size_t>(i)];
    if (seen >= target) {
      return lo_ns_ + (i + 1) * span_ns_ / n;  // upper edge: report pessimistically
    }
  }
  return max_;
}

std::string LatencyHistogram::format_row() const {
  // Columns: name, mean, p99, p99.9, max -- all in microseconds.
  char buf[160];
  std::snprintf(buf, sizeof(buf), "%-18s%9.1fu%9.1fu%9.1fu%9.1fu",
                name_.c_str(),
                mean_ns() / 1000.0,
                static_cast<double>(percentile_ns(0.99)) / 1000.0,
                static_cast<double>(percentile_ns(0.999)) / 1000.0,
                static_cast<double>(max_ns()) / 1000.0);
  return buf;
}

std::string LatencyHistogram::format_chart(int width) const {
  std::ostringstream os;
  if (count_ == 0) {
    return "    (no samples)\n";
  }
  const std::uint64_t peak = std::max<std::uint64_t>(
      1, *std::max_element(buckets_.begin(), buckets_.end()));
  const auto n = static_cast<std::int64_t>(buckets_.size());
  os.setf(std::ios::fixed);
  os.precision(1);
  if (underflow_) {
    os << "    <   below range | " << underflow_ << " samples\n";
  }
  for (std::int64_t i = 0; i < n; ++i) {
    const std::uint64_t c = buckets_[static_cast<std::size_t>(i)];
    if (c == 0) {
      continue;  // linear buckets over a wide range are mostly empty; skip them
    }
    const double edge_us = static_cast<double>(lo_ns_ + i * span_ns_ / n) / 1000.0;
    const int bar = static_cast<int>(static_cast<double>(width) *
                                     static_cast<double>(c) / static_cast<double>(peak));
    os << "    ";
    os.width(10);
    os << edge_us << " us | " << std::string(static_cast<std::size_t>(std::max(0, bar)), '#')
       << std::string(static_cast<std::size_t>(std::max(0, width - bar)), ' ') << ' ' << c << '\n';
  }
  if (overflow_) {
    os << "    >   above range | " << overflow_ << " samples\n";
  }
  return os.str();
}

}  // namespace rc::rt
