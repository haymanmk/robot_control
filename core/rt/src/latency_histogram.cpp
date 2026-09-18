#include "rc/rt/latency_histogram.hpp"

#include <algorithm>
#include <cstdio>
#include <cmath>
#include <sstream>
#include <utility>

namespace rc::rt {

LatencyHistogram::LatencyHistogram(std::string name, std::int64_t low_nanoseconds_,
                                   std::int64_t high_nanoseconds_, std::size_t bucket_count)
    : label(std::move(name)),
      low_nanoseconds(low_nanoseconds_),
      high_nanoseconds(high_nanoseconds_),
      span_nanoseconds(std::max<std::int64_t>(1, high_nanoseconds_ - low_nanoseconds_)),
      buckets(std::max<std::size_t>(1, bucket_count), 0) {}

void LatencyHistogram::record(std::int64_t value_nanoseconds) noexcept {
  ++sample_count;
  sum += static_cast<double>(value_nanoseconds);
  minimum = std::min(minimum, value_nanoseconds);
  maximum = std::max(maximum, value_nanoseconds);

  if (value_nanoseconds < low_nanoseconds) {
    ++underflow;
    return;
  }
  if (value_nanoseconds > high_nanoseconds) {
    ++overflow;
    return;
  }
  const auto bucket_count = static_cast<std::int64_t>(buckets.size());
  auto bucket_index = (value_nanoseconds - low_nanoseconds) * bucket_count / span_nanoseconds;
  bucket_index = std::clamp<std::int64_t>(bucket_index, 0, bucket_count - 1);
  ++buckets[static_cast<std::size_t>(bucket_index)];
}

std::int64_t LatencyHistogram::minimum_nanoseconds() const noexcept { return sample_count ? minimum : 0; }
std::int64_t LatencyHistogram::maximum_nanoseconds() const noexcept { return sample_count ? maximum : 0; }

double LatencyHistogram::mean_nanoseconds() const noexcept {
  return sample_count ? sum / static_cast<double>(sample_count) : 0.0;
}

std::int64_t LatencyHistogram::percentile_nanoseconds(double fraction) const noexcept {
  if (sample_count == 0) {
    return 0;
  }
  const auto target = static_cast<std::uint64_t>(std::ceil(fraction * static_cast<double>(sample_count)));
  std::uint64_t seen = underflow;
  if (seen >= target) {
    return minimum;  // percentile lies below the histogram range
  }
  const auto bucket_count = static_cast<std::int64_t>(buckets.size());
  for (std::int64_t bucket = 0; bucket < bucket_count; ++bucket) {
    seen += buckets[static_cast<std::size_t>(bucket)];
    if (seen >= target) {
      // Report the bucket's upper edge -- pessimistic, which is the right bias
      // for a latency number you will have to defend. But clamp to the observed
      // extremes: an instrument that reports p99.9 above max is not one anyone
      // should trust, however defensible the arithmetic.
      return std::clamp(low_nanoseconds + (bucket + 1) * span_nanoseconds / bucket_count, minimum, maximum);
    }
  }
  return maximum;
}

std::string LatencyHistogram::format_row() const {
  // Columns: name, mean, p99, p99.9, max -- all in microseconds.
  char row[160];
  std::snprintf(row, sizeof(row), "%-18s%9.1fu%9.1fu%9.1fu%9.1fu",
                label.c_str(),
                mean_nanoseconds() / 1000.0,
                static_cast<double>(percentile_nanoseconds(0.99)) / 1000.0,
                static_cast<double>(percentile_nanoseconds(0.999)) / 1000.0,
                static_cast<double>(maximum_nanoseconds()) / 1000.0);
  return row;
}

std::string LatencyHistogram::format_chart(int width) const {
  std::ostringstream out;
  if (sample_count == 0) {
    return "    (no samples)\n";
  }
  const std::uint64_t peak = std::max<std::uint64_t>(
      1, *std::max_element(buckets.begin(), buckets.end()));
  const auto bucket_count = static_cast<std::int64_t>(buckets.size());
  out.setf(std::ios::fixed);
  out.precision(1);
  if (underflow) {
    out << "    <   below range | " << underflow << " samples\n";
  }
  for (std::int64_t bucket = 0; bucket < bucket_count; ++bucket) {
    const std::uint64_t samples = buckets[static_cast<std::size_t>(bucket)];
    if (samples == 0) {
      continue;  // linear buckets over a wide range are mostly empty; skip them
    }
    const double edge_microseconds =
        static_cast<double>(low_nanoseconds + bucket * span_nanoseconds / bucket_count) / 1000.0;
    const int bar = static_cast<int>(static_cast<double>(width) *
                                     static_cast<double>(samples) / static_cast<double>(peak));
    out << "    ";
    out.width(10);
    out << edge_microseconds << " us | " << std::string(static_cast<std::size_t>(std::max(0, bar)), '#')
       << std::string(static_cast<std::size_t>(std::max(0, width - bar)), ' ') << ' ' << samples << '\n';
  }
  if (overflow) {
    out << "    >   above range | " << overflow << " samples\n";
  }
  return out.str();
}

}  // namespace rc::rt
