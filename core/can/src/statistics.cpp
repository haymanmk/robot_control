#include "robot_control/can/statistics.hpp"

#include <cstdio>
#include <ctime>
#include <fstream>

namespace robot_control::can {
namespace {

bool read_counter(const std::string& directory, const char* name, std::uint64_t& value) {
  std::ifstream file(directory + name);
  return static_cast<bool>(file >> value);
}

}  // namespace

InterfaceCounters InterfaceCounters::difference(const InterfaceCounters& earlier,
                                                const InterfaceCounters& later) noexcept {
  InterfaceCounters delta;
  delta.valid = earlier.valid && later.valid;
  delta.received_frames = later.received_frames - earlier.received_frames;
  delta.transmitted_frames = later.transmitted_frames - earlier.transmitted_frames;
  delta.received_bytes = later.received_bytes - earlier.received_bytes;
  delta.transmitted_bytes = later.transmitted_bytes - earlier.transmitted_bytes;
  delta.receive_errors = later.receive_errors - earlier.receive_errors;
  delta.transmit_errors = later.transmit_errors - earlier.transmit_errors;
  delta.receive_dropped = later.receive_dropped - earlier.receive_dropped;
  delta.transmit_dropped = later.transmit_dropped - earlier.transmit_dropped;
  delta.read_at_nanoseconds = later.read_at_nanoseconds - earlier.read_at_nanoseconds;
  return delta;
}

InterfaceCounters read_interface_counters(const std::string& interface) {
  InterfaceCounters counters;
  const std::string directory = "/sys/class/net/" + interface + "/statistics/";
  timespec now{};
  clock_gettime(CLOCK_MONOTONIC, &now);
  counters.read_at_nanoseconds =
      static_cast<std::int64_t>(now.tv_sec) * 1'000'000'000LL + static_cast<std::int64_t>(now.tv_nsec);
  counters.valid = read_counter(directory, "rx_packets", counters.received_frames) &&
                   read_counter(directory, "tx_packets", counters.transmitted_frames) &&
                   read_counter(directory, "rx_bytes", counters.received_bytes) &&
                   read_counter(directory, "tx_bytes", counters.transmitted_bytes) &&
                   read_counter(directory, "rx_errors", counters.receive_errors) &&
                   read_counter(directory, "tx_errors", counters.transmit_errors) &&
                   read_counter(directory, "rx_dropped", counters.receive_dropped) &&
                   read_counter(directory, "tx_dropped", counters.transmit_dropped);
  return counters;
}

std::string format_statistics(const BusStatistics& statistics) {
  std::string out;
  char line[96];
  const auto row = [&](const char* name, std::uint64_t value) {
    if (value != 0) {
      std::snprintf(line, sizeof(line), "  %-32s %12llu\n", name, static_cast<unsigned long long>(value));
      out += line;
    }
  };
  row("frames sent", statistics.frames_sent);
  row("bytes sent", statistics.bytes_sent);
  row("frames received", statistics.frames_received);
  row("bytes received", statistics.bytes_received);
  row("send failures (queue full)", statistics.send_failures);
  row("error frames received", statistics.receive_errors);
  row("frames without kernel timestamp", statistics.frames_without_kernel_timestamp);
  row("bits on wire, minimum", statistics.minimum_bits_on_wire);
  row("bits on wire, maximum", statistics.maximum_bits_on_wire);
  if (out.empty()) {
    out = "  (no frames)\n";
  }
  return out;
}

}  // namespace robot_control::can
