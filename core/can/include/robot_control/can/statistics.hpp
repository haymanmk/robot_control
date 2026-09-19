#pragma once

/// @file statistics.hpp
/// What the bus is doing, counted two ways.
///
/// `BusStatistics` is what *our socket* saw: frames and bytes each way, sends
/// the driver refused, error frames, and the bit-time range those frames cost.
/// `InterfaceCounters` is what *the driver* saw, read from
/// `/sys/class/net/<interface>/statistics/`. They disagree in useful ways: the
/// driver counts every frame on the bus, ours counts only what passed the
/// socket's filters, and the driver's `rx_dropped` says whether the kernel ran
/// out of buffer before we drained it.
///
/// Utilisation is derived, never measured directly: classic CAN adapters do
/// not report bus load. It is bits on the wire over an interval divided by the
/// bit rate, and because stuffing is data-dependent it comes out as a range.
/// Lab 02 compares this range with the 90% estimate in
/// [ADR-0002](../../../../docs/adr/0002-target-platform-rebot-b601-rs.md).

#include <cstdint>
#include <string>

namespace robot_control::can {

/// Counters kept by a CanTransport. A snapshot: the fields are read one at a
/// time from counters the cyclic thread is still updating, so they may be from
/// slightly different instants. Fine for a report; do not compute differences
/// between fields of one snapshot and expect them to be exact.
struct BusStatistics {
  std::uint64_t frames_sent = 0;      ///< frames the driver accepted for transmission
  std::uint64_t bytes_sent = 0;       ///< payload bytes in those frames
  std::uint64_t frames_received = 0;  ///< frames read from the socket, error frames excluded
  std::uint64_t bytes_received = 0;   ///< payload bytes in those frames
  /// send() returned false. On SocketCAN that is usually ENOBUFS: the driver's
  /// transmit queue is full, which is the bus telling you it is saturated.
  std::uint64_t send_failures = 0;
  /// Error frames delivered by the driver (bus-off, error-passive, lost
  /// arbitration, ...). Zero on a healthy bus.
  std::uint64_t receive_errors = 0;
  /// Frames that came without a kernel timestamp and were stamped in user
  /// space instead. Should be zero; if not, the timestamps measure our
  /// scheduler and Lab 02's latency numbers are not to be believed.
  std::uint64_t frames_without_kernel_timestamp = 0;
  /// Bit-time cost of every frame sent or received, from bits_on_wire().
  std::uint64_t minimum_bits_on_wire = 0;
  std::uint64_t maximum_bits_on_wire = 0;  ///< see minimum_bits_on_wire
};

/// Fraction of bus time in use over an interval, as a range because stuffing
/// is data-dependent. Values above 1.0 mean the frames could not have fit and
/// some were queued past the interval.
struct Utilisation {
  double minimum = 0.0;  ///< assuming no stuff bits
  double maximum = 0.0;  ///< assuming worst-case stuffing
};

/// Utilisation from a bit count over @p interval_nanoseconds at @p bit_rate
/// bit/s. Returns zeros for a zero interval or bit rate.
[[nodiscard]] constexpr Utilisation bus_utilisation(std::uint64_t minimum_bits,
                                                    std::uint64_t maximum_bits,
                                                    std::int64_t interval_nanoseconds,
                                                    std::uint32_t bit_rate) noexcept {
  if (interval_nanoseconds <= 0 || bit_rate == 0) {
    return {};
  }
  const double capacity_bits =
      static_cast<double>(bit_rate) * static_cast<double>(interval_nanoseconds) / 1e9;
  return {static_cast<double>(minimum_bits) / capacity_bits,
          static_cast<double>(maximum_bits) / capacity_bits};
}

/// The driver's own counters from sysfs. Frames are what the kernel calls
/// packets. `valid` is false when the interface or any counter file is missing.
struct InterfaceCounters {
  bool valid = false;                     ///< every counter file was read
  std::uint64_t received_frames = 0;      ///< rx_packets
  std::uint64_t transmitted_frames = 0;   ///< tx_packets
  std::uint64_t received_bytes = 0;       ///< rx_bytes
  std::uint64_t transmitted_bytes = 0;    ///< tx_bytes
  std::uint64_t receive_errors = 0;       ///< rx_errors
  std::uint64_t transmit_errors = 0;      ///< tx_errors
  std::uint64_t receive_dropped = 0;      ///< rx_dropped: the kernel had nowhere to put a frame
  std::uint64_t transmit_dropped = 0;     ///< tx_dropped
  /// CLOCK_MONOTONIC when the counters were read, so two snapshots give a rate.
  std::int64_t read_at_nanoseconds = 0;

  /// Counter-by-counter difference @p later minus @p earlier, with `valid` only
  /// if both are. The interval is in read_at_nanoseconds.
  [[nodiscard]] static InterfaceCounters difference(const InterfaceCounters& earlier,
                                                    const InterfaceCounters& later) noexcept;
};

/// Reads `/sys/class/net/<interface>/statistics/`. Does file I/O, so never on
/// the cyclic path. Works for any network interface, which is how it is tested
/// without CAN hardware.
[[nodiscard]] InterfaceCounters read_interface_counters(const std::string& interface);

/// One line per non-zero counter, for a terminal report.
[[nodiscard]] std::string format_statistics(const BusStatistics& statistics);

}  // namespace robot_control::can
