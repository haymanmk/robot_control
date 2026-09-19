#pragma once

/// @file socketcan.hpp
/// CanTransport on a Linux SocketCAN raw socket.
///
/// What open() sets up, and why each piece is there:
///
///   * **Non-blocking socket.** send() and receive() are on the cyclic path
///     and may not wait. A full transmit queue makes send() fail with ENOBUFS
///     rather than block; an empty receive queue makes receive() return false.
///   * **`SO_TIMESTAMPING`** with software receive timestamps, and hardware
///     receive timestamps requested from the adapter with `SIOCSHWTSTAMP`.
///     An adapter without a clock refuses the request and the transport
///     carries on with software timestamps; hardware_timestamps_enabled()
///     says which you got. On `vcan0` you always get software only.
///   * **Error frames enabled** (`CAN_RAW_ERR_FILTER`), so bus-off and
///     error-passive show up in BusStatistics::receive_errors instead of as a
///     silent absence of feedback.
///
/// The kernel's software timestamp is on CLOCK_REALTIME; there is no
/// monotonic option in the socket API. receive() translates it by reading
/// both clocks at that moment and applying the offset, which costs two vDSO
/// reads and no system call. The error is the change in the offset between
/// the kernel stamping the frame and us reading it: with NTP slewing at most
/// 500 ppm, that is under a nanosecond over a whole 2 ms cycle. A step
/// adjustment of the wall clock would corrupt the frames in flight at that
/// instant, and nothing else; the loop must never be timed against the wall
/// clock in the first place ([AGENTS.md](../../../../AGENTS.md)).
///
/// Worst-case cost of the cyclic calls: one `send()` or `recvmsg()` system
/// call. On a stock kernel that is a few microseconds; the drive layer budgets
/// for 14 of them per cycle.

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_control/can/transport.hpp"

namespace robot_control::can {

/// CanTransport over `AF_CAN`/`CAN_RAW` with kernel receive timestamps. One
/// instance per interface per thread pair: send() and receive() are safe to
/// call from one thread while statistics() is read from another, and nothing
/// more.
class SocketCanTransport final : public CanTransport {
 public:
  SocketCanTransport() = default;
  /// Closes the socket if still open.
  ~SocketCanTransport() override;
  SocketCanTransport(const SocketCanTransport&) = delete;
  SocketCanTransport& operator=(const SocketCanTransport&) = delete;

  /// Opens a raw socket bound to @p interface and configures it as the file
  /// comment describes. Non-cyclic.
  [[nodiscard]] bool open(const std::string& interface) override;
  void close() noexcept override;
  [[nodiscard]] bool is_open() const noexcept override { return descriptor >= 0; }
  [[nodiscard]] bool set_filters(const CanFilter* filters, std::size_t count) override;
  [[nodiscard]] bool send(const CanFrame& frame) noexcept override;
  [[nodiscard]] bool receive(CanFrame& frame) noexcept override;
  [[nodiscard]] BusStatistics statistics() const noexcept override;
  [[nodiscard]] const std::string& last_error() const noexcept override { return error; }

  /// True if the adapter accepted the hardware receive-timestamp request.
  [[nodiscard]] bool hardware_timestamps_enabled() const noexcept { return hardware_timestamps; }
  /// Interface name given to open().
  [[nodiscard]] const std::string& interface_name() const noexcept { return interface; }

  /// Blocks up to @p timeout_milliseconds for a frame to become readable.
  /// Non-cyclic: for capture tools and tests, never for the loop, which is
  /// paced by CyclicTask and polls with receive(). Returns true if readable.
  [[nodiscard]] bool wait_readable(int timeout_milliseconds) noexcept;

 private:
  /// Records @p what plus errno's message in error and returns false.
  bool fail(const char* what);

  int descriptor = -1;
  std::string interface;
  std::string error;
  bool hardware_timestamps = false;

  // Written by the cyclic thread only, read by any thread. Relaxed atomics,
  // and single-writer increments (load, add, store) so the hot path is a plain
  // memory write, not a locked read-modify-write.
  std::atomic<std::uint64_t> frames_sent{0};
  std::atomic<std::uint64_t> bytes_sent{0};
  std::atomic<std::uint64_t> frames_received{0};
  std::atomic<std::uint64_t> bytes_received{0};
  std::atomic<std::uint64_t> send_failures{0};
  std::atomic<std::uint64_t> receive_errors{0};
  std::atomic<std::uint64_t> frames_without_kernel_timestamp{0};
  std::atomic<std::uint64_t> minimum_bits_on_wire{0};
  std::atomic<std::uint64_t> maximum_bits_on_wire{0};
};

}  // namespace robot_control::can
