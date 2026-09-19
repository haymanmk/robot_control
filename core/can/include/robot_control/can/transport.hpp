#pragma once

/// @file transport.hpp
/// The interface between the control core and a CAN bus.
///
/// Why an interface and not just the socket class: the drive layer above will
/// be tested against a transport that replays a log or simulates seven motors,
/// and the cyclic loop must not know which one it has. The cost is one
/// virtual call per frame, which is bounded and fine.
///
/// The methods split into two groups, and the split is the point:
///
///   * **Non-cyclic:** open(), close(), set_filters(), statistics(),
///     last_error(). They allocate, block and do file I/O. Call them from the
///     thread that sets the loop up, before it starts.
///   * **Cyclic:** send() and receive(). `noexcept`, never block, never
///     allocate, and cost one system call each. That system call is the one
///     I/O the cyclic-path rules allow, because it *is* the fieldbus
///     ([ADR-0003](../../../../docs/adr/0003-cpp-control-core-and-layering.md)).
///
/// receive() returns one frame at a time and false when the queue is empty.
/// The loop drains with `while (transport.receive(frame)) { ... }` and bounds
/// the drain itself; a transport does not know how many frames a cycle may
/// afford to process.

#include <cstddef>
#include <cstdint>
#include <string>

#include "robot_control/can/frame.hpp"
#include "robot_control/can/statistics.hpp"

namespace robot_control::can {

/// Accept a received frame when `(frame.identifier & mask) == (identifier & mask)`.
/// Applies to the identifier bits only; standard and extended frames both pass
/// a filter they match. A frame is delivered if it matches any filter set.
struct CanFilter {
  std::uint32_t identifier = 0;  ///< identifier bits to match, after masking
  std::uint32_t mask = 0;        ///< which identifier bits matter; 0 accepts everything
};

/// Abstract CAN bus. See the file comment for which methods are cyclic.
class CanTransport {
 public:
  virtual ~CanTransport() = default;

  /// Opens the bus on @p interface (for SocketCAN, a network interface name
  /// such as "can0"). Non-cyclic. On failure returns false and last_error()
  /// says why.
  [[nodiscard]] virtual bool open(const std::string& interface) = 0;
  /// Closes the bus. Safe to call when not open, and from the destructor.
  virtual void close() noexcept = 0;
  /// True between a successful open() and close().
  [[nodiscard]] virtual bool is_open() const noexcept = 0;

  /// Replaces the receive filters with @p count entries from @p filters. Zero
  /// entries restores the default, which accepts everything. Non-cyclic.
  [[nodiscard]] virtual bool set_filters(const CanFilter* filters, std::size_t count) = 0;

  /// Hands @p frame to the driver without waiting. Cyclic: noexcept, one
  /// system call, no allocation. Returns false and counts a send failure when
  /// the driver refuses, typically because its transmit queue is full.
  [[nodiscard]] virtual bool send(const CanFrame& frame) noexcept = 0;

  /// Takes the oldest queued frame into @p frame without waiting. Cyclic:
  /// noexcept, one system call, no allocation. Returns false when nothing is
  /// queued. Error frames are delivered too, with CanFrame::error() set, so a
  /// caller decodes only after checking that.
  [[nodiscard]] virtual bool receive(CanFrame& frame) noexcept = 0;

  /// Snapshot of the counters. Non-cyclic in intent (a report), though it does
  /// not block or allocate.
  [[nodiscard]] virtual BusStatistics statistics() const noexcept = 0;

  /// Why the last non-cyclic call failed; empty if it did not.
  [[nodiscard]] virtual const std::string& last_error() const noexcept = 0;
};

}  // namespace robot_control::can
