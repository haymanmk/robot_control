#pragma once

/// @file frame.hpp
/// One classic CAN frame, as the rest of the control core sees it.
///
/// This is the unit that crosses every boundary in `core/can`: it comes out of
/// the socket, goes into rings, and is copied into telemetry unchanged. So it is
/// a plain 32-byte struct with no pointers, trivially copyable, and its layout
/// is asserted below. The kernel's `struct can_frame` is not used outside
/// `socketcan.cpp`, because it carries flag bits inside the identifier field
/// and a layout that has changed between kernel versions.
///
/// Two timestamps, both filled by the transport on receive and both zero on a
/// frame we built ourselves:
///
///   * `receive_timestamp_nanoseconds` is the kernel's software receive
///     timestamp, translated to CLOCK_MONOTONIC so it can be compared with the
///     cycle's wake time. It marks when the driver handed the frame to the
///     network stack, not when our thread got around to reading it. That is the
///     whole reason for `SO_TIMESTAMPING`
///     ([ADR-0004](../../../../docs/adr/0004-system-decomposition.md) §4): a
///     timestamp taken in user space measures our scheduler, not the bus.
///   * `hardware_timestamp_nanoseconds` is the adapter's own clock, if the
///     adapter has one. It is on no system time base, so only differences
///     between two hardware timestamps mean anything.
///
/// `bits_on_wire()` gives the bus time a frame costs. It is a range, not a
/// number: the exact count depends on bit stuffing, which depends on the data
/// and the CRC. The range is what Lab 02 needs to turn a frame log into a bus
/// utilisation figure with honest error bars.

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <type_traits>

namespace robot_control::can {

/// Bits of a CanFrame::flags byte.
enum FrameFlag : std::uint8_t {
  frame_extended = 1u << 0,  ///< 29-bit identifier (CAN 2.0B) rather than 11-bit
  frame_remote   = 1u << 1,  ///< remote transmission request: no data, asks for a reply
  frame_error    = 1u << 2,  ///< not a bus frame: the driver reporting a bus error
};

/// Largest classic CAN payload. CAN FD goes to 64 and is deliberately not
/// supported until Lab 02 shows the arm's bus needs it.
inline constexpr std::uint8_t max_data_length = 8;
/// Identifier bits of an 11-bit (standard) frame.
inline constexpr std::uint32_t standard_identifier_mask = 0x7FFu;
/// Identifier bits of a 29-bit (extended) frame.
inline constexpr std::uint32_t extended_identifier_mask = 0x1FFFFFFFu;

/// One classic CAN frame with the kernel's receive timestamps. 32 bytes,
/// trivially copyable, no pointers; safe in rings and telemetry.
struct CanFrame {
  std::uint32_t identifier = 0;    ///< 11- or 29-bit identifier, no flag bits mixed in
  std::uint8_t flags = 0;          ///< bitwise OR of FrameFlag
  std::uint8_t length = 0;         ///< bytes of data that are meaningful, 0..8
  std::uint8_t reserved[2] = {};   ///< explicit padding so the layout is stable and comparable
  std::uint8_t data[max_data_length] = {};  ///< payload; bytes past length are zero
  std::int64_t receive_timestamp_nanoseconds = 0;   ///< kernel receive time on CLOCK_MONOTONIC; 0 if not received
  std::int64_t hardware_timestamp_nanoseconds = 0;  ///< adapter clock; 0 if the adapter has none

  /// True for a 29-bit identifier.
  [[nodiscard]] constexpr bool extended() const noexcept { return (flags & frame_extended) != 0; }
  /// True for a remote transmission request.
  [[nodiscard]] constexpr bool remote() const noexcept { return (flags & frame_remote) != 0; }
  /// True when this is the driver reporting an error, not a frame from the bus.
  [[nodiscard]] constexpr bool error() const noexcept { return (flags & frame_error) != 0; }
};

static_assert(sizeof(CanFrame) == 32, "CanFrame is copied into rings and telemetry; its size is a budget");
static_assert(std::is_trivially_copyable_v<CanFrame>, "CanFrame must be memcpy-safe");
static_assert(std::is_standard_layout_v<CanFrame>, "CanFrame must have one layout in every process");

/// Builds a data frame. @p length is clamped to max_data_length; the copy of
/// @p data is bounded by it. Usable on the cyclic path: no allocation, O(8).
[[nodiscard]] inline CanFrame make_data_frame(std::uint32_t identifier, const std::uint8_t* data,
                                              std::uint8_t length, bool extended) noexcept {
  CanFrame frame;
  frame.identifier = identifier & (extended ? extended_identifier_mask : standard_identifier_mask);
  frame.flags = extended ? frame_extended : 0;
  frame.length = length > max_data_length ? max_data_length : length;
  if (data != nullptr && frame.length > 0) {
    std::memcpy(frame.data, data, frame.length);
  }
  return frame;
}

/// Bus time a frame occupies, in bit times, as a range.
///
/// `minimum` assumes no stuff bits; `maximum` assumes the worst case, one stuff
/// bit per four bits of the stuffed region (start-of-frame through CRC). Both
/// include the 3-bit interframe space, because no other frame can start during
/// it, so it is bus time the frame consumed. Divide by the bit rate for seconds.
struct WireBits {
  std::uint32_t minimum = 0;  ///< no stuff bits, plus interframe space
  std::uint32_t maximum = 0;  ///< worst-case stuffing, plus interframe space
};

/// Bit-time cost of @p frame on a classic CAN bus. An error frame costs 0 here:
/// it is the driver's report, not something we can size from user space.
/// Worked example, 8 data bytes: standard 111..135 bits, extended 131..160.
/// That second range is why "14 frames at 500 Hz on 1 Mbit/s" is not a
/// comfortable 90% but somewhere between 92% and 112%.
[[nodiscard]] constexpr WireBits bits_on_wire(const CanFrame& frame) noexcept {
  if (frame.error()) {
    return {};
  }
  const std::uint32_t data_bits = frame.remote() ? 0u : 8u * static_cast<std::uint32_t>(frame.length);
  // Fixed fields. Standard: SOF 1, ID 11, RTR 1, IDE 1, r0 1, DLC 4, CRC 15,
  // CRC delimiter 1, ACK 1, ACK delimiter 1, EOF 7 = 44. Extended adds SRR 1,
  // 18 more ID bits and r1 = 64. The stuffed region stops after the CRC field.
  const std::uint32_t fixed_bits = frame.extended() ? 64u : 44u;
  const std::uint32_t stuffed_region = (frame.extended() ? 54u : 34u) + data_bits;
  constexpr std::uint32_t interframe_space = 3;
  const std::uint32_t minimum = fixed_bits + data_bits + interframe_space;
  const std::uint32_t maximum_stuff_bits = (stuffed_region - 1u) / 4u;
  return {minimum, minimum + maximum_stuff_bits};
}

}  // namespace robot_control::can
