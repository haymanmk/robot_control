#pragma once

/// @file record.hpp
/// One fixed-size record per control cycle — the flight recorder's unit.
///
/// [ADR-0004](../../../../docs/adr/0004-system-decomposition.md) makes telemetry
/// always-on, so the record's *size* is a design constraint, not an afterthought:
/// it is memcpy'd inside the cyclic path and it sets the disk rate.
///
///     256 B x 500 Hz = 128 KB/s = 7.7 MB per minute
///
/// Two decisions follow from that budget:
///
///   * **Timestamps are int64 nanoseconds**, never double. A double holds ~15-16
///     significant digits; CLOCK_MONOTONIC nanoseconds since boot already needs
///     ~15, so after a few weeks of uptime a double silently loses sub-microsecond
///     resolution — in the one field where sub-microsecond resolution is the
///     entire point.
///   * **Joint data is float**, not double. A float carries ~7 significant
///     digits, so a joint angle within +/-6.28 rad resolves to ~1e-6 rad
///     (0.00006 deg). The encoders are nowhere near that, so double would be
///     storing noise at twice the price.

#include <cstdint>

namespace robot_control::telemetry {

/// Seven on the B601-RS (6 arm + gripper); rounded up so the record stays a
/// round 256 bytes and one more joint does not change the wire format.
inline constexpr unsigned max_joints = 8;

/// Per-cycle condition flags. Bitfield so a cycle can report several at once.
enum RecordFlag : std::uint32_t {
  flag_none            = 0u,
  flag_overrun         = 1u << 0,  ///< period error beyond the overrun threshold
  flag_missed_deadline = 1u << 1,  ///< body() ran longer than the period
  flag_stale_feedback  = 1u << 2,  ///< a joint's feedback is older than allowed
  flag_command_dropped = 1u << 3,  ///< command ring was full; a command was lost
  flag_telemetry_lost  = 1u << 4,  ///< telemetry ring was full; records were lost
  flag_client_lost     = 1u << 5,  ///< heartbeat watchdog fired (ADR-0005)
  flag_limit_violation = 1u << 6,  ///< a soft limit was hit and clamped
  flag_drive_fault     = 1u << 7,  ///< at least one drive reports a fault
  flag_stopping        = 1u << 8,  ///< executing a Category 2 ramp
  flag_holding         = 1u << 9,  ///< compliant hold after a stop
};

/// Control mode in force for this cycle.
enum class ControlMode : std::uint32_t {
  idle = 0,               ///< enabled but commanding nothing
  mit = 1,                ///< impedance: position, velocity, gains and torque
  position_velocity = 2,  ///< drive-side position loop with a velocity limit
  velocity = 3,
  stopping = 4,           ///< Category 2 deceleration ramp
  holding = 5,            ///< compliant hold
};

/// 256 bytes. Trivially copyable, no pointers, identical layout in every
/// process that maps it.
struct TelemetryRecord {
  std::uint64_t cycle;                      ///< monotonically increasing cycle index
  std::int64_t deadline_nanoseconds;        ///< when this cycle should have woken
  std::int64_t wake_nanoseconds;            ///< when it actually woke
  std::int64_t execution_nanoseconds;       ///< how long the cycle body ran
  std::int64_t can_transmit_nanoseconds;    ///< first command frame handed to the driver
  std::int64_t can_receive_nanoseconds;     ///< newest feedback frame, KERNEL timestamp

  std::uint32_t joint_count;                ///< how many of the per-joint arrays are meaningful
  std::uint32_t flags;                      ///< bitwise OR of RecordFlag
  std::uint32_t fault_mask;                 ///< bit per joint: drive reported a fault
  std::uint32_t mode;                       ///< ControlMode

  float commanded_position[max_joints];     ///< commanded position, rad
  float commanded_velocity[max_joints];     ///< commanded velocity, rad/s
  float commanded_torque[max_joints];       ///< commanded torque, N m
  float measured_position[max_joints];      ///< measured position, rad
  float measured_velocity[max_joints];      ///< measured velocity, rad/s
  float measured_torque[max_joints];        ///< measured torque, N m
};

static_assert(sizeof(TelemetryRecord) == 256,
              "the record size is a budget, not an accident -- if this fires, "
              "re-derive the disk rate before changing the assertion");

/// Live snapshot for "what is it doing right now": a UI, a plot, a state
/// publisher. Deliberately smaller than TelemetryRecord — a viewer wants
/// position and health, not CAN timestamps.
struct StateSnapshot {
  std::uint64_t cycle;              ///< cycle this snapshot was taken in
  std::int64_t wake_nanoseconds;    ///< CLOCK_MONOTONIC when that cycle woke
  std::uint32_t joint_count;        ///< how many of the per-joint arrays are meaningful
  std::uint32_t flags;              ///< bitwise OR of RecordFlag
  std::uint32_t fault_mask;         ///< bit per joint: drive reported a fault
  std::uint32_t mode;               ///< ControlMode
  float position[max_joints];       ///< measured position, rad
  float velocity[max_joints];       ///< measured velocity, rad/s
  float torque[max_joints];         ///< measured torque, N m
};

}  // namespace robot_control::telemetry
