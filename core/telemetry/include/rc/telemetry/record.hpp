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

namespace rc::telemetry {

/// Seven on the B601-RS (6 arm + gripper); rounded up so the record stays a
/// round 256 bytes and one more joint does not change the wire format.
inline constexpr unsigned kMaxJoints = 8;

/// Per-cycle condition flags. Bitfield so a cycle can report several at once.
enum RecordFlag : std::uint32_t {
  kFlagNone            = 0u,
  kFlagOverrun         = 1u << 0,  ///< period error beyond the overrun threshold
  kFlagMissedDeadline  = 1u << 1,  ///< body() ran longer than the period
  kFlagStaleFeedback   = 1u << 2,  ///< a joint's feedback is older than allowed
  kFlagCommandDropped  = 1u << 3,  ///< command ring was full; a command was lost
  kFlagTelemetryLost   = 1u << 4,  ///< telemetry ring was full; records were lost
  kFlagClientLost      = 1u << 5,  ///< heartbeat watchdog fired (ADR-0005)
  kFlagLimitViolation  = 1u << 6,  ///< a soft limit was hit and clamped
  kFlagDriveFault      = 1u << 7,  ///< at least one drive reports a fault
  kFlagStopping        = 1u << 8,  ///< executing a Category 2 ramp
  kFlagHolding         = 1u << 9,  ///< compliant hold after a stop
};

/// Control mode in force for this cycle.
enum class ControlMode : std::uint32_t {
  kIdle = 0,       ///< enabled but commanding nothing
  kMit = 1,        ///< impedance: pos, vel, kp, kd, tau
  kPosVel = 2,     ///< drive-side position loop with a velocity limit
  kVelocity = 3,
  kStopping = 4,   ///< Category 2 deceleration ramp
  kHolding = 5,    ///< compliant hold
};

/// 256 bytes. Trivially copyable, no pointers, identical layout in every
/// process that maps it.
struct TelemetryRecord {
  std::uint64_t cycle;        ///< monotonically increasing cycle index
  std::int64_t deadline_ns;   ///< when this cycle should have woken
  std::int64_t wake_ns;       ///< when it actually woke
  std::int64_t exec_ns;       ///< how long the cycle body ran
  std::int64_t can_tx_ns;     ///< first command frame handed to the driver
  std::int64_t can_rx_ns;     ///< newest feedback frame, KERNEL timestamp

  std::uint32_t joint_count;
  std::uint32_t flags;        ///< bitwise OR of RecordFlag
  std::uint32_t fault_mask;   ///< bit per joint: drive reported a fault
  std::uint32_t mode;         ///< ControlMode

  float cmd_pos[kMaxJoints];
  float cmd_vel[kMaxJoints];
  float cmd_tau[kMaxJoints];
  float meas_pos[kMaxJoints];
  float meas_vel[kMaxJoints];
  float meas_tau[kMaxJoints];
};

static_assert(sizeof(TelemetryRecord) == 256,
              "the record size is a budget, not an accident -- if this fires, "
              "re-derive the disk rate before changing the assertion");

/// Live snapshot for "what is it doing right now": a UI, a plot, a state
/// publisher. Deliberately smaller than TelemetryRecord — a viewer wants
/// position and health, not CAN timestamps.
struct StateSnapshot {
  std::uint64_t cycle;
  std::int64_t wake_ns;
  std::uint32_t joint_count;
  std::uint32_t flags;
  std::uint32_t fault_mask;
  std::uint32_t mode;
  float pos[kMaxJoints];
  float vel[kMaxJoints];
  float tau[kMaxJoints];
};

}  // namespace rc::telemetry
