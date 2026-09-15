#pragma once

/// @file layout.hpp
/// The shared-memory contract between the RT core and its clients.
///
/// This header *is* the interface described in
/// [ADR-0006](../../../../docs/adr/0006-process-topology-and-rt-client-transport.md).
/// Python, ROS2 and any future UI all speak exactly this. Two rules keep it
/// honest:
///
///   1. **Everything here is trivially copyable, with no pointers.** Two
///      processes map this region at different virtual addresses; a pointer
///      stored inside would be a wild pointer on the other side.
///   2. **Any change to a record or a capacity bumps `kLayoutVersion`.** The
///      header carries the version *and* every struct size, and attach() refuses
///      a mismatch. A client built against an older layout must fail loudly at
///      attach rather than silently misread torque as position.

#include <atomic>
#include <cstdint>

#include "rc/rt/seqlock.hpp"
#include "rc/rt/spsc_ring.hpp"
#include "rc/telemetry/record.hpp"

namespace rc::bridge {

/// "RCBRIDG1" — identifies the region and catches a stale or foreign mapping.
inline constexpr std::uint64_t kMagic = 0x3147495242435223ULL;

/// Bump on ANY change below: record fields, capacities, ordering.
inline constexpr std::uint32_t kLayoutVersion = 1;

/// 8 s of history at 500 Hz. The drain thread only has to keep up on average;
/// this is the margin for a page-cache flush or a scheduling hiccup on the
/// non-RT side, not a place to store the session.
inline constexpr std::size_t kTelemetryCapacity = 4096;

/// Commands are events (move, enable, stop), not a stream. If 256 are
/// outstanding, the client is misusing the interface and should be told.
inline constexpr std::size_t kCommandCapacity = 256;

enum class CommandType : std::uint32_t {
  kNone = 0,
  kEnable = 1,
  kDisable = 2,
  kSetMode = 3,
  kSetTarget = 4,       ///< joint-space setpoint for the interpolator
  kStop = 5,            ///< request Category 2 (ADR-0005)
  kReturnHome = 6,      ///< graceful-shutdown / recovery only; server enforces preconditions
  kClearFault = 7,
};

/// One client -> RT command. Fixed size; the union of every command's payload,
/// because a variable-length encoding in shared memory buys nothing at this size
/// and costs a parser in the cyclic path.
struct CommandRecord {
  std::uint64_t sequence;     ///< client-assigned, strictly increasing; gaps mean loss
  std::int64_t issued_ns;     ///< CLOCK_MONOTONIC at the client
  std::uint32_t type;         ///< CommandType
  std::uint32_t joint_count;
  std::uint32_t mode;         ///< telemetry::ControlMode, for kSetMode
  std::uint32_t flags;
  float pos[rc::telemetry::kMaxJoints];
  float vel[rc::telemetry::kMaxJoints];
  float tau[rc::telemetry::kMaxJoints];
  float kp[rc::telemetry::kMaxJoints];
  float kd[rc::telemetry::kMaxJoints];
};

static_assert(sizeof(CommandRecord) == 192, "layout change requires a kLayoutVersion bump");

/// What the RT core is doing. Clients poll this; it is also what tells a client
/// that the server went away.
enum class ServerState : std::uint32_t {
  kStarting = 0,
  kIdle = 1,        ///< running, no client has taken control
  kControlled = 2,  ///< a client holds control; the watchdog is armed
  kStopping = 3,    ///< Category 2 ramp in progress
  kHolding = 4,     ///< compliant hold after a fault or stop
  kShutdown = 5,
};

/// Fixed-size preamble. Every field before the rings is validated at attach.
///
/// `magic` is atomic and is written **last** by the server with release
/// semantics; a client loads it with acquire before trusting anything else.
/// That makes it the publish gate: the region is visible by name the moment
/// shm_open() returns, and without this a client attaching a few hundred
/// microseconds early would read a zero period or a half-written header.
struct BridgeHeader {
  std::atomic<std::uint64_t> magic;
  std::uint32_t layout_version;
  std::uint32_t header_size;
  std::uint32_t telemetry_record_size;
  std::uint32_t command_record_size;
  std::uint32_t snapshot_size;
  std::uint32_t telemetry_capacity;
  std::uint32_t command_capacity;
  std::uint32_t control_period_ns;
  std::int64_t server_start_ns;
  std::uint64_t server_pid;

  /// Client liveness. A **counter**, not a timestamp: two processes need not
  /// agree on a clock for a counter to prove progress, and a frozen client that
  /// keeps republishing an old timestamp would look alive.
  alignas(rc::rt::kCacheLine) std::atomic<std::uint64_t> client_heartbeat;

  /// Server liveness, so a client can tell "the RT core died" from "the RT core
  /// is idle".
  alignas(rc::rt::kCacheLine) std::atomic<std::uint64_t> server_heartbeat;

  /// Non-zero once a client has taken control. The watchdog arms only then:
  /// an unattended core must not fault itself for the absence of a client that
  /// never arrived.
  alignas(rc::rt::kCacheLine) std::atomic<std::uint64_t> control_token;

  alignas(rc::rt::kCacheLine) std::atomic<std::uint32_t> server_state;
  std::atomic<std::uint32_t> watchdog_timeout_cycles;

  /// Counters the RT side owns. Published so a client can see loss without
  /// inferring it.
  alignas(rc::rt::kCacheLine) std::atomic<std::uint64_t> telemetry_dropped;
  std::atomic<std::uint64_t> commands_rejected;
  std::atomic<std::uint64_t> watchdog_trips;
};

/// The whole region. Placement-new'd by the server into the mapping; clients
/// reinterpret the same bytes.
struct BridgeRegion {
  BridgeHeader header;
  rc::rt::Seqlock<rc::telemetry::StateSnapshot> snapshot;
  rc::rt::SpscRing<rc::telemetry::TelemetryRecord, kTelemetryCapacity> telemetry;
  rc::rt::SpscRing<CommandRecord, kCommandCapacity> commands;
};

/// Default name; the leading slash is required by shm_open(3).
inline constexpr const char* kDefaultRegionName = "/rc_bridge";

}  // namespace rc::bridge
