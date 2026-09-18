#pragma once

/// @file server.hpp
/// The RT core's end of the bridge. Every method marked "cyclic" is callable
/// from inside the 500 Hz loop: wait-free, allocation-free, syscall-free.
///
/// It also owns the client watchdog, because
/// [ADR-0005](../../../../docs/adr/0005-safe-state-and-stop-architecture.md)
/// requires the watchdog to have the fewest possible dependencies, and the
/// cyclic loop is already running. A separate monitor process would be one more
/// thing that can die.

#include <cstdint>
#include <string>

#include "rc/bridge/layout.hpp"
#include "rc/bridge/shared_region.hpp"

namespace rc::bridge {

/// The RT core's end of the bridge. Owns the shared region, publishes
/// telemetry and state snapshots, polls commands, and runs the client
/// watchdog. Methods in the "cyclic" section are safe inside the 500 Hz loop.
class BridgeServer {
 public:
  BridgeServer() = default;

  /// Creates the shared region. Call once, before the cyclic loop starts —
  /// this allocates and syscalls.
  [[nodiscard]] RegionError open(const std::string& name = kDefaultRegionName,
                                 std::uint32_t control_period_ns = 2'000'000,
                                 bool lock_memory = true);

  /// Sets how many consecutive cycles without client progress trip the
  /// watchdog. At 500 Hz the default 50 is 100 ms.
  void set_watchdog_timeout_cycles(std::uint32_t cycles) noexcept;

  // ── cyclic ────────────────────────────────────────────────────────────────

  /// Publish one record. Never blocks. If the ring is full the record is
  /// dropped and counted: a stalled drain thread must not stall control.
  /// @return false if the record was dropped.
  bool publish(const rc::telemetry::TelemetryRecord& record) noexcept;

  /// Publish the "what is it doing now" snapshot for live viewers.
  void publish_snapshot(const rc::telemetry::StateSnapshot& snapshot) noexcept;

  /// Pop one pending command, if any.
  [[nodiscard]] bool poll_command(CommandRecord& out) noexcept;

  /// Advance the server heartbeat and evaluate the client watchdog. Call once
  /// per cycle, before poll_command().
  ///
  /// Liveness is judged per *token*: the watchdog baselines whenever the
  /// control token changes value, so a client that releases and re-takes
  /// control back-to-back is re-armed even if no cycle observed the gap.
  ///
  /// On a trip the server **revokes** the token (stores 0). The dead client
  /// held nothing worth keeping, and a successor must be able to take control
  /// without restarting the RT core -- that is what makes a Jupyter kernel
  /// restart recoverable. A stalled-not-dead client that later resumes
  /// discovers the revocation on its next heartbeat() and stands down.
  ///
  /// @return true if the watchdog tripped **on this cycle** — the caller starts
  ///         the Category 2 ramp. It fires once per trip, never re-triggering
  ///         mid-ramp.
  [[nodiscard]] bool tick(std::uint64_t cycle) noexcept;

  /// True once a client has taken control and has not been lost.
  [[nodiscard]] bool client_in_control() const noexcept;

  /// Publish the server's state for clients to read.
  void set_state(ServerState state) noexcept;
  /// The state most recently published with set_state().
  [[nodiscard]] ServerState state() const noexcept;

  // ── non-cyclic ────────────────────────────────────────────────────────────

  /// Drain queued telemetry, e.g. from the file-sink thread.
  /// @return number of records written into @p out.
  [[nodiscard]] std::size_t drain(rc::telemetry::TelemetryRecord* out, std::size_t max_records) noexcept;

  /// Records publish() dropped because the telemetry ring was full.
  [[nodiscard]] std::uint64_t telemetry_dropped() const noexcept;
  /// Times the client watchdog fired since open().
  [[nodiscard]] std::uint64_t watchdog_trips() const noexcept;
  /// True between a successful open() and close().
  [[nodiscard]] bool valid() const noexcept { return region_.valid(); }

  /// Mark shutdown and remove the region name.
  void close() noexcept;

 private:
  SharedRegion region_;
  std::uint64_t watched_token_ = 0;     ///< token value the watchdog is judging
  std::uint64_t last_heartbeat_ = 0;
  std::uint64_t last_progress_cycle_ = 0;
  std::uint32_t timeout_cycles_ = 50;
  std::uint64_t server_beat_ = 0;
};

}  // namespace rc::bridge
