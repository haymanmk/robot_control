#pragma once

/// @file client.hpp
/// The client end of the bridge — what Python, ROS2 and any UI bind to.
///
/// Note what is absent: there is no `step()`, no `send_now()`, no per-cycle
/// callback. That is [ADR-0004](../../../../docs/adr/0004-system-decomposition.md)'s
/// rule made structural. A client can *ask* for things and *observe* things; it
/// cannot drive the loop, because the loop is in another process.
///
/// Liveness is a two-step handshake on purpose:
///
///   attach()        -> may read telemetry and state. Watchdog stays disarmed.
///   take_control()  -> may command. Watchdog arms. From here the client MUST
///                      call heartbeat() regularly or the RT core executes a
///                      Category 2 stop (ADR-0005).
///
/// A monitoring or plotting client never calls take_control(), so it can crash,
/// hang or be killed with no effect on the arm. Only a client that took
/// responsibility is held to it.

#include <cstdint>
#include <string>

#include "rc/bridge/layout.hpp"
#include "rc/bridge/shared_region.hpp"

namespace rc::bridge {

/// The client end of the bridge. Attaches to the RT core's shared region to
/// read state and telemetry; optionally takes control to send commands, at
/// which point it must heartbeat() or the RT-side watchdog stops the arm.
class BridgeClient {
 public:
  BridgeClient() = default;
  ~BridgeClient();
  BridgeClient(const BridgeClient&) = delete;
  BridgeClient& operator=(const BridgeClient&) = delete;

  /// Map the RT core's region for reading. Does not arm the watchdog; a
  /// monitoring client stops here.
  [[nodiscard]] RegionError attach(const std::string& name = default_region_name);

  /// Take responsibility for commanding the arm. Arms the RT-side watchdog.
  /// @return false if another client already holds control.
  [[nodiscard]] bool take_control() noexcept;

  /// Give control back cleanly and disarm the watchdog. Called by the
  /// destructor, so a normal exit — including a Python interpreter shutting
  /// down tidily — releases without tripping anything.
  void release_control() noexcept;

  /// Prove liveness. Must be called more often than the watchdog timeout.
  ///
  /// Only a client that holds control writes the heartbeat. An observer's
  /// heartbeat would otherwise keep a *dead* controller looking alive -- the
  /// watchdog cannot tell whose hand is on the counter, so only the responsible
  /// hand may touch it. If the server has revoked our token (watchdog trip
  /// while we were stalled), this discovers it and stands down.
  void heartbeat() noexcept;

  /// Queue a command. Non-blocking. Refused unless this client holds control.
  /// @return false if not in control, or if the ring is full — the RT core is
  ///         not draining, which is itself a diagnosis, so the caller should
  ///         surface it rather than spin.
  [[nodiscard]] bool send(CommandRecord& command) noexcept;

  /// Latest state. Wait-free with respect to the RT writer.
  /// @return false only if a burst of writes prevented a clean read.
  [[nodiscard]] bool state(rc::telemetry::StateSnapshot& out) const noexcept;

  /// The RT core's own view of its state. Pair with server_alive() to tell
  /// "idle" from "dead".
  [[nodiscard]] ServerState server_state() const noexcept;

  /// Has the RT core advanced since the last call? Distinguishes "idle" from
  /// "dead", which the state enum alone cannot.
  [[nodiscard]] bool server_alive() noexcept;

  /// The RT loop's nominal period, from the region header.
  [[nodiscard]] std::uint32_t control_period_ns() const noexcept;
  /// Telemetry records the RT side dropped because the ring was full.
  [[nodiscard]] std::uint64_t telemetry_dropped() const noexcept;
  /// True after a successful attach().
  [[nodiscard]] bool attached() const noexcept { return region_.valid(); }

  /// Live check against the shared token, so a revocation by the server is
  /// visible here without waiting for the next heartbeat().
  [[nodiscard]] bool in_control() noexcept { return verify_control(); }

 private:
  /// Confirms our token is still the one in the region; clears local state if
  /// the server revoked it.
  [[nodiscard]] bool verify_control() noexcept;

  SharedRegion region_;
  std::uint64_t beat_ = 0;         ///< heartbeat counter
  std::uint64_t command_seq_ = 0;  ///< command sequence: separate, so gaps mean loss
  std::uint64_t token_ = 0;
  std::uint64_t last_server_beat_ = 0;
  bool holds_control_ = false;
};

}  // namespace rc::bridge
