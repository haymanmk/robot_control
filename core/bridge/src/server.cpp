#include "robot_control/bridge/server.hpp"

namespace robot_control::bridge {

RegionError BridgeServer::open(const std::string& name, std::uint32_t control_period_nanoseconds,
                               bool lock_memory) {
  const RegionError error =
      SharedRegion::create(name, region, control_period_nanoseconds, timeout_cycles, lock_memory);
  // mlock_failed still yields a usable region; the caller decides whether a
  // pageable bridge is acceptable. Every other error leaves region invalid.
  if (error != RegionError::ok && error != RegionError::mlock_failed) {
    return error;
  }
  region.get()->header.server_state.store(static_cast<std::uint32_t>(ServerState::idle),
                                           std::memory_order_release);
  return error;
}

void BridgeServer::set_watchdog_timeout_cycles(std::uint32_t cycles) noexcept {
  timeout_cycles = cycles;
  if (region.valid()) {
    region.get()->header.watchdog_timeout_cycles.store(cycles, std::memory_order_release);
  }
}

bool BridgeServer::publish(const robot_control::telemetry::TelemetryRecord& record) noexcept {
  if (!region.valid()) {
    return false;
  }
  BridgeRegion& mapped = *region.get();
  if (mapped.telemetry.push(record)) {
    return true;
  }
  mapped.header.telemetry_dropped.fetch_add(1, std::memory_order_relaxed);
  return false;
}

void BridgeServer::publish_snapshot(const robot_control::telemetry::StateSnapshot& snapshot) noexcept {
  if (region.valid()) {
    region.get()->snapshot.store(snapshot);
  }
}

bool BridgeServer::poll_command(CommandRecord& out) noexcept {
  return region.valid() && region.get()->commands.pop(out);
}

bool BridgeServer::tick(std::uint64_t cycle) noexcept {
  if (!region.valid()) {
    return false;
  }
  BridgeHeader& header = region.get()->header;
  header.server_heartbeat.store(++server_beat, std::memory_order_release);

  const std::uint64_t token = header.control_token.load(std::memory_order_acquire);

  if (token != watched_token) {
    // A different client (or none). Baseline against *its* heartbeat rather
    // than comparing to whatever the previous holder left behind. This is also
    // what re-arms after a release()+take_control() pair that happened between
    // two ticks: the token value moved, even if it was never seen as zero.
    watched_token = token;
    last_heartbeat = header.client_heartbeat.load(std::memory_order_acquire);
    last_progress_cycle = cycle;
    return false;
  }

  if (token == 0) {
    // Nobody has taken control. An unattended core must not fault itself for
    // the absence of a client that never arrived -- it simply idles, safely.
    last_progress_cycle = cycle;
    return false;
  }

  const std::uint64_t beat = header.client_heartbeat.load(std::memory_order_acquire);
  if (beat != last_heartbeat) {
    last_heartbeat = beat;
    last_progress_cycle = cycle;
    return false;
  }

  if (cycle - last_progress_cycle < timeout_cycles) {
    return false;
  }

  // Trip. Revoke the token so a successor can take control, and so that this
  // holder -- if it was merely stalled -- learns on its next heartbeat that it
  // no longer commands anything. Because the token now reads 0, the next tick
  // takes the `token != watched_token` branch and cannot re-fire.
  std::uint64_t expected = token;
  (void)header.control_token.compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                                std::memory_order_acquire);
  header.watchdog_trips.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool BridgeServer::client_in_control() const noexcept {
  if (!region.valid()) {
    return false;
  }
  // A tripped holder has had its token revoked, so "token present" is exactly
  // "a live client is responsible for the arm".
  return region.get()->header.control_token.load(std::memory_order_acquire) != 0;
}

void BridgeServer::set_state(ServerState state) noexcept {
  if (region.valid()) {
    region.get()->header.server_state.store(static_cast<std::uint32_t>(state),
                                             std::memory_order_release);
  }
}

ServerState BridgeServer::state() const noexcept {
  if (!region.valid()) {
    return ServerState::shutdown;
  }
  return static_cast<ServerState>(
      region.get()->header.server_state.load(std::memory_order_acquire));
}

std::size_t BridgeServer::drain(robot_control::telemetry::TelemetryRecord* out, std::size_t max_records) noexcept {
  if (!region.valid() || out == nullptr) {
    return 0;
  }
  std::size_t count = 0;
  while (count < max_records && region.get()->telemetry.pop(out[count])) {
    ++count;
  }
  return count;
}

std::uint64_t BridgeServer::telemetry_dropped() const noexcept {
  return region.valid()
             ? region.get()->header.telemetry_dropped.load(std::memory_order_relaxed)
             : 0;
}

std::uint64_t BridgeServer::watchdog_trips() const noexcept {
  return region.valid() ? region.get()->header.watchdog_trips.load(std::memory_order_relaxed)
                         : 0;
}

void BridgeServer::close() noexcept {
  if (region.valid()) {
    set_state(ServerState::shutdown);
    SharedRegion::unlink(region.name());
  }
  region = SharedRegion{};
}

}  // namespace robot_control::bridge
