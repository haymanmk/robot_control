#include "rc/bridge/server.hpp"

namespace rc::bridge {

RegionError BridgeServer::open(const std::string& name, std::uint32_t control_period_ns,
                               bool lock_memory) {
  const RegionError err =
      SharedRegion::create(name, region_, control_period_ns, timeout_cycles_, lock_memory);
  // kMlockFailed still yields a usable region; the caller decides whether a
  // pageable bridge is acceptable. Every other error leaves region_ invalid.
  if (err != RegionError::kOk && err != RegionError::kMlockFailed) {
    return err;
  }
  region_.get()->header.server_state.store(static_cast<std::uint32_t>(ServerState::kIdle),
                                           std::memory_order_release);
  return err;
}

void BridgeServer::set_watchdog_timeout_cycles(std::uint32_t cycles) noexcept {
  timeout_cycles_ = cycles;
  if (region_.valid()) {
    region_.get()->header.watchdog_timeout_cycles.store(cycles, std::memory_order_release);
  }
}

bool BridgeServer::publish(const rc::telemetry::TelemetryRecord& record) noexcept {
  if (!region_.valid()) {
    return false;
  }
  BridgeRegion& r = *region_.get();
  if (r.telemetry.push(record)) {
    return true;
  }
  r.header.telemetry_dropped.fetch_add(1, std::memory_order_relaxed);
  return false;
}

void BridgeServer::publish_snapshot(const rc::telemetry::StateSnapshot& snapshot) noexcept {
  if (region_.valid()) {
    region_.get()->snapshot.store(snapshot);
  }
}

bool BridgeServer::poll_command(CommandRecord& out) noexcept {
  return region_.valid() && region_.get()->commands.pop(out);
}

bool BridgeServer::tick(std::uint64_t cycle) noexcept {
  if (!region_.valid()) {
    return false;
  }
  BridgeHeader& h = region_.get()->header;
  h.server_heartbeat.store(++server_beat_, std::memory_order_release);

  const std::uint64_t token = h.control_token.load(std::memory_order_acquire);

  if (token != watched_token_) {
    // A different client (or none). Baseline against *its* heartbeat rather
    // than comparing to whatever the previous holder left behind. This is also
    // what re-arms after a release()+take_control() pair that happened between
    // two ticks: the token value moved, even if it was never seen as zero.
    watched_token_ = token;
    last_heartbeat_ = h.client_heartbeat.load(std::memory_order_acquire);
    last_progress_cycle_ = cycle;
    return false;
  }

  if (token == 0) {
    // Nobody has taken control. An unattended core must not fault itself for
    // the absence of a client that never arrived -- it simply idles, safely.
    last_progress_cycle_ = cycle;
    return false;
  }

  const std::uint64_t beat = h.client_heartbeat.load(std::memory_order_acquire);
  if (beat != last_heartbeat_) {
    last_heartbeat_ = beat;
    last_progress_cycle_ = cycle;
    return false;
  }

  if (cycle - last_progress_cycle_ < timeout_cycles_) {
    return false;
  }

  // Trip. Revoke the token so a successor can take control, and so that this
  // holder -- if it was merely stalled -- learns on its next heartbeat that it
  // no longer commands anything. Because the token now reads 0, the next tick
  // takes the `token != watched_token_` branch and cannot re-fire.
  std::uint64_t expected = token;
  (void)h.control_token.compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                                std::memory_order_acquire);
  h.watchdog_trips.fetch_add(1, std::memory_order_relaxed);
  return true;
}

bool BridgeServer::client_in_control() const noexcept {
  if (!region_.valid()) {
    return false;
  }
  // A tripped holder has had its token revoked, so "token present" is exactly
  // "a live client is responsible for the arm".
  return region_.get()->header.control_token.load(std::memory_order_acquire) != 0;
}

void BridgeServer::set_state(ServerState state) noexcept {
  if (region_.valid()) {
    region_.get()->header.server_state.store(static_cast<std::uint32_t>(state),
                                             std::memory_order_release);
  }
}

ServerState BridgeServer::state() const noexcept {
  if (!region_.valid()) {
    return ServerState::kShutdown;
  }
  return static_cast<ServerState>(
      region_.get()->header.server_state.load(std::memory_order_acquire));
}

std::size_t BridgeServer::drain(rc::telemetry::TelemetryRecord* out, std::size_t max) noexcept {
  if (!region_.valid() || out == nullptr) {
    return 0;
  }
  std::size_t n = 0;
  while (n < max && region_.get()->telemetry.pop(out[n])) {
    ++n;
  }
  return n;
}

std::uint64_t BridgeServer::telemetry_dropped() const noexcept {
  return region_.valid()
             ? region_.get()->header.telemetry_dropped.load(std::memory_order_relaxed)
             : 0;
}

std::uint64_t BridgeServer::watchdog_trips() const noexcept {
  return region_.valid() ? region_.get()->header.watchdog_trips.load(std::memory_order_relaxed)
                         : 0;
}

void BridgeServer::close() noexcept {
  if (region_.valid()) {
    set_state(ServerState::kShutdown);
    SharedRegion::unlink(region_.name());
  }
  region_ = SharedRegion{};
}

}  // namespace rc::bridge
