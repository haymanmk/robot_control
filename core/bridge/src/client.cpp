#include "rc/bridge/client.hpp"

#include <unistd.h>

namespace rc::bridge {

BridgeClient::~BridgeClient() { release_control(); }

RegionError BridgeClient::attach(const std::string& name) {
  return SharedRegion::attach(name, region, /*read_only=*/false);
}

bool BridgeClient::take_control() noexcept {
  if (!region.valid()) {
    return false;
  }
  if (holds_control && verify_control()) {
    return true;
  }
  BridgeHeader& header = region.get()->header;

  // The PID is the token: unique among live processes, and it identifies who
  // holds control in a crash dump.
  const auto desired = static_cast<std::uint64_t>(::getpid());
  std::uint64_t expected = 0;
  if (!header.control_token.compare_exchange_strong(expected, desired, std::memory_order_acq_rel,
                                               std::memory_order_acquire)) {
    return false;  // someone else holds it -- and we must not touch the heartbeat
  }
  token = desired;
  holds_control = true;

  // Only now, as the holder, prove liveness. A failed attempt above must leave
  // no trace, or a client polling for control would keep a dead holder alive.
  heartbeat();
  return true;
}

bool BridgeClient::verify_control() noexcept {
  if (!region.valid() || !holds_control) {
    return false;
  }
  const std::uint64_t live =
      region.get()->header.control_token.load(std::memory_order_acquire);
  if (live != token) {
    // Revoked: the watchdog tripped while we were stalled and a successor may
    // already be in charge. Stand down rather than command an arm we lost.
    holds_control = false;
    token = 0;
  }
  return holds_control;
}

void BridgeClient::release_control() noexcept {
  if (!region.valid() || !holds_control) {
    return;
  }
  BridgeHeader& header = region.get()->header;
  std::uint64_t expected = token;
  // Only clear our own token: a stale client must never release a successor's.
  (void)header.control_token.compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                                std::memory_order_acquire);
  holds_control = false;
  token = 0;
}

void BridgeClient::heartbeat() noexcept {
  if (!verify_control()) {
    return;
  }
  region.get()->header.client_heartbeat.store(++beat, std::memory_order_release);
}

bool BridgeClient::send(CommandRecord& command) noexcept {
  if (!verify_control()) {
    return false;
  }
  // Commands carry their own contiguous sequence so the server can detect loss
  // by gaps; the heartbeat counter is a separate thing and advances on its own.
  command.sequence = ++command_sequence;
  BridgeRegion& mapped = *region.get();
  if (!mapped.commands.push(command)) {
    mapped.header.commands_rejected.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  heartbeat();  // sending is itself proof of life
  return true;
}

bool BridgeClient::state(rc::telemetry::StateSnapshot& out) const noexcept {
  return region.valid() && region.get()->snapshot.load(out);
}

ServerState BridgeClient::server_state() const noexcept {
  if (!region.valid()) {
    return ServerState::shutdown;
  }
  return static_cast<ServerState>(
      region.get()->header.server_state.load(std::memory_order_acquire));
}

bool BridgeClient::server_alive() noexcept {
  if (!region.valid()) {
    return false;
  }
  const std::uint64_t observed_beat =
      region.get()->header.server_heartbeat.load(std::memory_order_acquire);
  const bool advanced = observed_beat != last_server_beat;
  last_server_beat = observed_beat;
  return advanced;
}

std::uint32_t BridgeClient::control_period_nanoseconds() const noexcept {
  return region.valid() ? region.get()->header.control_period_nanoseconds : 0;
}

std::uint64_t BridgeClient::telemetry_dropped() const noexcept {
  return region.valid()
             ? region.get()->header.telemetry_dropped.load(std::memory_order_relaxed)
             : 0;
}

}  // namespace rc::bridge
