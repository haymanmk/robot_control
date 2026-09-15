#include "rc/bridge/client.hpp"

#include <unistd.h>

namespace rc::bridge {

BridgeClient::~BridgeClient() { release_control(); }

RegionError BridgeClient::attach(const std::string& name) {
  return SharedRegion::attach(name, region_, /*read_only=*/false);
}

bool BridgeClient::take_control() noexcept {
  if (!region_.valid()) {
    return false;
  }
  if (holds_control_ && verify_control()) {
    return true;
  }
  BridgeHeader& h = region_.get()->header;

  // The PID is the token: unique among live processes, and it identifies who
  // holds control in a crash dump.
  const auto desired = static_cast<std::uint64_t>(::getpid());
  std::uint64_t expected = 0;
  if (!h.control_token.compare_exchange_strong(expected, desired, std::memory_order_acq_rel,
                                               std::memory_order_acquire)) {
    return false;  // someone else holds it -- and we must not touch the heartbeat
  }
  token_ = desired;
  holds_control_ = true;

  // Only now, as the holder, prove liveness. A failed attempt above must leave
  // no trace, or a client polling for control would keep a dead holder alive.
  heartbeat();
  return true;
}

bool BridgeClient::verify_control() noexcept {
  if (!region_.valid() || !holds_control_) {
    return false;
  }
  const std::uint64_t live =
      region_.get()->header.control_token.load(std::memory_order_acquire);
  if (live != token_) {
    // Revoked: the watchdog tripped while we were stalled and a successor may
    // already be in charge. Stand down rather than command an arm we lost.
    holds_control_ = false;
    token_ = 0;
  }
  return holds_control_;
}

void BridgeClient::release_control() noexcept {
  if (!region_.valid() || !holds_control_) {
    return;
  }
  BridgeHeader& h = region_.get()->header;
  std::uint64_t expected = token_;
  // Only clear our own token: a stale client must never release a successor's.
  (void)h.control_token.compare_exchange_strong(expected, 0, std::memory_order_acq_rel,
                                                std::memory_order_acquire);
  holds_control_ = false;
  token_ = 0;
}

void BridgeClient::heartbeat() noexcept {
  if (!verify_control()) {
    return;
  }
  region_.get()->header.client_heartbeat.store(++beat_, std::memory_order_release);
}

bool BridgeClient::send(CommandRecord& command) noexcept {
  if (!verify_control()) {
    return false;
  }
  // Commands carry their own contiguous sequence so the server can detect loss
  // by gaps; the heartbeat counter is a separate thing and advances on its own.
  command.sequence = ++command_seq_;
  BridgeRegion& r = *region_.get();
  if (!r.commands.push(command)) {
    r.header.commands_rejected.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  heartbeat();  // sending is itself proof of life
  return true;
}

bool BridgeClient::state(rc::telemetry::StateSnapshot& out) const noexcept {
  return region_.valid() && region_.get()->snapshot.load(out);
}

ServerState BridgeClient::server_state() const noexcept {
  if (!region_.valid()) {
    return ServerState::kShutdown;
  }
  return static_cast<ServerState>(
      region_.get()->header.server_state.load(std::memory_order_acquire));
}

bool BridgeClient::server_alive() noexcept {
  if (!region_.valid()) {
    return false;
  }
  const std::uint64_t beat =
      region_.get()->header.server_heartbeat.load(std::memory_order_acquire);
  const bool advanced = beat != last_server_beat_;
  last_server_beat_ = beat;
  return advanced;
}

std::uint32_t BridgeClient::control_period_ns() const noexcept {
  return region_.valid() ? region_.get()->header.control_period_ns : 0;
}

std::uint64_t BridgeClient::telemetry_dropped() const noexcept {
  return region_.valid()
             ? region_.get()->header.telemetry_dropped.load(std::memory_order_relaxed)
             : 0;
}

}  // namespace rc::bridge
