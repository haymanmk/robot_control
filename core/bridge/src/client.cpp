#include "rc/bridge/client.hpp"

#include <unistd.h>

namespace rc::bridge {

BridgeClient::~BridgeClient() { release_control(); }

RegionError BridgeClient::attach(const std::string& name) {
  return SharedRegion::attach(name, region_, /*read_only=*/false);
}

bool BridgeClient::take_control() noexcept {
  if (!region_.valid() || holds_control_) {
    return holds_control_;
  }
  BridgeHeader& h = region_.get()->header;

  // Publish a heartbeat before claiming the token. The server samples the
  // heartbeat on the first cycle it sees the token; if the token appeared first
  // it could baseline against a previous client's stale value.
  beat_ = h.client_heartbeat.load(std::memory_order_acquire) + 1;
  h.client_heartbeat.store(beat_, std::memory_order_release);

  // The PID is the token: unique among live processes, and it identifies who
  // holds control in a crash dump.
  const auto desired = static_cast<std::uint64_t>(::getpid());
  std::uint64_t expected = 0;
  if (!h.control_token.compare_exchange_strong(expected, desired, std::memory_order_acq_rel,
                                               std::memory_order_acquire)) {
    return false;  // someone else holds it
  }
  token_ = desired;
  holds_control_ = true;
  return true;
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
  if (region_.valid()) {
    region_.get()->header.client_heartbeat.store(++beat_, std::memory_order_release);
  }
}

bool BridgeClient::send(CommandRecord& command) noexcept {
  if (!region_.valid()) {
    return false;
  }
  // The client owns the sequence numbers, so the server can detect loss rather
  // than infer it from silence.
  command.sequence = ++beat_;
  BridgeRegion& r = *region_.get();
  if (!r.commands.push(command)) {
    r.header.commands_rejected.fetch_add(1, std::memory_order_relaxed);
    return false;
  }
  // Sending is itself proof of life; this keeps a busy client from having to
  // remember a separate heartbeat call.
  r.header.client_heartbeat.store(command.sequence, std::memory_order_release);
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
