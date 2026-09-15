/// Two-process tests for the bridge.
///
/// The one that matters is test_watchdog_trips_on_client_kill(): it forks a real
/// client, has it take control and heartbeat, then SIGKILLs it -- the exact
/// event ADR-0006 restructured the architecture around. If the RT loop survives
/// that and trips its watchdog, the Category 2 stop in ADR-0005 is reachable.
/// If it did not, none of the safety design would be worth anything.

#include <signal.h>
#include <sys/wait.h>
#include <unistd.h>

#include <chrono>
#include <cstring>
#include <string>
#include <thread>

#include "rc/bridge/client.hpp"
#include "rc/bridge/server.hpp"
#include "rc/telemetry/file_sink.hpp"
#include "test_support.hpp"

using namespace rc::bridge;
using rc::telemetry::Provenance;
using rc::telemetry::StateSnapshot;
using rc::telemetry::TelemetryRecord;

namespace {

constexpr std::uint32_t kPeriodNs = 1'000'000;  // 1 ms, to keep tests quick

void sleep_ms(unsigned ms) { std::this_thread::sleep_for(std::chrono::milliseconds(ms)); }

void test_create_and_attach() {
  const std::string name = "/rc_test_attach";
  SharedRegion::unlink(name);

  BridgeServer server;
  const RegionError err = server.open(name, kPeriodNs, /*lock_memory=*/false);
  CHECK_MSG(err == RegionError::kOk, to_string(err));
  CHECK(server.valid());
  CHECK(server.state() == ServerState::kIdle);

  BridgeClient client;
  CHECK(client.attach(name) == RegionError::kOk);
  CHECK(client.attached());
  CHECK_EQ(client.control_period_ns(), kPeriodNs);
  CHECK_MSG(!client.in_control(), "attaching must not imply taking control");

  server.close();
  SharedRegion::unlink(name);
}

void test_attach_without_server() {
  SharedRegion::unlink("/rc_test_absent");
  BridgeClient client;
  const RegionError err = client.attach("/rc_test_absent");
  CHECK_MSG(err == RegionError::kNotFound,
            std::string("expected kNotFound, got: ") + to_string(err));
}

void test_command_round_trip() {
  const std::string name = "/rc_test_cmd";
  SharedRegion::unlink(name);
  BridgeServer server;
  CHECK(server.open(name, kPeriodNs, false) == RegionError::kOk);

  BridgeClient client;
  CHECK(client.attach(name) == RegionError::kOk);
  CHECK(client.take_control());

  CommandRecord cmd{};
  cmd.type = static_cast<std::uint32_t>(CommandType::kSetTarget);
  cmd.joint_count = 6;
  for (unsigned j = 0; j < 6; ++j) {
    cmd.pos[j] = static_cast<float>(j) * 0.1f;
  }
  CHECK(client.send(cmd));

  CommandRecord got{};
  CHECK(server.poll_command(got));
  CHECK_EQ(got.joint_count, 6u);
  CHECK_EQ(got.type, static_cast<std::uint32_t>(CommandType::kSetTarget));
  CHECK(got.pos[3] > 0.29f && got.pos[3] < 0.31f);
  CHECK_MSG(!server.poll_command(got), "queue must be empty after one send");

  // Overflow must be reported, never silently dropped or blocking.
  bool rejected = false;
  for (std::size_t i = 0; i < kCommandCapacity + 8; ++i) {
    if (!client.send(cmd)) {
      rejected = true;
      break;
    }
  }
  CHECK_MSG(rejected, "a full command ring must refuse rather than block");

  server.close();
  SharedRegion::unlink(name);
}

void test_snapshot_round_trip() {
  const std::string name = "/rc_test_snap";
  SharedRegion::unlink(name);
  BridgeServer server;
  CHECK(server.open(name, kPeriodNs, false) == RegionError::kOk);
  BridgeClient client;
  CHECK(client.attach(name) == RegionError::kOk);

  StateSnapshot snap{};
  snap.cycle = 42;
  snap.joint_count = 7;
  snap.pos[2] = 1.25f;
  server.publish_snapshot(snap);

  StateSnapshot got{};
  CHECK(client.state(got));
  CHECK_EQ(got.cycle, 42u);
  CHECK_EQ(got.joint_count, 7u);
  CHECK(got.pos[2] > 1.24f && got.pos[2] < 1.26f);

  server.close();
  SharedRegion::unlink(name);
}

void test_telemetry_drop_is_counted_not_blocking() {
  const std::string name = "/rc_test_drop";
  SharedRegion::unlink(name);
  BridgeServer server;
  CHECK(server.open(name, kPeriodNs, false) == RegionError::kOk);

  TelemetryRecord rec{};
  std::uint64_t pushed = 0;
  bool dropped = false;
  // Publish well past capacity with nobody draining. The RT side must keep
  // returning promptly -- a full ring is a dropped record, never a stall.
  for (std::size_t i = 0; i < kTelemetryCapacity * 2; ++i) {
    rec.cycle = i;
    if (server.publish(rec)) {
      ++pushed;
    } else {
      dropped = true;
    }
  }
  CHECK_MSG(dropped, "a full telemetry ring must drop");
  CHECK_EQ(pushed, static_cast<std::uint64_t>(kTelemetryCapacity));
  CHECK_MSG(server.telemetry_dropped() == kTelemetryCapacity,
            "drops must be counted so loss is visible, not inferred");

  server.close();
  SharedRegion::unlink(name);
}

void test_file_sink() {
  const std::string name = "/rc_test_sink";
  SharedRegion::unlink(name);
  BridgeServer server;
  CHECK(server.open(name, kPeriodNs, false) == RegionError::kOk);

  Provenance prov = Provenance::collect();
  prov.label = "unit test";
  prov.control_rate_hz = 1000.0;

  rc::telemetry::FileSink sink;
  const std::string prefix = "/tmp/rc_test_telemetry";
  CHECK(sink.open(prefix, prov));
  sink.start(server, /*poll_interval_ms=*/2);

  constexpr std::uint64_t kRecords = 500;
  for (std::uint64_t i = 0; i < kRecords; ++i) {
    TelemetryRecord rec{};
    rec.cycle = i;
    rec.joint_count = 7;
    rec.meas_pos[0] = static_cast<float>(i);
    CHECK(server.publish(rec));
    if (i % 64 == 0) {
      sleep_ms(1);  // let the drain thread keep up
    }
  }
  sleep_ms(30);
  sink.stop();
  CHECK_EQ(sink.records_written(), kRecords);

  // The file must be exactly N fixed-size records: that is what lets numpy read
  // it with one fromfile() and no parser.
  std::FILE* f = std::fopen((prefix + ".bin").c_str(), "rb");
  CHECK_MSG(f != nullptr, "telemetry file should exist");
  if (f != nullptr) {
    std::fseek(f, 0, SEEK_END);
    const long size = std::ftell(f);
    CHECK_EQ(static_cast<std::uint64_t>(size),
             kRecords * static_cast<std::uint64_t>(sizeof(TelemetryRecord)));
    std::fseek(f, 0, SEEK_SET);
    TelemetryRecord first{};
    CHECK_EQ(std::fread(&first, sizeof(first), 1, f), 1u);
    CHECK_EQ(first.joint_count, 7u);
    std::fclose(f);
  }

  server.close();
  SharedRegion::unlink(name);
}

/// Spawns a child that attaches, optionally takes control, heartbeats, and waits
/// to be killed. Returns the child pid.
pid_t spawn_client(const std::string& name, bool take_control) {
  const pid_t pid = ::fork();
  if (pid != 0) {
    return pid;
  }
  // ── child ──
  BridgeClient client;
  if (client.attach(name) != RegionError::kOk) {
    ::_exit(2);
  }
  if (take_control && !client.take_control()) {
    ::_exit(3);
  }
  for (;;) {
    client.heartbeat();
    ::usleep(500);  // 2 kHz: comfortably faster than the watchdog timeout
  }
}

/// The central test. A client takes control, proves liveness, then dies without
/// releasing -- SIGKILL, so no destructor, no handler, nothing.
void test_watchdog_trips_on_client_kill() {
  const std::string name = "/rc_test_watchdog";
  SharedRegion::unlink(name);
  BridgeServer server;
  CHECK(server.open(name, kPeriodNs, false) == RegionError::kOk);
  server.set_watchdog_timeout_cycles(20);  // 20 ms at 1 kHz

  const pid_t child = spawn_client(name, /*take_control=*/true);
  CHECK_MSG(child > 0, "fork failed");

  std::uint64_t cycle = 0;
  bool tripped_while_alive = false;
  bool tripped_after_kill = false;
  std::uint64_t trip_cycle = 0;
  bool killed = false;

  // 1 kHz loop, as the real core would run.
  for (; cycle < 3000; ++cycle) {
    const bool trip = server.tick(cycle);

    if (!killed) {
      if (trip) {
        tripped_while_alive = true;  // false positive: would stop a healthy arm
      }
      // Kill only once the client has demonstrably taken control and been seen.
      if (cycle > 200 && server.client_in_control()) {
        ::kill(child, SIGKILL);
        int status = 0;
        ::waitpid(child, &status, 0);
        killed = true;
      }
    } else if (trip && !tripped_after_kill) {
      tripped_after_kill = true;
      trip_cycle = cycle;
    }
    ::usleep(1000);
    if (tripped_after_kill && cycle > trip_cycle + 100) {
      break;
    }
  }

  CHECK_MSG(killed, "client never took control, so the watchdog was never armed");
  CHECK_MSG(!tripped_while_alive, "watchdog fired while the client was heartbeating");
  CHECK_MSG(tripped_after_kill, "watchdog did NOT fire after the client was SIGKILLed");
  CHECK_MSG(server.watchdog_trips() == 1,
            "watchdog must fire exactly once, not re-trigger every cycle mid-ramp: got " +
                std::to_string(server.watchdog_trips()));

  server.close();
  SharedRegion::unlink(name);
}

/// The corollary, and the reason take_control() is a separate step: a client
/// that only observes -- a plot, a logger, a UI -- can die freely. Only a client
/// that accepted responsibility is held to it.
void test_observer_death_does_not_trip_watchdog() {
  const std::string name = "/rc_test_observer";
  SharedRegion::unlink(name);
  BridgeServer server;
  CHECK(server.open(name, kPeriodNs, false) == RegionError::kOk);
  server.set_watchdog_timeout_cycles(20);

  const pid_t child = spawn_client(name, /*take_control=*/false);
  CHECK_MSG(child > 0, "fork failed");
  sleep_ms(50);
  ::kill(child, SIGKILL);
  int status = 0;
  ::waitpid(child, &status, 0);

  bool tripped = false;
  for (std::uint64_t cycle = 0; cycle < 500; ++cycle) {
    if (server.tick(cycle)) {
      tripped = true;
    }
    ::usleep(200);
  }
  CHECK_MSG(!tripped, "an observer that never took control must not arm the watchdog");

  server.close();
  SharedRegion::unlink(name);
}

/// A second client must not be able to seize control from the first.
void test_control_is_exclusive() {
  const std::string name = "/rc_test_excl";
  SharedRegion::unlink(name);
  BridgeServer server;
  CHECK(server.open(name, kPeriodNs, false) == RegionError::kOk);

  BridgeClient a;
  BridgeClient b;
  CHECK(a.attach(name) == RegionError::kOk);
  CHECK(b.attach(name) == RegionError::kOk);
  CHECK(a.take_control());
  CHECK_MSG(!b.take_control(), "control must be exclusive");
  a.release_control();
  CHECK_MSG(b.take_control(), "control must be available after a clean release");

  server.close();
  SharedRegion::unlink(name);
}

}  // namespace

int main() {
  test_create_and_attach();
  test_attach_without_server();
  test_command_round_trip();
  test_snapshot_round_trip();
  test_telemetry_drop_is_counted_not_blocking();
  test_file_sink();
  test_control_is_exclusive();
  test_observer_death_does_not_trip_watchdog();
  test_watchdog_trips_on_client_kill();
  return rc::test::finish("bridge");
}
