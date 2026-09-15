/// rc_core_demo — the walking skeleton.
///
/// A 500 Hz cyclic loop with a simulated 6-joint plant, publishing telemetry and
/// taking commands over the bridge. No hardware, no CAN yet: what it demonstrates
/// is the *architecture* — that a client can die however it likes and the control
/// loop survives to execute a Category 2 stop
/// ([ADR-0005](../../docs/adr/0005-safe-state-and-stop-architecture.md)).
///
/// Two terminals:
///
///     ./rc_core_demo --server
///     ./rc_core_demo --client          # then: kill -9 the client
///
/// Watch the server ramp and hold. Then try Ctrl-C on the client instead, and
/// watch it release cleanly with no fault at all. That difference is the whole
/// point of the three-trigger taxonomy.

#include <signal.h>
#include <unistd.h>

#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <string>

#include "rc/bridge/client.hpp"
#include "rc/bridge/server.hpp"
#include "rc/rt/cyclic_task.hpp"
#include "rc/telemetry/file_sink.hpp"

using namespace rc;
using bridge::CommandType;
using bridge::ServerState;
using telemetry::ControlMode;

namespace {

constexpr unsigned kJoints = 6;
constexpr double kRateHz = 500.0;

/// Set by the signal handler and read by the loop. The handler itself does
/// nothing else: async-signal-safety means setting a flag, never doing work.
std::atomic<bool> g_shutdown{false};

void on_signal(int) { g_shutdown.store(true, std::memory_order_release); }

void install_signal_handlers() {
  struct sigaction sa {};
  sa.sa_handler = on_signal;
  ::sigemptyset(&sa.sa_mask);
  ::sigaction(SIGINT, &sa, nullptr);
  ::sigaction(SIGTERM, &sa, nullptr);
}

/// Stand-in for the arm: each joint tracks its target through a first-order lag.
/// Enough to produce plausible telemetry; it is not a dynamics model and does
/// not pretend to be one — core/model owns that.
struct Plant {
  double pos[kJoints]{};
  double vel[kJoints]{};

  void step(const double* target, double dt, double bandwidth_hz) {
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * bandwidth_hz * dt);
    for (unsigned j = 0; j < kJoints; ++j) {
      const double next = pos[j] + (target[j] - pos[j]) * alpha;
      vel[j] = (next - pos[j]) / dt;
      pos[j] = next;
    }
  }
};

int run_server() {
  install_signal_handlers();

  bridge::BridgeServer server;
  const auto period_ns = static_cast<std::uint32_t>(1e9 / kRateHz);
  const bridge::RegionError err = server.open(bridge::kDefaultRegionName, period_ns,
                                              /*lock_memory=*/true);
  if (err != bridge::RegionError::kOk && err != bridge::RegionError::kMlockFailed) {
    std::fprintf(stderr, "bridge open failed: %s\n", to_string(err));
    return 1;
  }
  if (err == bridge::RegionError::kMlockFailed) {
    std::fprintf(stderr, "warning: %s -- the bridge is pageable, timing will be worse\n",
                 to_string(err));
  }
  server.set_watchdog_timeout_cycles(50);  // 100 ms at 500 Hz

  telemetry::Provenance prov = telemetry::Provenance::collect();
  prov.label = "rc_core_demo (simulated plant)";
  prov.control_rate_hz = kRateHz;
  std::printf("rc_core_demo server\n%s", prov.to_summary().c_str());
  std::string why_not;
  if (!prov.suitable_as_baseline(why_not)) {
    std::printf("  baseline  NOT suitable: %s\n", why_not.c_str());
  }

  telemetry::FileSink sink;
  const std::string prefix = "/tmp/rc_demo_telemetry";
  if (!sink.open(prefix, prov)) {
    std::fprintf(stderr, "could not open %s.bin\n", prefix.c_str());
    return 1;
  }
  sink.start(server);
  std::printf("  telemetry %s.bin (+ .json)\n  waiting for a client...\n\n", prefix.c_str());

  Plant plant;
  double target[kJoints]{};
  double hold[kJoints]{};
  ControlMode mode = ControlMode::kIdle;
  std::int64_t stop_started_ns = 0;
  constexpr std::int64_t kRampNs = 300'000'000;  // ADR-0005 default T_stop

  rt::CyclicConfig cfg;
  cfg.period = rt::Nanos{period_ns};
  cfg.rt.priority = 0;  // raise once RLIMIT_RTPRIO is configured; see ADR-0007
  cfg.rt.lock_memory = true;
  rt::CyclicTask task(cfg);

  std::atomic<bool> stop_loop{false};
  std::uint64_t commands_seen = 0;
  // A publish() failure is only known after the record is gone, so the flag
  // rides on the *next* record. Otherwise it never reaches the file at all.
  bool telemetry_lost_pending = false;

  const rt::CyclicReport report = task.run_until(stop_loop, [&](std::uint64_t cycle, rt::Nanos dt) {
    const rt::Nanos now = rt::monotonic_now();
    std::uint32_t flags = telemetry::kFlagNone;
    if (telemetry_lost_pending) {
      flags |= telemetry::kFlagTelemetryLost;
      telemetry_lost_pending = false;
    }

    // ── 1. watchdog, before anything else trusts a client ──
    if (server.tick(cycle)) {
      std::printf("[cycle %llu] WATCHDOG: client lost -- Category 2 stop\n",
                  static_cast<unsigned long long>(cycle));
      std::fflush(stdout);
      // kFlagClientLost is what lets the flight recorder distinguish "the
      // client died" from "the client asked us to stop": both reach kStopping.
      flags |= telemetry::kFlagClientLost;
      mode = ControlMode::kStopping;
      stop_started_ns = now.count();
      std::memcpy(hold, plant.pos, sizeof(hold));
      server.set_state(ServerState::kStopping);
    }

    // ── 2. commands, only while a client is genuinely in control ──
    bridge::CommandRecord cmd{};
    while (server.poll_command(cmd)) {
      ++commands_seen;
      const auto type = static_cast<CommandType>(cmd.type);
      if (mode == ControlMode::kStopping || mode == ControlMode::kHolding) {
        // A stopped arm does not accept setpoints. Recovery is an explicit,
        // operator-initiated act (ADR-0005 §4) -- and only once the ramp has
        // finished, never mid-deceleration.
        if (type == CommandType::kClearFault && mode == ControlMode::kHolding) {
          std::printf("[cycle %llu] fault cleared by operator; idle\n",
                      static_cast<unsigned long long>(cycle));
          std::fflush(stdout);
          mode = ControlMode::kIdle;
          server.set_state(ServerState::kIdle);
        }
        continue;
      }
      switch (type) {
        case CommandType::kSetTarget:
          for (unsigned j = 0; j < kJoints && j < cmd.joint_count; ++j) {
            target[j] = static_cast<double>(cmd.pos[j]);
          }
          mode = ControlMode::kMit;
          server.set_state(ServerState::kControlled);
          break;
        case CommandType::kStop:
          mode = ControlMode::kStopping;
          stop_started_ns = now.count();
          std::memcpy(hold, plant.pos, sizeof(hold));
          server.set_state(ServerState::kStopping);
          break;
        case CommandType::kDisable:
          mode = ControlMode::kIdle;
          server.set_state(ServerState::kIdle);
          break;
        default:
          break;
      }
    }

    // ── 3. the Category 2 ramp: decelerate to the frozen pose, then hold ──
    if (mode == ControlMode::kStopping) {
      flags |= telemetry::kFlagStopping;
      const std::int64_t elapsed = now.count() - stop_started_ns;
      for (unsigned j = 0; j < kJoints; ++j) {
        target[j] = hold[j];  // decelerate toward where we were when it tripped
      }
      if (elapsed >= kRampNs) {
        mode = ControlMode::kHolding;
        // Correct the flag in the same cycle: a record that says mode=holding
        // while flagged stopping is a lie to whoever reads the telemetry later.
        flags = (flags & ~telemetry::kFlagStopping) | telemetry::kFlagHolding;
        server.set_state(ServerState::kHolding);
        std::printf("[cycle %llu] holding compliantly at the stop pose\n",
                    static_cast<unsigned long long>(cycle));
        std::fflush(stdout);
      }
    } else if (mode == ControlMode::kHolding) {
      flags |= telemetry::kFlagHolding;
    }

    // ── 4. plant + publish ──
    plant.step(target, static_cast<double>(dt.count()) / 1e9, /*bandwidth_hz=*/5.0);

    telemetry::TelemetryRecord rec{};
    rec.cycle = cycle;
    rec.deadline_ns = now.count();
    rec.wake_ns = now.count();
    rec.joint_count = kJoints;
    rec.mode = static_cast<std::uint32_t>(mode);
    rec.flags = flags;
    for (unsigned j = 0; j < kJoints; ++j) {
      rec.cmd_pos[j] = static_cast<float>(target[j]);
      rec.meas_pos[j] = static_cast<float>(plant.pos[j]);
      rec.meas_vel[j] = static_cast<float>(plant.vel[j]);
    }
    if (!server.publish(rec)) {
      telemetry_lost_pending = true;  // surfaces on the next record
      flags |= telemetry::kFlagTelemetryLost;  // and on this cycle's live snapshot
    }

    telemetry::StateSnapshot snap{};
    snap.cycle = cycle;
    snap.wake_ns = now.count();
    snap.joint_count = kJoints;
    snap.mode = rec.mode;
    snap.flags = flags;
    for (unsigned j = 0; j < kJoints; ++j) {
      snap.pos[j] = rec.meas_pos[j];
      snap.vel[j] = rec.meas_vel[j];
    }
    server.publish_snapshot(snap);

    if (g_shutdown.load(std::memory_order_acquire)) {
      stop_loop.store(true, std::memory_order_release);
    }
  });

  const std::uint64_t dropped = server.telemetry_dropped();
  const std::uint64_t trips = server.watchdog_trips();
  sink.stop();
  server.close();

  std::printf("\n%s", report.format().c_str());
  std::printf("  commands  %llu received\n  telemetry %llu records written, %llu dropped\n"
              "  watchdog  %llu trips\n",
              static_cast<unsigned long long>(commands_seen),
              static_cast<unsigned long long>(sink.records_written()),
              static_cast<unsigned long long>(dropped),
              static_cast<unsigned long long>(trips));
  return 0;
}

int run_client(bool clear_fault) {
  install_signal_handlers();
  bridge::BridgeClient client;
  bridge::RegionError err = client.attach();
  for (int attempt = 0; err == bridge::RegionError::kNotReady && attempt < 50; ++attempt) {
    ::usleep(10'000);  // server is mid-open(); this is the retryable case
    err = client.attach();
  }
  if (err != bridge::RegionError::kOk) {
    std::fprintf(stderr, "attach failed: %s\n", to_string(err));
    return 1;
  }
  if (!client.take_control()) {
    std::fprintf(stderr, "another client holds control\n");
    return 1;
  }
  if (client.server_state() == ServerState::kHolding) {
    if (!clear_fault) {
      std::fprintf(stderr,
                   "server is HOLDING after a fault. Recovery is deliberate: re-run with "
                   "--clear-fault once you have checked the arm.\n");
      return 3;
    }
    bridge::CommandRecord clear{};
    clear.type = static_cast<std::uint32_t>(CommandType::kClearFault);
    if (!client.send(clear)) {
      std::fprintf(stderr, "could not send kClearFault\n");
      return 1;
    }
    std::printf("client: fault cleared\n");
  }
  std::printf("client: in control, period %u ns. Ctrl-C releases cleanly; "
              "`kill -9 %d` does not.\n",
              client.control_period_ns(), ::getpid());

  double t = 0.0;
  while (!g_shutdown.load(std::memory_order_acquire)) {
    bridge::CommandRecord cmd{};
    cmd.type = static_cast<std::uint32_t>(CommandType::kSetTarget);
    cmd.joint_count = kJoints;
    for (unsigned j = 0; j < kJoints; ++j) {
      cmd.pos[j] = static_cast<float>(0.4 * std::sin(t + 0.3 * j));
    }
    if (!client.send(cmd)) {
      std::fprintf(stderr, "command ring full -- is the server draining?\n");
    }
    t += 0.02;

    telemetry::StateSnapshot snap{};
    if (client.state(snap) && snap.cycle % 250 == 0) {
      std::printf("  cycle %8llu  j0=%+.3f  mode=%u\n",
                  static_cast<unsigned long long>(snap.cycle),
                  static_cast<double>(snap.pos[0]), snap.mode);
      std::fflush(stdout);
    }
    ::usleep(20'000);  // 50 Hz: clients command at human rates, not loop rates
  }

  std::printf("client: releasing control cleanly (no fault)\n");
  client.release_control();
  return 0;
}

}  // namespace

int main(int argc, char** argv) {
  const std::string mode = argc > 1 ? argv[1] : "--server";
  const bool clear_fault = argc > 2 && std::string(argv[2]) == "--clear-fault";
  if (mode == "--client") {
    return run_client(clear_fault);
  }
  if (mode == "--server") {
    return run_server();
  }
  std::printf("usage: %s [--server | --client [--clear-fault]]\n", argv[0]);
  return 2;
}
