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

constexpr unsigned simulated_joints = 6;
constexpr double rate_hertz = 500.0;

/// Set by the signal handler and read by the loop. The handler itself does
/// nothing else: async-signal-safety means setting a flag, never doing work.
std::atomic<bool> shutdown_requested{false};

void on_signal(int) { shutdown_requested.store(true, std::memory_order_release); }

void install_signal_handlers() {
  struct sigaction action {};
  action.sa_handler = on_signal;
  ::sigemptyset(&action.sa_mask);
  ::sigaction(SIGINT, &action, nullptr);
  ::sigaction(SIGTERM, &action, nullptr);
}

/// Stand-in for the arm: each joint tracks its target through a first-order lag.
/// Enough to produce plausible telemetry; it is not a dynamics model and does
/// not pretend to be one — core/model owns that.
struct Plant {
  double pos[simulated_joints]{};
  double vel[simulated_joints]{};

  void step(const double* target, double time_step, double bandwidth_hertz) {
    const double alpha = 1.0 - std::exp(-2.0 * M_PI * bandwidth_hertz * time_step);
    for (unsigned joint = 0; joint < simulated_joints; ++joint) {
      const double next = pos[joint] + (target[joint] - pos[joint]) * alpha;
      vel[joint] = (next - pos[joint]) / time_step;
      pos[joint] = next;
    }
  }
};

int run_server() {
  install_signal_handlers();
  // Before any thread exists: one malloc arena, small thread stacks. Without
  // this, mlockall(MCL_FUTURE) locks ~72 MiB per background thread.
  for (const auto& note : rt::prepare_process()) {
    std::printf("  rt        %s\n", note.c_str());
  }

  bridge::BridgeServer server;
  const auto period_nanoseconds = static_cast<std::uint32_t>(1e9 / rate_hertz);
  const bridge::RegionError error = server.open(bridge::default_region_name, period_nanoseconds,
                                              /*lock_memory=*/true);
  if (error != bridge::RegionError::ok && error != bridge::RegionError::mlock_failed) {
    std::fprintf(stderr, "bridge open failed: %s\n", to_string(error));
    return 1;
  }
  if (error == bridge::RegionError::mlock_failed) {
    std::fprintf(stderr, "warning: %s -- the bridge is pageable, timing will be worse\n",
                 to_string(error));
  }
  server.set_watchdog_timeout_cycles(50);  // 100 ms at 500 Hz

  telemetry::Provenance provenance = telemetry::Provenance::collect();
  provenance.label = "rc_core_demo (simulated plant)";
  provenance.control_rate_hz = rate_hertz;
  std::printf("rc_core_demo server\n%s", provenance.to_summary().c_str());
  std::string why_not;
  if (!provenance.suitable_as_baseline(why_not)) {
    std::printf("  baseline  NOT suitable: %s\n", why_not.c_str());
  }

  telemetry::FileSink sink;
  const std::string prefix = "/tmp/rc_demo_telemetry";
  if (!sink.open(prefix, provenance)) {
    std::fprintf(stderr, "could not open %s.bin\n", prefix.c_str());
    return 1;
  }
  sink.start(server);
  std::printf("  telemetry %s.bin (+ .json)\n  waiting for a client...\n\n", prefix.c_str());

  Plant plant;
  double target[simulated_joints]{};
  double hold[simulated_joints]{};
  ControlMode mode = ControlMode::idle;
  std::int64_t stop_started_ns = 0;
  constexpr std::int64_t ramp_nanoseconds = 300'000'000;  // ADR-0005 default T_stop

  rt::CyclicConfig config;
  config.period = rt::nanoseconds{period_nanoseconds};
  config.rt.priority = 0;  // raise once RLIMIT_RTPRIO is configured; see ADR-0007
  config.rt.lock_memory = true;
  rt::CyclicTask task(config);

  std::atomic<bool> stop_loop{false};
  std::uint64_t commands_seen = 0;
  // A publish() failure is only known after the record is gone, so the flag
  // rides on the *next* record. Otherwise it never reaches the file at all.
  bool telemetry_lost_pending = false;

  const rt::CyclicReport report = task.run_until(stop_loop, [&](std::uint64_t cycle, rt::nanoseconds period) {
    const rt::nanoseconds now = rt::monotonic_now();
    std::uint32_t flags = telemetry::flag_none;
    if (telemetry_lost_pending) {
      flags |= telemetry::flag_telemetry_lost;
      telemetry_lost_pending = false;
    }

    // ── 1. watchdog, before anything else trusts a client ──
    if (server.tick(cycle)) {
      std::printf("[cycle %llu] WATCHDOG: client lost -- Category 2 stop\n",
                  static_cast<unsigned long long>(cycle));
      std::fflush(stdout);
      // flag_client_lost is what lets the flight recorder distinguish "the
      // client died" from "the client asked us to stop": both reach stopping.
      flags |= telemetry::flag_client_lost;
      mode = ControlMode::stopping;
      stop_started_ns = now.count();
      std::memcpy(hold, plant.pos, sizeof(hold));
      server.set_state(ServerState::stopping);
    }

    // ── 2. commands, only while a client is genuinely in control ──
    bridge::CommandRecord command{};
    while (server.poll_command(command)) {
      ++commands_seen;
      const auto type = static_cast<CommandType>(command.type);
      if (mode == ControlMode::stopping || mode == ControlMode::holding) {
        // A stopped arm does not accept setpoints. Recovery is an explicit,
        // operator-initiated act (ADR-0005 §4) -- and only once the ramp has
        // finished, never mid-deceleration.
        if (type == CommandType::clear_fault && mode == ControlMode::holding) {
          std::printf("[cycle %llu] fault cleared by operator; idle\n",
                      static_cast<unsigned long long>(cycle));
          std::fflush(stdout);
          mode = ControlMode::idle;
          server.set_state(ServerState::idle);
        }
        continue;
      }
      switch (type) {
        case CommandType::set_target:
          for (unsigned joint = 0; joint < simulated_joints && joint < command.joint_count; ++joint) {
            target[joint] = static_cast<double>(command.pos[joint]);
          }
          mode = ControlMode::mit;
          server.set_state(ServerState::controlled);
          break;
        case CommandType::stop:
          mode = ControlMode::stopping;
          stop_started_ns = now.count();
          std::memcpy(hold, plant.pos, sizeof(hold));
          server.set_state(ServerState::stopping);
          break;
        case CommandType::disable:
          mode = ControlMode::idle;
          server.set_state(ServerState::idle);
          break;
        default:
          break;
      }
    }

    // ── 3. the Category 2 ramp: decelerate to the frozen pose, then hold ──
    if (mode == ControlMode::stopping) {
      flags |= telemetry::flag_stopping;
      const std::int64_t elapsed = now.count() - stop_started_ns;
      for (unsigned joint = 0; joint < simulated_joints; ++joint) {
        target[joint] = hold[joint];  // decelerate toward where we were when it tripped
      }
      if (elapsed >= ramp_nanoseconds) {
        mode = ControlMode::holding;
        // Correct the flag in the same cycle: a record that says mode=holding
        // while flagged stopping is a lie to whoever reads the telemetry later.
        flags = (flags & ~telemetry::flag_stopping) | telemetry::flag_holding;
        server.set_state(ServerState::holding);
        std::printf("[cycle %llu] holding compliantly at the stop pose\n",
                    static_cast<unsigned long long>(cycle));
        std::fflush(stdout);
      }
    } else if (mode == ControlMode::holding) {
      flags |= telemetry::flag_holding;
    }

    // ── 4. plant + publish ──
    plant.step(target, static_cast<double>(period.count()) / 1e9, /*bandwidth_hertz=*/5.0);

    telemetry::TelemetryRecord record{};
    record.cycle = cycle;
    record.deadline_ns = now.count();
    record.wake_ns = now.count();
    record.joint_count = simulated_joints;
    record.mode = static_cast<std::uint32_t>(mode);
    record.flags = flags;
    for (unsigned joint = 0; joint < simulated_joints; ++joint) {
      record.cmd_pos[joint] = static_cast<float>(target[joint]);
      record.meas_pos[joint] = static_cast<float>(plant.pos[joint]);
      record.meas_vel[joint] = static_cast<float>(plant.vel[joint]);
    }
    if (!server.publish(record)) {
      telemetry_lost_pending = true;  // surfaces on the next record
      flags |= telemetry::flag_telemetry_lost;  // and on this cycle's live snapshot
    }

    telemetry::StateSnapshot snapshot{};
    snapshot.cycle = cycle;
    snapshot.wake_ns = now.count();
    snapshot.joint_count = simulated_joints;
    snapshot.mode = record.mode;
    snapshot.flags = flags;
    for (unsigned joint = 0; joint < simulated_joints; ++joint) {
      snapshot.pos[joint] = record.meas_pos[joint];
      snapshot.vel[joint] = record.meas_vel[joint];
    }
    server.publish_snapshot(snapshot);

    if (shutdown_requested.load(std::memory_order_acquire)) {
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
  bridge::RegionError error = client.attach();
  for (int attempt = 0; error == bridge::RegionError::not_ready && attempt < 50; ++attempt) {
    ::usleep(10'000);  // server is mid-open(); this is the retryable case
    error = client.attach();
  }
  if (error != bridge::RegionError::ok) {
    std::fprintf(stderr, "attach failed: %s\n", to_string(error));
    return 1;
  }
  if (!client.take_control()) {
    std::fprintf(stderr, "another client holds control\n");
    return 1;
  }
  if (client.server_state() == ServerState::holding) {
    if (!clear_fault) {
      std::fprintf(stderr,
                   "server is HOLDING after a fault. Recovery is deliberate: re-run with "
                   "--clear-fault once you have checked the arm.\n");
      return 3;
    }
    bridge::CommandRecord clear{};
    clear.type = static_cast<std::uint32_t>(CommandType::clear_fault);
    if (!client.send(clear)) {
      std::fprintf(stderr, "could not send clear_fault\n");
      return 1;
    }
    std::printf("client: fault cleared\n");
  }
  std::printf("client: in control, period %u ns. Ctrl-C releases cleanly; "
              "`kill -9 %d` does not.\n",
              client.control_period_ns(), ::getpid());

  double phase = 0.0;
  while (!shutdown_requested.load(std::memory_order_acquire)) {
    bridge::CommandRecord command{};
    command.type = static_cast<std::uint32_t>(CommandType::set_target);
    command.joint_count = simulated_joints;
    for (unsigned joint = 0; joint < simulated_joints; ++joint) {
      command.pos[joint] = static_cast<float>(0.4 * std::sin(phase + 0.3 * joint));
    }
    if (!client.send(command)) {
      std::fprintf(stderr, "command ring full -- is the server draining?\n");
    }
    phase += 0.02;

    telemetry::StateSnapshot snapshot{};
    if (client.state(snapshot) && snapshot.cycle % 250 == 0) {
      std::printf("  cycle %8llu  j0=%+.3f  mode=%u\n",
                  static_cast<unsigned long long>(snapshot.cycle),
                  static_cast<double>(snapshot.pos[0]), snapshot.mode);
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
