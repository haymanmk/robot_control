# core — the control library

C++20, plain CMake, **no ROS and no Python.** That constraint is deliberate
([ADR-0003](../docs/adr/0003-cpp-control-core-and-layering.md)). The core
builds, tests, and measures on its own. That is what keeps the real-time
boundary honest as features are added.

| Module | Status | Contents |
|---|---|---|
| [`rt/`](rt/) | done | `CLOCK_MONOTONIC` clock; phase-locked `CyclicTask`; `SpscRing`; `Seqlock`; real-time privileges; allocation-free histograms |
| [`telemetry/`](telemetry/) | done | fixed-size per-cycle record; provenance collection; file sink |
| [`bridge/`](bridge/) | done | shared-memory transport; watchdog; server and client endpoints |
| `can/` | — | SocketCAN with `SO_TIMESTAMPING`; frame encoding; bus statistics |
| `drive/` | — | RobStride protocol |
| `model/` | — | kinematics and dynamics (Pinocchio) |
| `control/` | — | gravity compensation, impedance, trajectory tracking |
| `safety/` | — | limits; stop-category supervisor ([ADR-0005](../docs/adr/0005-safe-state-and-stop-architecture.md)) |

## Rules for the cyclic path

These are not style preferences. They are what "real-time" means in this
project, and every function that can be called from inside the 500 Hz loop
follows all of them:

- **No allocation.** No `new`, no growing containers, no `std::string`.
- **No locks that a non-real-time thread could hold.** Lock-free only. With a
  mutex, a client killed in the middle of a critical section could block the
  control loop forever.
- **No logging and no I/O,** except the fieldbus itself.
- **No exceptions across the cycle boundary.**
- **Every cyclic function is `noexcept`** and documents its worst-case cost.

## The bridge in one picture

```
   RT core process                      client process (Python / ROS2 / UI)
  ┌──────────────────┐                 ┌──────────────────────────────┐
  │ CyclicTask 500Hz │                 │  BridgeClient                │
  │   tick()  ───────┼── heartbeat ◄───┼──  heartbeat()               │
  │   poll_command()◄┼── commands   ◄──┼──  send()                    │
  │   publish()   ───┼── telemetry  ───┼─►  (drained by FileSink)     │
  │   snapshot()  ───┼── latest     ───┼─►  state()                   │
  └────────┬─────────┘   POSIX shm     └──────────────────────────────┘
           │
      FileSink thread ──► run.bin + run.json
```

Two processes, so a client can crash, hang, or be killed with `kill -9`, and
the loop survives to carry out a Category 2 stop. That is the whole reason for
the separation ([ADR-0006](../docs/adr/0006-process-topology-and-rt-client-transport.md)).

Note that `take_control()` is a *separate step* from `attach()`. A client that
only observes never arms the watchdog, so a plotting script that dies is a
non-event. Only a client that has accepted responsibility is held to it.

## Try it

```bash
cmake -B build && cmake --build build -j && ctest --test-dir build --output-on-failure

# first: can this user run real-time code on this machine?
./build/app/rc_rtcheck/rc_rtcheck        # see docs/rt-setup.md if it says NOT READY

# in two terminals
./build/app/rc_core_demo/rc_core_demo --server
./build/app/rc_core_demo/rc_core_demo --client

# then kill -9 the client, and read what happened
python3 tools/telemetry_dump.py /tmp/rc_demo_telemetry

# a new client is refused while the server is holding after a fault;
# recovery is a deliberate act
./build/app/rc_core_demo/rc_core_demo --client --clear-fault
```

`Ctrl-C` on the client releases control cleanly and causes no fault. `kill -9`
trips the watchdog and starts the ramp. That difference is the three-trigger
table in ADR-0005, made visible.

## Checking the lock-free code

Wrong memory ordering fails rarely, is hard to reproduce, and never shows up
under a debugger. So:

```bash
cmake -B build-tsan -DRC_SANITIZE=thread && cmake --build build-tsan -j
ctest --test-dir build-tsan --output-on-failure
```

Both test suites pass under ThreadSanitizer. This was checked against a known
race first, so a clean result means something.

One limit to know about: GCC's ThreadSanitizer does **not** model
`atomic_thread_fence` (it prints a warning saying so). The seqlock therefore
stores its payload through word-sized `std::atomic_ref`, so a clean TSan result
proves "no data race" by construction. The argument that the fence *ordering*
is right comes from Boehm (2012), and the tearing check in
`tests/test_rings.cpp` is its empirical test.

## Review history

The first version of the bridge shipped with three watchdog defects. A code
review caught and reproduced them before any hardware saw the code:

- a client polling for control could keep a dead controller looking alive;
- a tripped token was never revoked, so no successor could ever recover;
- releasing and re-taking control between two ticks left the watchdog stuck.

Each has a named regression test in `tests/test_bridge.cpp`. The lesson is not
that the code was careless. It had tests, and it passed them. The lesson is
that a watchdog's tests must include the *badly behaved* client, not only the
well-behaved one.
