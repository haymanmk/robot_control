# core — the control library

C++20, plain CMake, **no ROS and no Python**. That constraint is deliberate
([ADR-0003](../docs/adr/0003-cpp-control-core-and-layering.md)): the core builds,
tests and measures on its own, which is what keeps the real-time boundary honest
as features accrete.

| Module | Status | Contents |
|---|---|---|
| [`rt/`](rt/) | ✅ | `CLOCK_MONOTONIC` clock, phase-locked `CyclicTask`, `SpscRing`, `Seqlock`, RT privileges, allocation-free histograms |
| [`telemetry/`](telemetry/) | ✅ | fixed-size per-cycle record, provenance collection, file sink |
| [`bridge/`](bridge/) | ✅ | shared-memory transport, watchdog, server/client endpoints |
| `can/` | — | SocketCAN with `SO_TIMESTAMPING`, frame codec, bus statistics |
| `drive/` | — | RobStride protocol |
| `model/` | — | kinematics + dynamics (Pinocchio) |
| `control/` | — | gravity compensation, impedance, trajectory tracking |
| `safety/` | — | limits, stop-category supervisor ([ADR-0005](../docs/adr/0005-safe-state-and-stop-architecture.md)) |

## Rules for the cyclic path

Not style preferences. These are what "real-time" means here, and every function
callable from inside the 500 Hz loop obeys all of them:

- **No allocation.** No `new`, no growing containers, no `std::string`.
- **No locks a non-RT thread can hold.** Lock-free only. A mutex lets a client
  that was killed mid-critical-section block the control loop forever.
- **No logging, no I/O** except the fieldbus itself.
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

Two processes, so a client can segfault, hang, or be `SIGKILL`ed and the loop
survives to execute a Category 2 stop. That is the whole reason for the
separation ([ADR-0006](../docs/adr/0006-process-topology-and-rt-client-transport.md)).

Note `take_control()` is a *separate step* from `attach()`. A client that only
observes never arms the watchdog, so a plotting script dying is a non-event;
only a client that accepted responsibility is held to it.

## Try it

```bash
cmake -B build && cmake --build build -j && ctest --test-dir build --output-on-failure

# two terminals
./build/app/rc_core_demo/rc_core_demo --server
./build/app/rc_core_demo/rc_core_demo --client

# then kill -9 the client, and read what happened
python3 tools/telemetry_dump.py /tmp/rc_demo_telemetry
```

`Ctrl-C` on the client releases cleanly and causes no fault. `kill -9` trips the
watchdog and starts the ramp. That difference is the three-trigger taxonomy in
ADR-0005, made observable.

## Verifying the lock-free code

Wrong memory ordering fails rarely, unreproducibly, and never under a debugger.
So:

```bash
cmake -B build-tsan -DRC_SANITIZE=thread && cmake --build build-tsan -j
ctest --test-dir build-tsan --output-on-failure
```

Both suites are clean under ThreadSanitizer, verified against a positive control
so the silence means something.
