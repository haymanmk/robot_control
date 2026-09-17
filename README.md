# robot_control

A robot control system with real-time performance and a full toolchain. It is
built from the ground up on a **Seeed Studio reBot Arm B601-RS**. The goal is
to learn the fundamentals of robot control by building a real system, not by
reading about one.

## Target hardware

The arm has 6 joints plus a gripper. It uses 7 RobStride quasi-direct-drive
motors on a **CAN bus**, with a nominal control rate of 500 Hz. The full
details, and what they mean for the design, are in
[ADR-0002](docs/adr/0002-target-platform-rebot-b601-rs.md).

## Reference baseline

The manufacturer has open-sourced the whole stack (software under Apache-2.0,
hardware under CERN-OHL-W-2.0). We read it and learn from it, then build our
own control path:

- [`Seeed-Projects/reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py) — hardware layer, Pinocchio kinematics and dynamics, controllers, trajectories, gravity calibration
- [`Seeed-Projects/reBot-DevArm`](https://github.com/Seeed-Projects/reBot-DevArm) — hardware, bill of materials, URDF

## Build

```bash
cmake -B build && cmake --build build -j
ctest --test-dir build --output-on-failure
./build/bench/loop_timing/bench_loop_timing --seconds 10
```

C++20, plain CMake, no dependencies beyond the C library. The control core
does not need ROS to build, test, or measure. That is on purpose; see
[ADR-0003](docs/adr/0003-cpp-control-core-and-layering.md).

## Status

| Component | State |
|---|---|
| `core/rt` — clock, cyclic executive, lock-free rings, real-time setup | done |
| `core/telemetry` — per-cycle record, provenance, file sink | done |
| `core/bridge` — shared-memory transport, watchdog | done |
| `core/can`, `core/drive`, `core/model`, `core/control`, `core/safety` | next |

See [`core/README.md`](core/README.md) for the layer map and the rules for
real-time code.

## Where to start

- [`docs/ROADMAP.md`](docs/ROADMAP.md) — the sequence of labs and what each one teaches
- [`bench/`](bench/) — the performance benchmarks; [loop_timing](bench/loop_timing/) needs no hardware
- [`notebooks/`](notebooks/) — analysis and write-ups, kept as plain `.py` files
- `tools/telemetry_dump.py` — reads a telemetry run with nothing installed
- [`docs/rt-setup.md`](docs/rt-setup.md) — locked memory and real-time priority limits: examine, then raise
- [`docs/adr/`](docs/adr/) — the design decisions and the reasons behind them

## Decisions so far

| ADR | Decision |
|---|---|
| [0001](docs/adr/0001-rtos-and-middleware-selection.md) | Three tiers: a hard real-time core, a lock-free bridge, and a ROS2/UI tier. Start on PREEMPT_RT; consider Xenomai only if measurements demand it. |
| [0002](docs/adr/0002-target-platform-rebot-b601-rs.md) | The target is the B601-RS over SocketCAN. Our loop sends setpoints; the drives close their own current loops. CAN bus load, not kernel latency, is the main constraint. |
| [0003](docs/adr/0003-cpp-control-core-and-layering.md) | C++20 control core with a plain CMake build, layered `rt → can → drive → model → control`. Chosen for control over the cyclic path, not for speed. |
| [0004](docs/adr/0004-system-decomposition.md) | A standalone C++ `core/` with no ROS and no Python. Telemetry is always on and is part of the product. A four-tier list of what we measure. |
| [0005](docs/adr/0005-safe-state-and-stop-architecture.md) | The arm has no brakes, so there is no safe *unpowered* state. The default fault response is a controlled stop that stays powered (IEC 60204-1 Category 2). Return-to-home is a shutdown and recovery feature, never a fault response. |
| [0006](docs/adr/0006-process-topology-and-rt-client-transport.md) | Live control runs in a **separate process** from any client, over shared memory. A client can crash without taking the control loop with it. |
| [0007](docs/adr/0007-rt-platform-on-a-cuda-laptop.md) | One laptop with the NVIDIA driver, needed for CUDA inference. PREEMPT_RT with CPU isolation first; Xenomai only as a fallback. Run `hwlatdetect` before drawing any conclusion about kernels. |
