# robot_control

A robot control system with real-time performance and a full toolchain — built
from the ground up on a **Seeed Studio reBot Arm B601-RS**, as a way of learning
the fundamentals by doing rather than by reading.

## Target hardware

6 DoF + gripper, 7 RobStride quasi-direct-drive motors on a **CAN** bus,
500 Hz nominal outer loop. Full details and the architectural consequences:
[ADR-0002](docs/adr/0002-target-platform-rebot-b601-rs.md).

## Reference baseline

The manufacturer open-sourced the whole stack (Apache-2.0 software,
CERN-OHL-W-2.0 hardware). We read it, learn from it, and re-derive the control
path ourselves:

- [`Seeed-Projects/reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py) — HAL, Pinocchio kinematics/dynamics, controllers, trajectories, gravity calibration
- [`Seeed-Projects/reBot-DevArm`](https://github.com/Seeed-Projects/reBot-DevArm) — hardware, BOM, URDF

## Build

```bash
cmake -B build && cmake --build build -j
./build/bench/loop_timing/bench_loop_timing --seconds 10
```

C++20, plain CMake, no dependencies beyond libc. The control core deliberately
does not require ROS to build, test, or measure — see [ADR-0003](docs/adr/0003-cpp-control-core-and-layering.md).

## Where to start

- [`docs/ROADMAP.md`](docs/ROADMAP.md) — the milestone sequence and what each one teaches
- [`bench/`](bench/) — the performance regression suite; **[loop_timing](bench/loop_timing/) needs no hardware**
- [`notebooks/`](notebooks/) — analysis and write-ups, as plain `.py`
- [`docs/adr/`](docs/adr/) — architecture decisions and why they were made

## Decisions so far

| ADR | Decision |
|---|---|
| [0001](docs/adr/0001-rtos-and-middleware-selection.md) | Three-tier architecture: hard-RT core / lock-free bridge / ROS2 + UI. Prototype on PREEMPT_RT before considering Xenomai. |
| [0002](docs/adr/0002-target-platform-rebot-b601-rs.md) | Target is B601-RS over SocketCAN. Xenomai closed for this robot: our loop is an outer setpoint loop, and CAN bus load — not kernel latency — is the binding constraint. |
| [0007](docs/adr/0007-rt-platform-on-a-cuda-laptop.md) | One laptop, NVIDIA driver mandatory for CUDA inference. PREEMPT_RT + CPU isolation first, measured under inference load; Xenomai reopened as a contingency. `hwlatdetect` runs before any kernel conclusion — if firmware SMIs are the floor, no kernel fixes it. |
| [0006](docs/adr/0006-process-topology-and-rt-client-transport.md) | Live control runs in a **separate process** from the client, over a shared-memory bridge. In-process bindings would take the RT core down with the interpreter — exactly when the watchdog must survive. DDS stays at the ROS2 tier; in-process pybind11 is kept for offline analysis. |
| [0005](docs/adr/0005-safe-state-and-stop-architecture.md) | No brakes, so there is no unpowered safe state. Default fault response is IEC 60204-1 **Category 2**: decelerate, then hold *powered* at low stiffness with gravity feedforward. Return-to-home is a graceful-shutdown and recovery feature, never a fault response. |
| [0004](docs/adr/0004-system-decomposition.md) | Standalone C++ `core/` (no ROS, no Python), pybind11 bindings that can only *command and observe*, ROS2 last. Telemetry is always-on and part of the product, with a four-tier measurement taxonomy. |
| [0003](docs/adr/0003-cpp-control-core-and-layering.md) | C++20 control core with a plain-CMake build, layered `rt → can → drive → model → control`. Chosen for control over the cyclic path, not for speed — Lab 01 shows C++ and Python hit a 2 ms deadline equally well. |
