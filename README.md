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
./build/labs/01_loop_timing/cpp/lab01_loop_timing --seconds 10
```

C++20, plain CMake, no dependencies beyond libc. The control core deliberately
does not require ROS to build, test, or measure — see [ADR-0003](docs/adr/0003-cpp-control-core-and-layering.md).

## Where to start

- [`docs/ROADMAP.md`](docs/ROADMAP.md) — the milestone sequence and what each one teaches
- [`labs/`](labs/) — runnable experiments; **[Lab 01](labs/01_loop_timing/) needs no hardware**
- [`docs/adr/`](docs/adr/) — architecture decisions and why they were made

## Decisions so far

| ADR | Decision |
|---|---|
| [0001](docs/adr/0001-rtos-and-middleware-selection.md) | Three-tier architecture: hard-RT core / lock-free bridge / ROS2 + UI. Prototype on PREEMPT_RT before considering Xenomai. |
| [0002](docs/adr/0002-target-platform-rebot-b601-rs.md) | Target is B601-RS over SocketCAN. Xenomai closed for this robot: our loop is an outer setpoint loop, and CAN bus load — not kernel latency — is the binding constraint. |
| [0003](docs/adr/0003-cpp-control-core-and-layering.md) | C++20 control core with a plain-CMake build, layered `rt → can → drive → model → control`. Chosen for control over the cyclic path, not for speed — Lab 01 shows C++ and Python hit a 2 ms deadline equally well. |
