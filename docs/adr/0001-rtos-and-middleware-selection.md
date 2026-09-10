# ADR-0001: RTOS and Middleware Selection for the Robot Control System

**Status:** Proposed
**Date:** 2026-09-10
**Deciders:** TBD

## Context

We are drafting the system architecture for a robot control system with two headline requirements:

1. **Real-time performance** — deterministic execution of the control path (fieldbus cycle, motor/servo control, servo-level interpolation).
2. **Full-stack toolchain** — simulation, visualization, planning, and a control interface (UI), ideally from an existing ecosystem rather than built from scratch.

The initial proposal is **Xenomai** (dual-kernel Cobalt) for the hard real-time tier — motivated by benchmarks showing lower worst-case latency than PREEMPT_RT — combined with **ROS2** for IPC and the surrounding toolchain, with time-sensitive tasks (safety, IK, trajectory control, low-level motor control) pinned to isolated CPUs, and a backend/frontend paradigm for the UI.

Forces at play:

- Worst-case latency/jitter requirements are **not yet quantified** (loop rate, jitter budget, axis count, fieldbus are TBD).
- ROS2/DDS provides no hard real-time guarantees; any architecture must keep it out of the cyclic control path.
- PREEMPT_RT was merged into the mainline kernel (6.12, late 2024), changing its maintenance profile.
- Xenomai 3 project maintenance has been uncertain; the lead developer's effort moved to EVL ("Xenomai 4"/Dovetail), which has a different API.
- If the robot operates near people or requires certification, functional safety is governed by ISO 13849 / IEC 61508 / ISO 10218.

## Decision

Proposed (pending measurement):

1. **Prototype the hard-RT tier on mainline PREEMPT_RT with CPU isolation first**, and adopt Xenomai only if measured worst-case latency on target hardware fails the (to-be-written) timing budget.
2. **Treat the RT/non-RT boundary as the first-class design element**: a lock-free shared-memory interface carrying fixed-size structs between the RT core and the ROS2 tier, regardless of kernel choice.
3. **Keep ROS2/DDS entirely out of the hard-RT path**; evaluate `ros2_control` before hand-rolling the RT core.
4. **Implement functional safety in hardware** (STO, safety relays/PLC, safety-rated e-stops); software performs supervisory checks only.
5. **Backend/frontend UI in the non-RT tier**, with Foxglove for the engineering UI and custom frontend effort reserved for the operator UI.

## Options Considered

### Option A: Xenomai (Cobalt dual-kernel) + ROS2

| Dimension | Assessment |
|-----------|------------|
| Complexity | High — dual-domain programming model, RTDM drivers, hand-rolled RT↔ROS2 bridge (XDDP/shared memory) |
| Worst-case latency | Best in class (single-digit µs typical) |
| Ecosystem fit | Poor — ROS2, `ros2_control`, and standard drivers are not Cobalt-aware |
| Maintenance risk | High — Xenomai 3 maintenance uncertain; successor (EVL/Xenomai 4) is a different API |
| Team familiarity | TBD |

**Pros:**
- Lowest achievable worst-case latency; suits ≥5–20 kHz loops or sub-5 µs jitter budgets (e.g., software FOC/current loops).
- Strong isolation of the RT domain from Linux activity by construction.

**Cons:**
- RT threads must avoid ordinary Linux syscalls or they silently migrate to the Linux domain; mode-switch bugs are hard to find.
- Drivers in the RT path must be RTDM; RT Ethernet (RTnet) is essentially unmaintained — verify EtherCAT master support before committing.
- Gives up most of `ros2_control`; the RT↔ROS2 bridge and controller lifecycle are built and maintained in-house.
- Weaker debugging/observability tooling; small community.

### Option B: Mainline PREEMPT_RT + CPU isolation + ros2_control (recommended starting point)

| Dimension | Assessment |
|-----------|------------|
| Complexity | Moderate — single kernel, standard POSIX RT APIs, established tuning recipe |
| Worst-case latency | Tens of µs worst case on tuned hardware; often <10 µs — ample for 1 kHz servo/EtherCAT cycles |
| Ecosystem fit | Excellent — `ros2_control`, `gz_ros2_control`, UR/Franka/Kuka drivers all use this path |
| Maintenance risk | Low — mainline kernel feature since 6.12 |
| Team familiarity | TBD |

**Pros:**
- Same controller code runs in simulation (Gazebo via `gz_ros2_control`) and on hardware — directly serves the full-toolchain goal.
- One kernel, one toolchain, standard debugging (ftrace, perf, gdb).
- EtherCAT masters (IgH, SOEM) are proven on PREEMPT_RT.

**Cons:**
- Higher worst-case latency than Xenomai; must be verified under load on target hardware, not assumed.
- Requires disciplined tuning: `isolcpus`, `nohz_full`, `rcu_nocbs`, IRQ affinity, NIC queue steering, BIOS C-state/SMI management.
- RT paths must still avoid allocation, page faults (`mlockall`), and priority-inversion hazards — determinism is earned, not granted.

### Option C: Hybrid — separate MCU/hardware for the servo loop

Low-level motor control on a dedicated MCU or the drives themselves (e.g., CiA 402 drives closing current/velocity loops); the Linux side runs at trajectory/interpolation rate.

**Pros:** Relaxes the Linux-side jitter budget dramatically; often eliminates the Xenomai question entirely.
**Cons:** More hardware integration; firmware toolchain to maintain; partitioning decisions move into hardware.

*Not mutually exclusive with A/B — worth considering if drives already close their own loops.*

## Trade-off Analysis

The deciding variable is the **quantified timing budget**, which does not exist yet. For the classic 1 kHz cycle, Option B's measured worst-case jitter on tuned hardware comfortably meets requirements while preserving the entire ROS2 toolchain — the ecosystem cost of Option A buys latency headroom the system may not need. Option A becomes rational only when measurements prove B insufficient (roughly: ≥5–20 kHz loops or <5 µs jitter).

Benchmark posts favoring Xenomai frequently use idle systems, older kernels, or Raspberry Pi-class hardware; they are not evidence about *our* target under *our* load. The only benchmark that counts is `cyclictest` plus in-loop instrumentation on target hardware under representative load, run for 24 h or more.

Independent of kernel choice, the architecture is the same three tiers:

1. **Hard-RT tier** (isolated CPUs): fieldbus cycle, motor/servo control, servo-level interpolation, servo-level IK (if 1 kHz Cartesian servoing is required). No ROS, no DDS, no allocation, no logging in the cyclic path.
2. **Bridge:** lock-free shared-memory ring buffers with fixed-size structs (setpoints in, joint states out). Specified early, versioned.
3. **ROS2 tier** (non-isolated CPUs): planning-level IK and trajectory *generation* (MoveIt2), state publishing, diagnostics, bagging, simulation, UI backend. DDS threads and NIC IRQs pinned away from isolated cores; Cyclone DDS + iceoryx zero-copy transport if high-rate local traffic emerges.

**Task placement corrections to the original proposal:**

- *IK/trajectory:* servo-level interpolation and servo-level IK belong in the RT tier; planning-level IK and trajectory generation are non-deterministic and belong in the ROS2 tier, delivering time-parameterized trajectories for the RT interpolator to sample.
- *Safety:* functional safety must not be a software task on the control PC. The safety chain is hardware (STO on drives, safety relays/PLC, safety-rated e-stops, encoder-based SLS as needed) per ISO 13849 / IEC 61508. Software implements supervisory checks (soft limits, watchdogs, torque sanity) as a second line of defense, not the safety case.

**UI:** backend (rclcpp/rclpy or rosbridge/Foxglove WebSocket bridge) + web frontend, strictly non-RT. Use Foxglove for the engineering UI (plots, 3D, topic inspection, bag playback); spend custom frontend effort on the operator UI (jogging, program management, status).

## Consequences

- **Easier:** simulation-to-hardware parity (`ros2_control` in Gazebo and on the robot), hiring/onboarding (mainline kernel, standard ROS2), debugging, long-term maintenance.
- **Easier:** the kernel decision stays reversible — the RT core sits behind a thin platform abstraction (threading, clocking, IPC), so a later Xenomai migration changes the platform layer, not the controllers.
- **Harder:** we must write and defend a timing budget, build a latency test rig, and hold the "no DDS in the RT path" line as features accrete.
- **Revisit if:** measured worst-case latency fails the budget; loop rates rise toward ≥5–20 kHz; drive hardware changes (e.g., drives close their own loops, enabling Option C); EVL/Xenomai 4 matures with credible EtherCAT and community support.

## Action Items

1. [ ] Write the timing budget: loop rates, jitter budget, axis count, fieldbus, safety category.
2. [ ] Stand up PREEMPT_RT on target hardware with CPU isolation (`isolcpus`, `nohz_full`, `rcu_nocbs`, IRQ affinity, BIOS C-state/SMI tuning).
3. [ ] Run `cyclictest` + in-loop instrumentation under representative load for ≥24 h; record worst-case numbers against the budget.
4. [ ] Evaluate `ros2_control` against the drive interface (IgH or SOEM EtherCAT master) before hand-rolling an RT core.
5. [ ] Define the RT↔ROS2 shared-memory interface (fixed-size structs, versioning) as a standalone spec.
6. [ ] Decide the hardware safety architecture (STO, relays/PLC, e-stop chain) before the software architecture ossifies.
7. [ ] Prototype the UI path: Foxglove bridge for engineering; skeleton backend/frontend for operator UI.
