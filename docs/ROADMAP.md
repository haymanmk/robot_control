# Roadmap — learning a robot control system by building one

The rule of this project: **every concept arrives attached to something that runs
on the real arm.** No milestone is "read about X". Each one is a lab with a
measurable result, and the fundamentals show up as the thing you needed in order
to make the measurement come out right.

Reference baseline: [`Seeed-Projects/reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py)
(Apache-2.0) and [`Seeed-Projects/reBot-DevArm`](https://github.com/Seeed-Projects/reBot-DevArm)
(hardware, URDF). Target: **reBot Arm B601-RS** — see [ADR-0002](adr/0002-target-platform-rebot-b601-rs.md).

---

**Where a lab's artifacts live:** a lab is a pedagogical unit, not a directory.
It produces a regression benchmark in `bench/`, an analysis write-up in
`notebooks/`, and code in `core/` — see [ADR-0004](adr/0004-system-decomposition.md).

Implementation language is **C++20** for everything cyclic; see
[ADR-0003](adr/0003-cpp-control-core-and-layering.md) for the layering and the
rules the cyclic path lives by. `core/rt` — the clock, the cyclic executive, RT
privileges, and allocation-free instrumentation — is written and is what Lab 01
demonstrates.

## Phase 0 — Timing, the bus, and what the drives do when you die

> ⛔ **Lab 00 is blocking.** Nothing moves under its own power until it is done.

| Lab | Question it answers | Fundamentals it forces you to learn |
|---|---|---|
| **00 — drive comms loss** ⛔ | When command frames stop arriving, does a RobStride drive hold its last setpoint or disable? | Fail-safe vs fail-operational, why a brakeless arm has no unpowered safe state, dependency ranking of stop paths ([ADR-0005](adr/0005-safe-state-and-stop-architecture.md)) |
| **01 — loop timing** ✅ | How wrong is `time.sleep(dt - elapsed)`, in microseconds? | Monotonic vs wall clocks, absolute vs relative deadlines, phase-locked loops, drift vs jitter, percentiles over averages, `SCHED_FIFO`, `mlockall`, why the GIL is not your main problem here |
| 00b — the firmware floor ⛔ | Is the latency floor on this laptop set by the kernel or by SMM firmware? | `hwlatdetect`, System Management Interrupts, why a dual-kernel cannot fix a firmware stall ([ADR-0007](adr/0007-rt-platform-on-a-cuda-laptop.md)) |
| 02 — the CAN bus | What is actually on the wire at 500 Hz? | CAN framing, arbitration, bit stuffing, bus load, SocketCAN, `candump`/`cangen`, why bus utilisation — not the kernel — sets worst-case latency |
| 03 — round-trip latency | Command frame out → feedback frame back, distribution? | Dead time in a control loop, its effect on achievable gains, joint-to-joint skew without a distributed clock |

**Phase 0 deliverable:** the timing budget in ADR-0002's action item 2, written
from *measured* numbers.

## Phase 1 — Move one joint honestly

| Lab | Question | Fundamentals |
|---|---|---|
| 04 — one joint, open loop | Can I command J6 to a position and read it back? | RobStride protocol, MIT mode's `(pos, vel, kp, kd, tau)`, what "impedance control" means physically, enable/disable/zeroing, units and sign conventions |
| 05 — stop categories | Does the Category 2 ramp bound the extra travel, and can a human still push the arm aside while it holds? | IEC 60204-1 stop categories, compliant vs rigid hold, allocation-free gravity feedforward in the cyclic path, why return-to-home is recovery and never a fault response |
| 06 — joint velocity | The drive lies about `mechVel`. Now what? | Finite differencing, quantisation noise, low-pass and Savitzky-Golay filters, phase lag vs noise trade-off, why this is state estimation |

## Phase 2 — Make it a manipulator

| Lab | Question | Fundamentals |
|---|---|---|
| 07 — FK from scratch | Where is the tool, given the joint angles? | Rigid transforms, SE(3), homogeneous matrices, URDF joint frames, then Pinocchio as a *check* on your own derivation |
| 08 — the Jacobian | How does joint velocity map to tool velocity? | Geometric vs analytic Jacobian, twists, singular values, manipulability, what a singularity *feels* like on a real arm |
| 09 — IK | Given a pose, which joint angles? | Damped least squares, nullspace projection, joint-limit avoidance, why closed-form IK for a 6R wrist is worth deriving once |
| 10 — trajectories | How do I get there without jerking? | Trapezoidal vs S-curve profiles, quintic splines, time parameterisation under velocity/accel limits, continuity classes and what discontinuity does to a gearbox |

## Phase 3 — Torque, not just position

| Lab | Question | Fundamentals |
|---|---|---|
| 11 — gravity compensation | Can I make the arm weightless to push around? | Rigid-body dynamics, RNEA, why `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ`, and reproducing the vendor's calibration campaign in `tools/gravity_calibration/` |
| 12 — parameter identification | Are the URDF's masses right? | Least-squares identification, regressor form, persistent excitation, friction models (Coulomb + viscous), why your model is always wrong and how wrong is acceptable |
| 13 — impedance control | Can it comply with a push and return? | Stiffness/damping/inertia shaping, passivity, stability limits of a discrete impedance controller, why `kp` cannot be raised indefinitely |

## Phase 4 — A system, not a script

| Lab | Question | Fundamentals |
|---|---|---|
| 14 — the RT/non-RT split *(moved early)* | How do setpoints cross from a client process into the 500 Hz loop, and survive that client dying? | Lock-free SPSC ring buffers, memory ordering and why it fails unreproducibly, POSIX shared memory, heartbeats, `ThreadSanitizer`. Built in Phase 1, not Phase 4 — [ADR-0006](adr/0006-process-topology-and-rt-client-transport.md) makes it the watchdog's precondition. |
| 15 — PREEMPT_RT, *if needed* | Does the budget hold under load on the stock kernel — and if not, does PREEMPT_RT close the gap? | `cyclictest`, `isolcpus`/`nohz_full`/`rcu_nocbs`, IRQ affinity, priority inversion, ftrace. Note the order: measure the stock kernel **first**. A 2 ms outer loop with `SCHED_FIFO` and CPU isolation may already pass, and ADR-0001's own principle says do not adopt a kernel you have not proven you need. |
| 18 — repeatability, ISO 9283 | Is the ±0.1 mm claim true? | Pose repeatability vs accuracy, why the encoders cannot validate the encoders, dial-indicator procedure, sample size and confidence |
| 16 — ROS2 tier | MoveIt2, state publishing, Foxglove | `ros2_control` architecture, DDS, lifecycle nodes, and where the real-time boundary must sit |
| 17 — simulation parity | Same controller in sim and on hardware | MuJoCo/Gazebo, sim-to-real gap, what the URDF's collision meshes are for |

---

## How the mentoring works

For each lab:

1. **I state the question and the measurable success criterion.**
2. **You predict the answer before running it.** Writing down a wrong prediction
   is the part that makes the concept stick; skipping it turns the lab back into
   reading.
3. You run it, we look at the numbers together.
4. I explain the fundamentals that the numbers just demonstrated — at that point
   you have a hook to hang them on.
5. We read the corresponding piece of the vendor code and ask: *why did they do
   it that way, and what did it cost them?*

Ask me "why" at any point. If an explanation lands at the wrong level — too deep
or too shallow — say so and I will re-pitch it.
