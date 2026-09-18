# Roadmap — learning robot control by building a controller

The rule of this project is simple: **every concept is tied to something that
runs on the real arm.** No milestone is "read about X". Each milestone is a lab
with a measurable result. The theory comes in when you need it to make the
measurement come out right.

Reference code: [`Seeed-Projects/reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py)
(Apache-2.0) and [`Seeed-Projects/reBot-DevArm`](https://github.com/Seeed-Projects/reBot-DevArm)
(hardware and URDF). Target: **reBot Arm B601-RS** — see
[ADR-0002](adr/0002-target-platform-rebot-b601-rs.md).

## How labs map to the repository

A lab is a unit of learning, not a directory. Each lab produces:

- a benchmark in `bench/` that can run again later to catch regressions,
- an analysis write-up in `notebooks/`,
- and code in `core/`.

See [ADR-0004](adr/0004-system-decomposition.md).

All real-time code is **C++20**. [ADR-0003](adr/0003-cpp-control-core-and-layering.md)
gives the layer structure and the rules for the cyclic path. `core/realtime` — the
clock, the cyclic executive, real-time setup, and allocation-free measurement —
is finished, and Lab 01 shows it working.

## Phase 0 — Timing, the bus, and what the drives do when the controller dies

> **Lab 00 blocks everything.** Nothing moves under its own power until it is done.

| Lab | Question it answers | What you learn |
|---|---|---|
| **00 — drive behaviour on lost communication** (blocking) | When command frames stop arriving, does a RobStride drive hold its last setpoint or disable itself? | Fail-safe versus fail-operational; why an arm without brakes has no safe unpowered state; ranking stop paths by how little they depend on ([ADR-0005](adr/0005-safe-state-and-stop-architecture.md)) |
| **01 — loop timing** (done) | How wrong is `time.sleep(dt - elapsed)`, in microseconds? | Monotonic versus wall clocks; absolute versus relative deadlines; phase-locked loops; drift versus jitter; why percentiles beat averages; `SCHED_FIFO`; `mlockall`; why the GIL is not the main problem here |
| **00b — the firmware floor** (blocking) | On this laptop, is the latency floor set by the kernel or by firmware (SMM)? | `hwlatdetect`; System Management Interrupts; why a dual-kernel cannot fix a firmware stall ([ADR-0007](adr/0007-rt-platform-on-a-cuda-laptop.md)) |
| 02 — the CAN bus | What is actually on the wire at 500 Hz? | CAN framing, arbitration, bit stuffing, bus load; SocketCAN; `candump` and `cangen`; why bus utilisation, not the kernel, sets the worst-case latency |
| 03 — round-trip latency | From command frame out to feedback frame back: what is the distribution? | Dead time in a control loop and its effect on the gains you can use; joint-to-joint skew when there is no shared clock |

**Deliverable for Phase 0:** the timing budget from ADR-0002's action item 2,
written from measured numbers.

## Phase 1 — Move one joint properly

| Lab | Question | What you learn |
|---|---|---|
| 04 — one joint, open loop | Can I command joint 6 to a position and read it back? | The RobStride protocol; MIT mode's `(pos, vel, kp, kd, tau)`; what impedance control means physically; enable, disable, and zeroing; units and sign conventions |
| 05 — stop categories | Does the Category 2 ramp keep the extra travel small? Can a person still push the arm aside while it holds? | IEC 60204-1 stop categories; compliant versus rigid hold; gravity feedforward in the cyclic path without allocation; why return-to-home is recovery and never a fault response |
| 06 — joint velocity | The drive's reported velocity is not trustworthy. What now? | Finite differencing; quantisation noise; low-pass and Savitzky–Golay filters; the trade-off between phase lag and noise; why this is a state-estimation problem |

## Phase 2 — Make it a manipulator

| Lab | Question | What you learn |
|---|---|---|
| 07 — forward kinematics from scratch | Where is the tool, given the joint angles? | Rigid transforms; SE(3); homogeneous matrices; URDF joint frames; then Pinocchio as a check on your own derivation |
| 08 — the Jacobian | How does joint velocity map to tool velocity? | Geometric versus analytic Jacobian; twists; singular values; manipulability; what a singularity feels like on a real arm |
| 09 — inverse kinematics | Given a pose, which joint angles? | Damped least squares; nullspace projection; joint-limit avoidance; why closed-form IK for a 6R wrist is worth deriving once |
| 10 — trajectories | How do I get there without jerking? | Trapezoidal versus S-curve profiles; quintic splines; time parameterisation under velocity and acceleration limits; continuity classes and what a discontinuity does to a gearbox |

## Phase 3 — Torque, not just position

| Lab | Question | What you learn |
|---|---|---|
| 11 — gravity compensation | Can I make the arm feel weightless so I can push it around? | Rigid-body dynamics; RNEA; why `M(q)q̈ + C(q,q̇)q̇ + g(q) = τ`; repeating the vendor's calibration in `tools/gravity_calibration/` |
| 12 — parameter identification | Are the masses in the URDF right? | Least-squares identification; regressor form; persistent excitation; friction models (Coulomb and viscous); why the model is always wrong and how wrong is acceptable |
| 13 — impedance control | Can it give way to a push and then return? | Shaping stiffness, damping, and inertia; passivity; stability limits of a discrete impedance controller; why `kp` cannot be raised forever |

## Phase 4 — A system, not a script

| Lab | Question | What you learn |
|---|---|---|
| 14 — the real-time / non-real-time split (done) | How do setpoints get from a client process into the 500 Hz loop, and survive that client dying? | Lock-free single-producer/single-consumer rings; memory ordering and why it fails in ways that are hard to reproduce; POSIX shared memory; heartbeats; ThreadSanitizer. Built in Phase 1 rather than Phase 4, because [ADR-0006](adr/0006-process-topology-and-rt-client-transport.md) makes it a precondition for the watchdog. |
| 15 — PREEMPT_RT, if needed | Does the budget hold under load on the stock kernel? If not, does PREEMPT_RT close the gap? | `cyclictest`; `isolcpus`, `nohz_full`, `rcu_nocbs`; IRQ affinity; priority inversion; ftrace. Measure the stock kernel **first**. A 2 ms outer loop with `SCHED_FIFO` and CPU isolation may already pass, and ADR-0001 says not to adopt a kernel you have not proven you need. |
| 16 — ROS2 tier | MoveIt2, state publishing, Foxglove | `ros2_control` architecture; DDS; lifecycle nodes; where the real-time boundary must sit |
| 17 — simulation parity | The same controller in simulation and on hardware | MuJoCo or Gazebo; the sim-to-real gap; what the URDF collision meshes are for |
| 18 — repeatability (ISO 9283) | Is the ±0.1 mm claim true? | Repeatability versus accuracy; why the encoders cannot validate the encoders; the dial-indicator procedure; sample size and confidence |

## How the mentoring works

For each lab:

1. I state the question and the measurable success criterion.
2. **You predict the answer before running it.** Writing down a wrong
   prediction is what makes the concept stick. Skipping this step turns the lab
   back into reading.
3. You run it, and we look at the numbers together.
4. I explain the theory the numbers just demonstrated. By then you have
   something concrete to attach it to.
5. We read the matching part of the vendor code and ask: why did they do it
   that way, and what did it cost them?

Ask "why" at any point. If an explanation is pitched at the wrong level — too
deep or too shallow — say so and I will adjust it.
