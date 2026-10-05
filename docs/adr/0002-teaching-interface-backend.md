# ADR-0002: Control Backend for the Teach-and-Playback Interface

**Status:** Proposed
**Date:** 2026-10-05
**Deciders:** TBD
**Related:** ADR-0001 (RTOS and middleware selection)

## Context

The first user-facing feature of the control system is a **teaching interface** for the
reBot Arm B601-RS (Seeed Studio, RobStride motors, CAN bus): an operator moves the arm,
records taught points, and later plays the recorded program back.

Requirements gathered so far:

1. **Teaching by direct manipulation.** The operator pulls and drags the end-effector by
   hand. The UI is used only for fine adjustments (small jogs) and for the gripper.
2. **Playback with trajectory planning**, not plain joint-angle interpolation: the
   motion between taught points must be time-parameterized against velocity and
   acceleration limits, and Cartesian straight-line moves must be available.
3. **Short time to a working prototype**, with the robot we already own.

Constraints and findings from the hardware and the vendor software:

- Seeed publishes [`reBotArm_control_py`](https://github.com/Seeed-Projects/reBotArm_control_py):
  a Python library on top of `motorbridge` (CAN/serial motor bridge) and Pinocchio. It
  provides a configuration-driven actuator layer (`RebotArm` with `arm` and `gripper`
  joint groups, a 500 Hz control loop), forward/inverse kinematics, rigid-body dynamics
  (`g(q)` for gravity compensation), an SE(3) geodesic sampler with closed-loop IK
  tracking, and two reference controllers (gravity compensation, end-pose control).
- The RobStride motors close their own impedance loop in **MIT mode**
  (`pos, vel, kp, kd, tau` setpoints). The host only streams setpoints. This is
  ADR-0001's "Option C" situation: the servo loop is in the drives, so the host loop
  does not need hard real-time guarantees.
- The upstream gravity calibration for the RS build (2026-07-17) found the URDF mass
  model correct to ~5–11 % on load-bearing joints, 0.2–0.5 N·m Coulomb friction per
  joint, and two firmware quirks: cached positions can freeze unless feedback is
  explicitly requested, and the reported velocity (`mechVel`) is not in rad/s.
  Velocity must be finite-differenced from position.
- Upstream controllers each *own* the control loop (`start_control_loop` inside
  `start()`), so they cannot be composed into one application that switches between
  free-drive, jogging and playback.
- The upstream package is not on PyPI and resolves its `config/` and `urdf/` directories
  relative to its own source tree, so it must be consumed as a source checkout.
- Seeed also ships a ROS 2 stack for this arm. ADR-0001 places planning-level work in a
  ROS 2 tier in the long run.

## Decision

1. **Use `reBotArm_control_py` as the control backend** of the teaching interface,
   vendored as a git submodule and installed as an editable path dependency, so its
   configuration and URDF resolve correctly and upstream fixes can be pulled.
2. **Own the control loop in our application.** A single `TeachController` callback at
   the configured rate (500 Hz) keeps every motor in MIT mode and switches between
   states — `HOLD`, `FREE_DRIVE`, `JOG`, `PLAYBACK` — without re-negotiating motor
   modes. Upstream is used for kinematics, dynamics, the SE(3) sampler and CLIK
   tracking, and the actuator layer; the two upstream controllers are reference
   material, not runtime dependencies.
3. **Free-drive is gravity-compensated impedance control, never torque-off.**
   `tau = tau_scale · g(q)` feed-forward plus low PD gains, with an end-effector
   velocity lock (hold when still, follow when pushed) so a released arm does not
   drift. Every stop path falls back to a stiff position hold with gravity
   feed-forward; motors are disabled only by an explicit e-stop.
4. **Playback is planned, time-parameterized motion.** Joint moves use a multi-waypoint
   cubic spline with via-point velocity heuristics, time-scaled until per-joint
   velocity and acceleration limits hold. Cartesian moves use the upstream SE(3)
   geodesic sampler with CLIK tracking, bounded by Cartesian speed limits and
   re-checked against joint limits. The planner emits `(t, q, q̇)` samples that the
   control callback streams as MIT setpoints with `g(q)` feed-forward.
5. **The program model is transport-independent.** A taught point stores the joint
   configuration, the end-effector pose, the gripper state, the motion type
   (joint / linear), and a speed scale, serialized as versioned JSON. The planner and
   the program model depend only on NumPy and Pinocchio, so they survive a later
   move to ROS 2 / MoveIt 2 unchanged.
6. **A backend protocol with a simulation implementation.** The controller talks to an
   `ArmBackend` interface; `SimBackend` lets the UI, planner and server be developed
   and tested without hardware, and `RebotArmBackend` wraps the upstream actuator.
7. **UI as a web page served by a FastAPI backend** (REST commands + WebSocket state
   stream), consistent with ADR-0001's backend/frontend split and its guidance to spend
   custom frontend effort on the operator UI.

## Options Considered

### Option A: `reBotArm_control_py` + own application layer (chosen)

| Dimension | Assessment |
|-----------|------------|
| Time to prototype | Short — actuator, kinematics, dynamics and a validated gravity model exist |
| Fit to requirements | Good — MIT mode gives compliant free-drive; Pinocchio gives the planning primitives |
| Determinism | Adequate — drives close the servo loop; host jitter at 500 Hz is tolerable |
| Maintenance risk | Medium — young vendor library, no PyPI release, bilingual docs, no license file yet |
| Portability | Good for the model/planner; the backend adapter is the only vendor-specific code |

### Option B: Seeed ROS 2 stack + MoveIt 2 + `ros2_control`

Aligns with ADR-0001's target architecture and gives mature planning, visualization
(RViz/Foxglove) and a hardware-interface abstraction.

**Cons:** much larger footprint for a teach-and-playback prototype; MoveIt 2's planners
target collision-free point-to-point planning, while teaching needs deterministic
replays of recorded paths; compliant free-drive would still have to be written as a
custom `ros2_control` controller. Kept as the migration target once the control system
grows beyond one arm and one UI.

### Option C: Direct `motorbridge` driver with our own kinematics

**Pros:** full control, no vendor abstraction.
**Cons:** re-implements the actuator layer, URDF handling and the gravity calibration
work that upstream already did; no benefit for the requirements at hand.

## Consequences

- **Easier:** a working teach pendant quickly; development without hardware through the
  simulation backend; the planner and program format are reusable in the ROS 2 tier.
- **Harder:** we inherit RS firmware quirks (explicit feedback requests, finite-difference
  velocity) and the ~5–11 % gravity-model error plus wrist friction, which the PD terms
  must cover; free-drive quality will need per-joint gain tuning on the real arm.
- **Safety:** the software stop is a stiff hold, not a safety function. Per ADR-0001,
  a hardware e-stop that cuts motor power remains mandatory, and the arm must be kept
  clear of the table and of itself when free-drive starts (contact corrupts the gravity
  feed-forward).
- **Revisit if:** upstream stops being maintained or changes its layout; the control
  system gains a second robot or needs multi-node tooling (move to Option B); or free-
  drive quality on the real hardware proves insufficient with gravity feed-forward
  alone (consider a friction model or an external force estimator).

## Action Items

1. [ ] Implement the teaching interface in `robot_teaching` with the structure above.
2. [ ] Validate free-drive gains on the real arm from a clearance pose ("L" pose,
       joint2 ≈ 0.7 rad, joint3 ≈ 1.1 rad), one joint at a time.
3. [ ] Measure achievable playback speed limits per joint and set the configuration
       defaults from measurements rather than the URDF's nominal 40–50 rad/s.
4. [ ] Ask upstream for a license file; keep our use limited to importing the library.
