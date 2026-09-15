# ADR-0004: System Decomposition, the Python Boundary, and Telemetry as a Product Feature

**Status:** Accepted; §2 superseded by [ADR-0006](0006-process-topology-and-rt-client-transport.md)
**Date:** 2026-09-15
**Deciders:** haymanmk
**Builds on:** [ADR-0002](0002-target-platform-rebot-b601-rs.md), [ADR-0003](0003-cpp-control-core-and-layering.md)

## Context

Two requirements were stated. They look separate, but they are not.

1. The core — kinematics, dynamics, trajectory planning, motor control —
   should be a standalone library, wrapped by ROS2, in the same way Seeed kept
   `reBotArm_control_py` independent of any ROS distribution.
2. Performance must be measurable, both real-time behaviour *and* control
   accuracy. In the words of the requirement: "Otherwise, I cannot tell other
   people how good the system is."

The second requirement is not a development convenience. Every serious motion
controller records every cycle, and its diagnostics are a selling point. So
requirement 2 is a *product* requirement, and it belongs inside the library,
not in a test directory next to it.

## Decision

### 1. Repository layout

```
core/            C++20. No ROS, no Python. This is the product.
  rt/            clock, cyclic executive, real-time privileges, measurement
  telemetry/     lock-free ring, fixed-size records, file sink, run metadata
  can/           SocketCAN transport, frame encoding, bus statistics
  drive/         RobStride protocol: modes, scaling, enable, feedback
  model/         kinematics and dynamics (Pinocchio), driven by the URDF
  control/       gravity compensation, impedance, trajectory tracking
  safety/        limits, watchdog, stop-category supervisor  (ADR-0005)
  bridge/        lock-free hand-off between the real-time and non-real-time tiers
bindings/python/ pybind11 -> `rebot`. Command and observe only.
ros2/            ros2_control hardware_interface and MoveIt2 configuration
bench/           performance benchmarks; run in CI; gate releases
notebooks/       analysis and write-ups, as plain .py files (see §6)
```

`core/` builds and its tests run with **no ROS and no Python installed**. This
rule keeps the real-time boundary honest as features accumulate. The moment the
core needs ROS to build, ROS is in the control path in spirit, if not in fact.

For the Python bindings we use **pybind11** rather than nanobind. The binding
layer is not on the hot path, so maturity and ecosystem matter more than a
small speed difference. (What gets bound changed in ADR-0006: the bridge client
and `core/model`, not the real-time core.)

### 2. The Python boundary

> **Superseded by [ADR-0006](0006-process-topology-and-rt-client-transport.md).**
> The rule below still stands. The *mechanism* does not. In-process bindings
> would kill the real-time core whenever the Python interpreter dies, which is
> exactly when [ADR-0005](0005-safe-state-and-stop-architecture.md)'s watchdog
> must keep running. Live control now runs in a separate process over a
> shared-memory bridge. In-process bindings are kept only for offline analysis.

Python may **command and observe**. Python may **not** be in the cyclic path.

This is enforced by the shape of the API, not by documentation. The bindings
expose no `step()`, no `send_now()`, and no per-cycle callback into Python.
Only calls like these exist:

```python
arm.move_to(...)          # queue a motion
arm.set_target(...)       # update a setpoint asynchronously
arm.telemetry(last=5.0)   # read the recorder
arm.state()               # the most recent state snapshot
```

If the wrong design cannot be *expressed*, nobody will implement it by accident.

### 3. Telemetry is always on

One fixed-size record per cycle is written by the real-time thread into a
pre-allocated lock-free ring, and drained by a non-real-time writer thread. The
real-time side does no allocation, takes no locks, and does no I/O. Its only
cost is a bounded memory copy.

Sixty seconds at 500 Hz, at about 256 bytes per record, is about **7.7 MB**.
That is cheap, so telemetry is not optional. A recorder that is switched on
*after* the incident is a recorder you did not have.

Every record carries: the cycle index, the deadline, the wake time, the
execution time, per-joint commanded and measured position, velocity, and
torque, CAN transmit and receive kernel timestamps, and fault flags.

**Every run file is stamped with its provenance:** git commit, build type,
kernel release and whether it is `PREEMPT_RT`, CPU model and governor,
`isolcpus` state, the real-time priority and affinity actually granted, and the
CAN bit rate. Without this, a number recorded today means nothing in six
months. Comparison over time is the whole point of a regression suite, and
comparison needs provenance.

### 4. What we measure, in four tiers

The tiers are separate because they need different evidence.

| Tier | Metrics | Requires |
|---|---|---|
| **1 — Determinism** | wake latency, period error, execution time, drift, overruns — p99.9 and max | nothing |
| **2 — Fieldbus** | bus load, frame round trip, command-to-feedback latency, joint-to-joint skew, frame loss | the arm |
| **3 — Control accuracy** | joint tracking error (RMS and max), step-response overshoot and settling time, Cartesian path deviation, gravity-compensation drift | the arm |
| **4 — Task level** | pose repeatability, absolute accuracy | **external measuring equipment** |

Two rules of honesty go with this table:

- **The encoders cannot validate the encoders.** Tier 3 measures *tracking
  error*: commanded versus measured on the same sensor. That is real and
  useful, but it says nothing about where the tool actually is in space.
- **Tier 4 splits in two.** *Repeatability* can be measured at home with a dial
  indicator and the ISO 9283 procedure, for very little money; Seeed's ±0.1 mm
  claim can be checked independently. *Absolute accuracy* needs a laser tracker
  or a ballbar. We do not have one, so we will not claim it. Saying so is what
  makes the rest credible.

**The measuring instrument must be better than the thing measured.** Timestamps
taken in user space on CAN frames measure our own scheduler, not the bus.
`core/can` therefore uses `SO_TIMESTAMPING` to get kernel or hardware receive
timestamps. A latency number is only as good as the clock that produced it.

### 5. `bench/` gates releases; `notebooks/` explains

`bench/` produces JSON that is compared with a stored baseline. A regression
beyond a threshold fails CI. This turns "I measured it once" into "it is still
true." The explanation — what the numbers mean and what we learned — lives in
`notebooks/`.

The learning sequence in `docs/ROADMAP.md` still calls its units *labs*. A lab
now produces files in `bench/` and `notebooks/` rather than having its own
directory.

### 6. Notebooks are plain `.py` files in percent format

`.ipynb` is JSON with outputs embedded. Its diffs cannot be reviewed, and every
run creates a merge conflict. We commit `.py` files with `# %%` cell markers
instead. They:

- run as ordinary scripts with no Jupyter installed,
- run cell by cell with inline plots in VS Code, PyCharm, and Jupyter,
- and diff like code, because they are code.

No `.ipynb` is committed. (`jupytext` can pair one locally if you want a browser
notebook; the `.ipynb` stays ignored by git.)

### 7. Build order: a thin vertical slice, ROS2 last

We do not build six layers from the bottom up. The first milestone is narrow
and complete:

```
rt -> can -> drive -> telemetry -> safety (watchdog) -> ONE joint moving
```

The drive-behaviour experiment in [ADR-0005](0005-safe-state-and-stop-architecture.md)
comes *before* anything moves under its own power. Then we widen to seven
joints, then add `model/`, then `control/`.

ROS2 comes last. `ros2_control`'s `controller_manager` wants to own the update
loop; building against it early produces a core that cannot run without it.
Our core owns its own executive, and `hardware_interface` becomes one more
client of the bridge. (Exactly how that fits together is left to a later ADR,
once `bridge/` exists and `ros2_control`'s constraints have been measured
rather than assumed.)

## Consequences

- **Easier:** telemetry exists from the first cycle, so every controller is
  measurable by default, and "how good is it?" has an answer backed by data.
- **Easier:** `core/` is portable. The same library works under a Python
  session, a ROS2 node, or a bare `main()`, and the same binary runs against
  `vcan0` or real hardware.
- **Easier:** the risk of a Jupyter kernel restart is closed by design: the
  heartbeat stops and ADR-0005's Category 2 response runs.
- **Harder:** telemetry has to fit inside the cycle budget. It is now part of
  the worst-case execution time, and Tier 1 numbers must be measured *with it
  on*, because that is how it ships.
- **Harder:** a stable, versioned bridge record layout must be designed early,
  because two consumers depend on it.
- **Harder:** "no allocation in the cyclic path" now applies to real code with
  real deadlines, not only to the executive. Expect to fight it.
