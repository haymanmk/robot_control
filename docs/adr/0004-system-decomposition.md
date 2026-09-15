# ADR-0004: System Decomposition, the Python Boundary, and Telemetry as a Product Feature

**Status:** Accepted
**Date:** 2026-09-15
**Deciders:** haymanmk
**Builds on:** [ADR-0002](0002-target-platform-rebot-b601-rs.md), [ADR-0003](0003-cpp-control-core-and-layering.md)

## Context

Two requirements were stated that look separate and are not:

1. The core — kinematics, dynamics, trajectory planning, motor control — should
   be a standalone library, wrapped by ROS2, in the way Seeed kept
   `reBotArm_control_py` independent of any ROS distribution.
2. Performance must be measurable: real-time determinism *and* control accuracy.
   "Otherwise, I cannot tell other people how good the system is."

The second is not a development convenience. Every serious motion controller
ships a flight recorder and sells its diagnostics as a feature. So requirement 2
is a *product* requirement, and it belongs inside the library, not in a test
directory beside it.

## Decision

### 1. Repository decomposition

```
core/            C++20. No ROS, no Python, no robot-specific policy. The product.
  rt/            clock, cyclic executive, RT privileges, instrumentation
  telemetry/     lock-free ring, fixed-size records, sinks, run metadata
  can/           SocketCAN transport, frame codec, bus statistics
  drive/         RobStride protocol: modes, scaling, enable, feedback
  model/         kinematics + dynamics (Pinocchio), URDF-driven
  control/       gravity compensation, impedance, trajectory tracking
  safety/        limits, watchdog, stop-category supervisor  (ADR-0005)
  bridge/        lock-free SPSC handoff between the RT and non-RT tiers
bindings/python/ pybind11 -> `rebot`. Command and observe only.
ros2/            ros2_control hardware_interface + MoveIt2 configuration
bench/           performance regression suite; runs in CI, gates releases
notebooks/       analysis and write-ups, as plain .py (see §4)
```

`core/` builds and its tests run with **no ROS and no Python installed**. That
constraint is what keeps the real-time boundary honest as features accrete; the
moment the core needs ROS to build, ROS is in the control path in spirit even if
not in fact.

**pybind11** over nanobind for the bindings: the binding layer is not in the hot
path, so maturity and ecosystem beat marginal speed.

### 2. The Python boundary — the rule that must not bend

Python may **command and observe**. Python may **not** be in the cyclic path.

`CyclicTask` runs on a C++ thread with the GIL released. Python pushes setpoints
and drains telemetry through `core/bridge`'s lock-free SPSC queues — the same
mechanism ROS2 uses. There is no code path by which the interpreter, the
allocator, or the GIL can appear inside the 2 ms budget.

**This is enforced by API shape, not by documentation.** The binding exposes no
`step()`, no `send_now()`, no per-cycle callback into Python. Only:

```python
arm.move_to(...)          # queue a motion
arm.set_target(...)       # update a setpoint asynchronously
arm.telemetry(last=5.0)   # drain the recorder
arm.state()               # most recent published state snapshot
```

If the wrong architecture cannot be *expressed*, nobody implements it at 3am.
A useful side effect: the bridge gains two independent consumers (Python and
ROS2) early, which is what proves an abstraction is real rather than assumed.

### 3. Telemetry is always on

A fixed-size record per cycle, written by the RT thread into a preallocated
lock-free ring, drained by a non-RT writer thread. No allocation, no locks, no
I/O on the RT side — the RT thread's only telemetry cost is a bounded memcpy.

60 s at 500 Hz × ~200 B ≈ **6 MB**. Affordable, so it is not optional. A flight
recorder switched on *after* the incident is a flight recorder you do not have.

Every record carries: cycle index, deadline, wake time, execution time, per-joint
commanded and measured position/velocity/torque, CAN tx/rx kernel timestamps,
and fault flags.

**Every run file is stamped with its provenance**: git SHA, build type, kernel
release and whether it is `PREEMPT_RT`, CPU model and governor, `isolcpus`
state, RT priority and affinity actually granted, CAN interface bitrate. Without
this, a number recorded today is worthless in six months — and comparison across
time is the entire point of a regression suite.

### 4. The measurement taxonomy

Four tiers, deliberately separated because they require different evidence:

| Tier | Metrics | Requires |
|---|---|---|
| **1 — Determinism** | wake latency, period error, execution time, drift, overruns — p99.9 and max | nothing |
| **2 — Fieldbus** | bus load %, frame round-trip, command→feedback latency, joint-to-joint skew, frame loss | the arm |
| **3 — Control accuracy** | joint tracking error (RMS/max), step response overshoot and settling, Cartesian path deviation, gravity-comp residual drift | the arm |
| **4 — Task level** | pose repeatability, absolute accuracy | **external metrology** |

Two honesty rules attach to this table:

- **The encoders cannot validate the encoders.** Tier 3 measures *tracking
  error* — commanded versus measured on the same sensor. That is real and
  useful, and it is not a statement about where the tool actually is in space.
- **Tier 4 splits.** *Repeatability* is measurable at home with a dial indicator
  and the ISO 9283 procedure, for the price of lunch; Seeed's ±0.1 mm claim can
  be independently checked. *Absolute accuracy* needs a laser tracker or ballbar,
  which we do not have, so we will not claim it. Saying so is what makes the rest
  credible.

**The instrument must outclass the measurement.** Userspace timestamps on CAN
frames measure our own scheduler, not the bus; `core/can` therefore uses
`SO_TIMESTAMPING` for kernel/hardware receive timestamps. A latency number is
only as good as the clock that produced it.

### 5. `bench/` gates releases; `notebooks/` tells the story

`bench/` emits JSON, compared against a stored baseline. A regression beyond
threshold fails CI. This is what turns "I measured it once" into "it is still
true." The narrative — what the numbers mean, what we learned — lives in
`notebooks/`.

The pedagogical sequence in `docs/ROADMAP.md` still calls its units *labs*; a
lab now produces artifacts in `bench/` and `notebooks/` rather than living in a
directory of its own.

### 6. Notebooks are plain `.py` in percent format

`.ipynb` is JSON with embedded output: unreviewable diffs, merge conflicts on
every execution. We commit `.py` files using `# %%` cell markers instead, which

- run as ordinary scripts with no Jupyter installed,
- execute cell-by-cell with inline plots in VS Code, PyCharm, and Jupyter itself,
- and diff like code, because they are code.

No `.ipynb` is committed. (`jupytext` can pair one locally if a browser notebook
is wanted; the `.ipynb` stays gitignored.)

### 7. Build order: thin vertical slice, ROS2 last

Not six layers bottom-up. The first milestone is narrow and complete:

```
rt -> can -> drive -> telemetry -> safety(watchdog) -> ONE joint moving
```

with the drive-behaviour experiment from [ADR-0005](0005-safe-state-and-stop-architecture.md)
performed *before* anything moves under its own power. Then widen to seven
joints, then add `model/`, then `control/`.

ROS2 comes last. `ros2_control`'s `controller_manager` wants to own the update
loop; building against it early produces a core that cannot run without it.
Our core owns its executive, and `hardware_interface` becomes one more bridge
client. (How exactly that composes is deferred to its own ADR once `bridge/`
exists and `ros2_control`'s constraints have been measured rather than assumed.)

## Consequences

- **Easier:** telemetry exists from cycle one, so every controller is measurable
  by default and "how good is it?" has an answer backed by stored data.
- **Easier:** `core/` is portable — same library under a Python REPL, a ROS2
  node, or a bare `main()`; same binary against `vcan0` or real hardware.
- **Easier:** the Jupyter-kernel-restart hazard is closed structurally by the
  heartbeat and ADR-0005's Category 2 response.
- **Harder:** telemetry must be budgeted in the cycle. It is now part of the
  worst-case execution time, and Tier 1 metrics must be measured *with it on*,
  since that is how it ships.
- **Harder:** a stable bridge record layout must be designed and versioned early,
  because two consumers depend on it.
- **Harder:** "no allocation in the cyclic path" now binds real code with real
  deadlines, not just the executive. Expect to fight it.
