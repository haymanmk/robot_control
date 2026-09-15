# ADR-0005: Safe State and Stop Architecture

**Status:** Accepted (one premise unverified — see §Must Measure)
**Date:** 2026-09-15
**Deciders:** haymanmk
**Supersedes:** the "ramp to hold and disable" phrasing used informally earlier, which was wrong.

## Context

The reBot Arm B601-RS has **no brakes**. The RobStride motors are
quasi-direct-drive with a low gear ratio, so they backdrive freely. Three
consequences follow immediately:

1. **Disabling a motor is a controlled fall.** Any "safe state" that involves
   removing torque drops the arm from wherever it is.
2. **Power loss is a fall, and no software can prevent it.** There is no STO, no
   safety relay, and no mechanical brake to fail safe into.
3. Therefore the arm's safe state must be a *powered* state, and the design
   question is not "how do we stop" but **"which stop, for which fault, with how
   few dependencies."**

### There is no single safe state

The instinct to define one "safe stop" is the mistake. IEC 60204-1 defines three
stop categories, and a brakeless arm needs different ones for different faults:

| Category | Meaning | On this arm |
|---|---|---|
| **0** | Immediate removal of power, uncontrolled | **Free fall.** Last resort only — it is the only stop that still works when everything else has failed. |
| **1** | Controlled deceleration, *then* power removed | Acceptable only if the arm is parked low first. |
| **2** | Controlled stop, **power retained** | **The default for this arm.** |

### Why "return to home" is not a stop

Returning to a home pose requires IK, trajectory generation, the dynamics model,
and a healthy control stack — i.e. exactly the subsystems whose failure triggered
the stop. A watchdog fires *because the controller is no longer trustworthy*; it
cannot then ask that controller to plan and execute a motion.

Worse, an arm that moves autonomously after a fault is actively dangerous,
because a human has usually just walked up to it. Industry practice is firm here:
**after a fault, no autonomous motion.** Recovery is a separate, operator-
initiated mode.

### The dependency principle

The property that decides whether a safe stop works is **how few things it needs
to be true.** Ranked by trust, fewest dependencies first:

```
drive firmware alone       <- survives our process being SIGKILLed
RT core + drive            <- survives Python/ROS2 dying
RT core + model            <- needs Pinocchio state to be sane
RT core + planner          <- needs the whole stack healthy   (NOT a stop)
```

Every stop response is designed to sit as high in that list as possible.

## Decision

### 1. The default safe state is Category 2: *decelerate, then compliant hold*

On any fault that does not indicate the drives themselves are unreliable:

1. **Freeze the setpoint source.** Stop accepting commands from the bridge.
2. **Decelerate** over a bounded ramp `T_stop` (default 300 ms), commanding
   a constant-deceleration profile from the current `(q, q̇)` so the extra travel
   is bounded and computable.
3. **Hold compliantly** at the pose reached: low position stiffness plus gravity
   feedforward, staying powered and enabled indefinitely.

**Low stiffness, not rigid, and this is deliberate.** A rigid hold turns the arm
into an immovable object, which is the wrong behaviour if a person or a workpiece
is trapped against it. Low `kp` plus gravity feedforward means the arm stays
where it was put but a human can still push it aside by hand.

Gravity feedforward alone (`kp = 0`) is rejected: model error makes it sag over
minutes. Position hold alone is rejected: it fights gravity with stiffness and
needs high gains to hold a stretched-out pose. The combination is what a
collaborative arm wants.

### 2. The watchdog lives inside the cyclic loop

Not in a monitor process, not in Python. The 500 Hz loop already runs; checking
"has the client posted a heartbeat within N cycles" is a handful of instructions
and adds **zero new dependencies**. A separate watchdog process is one more thing
that can die.

### 3. Response is selected per fault

| Fault | Detected by | Response | Needs |
|---|---|---|---|
| Client heartbeat lost (Jupyter kernel restart, ROS2 node crash, SSH drop) | RT loop | Cat 2: ramp → compliant hold | RT core + drive |
| Sustained cycle overrun | Cyclic executive | Cat 2, refuse new setpoints, log | RT core |
| Joint/velocity/torque limit exceeded | Safety supervisor | Clamp, then Cat 2 | RT core |
| Feedback stale from one joint | CAN layer | Cat 2 on all joints | RT core + drive |
| Drive reports internal fault | Drive layer | Cat 2 on healthy joints; faulted joint is already gone | drive |
| **RT core process dies** (SIGKILL, segfault, OOM) | *drive firmware only* | **UNKNOWN — see below** | drive firmware |
| Operator e-stop | hardware | Cat 0 → fall | hardware |
| Power loss | — | fall | nothing |

### 4. Return-to-home is a recovery mode, never a fault response

Preconditions, all required, all checked: fault cleared and acknowledged by the
operator; every joint reporting fresh feedback; model and planner healthy;
operator-initiated with a deliberate action. Executed at a reduced speed limit
(default 20% of normal). Never automatic, never on a timer.

### 5. Residual risk is acknowledged, not engineered away

**This arm cannot be made safe in software. It can be made predictable.**
Power loss drops it, and no watchdog changes that. Mitigations are therefore
physical and are part of the system design, not an afterthought:

- Nothing fragile — and no hands — underneath the workspace.
- Mount so that a full-extension collapse is survivable for the arm and the desk.
- Park low before any planned power-down; treat "powered down while extended" as
  an operating error.
- Consider a physical parking stand or counterbalance for unattended power-down.

## Must Measure (before any autonomous motion)

The whole design rests on one unverified premise: **what does a RobStride drive
do when command frames stop arriving?**

- If it **holds the last MIT setpoint**, we have the strongest safety property
  available on this hardware — the arm survives our process being `SIGKILL`ed,
  with zero dependency on our software.
- If it **disables after a comms timeout**, the arm falls whenever our process
  dies, and we need a different answer: a separate always-on holder process, a
  firmware parameter change, or physical mitigation.

**Experiment (Lab 05, promoted to blocking):** enable one joint under MIT mode at
a low, safe pose, hold the arm by hand, `SIGKILL` the sender, and observe. Repeat
with the joint lightly loaded. Record the timeout if one exists. Do this with the
arm low and something soft underneath, and do it before anything else moves.

Everything in §3's last rows is provisional until this number exists.

## Consequences

- **Easier:** the Jupyter/Python risk is contained by construction — a kernel
  restart drops the heartbeat, the RT core ramps and holds, nothing falls.
- **Easier:** "which stop?" has a documented answer per fault, so the safety
  supervisor is a table, not an accumulation of special cases.
- **Harder:** the compliant-hold controller must run in the cyclic path with no
  allocation, including the gravity term. Pinocchio's `Data` is preallocated once
  at startup; RNEA then allocates nothing.
- **Harder:** every subsystem must define its fault signal explicitly. "It threw
  an exception" is not a fault classification.
- **Revisit if:** brakes are added, an STO-capable drive is fitted, or the
  must-measure experiment shows the drives disable on comms loss.
