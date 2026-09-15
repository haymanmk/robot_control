# ADR-0005: Safe State and Stop Architecture

**Status:** Accepted, with one premise still unverified (see "Must measure")
**Date:** 2026-09-15
**Amended:** 2026-09-15 — three triggers instead of two; return-to-home recognised as a graceful-shutdown feature; §4b on gain persistence added
**Deciders:** haymanmk
**Replaces:** the informal phrase "ramp to hold and disable" used earlier, which was wrong

## Context

The reBot Arm B601-RS has **no brakes**. The RobStride motors are
quasi-direct-drive with a low gear ratio, so they turn freely when unpowered.
Three things follow at once:

1. **Disabling a motor is a controlled fall.** Any "safe state" that removes
   torque drops the arm from wherever it is.
2. **Losing power is a fall, and no software can prevent it.** There is no
   safe-torque-off, no safety relay, and no mechanical brake to fall back on.
3. So the arm's safe state must be a *powered* state. The design question is
   not "how do we stop" but **"which stop, for which fault, depending on how
   little."**

### There is no single safe state

Wanting one "safe stop" is the mistake. IEC 60204-1 defines three stop
categories, and an arm without brakes needs different ones for different faults:

| Category | Meaning | On this arm |
|---|---|---|
| **0** | Remove power immediately; uncontrolled | **Free fall.** Last resort only. It is the one stop that still works when everything else has failed. |
| **1** | Controlled deceleration, *then* remove power | Acceptable only if the arm is parked low first. |
| **2** | Controlled stop, **power kept on** | **The default for this arm.** |

### Three triggers, not two

An earlier draft treated "the application exited" and "the controller failed"
as one case. They are different events with different information available,
and treating them as one gave the wrong answer for both:

| Trigger | Example | Is the software healthy? | Response |
|---|---|---|---|
| **Graceful shutdown** | The user closes the application, presses stop, or sends `SIGINT`/`SIGTERM` | **Yes.** We are choosing to stop. | Optional **return-to-home**, then park, then hold |
| **Fault** | Heartbeat lost, limit exceeded, drive fault, sustained overrun | **No.** Something is wrong. | **Category 2:** ramp down, then compliant hold. No autonomous motion. |
| **Hard death** | `SIGKILL`, segfault, out-of-memory kill, power loss | **None of our software is running.** | Whatever the drive firmware does — see "Must measure" |

**Return-to-home belongs to the first row only, and it is a real feature.** If
a user closes an application that was moving the arm, leaving the arm stuck
mid-pose and expecting the user to work out a safe recovery by hand is bad
design. At that moment the software is healthy: IK, the model, and the planner
are all available, and we are stopping by choice, not because of a failure. So
we use them.

It must never be reachable from the second or third row. A watchdog fires
*because the controller can no longer be trusted*. Asking that same controller
to plan and carry out a motion is asking the broken part to rescue you. And an
arm that moves by itself after a *fault* is dangerous in a way it is not after
a *clean exit*, because a person has usually just walked up to it.

#### Making graceful shutdown work in practice

- `SIGINT`/`SIGTERM` handlers must be async-signal-safe: set an atomic flag and
  do nothing else. The cyclic loop sees the flag and runs the sequence.
- Return-to-home moves in **joint space** at a reduced speed limit (default
  20%). Joint-space motion is predictable and has no IK surprises. Slow motion
  means a person can react and push the arm aside.
- **Path safety is not free.** A simple interpolation from an arbitrary pose to
  home can sweep through the workspace, the table, or the operator. Until
  collision checking exists, return-to-home requires the operator to hold a
  key or button for the whole motion; releasing it stops the arm. If the
  confirmation times out, the default is Category 2, not motion.
- If the client dies *during* return-to-home, the heartbeat stops and the fault
  path takes over in the middle of the motion. Graceful shutdown is not a
  privileged mode; it is supervised like everything else.

### The dependency principle

What decides whether a safe stop works is **how few things must still be
working for it to happen.** Ranked from fewest dependencies to most:

```
drive firmware alone       <- survives our process being SIGKILLed
RT core + drive            <- survives Python or ROS2 dying
RT core + model            <- needs Pinocchio's state to be sane
RT core + planner          <- needs the whole stack healthy   (NOT a stop)
```

Every stop response is designed to sit as high in this list as it can.

## Decision

### 1. The default safe state is Category 2: slow down, then hold compliantly

For any fault that does not suggest the drives themselves are unreliable:

1. **Freeze the setpoint source.** Stop accepting commands from the bridge.
2. **Decelerate** over a bounded time `T_stop` (default 300 ms), using a
   constant-deceleration profile from the current position and velocity, so
   the extra travel is bounded and can be calculated.
3. **Hold compliantly** at the pose reached: low position stiffness plus
   gravity feedforward, staying powered and enabled indefinitely.

**Low stiffness, not rigid — on purpose.** A rigid hold makes the arm an
immovable object, which is the wrong behaviour if a person or a workpiece is
trapped against it. Low `kp` plus gravity feedforward means the arm stays where
it was put, but a person can still push it aside by hand.

Gravity feedforward alone (`kp = 0`) is rejected: model error makes the arm sag
over minutes. Position hold alone is rejected: it fights gravity with stiffness
and needs high gains to hold a stretched-out pose. The combination is what a
collaborative arm needs.

### 2. The watchdog lives inside the cyclic loop

Not in a separate monitor process, and not in Python. The 500 Hz loop is
already running; checking "has the client posted a heartbeat within N cycles?"
takes a few instructions and adds **no new dependencies**. A separate watchdog
process is one more thing that can die.

### 3. The response is chosen per fault

| Fault | Detected by | Response | Needs |
|---|---|---|---|
| Graceful shutdown requested (`SIGINT`, application close, stop button) | signal handler → real-time loop | Return-to-home if confirmed, otherwise park; then hold | the full stack |
| Client heartbeat lost (Jupyter kernel restart, ROS2 node crash, SSH dropped) | real-time loop | Category 2: ramp, then compliant hold | RT core + drive |
| Sustained cycle overrun | cyclic executive | Category 2; refuse new setpoints; log | RT core |
| Joint, velocity, or torque limit exceeded | safety supervisor | Clamp, then Category 2 | RT core |
| Feedback from one joint is stale | CAN layer | Category 2 on all joints | RT core + drive |
| A drive reports an internal fault | drive layer | Category 2 on the healthy joints; the faulted joint is already gone | drive |
| **The RT core process dies** (`SIGKILL`, segfault, out-of-memory) | *drive firmware only* | **UNKNOWN — see below** | drive firmware |
| Operator emergency stop | hardware | Category 0 → fall | hardware |
| Power loss | — | fall | nothing |

### 4. Return-to-home is a shutdown and recovery feature, never a fault response

It has two legitimate uses. Both are started by the operator, and both need a
healthy stack:

- **Graceful shutdown.** The application driving the arm is closing. Return to
  home so the next session starts from a known pose.
- **Recovery after a fault.** The fault has been cleared and acknowledged, and
  the operator now wants the arm back at its origin.

Preconditions — all required, all checked: every joint reporting fresh
feedback; model and planner healthy; no active fault; a deliberate operator
action; a held confirmation for the whole motion. Reduced speed limit (default
20%). Never automatic, never on a timer, never as the response to a fault.

### 4b. Never send gains you would not want held forever

If the drives hold their last MIT setpoint when communication is lost (the
outcome we want, and must verify), then **every MIT frame is a possible
permanent state.** If our process dies in the middle of a high-gain tracking
move, the arm holds rigidly, indefinitely, with nobody watching.

So the commanded `kp` and `kd` are limited to values that are safe to leave
unattended. Tracking gains high enough to be dangerous if frozen are simply not
allowed. If a motion needs them, it needs a different mechanism and its own
hazard analysis. This costs a little tracking performance and removes a whole
class of failure.

A hold-forever state is also a heat problem: motors energised against gravity
with no supervisor will get hot. A long-hold policy (alarm, then park low) is
postponed but noted here so it is not forgotten.

### 5. Residual risk is acknowledged, not engineered away

**This arm cannot be made safe in software. It can be made predictable.**
A power loss drops it, and no watchdog changes that. The remaining mitigations
are physical, and they are part of the system design, not an afterthought:

- Nothing fragile, and no hands, under the workspace.
- Mount the arm so that a full-extension collapse does not damage the arm or
  the desk.
- Park low before any planned power-down. Treat "powered down while extended"
  as an operating error.
- Consider a physical parking stand or a counterbalance for unattended
  power-down.

## Must measure (before any autonomous motion)

The whole design rests on one unverified premise: **what does a RobStride drive
do when command frames stop arriving?**

- If it **holds the last MIT setpoint**, we have the strongest safety property
  available on this hardware. The arm survives our process being `SIGKILL`ed,
  with no dependency on our software at all.
- If it **disables itself after a communication timeout**, the arm falls
  whenever our process dies, and we need a different answer: a separate
  always-on holder process, a firmware parameter change, or physical mitigation.

**Experiment (Lab 00, blocking):** enable one joint in MIT mode at a low, safe
pose; hold the arm by hand; `SIGKILL` the sender; observe. Repeat with the
joint lightly loaded. Record the timeout, if there is one. Keep the arm low,
with something soft underneath, and do this before anything else moves.

**Check the manual first.** Motor controllers of this type often have a *CAN
communication timeout* parameter: a non-zero value disables the motor after
N ms without a command; zero turns the timeout off, so the motor holds.
Whether RobStride has such a parameter, at which register, and with what
default, could not be confirmed from public sources. (The manuals sit behind a
CDN this session cannot reach, and RobStride's published sample program does
not touch it.) Check the parameter table in the manual that came with the arm,
and list the `0x70xx` register space if the manual says nothing. If the
parameter exists and defaults to a non-zero timeout, **setting it to zero is
the single most valuable configuration change available on this hardware.** It
turns "hard death drops the arm" into "hard death freezes the arm."

Note the trade-off before changing it: a motor that holds forever is safer
against falling and less safe against a runaway setpoint. §4b exists because of
exactly that trade-off.

Everything in the last rows of §3 is provisional until this measurement exists.

## Consequences

- **Easier:** the Jupyter and Python risk is contained by design. A kernel
  restart drops the heartbeat, the RT core ramps down and holds, and nothing
  falls.
- **Easier:** "which stop?" has a documented answer for each fault, so the
  safety supervisor is a table, not a pile of special cases.
- **Harder:** the compliant-hold controller must run in the cyclic path with no
  allocation, gravity term included. Pinocchio's `Data` is allocated once at
  start-up; after that, RNEA allocates nothing.
- **Harder:** every subsystem must define its fault signal explicitly. "It threw
  an exception" is not a fault classification.
- **Revisit if:** brakes are added, a drive with safe-torque-off is fitted, or
  the experiment shows that the drives disable themselves on lost communication.
