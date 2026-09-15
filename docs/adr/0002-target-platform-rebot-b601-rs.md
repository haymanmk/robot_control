# ADR-0002: Target Platform — reBot Arm B601-RS

**Status:** Accepted
**Date:** 2026-09-15
**Supersedes (in part):** the hardware assumptions implied by ADR-0001
**Deciders:** haymanmk

## Context

ADR-0001 chose a real-time strategy for a *hypothetical* robot: an EtherCAT
fieldbus, industrial CiA 402 drives, and a timing budget still to be written.
We now have a concrete target, the **Seeed Studio reBot Arm B601-RS**. Its
hardware and software are fully open source. Several of ADR-0001's assumptions
do not hold for it.

### What the hardware actually is

| Property | Value | Source |
|---|---|---|
| Kinematics | 6 revolute joints plus 1 gripper, serial chain | `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf` |
| Reach and payload | 754 mm; 2.5 kg; ±0.1 mm repeatability | Seeed product page |
| Actuators, joints 1–3 | RobStride **RS-06** quasi-direct-drive | `config/rebotarm_rs.yaml` |
| Actuators, joints 4–6 and gripper | RobStride **RS-00** | `config/rebotarm_rs.yaml` |
| Fieldbus | **CAN** via SocketCAN (`can0`); one motor per CAN ID, `0x01`–`0x07` | `config/rebotarm_rs.yaml` |
| Nominal control rate | **500 Hz** (`rate: 500`) | `config/rebotarm_rs.yaml` |
| Bus supply | 48 V | Seeed documentation |
| Drive control modes | MIT impedance (`pos, vel, kp, kd, tau`), position with velocity limit, velocity | `actuator/rebotarm.py` |

### What this changes compared with ADR-0001

1. **There is no EtherCAT.** The bus is CAN. The IgH and SOEM masters,
   distributed clocks, and the rest of the EtherCAT tooling in ADR-0001 do not
   apply. CAN has no hardware-synchronised cycle. Frames win the bus by
   identifier priority, so bus load — not the kernel — sets the worst-case
   command latency. This is the most important correction.

2. **ADR-0001's "Option C" is not an option here; it is the situation.** The
   RobStride drives close their own current loops on board. In position and
   velocity modes they close those loops too. Our loop is an *outer* loop: at
   500 Hz we send setpoints; we never close a current loop. That loosens the
   jitter budget by roughly a factor of ten compared with the software
   current-loop case that motivated Xenomai in ADR-0001.

3. **So Xenomai is closed for this robot, not just deferred.** ADR-0001 said
   to adopt it only if PREEMPT_RT proved insufficient. With a 2 ms period, an
   outer loop, and a CAN bus whose own arbitration jitter is larger than kernel
   scheduling jitter, no measurement could reach that bar.
   (ADR-0007 later reopens Xenomai as a *contingency* for a different reason:
   the control PC also runs a GPU driver.)

### A quick check on bus load

At 500 Hz with 7 motors, one command frame and one feedback frame each, a CAN
frame is 8 data bytes plus about 44–64 bits of overhead — roughly 108–130 bits
in the worst case with bit stuffing.

    14 frames per cycle × ~130 bits × 500 Hz ≈ 0.91 Mbit/s

At **1 Mbit/s that is about 90% bus utilisation**, far above the 30–50% where
latency stays predictable. So one of these must be true, and we need to find
out which on the hardware: the bus runs faster (CAN FD), feedback is not
requested from every motor every cycle, or the real rate is below 500 Hz. That
is action item 1.

No kernel tuning can fix this kind of constraint. That is why the fieldbus
budget must be written before the real-time tier.

### What the vendor software looks like

`Seeed-Projects/reBotArm_control_py` (Apache-2.0) is the baseline we learn from:

- `actuator/rebotarm.py` — a configuration-driven hardware layer over the
  `motorbridge` SDK. Joints are grouped, and each group has its own control mode.
- `kinematics/` and `dynamics/` — Pinocchio wrappers (FK, IK, RNEA, CRBA, centroidal).
- `controllers/` — gravity compensation and an end-pose (Cartesian) controller.
- `trajectory/` — planner, sampler, and a CLIK tracker.
- `tools/gravity_calibration/` — a real identification campaign with recorded data.

Its control loop, quoted from `actuator/rebotarm.py:802`:

```python
def _control_loop_impl(self) -> None:
    dt = 1.0 / self._ctrl_rate
    while self._running:
        t0 = time.perf_counter()
        self._ctrl_fn(self, dt)
        elapsed = time.perf_counter() - t0
        sleep_time = dt - elapsed
        if sleep_time > 0:
            time.sleep(sleep_time)
```

This is a *relative* sleep. Each cycle adds the kernel's wake-up delay to the
period and never gets it back, so the loop runs slower than nominal and the
error grows without limit. It also runs on a plain `threading.Thread`: default
scheduling class, subject to the GIL, no memory locking. It is honest, readable
code for getting a robot moving. It is not a real-time loop. The gap between
those two statements is what Lab 01 measures.

## Decision

1. **The target platform is the reBot Arm B601-RS over SocketCAN.** Timing
   budgets, labs, and interfaces are written for this robot, not a generic one.
2. **PREEMPT_RT only.** The Xenomai question from ADR-0001 is closed for this
   robot (see points 2 and 3 above). Reopen it only if the actuators change.
3. **Our loop is an outer setpoint loop at a nominal 500 Hz.** Torque-level
   control is delegated to the drives' MIT mode. We never attempt a current loop.
4. **The fieldbus budget is written before the real-time tier.** Kernel latency
   is not the binding constraint here; CAN bus utilisation is.
5. **The vendor stack is a reference, not a dependency.** We read it, reuse its
   calibration data and URDF, and derive the control path ourselves. That is
   the point of the project.
6. **Keep ADR-0001's three-tier shape:** a hard real-time core, a lock-free
   bridge, and a non-real-time ROS2 and UI tier. That structure survives the
   change of hardware unchanged.

## Consequences

- **Easier:** a 2 ms outer loop on PREEMPT_RT is a solved problem. We can spend
  our effort on kinematics, dynamics, and trajectories rather than on the kernel.
- **Easier:** SocketCAN is ordinary Linux. `candump`, `cangen`, `can-utils`, and
  kernel tracing all work, so the bus can be observed from the first day.
- **Harder:** CAN has no distributed clock. Command frames for the 7 joints are
  not applied at the same instant. The skew between joints within a cycle is
  real and must be measured, not assumed away.
- **Harder:** the vendor notes that the RobStride firmware's reported velocity
  (`mechVel`, register `0x701A`) is not reliable in rad/s (see
  `get_velocities` in `actuator/rebotarm.py`). Joint velocity must therefore be
  estimated from filtered position differences. That is a state-estimation
  problem, not a register read.

## Action items

1. [ ] **Measure the CAN bus:** actual bit rate, frames per cycle, bus load, and
       worst-case frame latency with all 7 joints commanded (`candump -ta`;
       bus load from `ip -details -statistics link show can0`).
2. [ ] Write the timing budget for *this* robot: period, jitter budget, allowed
       overrun rate, watchdog timeout, and joint-to-joint skew tolerance.
3. [ ] **Lab 01** — measure loop jitter for the vendor pattern and for a
       phase-locked loop, on a stock kernel and again on PREEMPT_RT with `SCHED_FIFO`.
4. [ ] Measure round-trip latency: command frame on the wire to feedback frame
       for the same joint.
5. [ ] Decide the safety architecture. The B601-RS has no safe-torque-off and no
       safety relay. A 2.5 kg-payload arm with 48 V motors is not safe by
       nature. Decide explicitly what the emergency-stop chain is, and record
       that this robot is a development platform, not a certifiable one.
       (Done in ADR-0005.)
