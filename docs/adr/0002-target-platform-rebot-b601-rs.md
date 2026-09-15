# ADR-0002: Target Platform — reBot Arm B601-RS

**Status:** Accepted
**Date:** 2026-09-15
**Supersedes (in part):** ADR-0001's implicit hardware assumptions
**Deciders:** haymanmk

## Context

ADR-0001 chose an RT strategy against a *hypothetical* robot: EtherCAT fieldbus,
CiA 402 drives, a timing budget yet to be written. We now have a concrete target:
the **Seeed Studio reBot Arm B601-RS**, whose hardware and software are fully
open source. Several of ADR-0001's premises do not survive contact with it.

### What the hardware actually is

| Property | Value | Source |
|---|---|---|
| Kinematics | 6 DoF + 1 gripper, serial revolute chain | `Rebot_Arm_description/RS/urdf/ReBot_Arm_RS.urdf` |
| Reach / payload | 754 mm / 2.5 kg, ±0.1 mm repeatability | Seeed product page |
| Actuators J1–J3 | RobStride **RS-06** quasi-direct-drive | `config/rebotarm_rs.yaml` |
| Actuators J4–J6, gripper | RobStride **RS-00** | `config/rebotarm_rs.yaml` |
| Fieldbus | **CAN** (SocketCAN `can0`), one motor per CAN node id `0x01`–`0x07` | `config/rebotarm_rs.yaml` |
| Nominal control rate | **500 Hz** (`rate: 500`) | `config/rebotarm_rs.yaml` |
| Bus supply | 48 V | Seeed docs |
| Drive-side control modes | MIT impedance (`pos, vel, kp, kd, tau`), POS_VEL, VEL | `actuator/rebotarm.py` |

### What this changes about ADR-0001

1. **No EtherCAT.** The bus is CAN, so IgH/SOEM, DC distributed clocks, and the
   whole EtherCAT tooling line in ADR-0001 do not apply. CAN has no
   hardware-synchronised cycle: frames arbitrate by identifier priority, and bus
   load — not the kernel — sets the floor on worst-case command latency.
   *This is the single most important correction.*

2. **ADR-0001's Option C is already the reality, not an option.** The RobStride
   drives close their own current and (in POS_VEL/VEL mode) velocity and position
   loops on-board at their own internal rate. Our loop is an **outer** loop:
   at 500 Hz we ship setpoints, we do not close a current loop. That relaxes the
   jitter budget by roughly an order of magnitude versus the software-FOC case
   that motivated Xenomai in ADR-0001.

3. **Therefore Xenomai is off the table for this robot.** ADR-0001 said adopt it
   only if measurement proves PREEMPT_RT insufficient; with a 2 ms period, an
   outer loop, and a CAN bus whose own arbitration jitter exceeds kernel
   scheduling jitter, there is no measurement that could reach that bar. The
   decision is now *closed* for B601-RS, not merely deferred.

### Bus-budget sanity check (the number that actually constrains us)

At 500 Hz with 7 motors, one command frame each and one feedback frame each,
a CAN frame is 8 data bytes plus ~44–64 bits of overhead ≈ 108–130 bits worst
case with stuffing.

    14 frames/cycle x ~130 bits x 500 Hz  ~=  0.91 Mbit/s

At **1 Mbit/s CAN that is ~90% bus utilisation** — far past the ~30–50% where
latency stays predictable. Conclusion to verify on hardware: either the bus runs
at a higher rate (CAN FD), or feedback is not polled from every motor every
cycle, or the effective rate is below 500 Hz. **Action item 1 below.**

This is the kind of constraint that no amount of kernel tuning fixes, and it is
why the fieldbus budget must be written before the RT tier.

### What the vendor stack looks like

`Seeed-Projects/reBotArm_control_py` (Apache-2.0), the baseline we learn from:

- `actuator/rebotarm.py` — config-driven HAL over the `motorbridge` SDK;
  joints collected into groups, each group with its own control mode.
- `kinematics/`, `dynamics/` — Pinocchio wrappers (FK, IK, RNEA, CRBA, centroidal).
- `controllers/` — gravity compensation, end-pose (Cartesian) controller.
- `trajectory/` — planner, sampler, CLIK tracker.
- `tools/gravity_calibration/` — a real identification campaign with recorded data.

Its control loop, verbatim (`actuator/rebotarm.py:802`):

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

This is a *relative* sleep: every cycle's wake-up latency is added to the period
and never repaid, so the loop runs slower than nominal and the error accumulates
without bound. It is also a plain `threading.Thread` — default scheduling class,
subject to the GIL, no memory locking. Excellent, honest baseline code for
getting a robot moving; it is not a real-time loop, and the gap between those two
statements is Lab 01.

## Decision

1. **Target platform is reBot Arm B601-RS over SocketCAN.** All timing budgets,
   labs, and interfaces are written against this robot, not a generic one.
2. **PREEMPT_RT only.** The Xenomai question from ADR-0001 is closed for this
   robot (see §2–3 above). Reopen only if the actuators change.
3. **Our loop is an outer setpoint loop at 500 Hz nominal.** Torque-level control
   is delegated to the drives' MIT mode; we never attempt a current loop.
4. **The fieldbus budget is written before the RT tier.** Kernel latency is not
   the binding constraint here; CAN bus utilisation is.
5. **Vendor stack is a reference, not a dependency.** We read it, we reuse its
   calibration data and URDF, and we re-derive the control path ourselves —
   that is the point of the project.
6. **Keep ADR-0001's three-tier shape**: hard-RT core / lock-free bridge /
   non-RT ROS2 + UI tier. That structure survives the hardware change intact.

## Consequences

- **Easier:** a 2 ms outer loop on PREEMPT_RT is a solved problem; we can spend
  our learning budget on kinematics, dynamics, and trajectory generation rather
  than on kernel archaeology.
- **Easier:** SocketCAN is ordinary Linux — `candump`, `cangen`, `can-utils`,
  and kernel tracing all work, so the bus is observable from day one.
- **Harder:** CAN offers no distributed clock. Command frames for the 7 joints
  are not applied simultaneously; joint-to-joint skew within a cycle is real and
  must be measured, not assumed away.
- **Harder:** we inherit the vendor's caveat that RobStride firmware's reported
  `mechVel` (`0x701A`) is not trustworthy in rad/s (`actuator/rebotarm.py`,
  `get_velocities`), so joint velocity must be derived by filtered
  finite-differencing — a state-estimation problem, not a read.

## Action Items

1. [ ] **Measure the CAN bus**: actual bitrate, frames per cycle, bus load, and
       worst-case frame latency under full 7-joint commanding (`candump -ta`,
       bus-load from `ip -details -statistics link show can0`).
2. [ ] Write the timing budget for *this* robot: period, jitter budget, allowed
       overrun rate, watchdog timeout, joint-to-joint skew tolerance.
3. [ ] **Lab 01** — quantify loop jitter for the vendor pattern vs a phase-locked
       loop, on ordinary Linux and again on PREEMPT_RT with SCHED_FIFO.
4. [ ] Measure round-trip latency: command frame on the wire to feedback frame
       for the same joint.
5. [ ] Decide the safety architecture. The B601-RS has no STO and no safety
       relay; a 2.5 kg-payload arm with 48 V QDD motors is not intrinsically
       safe. Decide explicitly what the e-stop chain is, and document that this
       robot is a development platform, not a certifiable one.
