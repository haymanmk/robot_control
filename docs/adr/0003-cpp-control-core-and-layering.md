# ADR-0003: C++ Control Core and Layer Boundaries

**Status:** Accepted
**Date:** 2026-09-15
**Deciders:** haymanmk
**Builds on:** [ADR-0001](0001-rtos-and-middleware-selection.md) (three-tier shape), [ADR-0002](0002-target-platform-rebot-b601-rs.md) (target hardware)

## Context

The vendor baseline, `reBotArm_control_py`, is Python throughout, including the
500 Hz cyclic loop. It works: the arm moves, gravity compensation holds, IK
solves. So the question is not whether Python *can* drive this arm. It clearly
can. The question is what we are trying to learn.

Two options were considered seriously.

**Python first, port later.** The least friction, and it would let us compare
our code line by line with the vendor's. But it postpones every question that
makes a control system *real-time* — allocation in the cyclic path, memory
locking, scheduling class, lock-free hand-off between tiers — to a rewrite
later. Those questions are the point of this project, and rewrites tend not to
happen.

**C++ from the start.** Slower at first. But the constraints that make
real-time code different from ordinary code are only *visible* in a language
that lets you break them. You cannot learn why the cyclic path must not
allocate in a language where every operation might.

### The measurement behind the decision

Lab 01 was written in both languages and run on the same machine:

```
                    Python                    C++
strategy            mean    max    drift      mean    max    drift
relative-sleep      94.2u  286.4u  188.4ms    93.0u  330.3u  185.8ms
absolute-sleep       0.0u  241.1u    0.1ms     0.1u 2269.4u    0.1ms
clock-nanosleep      0.0u 1713.4u    0.1ms     0.0u  293.0u    0.1ms
```

**The results are the same.** At 500 Hz with 400 µs of work per cycle, C++ is
not measurably better than Python at meeting a deadline. On a non-real-time
kernel the scheduler dominates, and the scheduler does not care which language
woke it up. The difference between two runs of the same language is larger than
the difference between the languages.

This is worth stating plainly, because "C++ is faster" would be the wrong
reason to choose it here and would set the wrong expectations. **C++ is chosen
for control, not speed.** It has no garbage collector, no GIL, and no hidden
allocation, and it lets us state and enforce "this function does not allocate,
block, or make a system call". Those properties matter in the *tail* of the
distribution, which is the only part a control system cares about. They will
matter more as the cycle body grows from a busy loop into real dynamics plus
CAN I/O.

## Decision

1. **The control core is C++20.** This covers the cyclic path, the hardware
   layer, kinematics and dynamics, trajectory generation, and safety supervision.
2. **Python is for tooling, not control:** calibration analysis, plotting,
   protocol exploration, offline trajectory checks. Python versions of labs for
   comparison (as in Lab 01) are welcome; they are evidence, not product code.
3. **Layers.** Each layer depends only on the layers above it in this list:

   ```
   core/rt/        real-time primitives: clock, cyclic executive, real-time
                   privileges, allocation-free measurement. Depends on libc only.
   core/telemetry/ per-cycle record, provenance, file sink.
   core/bridge/    shared-memory transport between the real-time core and clients.
   core/can/       SocketCAN transport, frame encoding, bus statistics.
   core/drive/     RobStride protocol: modes, scaling, enable and disable, feedback.
   core/model/     kinematics and dynamics (Pinocchio), driven by the URDF.
   core/control/   controllers: gravity compensation, impedance, trajectory tracking.
   core/safety/    limits, watchdog, stop-category supervisor.
   app/            the programs that actually run.
   ros2/           the non-real-time tier: ros2_control, MoveIt2, Foxglove.
   ```

4. **Rules for the cyclic path.** These are enforced by review and by the
   measurements built into `core/rt`, not by good intentions:
   - No allocation. No `new`, no growing containers, no `std::string`.
   - No locks that a non-real-time thread could hold. Lock-free queues only.
   - No logging and no I/O, except the fieldbus itself.
   - No exceptions thrown across the cycle boundary.
   - Every cyclic function is `noexcept` and documents its worst-case cost.

5. **`core/rt` is the foundation and is written first.** Everything cyclic runs
   on `CyclicTask`; nothing writes its own loop. When Lab 15 moves us to
   PREEMPT_RT with CPU isolation, that changes `RtOptions`, not any controller.

6. **The build is plain CMake.** No ROS build system in the core. The core must
   build, test, and measure without ROS installed. That is what keeps the
   real-time boundary honest as features are added. ROS2 wraps the core from
   the outside, in `ros2/`.

## Consequences

- **Easier:** the real-time rules are stated once, in `core/rt`, and inherited
  everywhere. Measurement (wake latency, period error, execution time, drift)
  is built into the executive, so every controller is measurable by default.
- **Easier:** the core has no dependency on ROS, Python, or a specific robot.
  It can be unit-tested on a laptop, and the same binary runs against a
  simulated or a real CAN bus.
- **Harder:** we lose the line-by-line comparison with the vendor code. To
  compensate, the vendor repository stays checked out as a reference, and each
  lab ends by reading the matching Python and asking what it cost them.
- **Harder:** Pinocchio's C++ API is less documented than its Python bindings,
  and every kinematics bug now needs a debugger instead of an interactive
  session. The Phase 2 labs derive FK and the Jacobian by hand first and use
  Pinocchio as a check, which was the teaching plan anyway.
- **Slower to first motion.** Accepted on purpose. The goal is not to make the
  arm move (the vendor code already does that), but to understand every layer
  between a desired pose and a CAN frame.

## Notes for later

- `CyclicTask::Body` is a `std::function`, so the cycle body is called through a
  pointer. It does not allocate per call, so it is not a real-time violation.
  If the body ever needs to be inlined, template the executive. Measure first.
- `CyclicTask` does not skip cycles to catch up after an overrun. It runs
  without sleeping until it recovers and reports `missed_deadlines`. Revisit
  once we know what the drives do with a late setpoint (Lab 03).
