# ADR-0003: C++ Control Core and Layer Boundaries

**Status:** Accepted
**Date:** 2026-09-15
**Deciders:** haymanmk
**Builds on:** [ADR-0001](0001-rtos-and-middleware-selection.md) (three-tier shape), [ADR-0002](0002-target-platform-rebot-b601-rs.md) (target hardware)

## Context

The vendor baseline (`reBotArm_control_py`) is Python throughout, including the
500 Hz cyclic path. It works — the arm moves, gravity compensation holds, IK
solves. The question is not whether Python *can* drive this arm; demonstrably it
can. The question is what we are trying to learn.

Two options were considered seriously.

**Python first, port later.** Lowest friction, and lets us diff our code
line-for-line against the vendor's. But it defers every question that makes a
control system *real-time* — allocation in the cyclic path, memory locking,
scheduling class, lock-free handoff between tiers — to a rewrite later. Those
questions are the fundamentals this project exists to learn, and a rewrite is
where good intentions go to die.

**C++ from the start.** Slower initially. But the constraints that make RT code
different from ordinary code are only *visible* in a language that lets you
violate them: you cannot learn why the cyclic path must not allocate in a
language where every operation might.

### The measurement that informed this

Lab 01 was written twice, once in each language, and run on the same machine:

```
                    Python                    C++
strategy            mean    max    drift      mean    max    drift
relative-sleep      94.2u  286.4u  188.4ms    93.0u  330.3u  185.8ms
absolute-sleep       0.0u  241.1u    0.1ms     0.1u 2269.4u    0.1ms
clock-nanosleep      0.0u 1713.4u    0.1ms     0.0u  293.0u    0.1ms
```

**They are the same.** At 500 Hz with 400 µs of work, C++ is not measurably
better than Python at hitting a deadline — because on a non-RT kernel the
scheduler dominates, and the scheduler does not care what language woke it up.
Run-to-run variation between the two exceeds the difference between them.

This is worth being explicit about, because "C++ is faster" is the wrong reason
to choose it here and would set the wrong expectations. **C++ is chosen for
control, not speed:** no garbage collector, no GIL, no hidden allocation, and
the ability to state and enforce "this function does not allocate, block, or
syscall". Those properties matter at the *tail* of the distribution, which is
the only part of it a control system cares about — and they will matter more as
the cycle body grows from a busy-spin into real dynamics plus CAN I/O.

## Decision

1. **The control core is C++20.** Cyclic path, HAL, kinematics/dynamics,
   trajectory generation, safety supervision.
2. **Python is kept for tooling, not control**: calibration analysis, plotting,
   protocol exploration, offline trajectory checking. Ports of labs into Python
   for comparison (as in Lab 01) are welcome — they are evidence, not code.
3. **Layering** (each layer depends only on those above it in this list):

   ```
   core/rt/        real-time primitives: clock, cyclic executive, RT privileges,
                   allocation-free instrumentation.  Depends on: libc only.
   core/can/       SocketCAN transport, frame codecs, bus statistics.
   core/drive/     RobStride protocol: modes, scaling, enable/disable, feedback.
   core/model/     kinematics + dynamics (Pinocchio), URDF-driven.
   core/control/   controllers: gravity comp, impedance, trajectory tracking.
   app/            composition roots: the actual programs that run.
   bridge/         lock-free RT <-> non-RT handoff (ADR-0001's tier boundary).
   ros2/           non-RT tier: ros2_control, MoveIt2, Foxglove.
   ```

4. **Rules for the cyclic path**, enforced by review and by Lab 01's
   instrumentation, not by hope:
   - No allocation. No `new`, no growing containers, no `std::string`.
   - No locks that a non-RT thread can hold. Lock-free SPSC queues only.
   - No logging, no I/O except the fieldbus itself.
   - No exceptions thrown across the cycle boundary.
   - Every cyclic function is `noexcept` and documented with its worst-case cost.

5. **`core/rt` is the foundation and is written first.** Everything cyclic runs
   on `CyclicTask`; nothing rolls its own loop. When Lab 15 moves us to
   PREEMPT_RT with CPU isolation, that is a change to `RtOptions`, not to any
   controller.

6. **Build is plain CMake**, no ROS build system in the core. The core must be
   buildable, testable, and measurable without ROS installed — that is what
   keeps the RT/non-RT boundary honest as features accrete. ROS2 wraps the core
   from outside, in `ros2/`.

## Consequences

- **Easier:** the RT discipline is stated once, in `core/rt`, and inherited
  everywhere. Instrumentation (wake latency, period error, execution time,
  drift) is built into the executive, so every controller is measurable by
  default rather than as an afterthought.
- **Easier:** the core has no dependency on ROS, Python, or a specific robot, so
  it can be unit-tested on a laptop and the same binary runs against a simulated
  or a real CAN bus.
- **Harder:** we give up direct line-by-line comparison with the vendor code.
  Mitigation: the vendor repo stays checked out as reference, and each lab ends
  by reading the corresponding Python and asking what it cost them.
- **Harder:** Pinocchio's C++ API is less documented than its Python bindings,
  and every kinematics bug now needs a debugger instead of a REPL. Mitigation:
  Phase 2 labs derive FK/Jacobian by hand first and use Pinocchio as the *check*
  — which was the pedagogical plan anyway.
- **Slower to first motion.** Accepted deliberately: the goal is not to make the
  arm move (the vendor code already does that, and it is installed), it is to
  understand every layer between a desired pose and a CAN frame.

## Notes for later

- The `std::function` in `CyclicTask::Body` is an indirect call per cycle. It
  does not allocate per call, so it is not an RT violation, but if the cycle
  body ever needs to be inlined, template the executive. Measure before changing.
- `CyclicTask` currently does not skip cycles to catch up after an overrun; it
  runs flat out until it recovers, and reports `missed_deadlines`. Revisit when
  we know what the drives do with a late setpoint (Lab 03).
