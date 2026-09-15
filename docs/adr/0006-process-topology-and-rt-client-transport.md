# ADR-0006: Process Topology and the RT↔Client Transport

**Status:** Accepted
**Date:** 2026-09-15
**Deciders:** haymanmk
**Supersedes:** [ADR-0004](0004-system-decomposition.md) §2 (in-process pybind11 bindings as the live-control interface)

## Context

ADR-0004 proposed pybind11 bindings as *the* Python interface: Python and the RT
core in one process, `CyclicTask` on a C++ thread with the GIL released, and the
rule "no Python in the cyclic path" enforced by API shape.

The challenge raised against it was simply: *why a binding at all — why not IPC
or DDS?* That question exposes a conflation in ADR-0004. Two independent
decisions were being treated as one:

- **(a)** Does the client run in the **same process** as the RT core?
- **(b)** What **transport** carries commands and telemetry?

pybind11 answers (a) with "same process" and makes (b) trivial. IPC answers (a)
with "separate" and makes (b) a real design question. ADR-0004 picked the binding
for (b)'s convenience without weighing (a) on its own merits — and (a) is a
safety decision, not an ergonomics one.

### The argument that decides it

[ADR-0005](0005-safe-state-and-stop-architecture.md) ranks stop paths by how few
things must still be working:

```
drive firmware alone     <- survives our process being SIGKILLed
RT core + drive          <- survives the client dying
RT core + model          <- needs Pinocchio state sane
RT core + planner        <- not a stop at all
```

**An in-process binding collapses the second row into the first.** If the
interpreter segfaults, is OOM-killed, or the Jupyter kernel is restarted, it
takes the RT core down with it — and the arm's fate rests entirely on drive
firmware behaviour we have not yet measured. Every fault the watchdog exists to
survive becomes unsurvivable, because the watchdog dies in the same instant.

Separate processes make ADR-0005's Category 2 response *reachable*: the client
dies, the heartbeat stops, the RT core — still running — ramps and holds.

That is worth more than zero-copy convenience. **Process isolation is the
watchdog's precondition, not an optimisation.**

A secondary benefit falls out: ADR-0004's "no Python in the cyclic path" rule
stops being a discipline enforced by API shape and becomes a **structural
impossibility**. Python is in a different address space. It cannot be in the
loop, however creative anyone gets at 3am.

## Decision

### 1. The Python surface splits in two, because it has two jobs

| Surface | Job | Topology | Mechanism |
|---|---|---|---|
| **Analysis** | FK/IK on a numpy array, telemetry parsing, calibration fitting, trajectory maths. No live robot. | **In-process** | pybind11 over `core/model` and the telemetry decoder |
| **Live control** | Commanding a running arm, streaming telemetry | **Separate process** | shared-memory bridge (below) |

Splitting them is what makes both correct. Making `fk(q)` an RPC would be absurd;
making a live arm handle die with the interpreter is dangerous. They are
different problems and get different answers.

### 2. Transport for the live path: shared memory with fixed-size records

This is [ADR-0001](0001-rtos-and-middleware-selection.md)'s tier-2 bridge, which
was always in the architecture. It is now built first rather than last, and gains
its second consumer immediately.

- POSIX shared memory segment, fixed-size records, lock-free SPSC rings:
  **commands in**, **telemetry out**, plus a **client heartbeat counter**.
- The RT side never blocks, never allocates, never syscalls to publish.
- A client attaches, writes commands, bumps the heartbeat, and drains telemetry.
  The RT loop reads the heartbeat each cycle — ADR-0005's watchdog falls out for
  free rather than being bolted on.
- Layout is versioned; a client with a mismatched version is refused at attach.
- Telemetry drains **zero-copy**: Python can `numpy` -view the ring directly,
  which matters because ADR-0004 makes telemetry always-on and high-rate.

`bindings/python` therefore binds the **~200-line bridge client**, not the RT
core. Still pybind11, still convenient — but the interpreter is in another
process, so nothing it does can reach the loop.

### 3. Alternatives considered

| Option | Verdict |
|---|---|
| **In-process pybind11** (ADR-0004) | Rejected for live control: kills the watchdog. Retained for analysis, where there is no robot to endanger. |
| **Unix domain socket** | Fine for commands (low rate, ~10–50 µs). A copy per telemetry record, and a syscall on the RT side to publish. Rejected as primary; may return as the control/discovery channel beside the shm data plane. |
| **DDS / ROS2 topics** | Right transport, wrong hop. DDS allocates, runs discovery, and offers no real-time guarantee — ADR-0001 already bars it from the RT path. It also drags a full ROS2 install into a Jupyter session that only wants to plot data. **Correct as the outer interface** (planning, remote UI, multi-machine), consumed by `ros2/` as one more bridge client. |
| **gRPC / ZeroMQ** | Serialisation and a dependency, for no local advantage over shm+UDS. |

### 4. Consequence for ROS2

`ros2_control`'s `hardware_interface` becomes a bridge client exactly like the
Python one. Three clients (Python, ROS2, a future operator UI) on one versioned
interface is what proves the abstraction is real rather than assumed.

## Consequences

- **Easier:** the watchdog can actually do its job; a Jupyter kernel restart is
  now a recoverable event by construction rather than by hope.
- **Easier:** the RT core becomes a long-lived service. Clients attach and detach
  freely; the arm's held state survives every one of them.
- **Easier:** the bridge is exercised by a real consumer from day one, so its
  design is validated before ROS2 depends on it.
- **Harder:** shared-memory ring buffers with correct memory ordering are
  genuinely difficult, and wrong memory ordering fails rarely and unreproducibly.
  This is a place for careful review and for tests under `ThreadSanitizer`.
- **Harder:** two processes to start, supervise and version. The RT core needs a
  lifecycle (systemd unit or equivalent), not just a `main()`.
- **Harder:** the analysis and live surfaces must not diverge into two different
  vocabularies for the same concepts.
