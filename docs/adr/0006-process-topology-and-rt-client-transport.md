# ADR-0006: Process Topology and the Transport Between the RT Core and Its Clients

**Status:** Accepted
**Date:** 2026-09-15
**Deciders:** haymanmk
**Supersedes:** [ADR-0004](0004-system-decomposition.md) §2 (in-process pybind11 bindings as the live-control interface)

## Context

ADR-0004 proposed pybind11 bindings as *the* Python interface: Python and the
real-time core in one process, `CyclicTask` on a C++ thread with the GIL
released, and the rule "no Python in the cyclic path" enforced by the shape of
the API.

The challenge to that was simple: *why a binding at all? Why not IPC or DDS?*
The question exposed a mix-up in ADR-0004. Two independent decisions were being
treated as one:

- **(a)** Does the client run in the **same process** as the real-time core?
- **(b)** What **transport** carries commands and telemetry?

pybind11 answers (a) with "same process", and then (b) is trivial. IPC answers
(a) with "separate", and then (b) is a real design question. ADR-0004 chose the
binding because it made (b) easy, without weighing (a) on its own. And (a) is a
safety decision, not a convenience.

### The argument that settles it

[ADR-0005](0005-safe-state-and-stop-architecture.md) ranks stop paths by how
few things must still be working:

```
drive firmware alone     <- survives our process being SIGKILLed
RT core + drive          <- survives the client dying
RT core + model          <- needs Pinocchio's state to be sane
RT core + planner        <- not a stop at all
```

**An in-process binding merges the second row into the first.** If the
interpreter segfaults, is killed for using too much memory, or its Jupyter
kernel is restarted, it takes the real-time core down with it. The arm's fate
then rests entirely on drive firmware behaviour we have not measured. Every
fault the watchdog exists to survive becomes unsurvivable, because the watchdog
dies at the same moment.

Separate processes make ADR-0005's Category 2 response *reachable*: the client
dies, the heartbeat stops, and the real-time core — still running — ramps down
and holds.

That is worth more than the convenience of zero-copy access. **Process
isolation is a precondition for the watchdog, not an optimisation.**

A second benefit follows. ADR-0004's rule "no Python in the cyclic path" stops
being a discipline enforced by API shape and becomes **physically impossible**.
Python is in a different address space. It cannot be in the loop.

## Decision

### 1. The Python surface splits in two, because it has two jobs

| Surface | Job | Where it runs | Mechanism |
|---|---|---|---|
| **Analysis** | FK and IK on a numpy array; reading telemetry; calibration fits; trajectory maths. No live robot. | **In the same process** | pybind11 over `core/model` and the telemetry decoder |
| **Live control** | Commanding a running arm; streaming telemetry | **A separate process** | the shared-memory bridge (below) |

Splitting them is what makes both correct. Making `fk(q)` a remote call would
be absurd; letting a live arm handle die with the interpreter is dangerous.
They are different problems and get different answers.

### 2. Transport for live control: shared memory with fixed-size records

This is the tier-2 bridge from [ADR-0001](0001-rtos-and-middleware-selection.md),
which was always part of the architecture. It is now built first rather than
last, and it gets its second consumer immediately.

- A POSIX shared-memory segment holding fixed-size records and lock-free
  single-producer/single-consumer rings: **commands in**, **telemetry out**, a
  **state snapshot**, and a **client heartbeat counter**.
- The real-time side never blocks, never allocates, and never makes a system
  call to publish.
- A client attaches, writes commands, advances the heartbeat, and reads
  telemetry. The real-time loop checks the heartbeat every cycle, so
  ADR-0005's watchdog comes for free instead of being bolted on.
- The layout is versioned. A client with a different layout version is refused
  at attach time.
- Telemetry can be read **without copying**: Python can view the ring directly
  as a numpy array. This matters because ADR-0004 makes telemetry always on and
  high rate.

`bindings/python` therefore binds the **small bridge client**, not the
real-time core. It is still pybind11 and still convenient, but the interpreter
is in another process, so nothing it does can reach the loop.

### 3. Alternatives considered

| Option | Verdict |
|---|---|
| **In-process pybind11** (ADR-0004) | Rejected for live control: it kills the watchdog. Kept for analysis, where there is no robot to endanger. |
| **Unix domain socket** | Fine for commands (low rate, 10–50 µs). But it copies every telemetry record and needs a system call on the real-time side to publish. Rejected as the main channel; it may return as a control or discovery channel beside the shared-memory data plane. |
| **DDS / ROS2 topics** | The right transport for the wrong hop. DDS allocates, runs discovery, and gives no real-time guarantee; ADR-0001 already keeps it out of the real-time path. It would also pull a full ROS2 install into a Jupyter session that only wants to plot data. **Correct as the outer interface** (planning, remote UI, multiple machines), used by `ros2/` as one more bridge client. |
| **gRPC / ZeroMQ** | Serialisation and an extra dependency, with no local advantage over shared memory plus a socket. |

### 4. What this means for ROS2

`ros2_control`'s `hardware_interface` becomes a bridge client, exactly like the
Python one. Three clients — Python, ROS2, and a future operator UI — on one
versioned interface is what proves the abstraction is real rather than assumed.

## Consequences

- **Easier:** the watchdog can do its job. A Jupyter kernel restart is now a
  recoverable event by design, not by luck.
- **Easier:** the real-time core becomes a long-running service. Clients attach
  and detach freely; the arm's held state outlives every one of them.
- **Easier:** the bridge has a real consumer from the first day, so its design
  is tested before ROS2 depends on it.
- **Harder:** shared-memory ring buffers with correct memory ordering are
  genuinely difficult, and wrong memory ordering fails rarely and in ways that
  are hard to reproduce. This code gets careful review and tests under
  ThreadSanitizer.
- **Harder:** two processes to start, supervise, and version. The real-time
  core needs a lifecycle (a systemd unit or similar), not just a `main()`.
- **Harder:** the analysis and live surfaces must not drift into two different
  vocabularies for the same ideas.
