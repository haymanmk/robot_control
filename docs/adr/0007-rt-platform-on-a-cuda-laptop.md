# ADR-0007: Real-Time Platform on a Single CUDA Laptop

**Status:** Accepted
**Date:** 2026-09-15
**Deciders:** haymanmk
**Amends:** [ADR-0002](0002-target-platform-rebot-b601-rs.md), "Xenomai is off the table"

## Context

ADR-0002 closed the Xenomai question because our loop is a 2 ms *outer*
setpoint loop and CAN bus load matters more than kernel latency. That reasoning
still holds for the *normal* case. But it assumed a control PC that does
nothing except control.

The actual platform is one laptop, and it must also:

- run **local CUDA inference**, so the proprietary NVIDIA driver is required.
  This is not a preference; the application needs it.
- serve as the developer's only machine, so it needs a working desktop. (The
  AMD driver under PREEMPT_RT gave unusable fractional scaling and intermittent
  flicker.)

GPU drivers are a well-known source of latency spikes under PREEMPT_RT: large
interrupt handlers, long sections that cannot be pre-empted, and an
out-of-tree module that DKMS must rebuild for every real-time kernel. So
ADR-0002 was reasoning about a machine we do not have. Xenomai returns as a
**fallback** — not because the loop got faster, but because the platform got
noisier.

## Decision

1. **One laptop with the NVIDIA proprietary driver, accepted as a constraint.**
   CUDA inference is part of the application. The driver is not negotiable, and
   the trade-off is understood.
2. **PREEMPT_RT with CPU isolation first.** Measure. Escalate only on evidence.
   This is ADR-0001's own principle applied to a harder platform.
3. **Run `hwlatdetect` before drawing any conclusion about kernels.** See below.
   It is the measurement that decides whether changing the kernel could help at all.
4. **Xenomai stays a documented fallback.** Reopen it only if the isolation
   work fails its budget *and* `hwlatdetect` shows the latency source is
   something a dual-kernel could actually fix.
5. **Nothing in the real-time path touches the GPU.** Inference belongs to the
   non-real-time tier ([ADR-0001](0001-rtos-and-middleware-selection.md) tier 3),
   and [ADR-0006](0006-process-topology-and-rt-client-transport.md)'s process
   separation makes that structural: inference is a bridge client like any other.

### The measurement that comes first

**If the latency source is firmware, no kernel fixes it.** Laptops are poor
real-time platforms for a reason that has nothing to do with Linux: System
Management Interrupts (SMIs). An SMI runs firmware code in System Management
Mode with interrupts disabled and the operating system unaware. It stalls
PREEMPT_RT and Xenomai equally, because neither kernel is running during one.
Thermal management, fan control, and battery handling all trigger SMIs, and
laptops use them more aggressively than servers.

So `hwlatdetect` (from `rt-tests`) runs first. It measures hardware-caused
latency with the kernel taken out of the picture. Its result splits the decision:

| `hwlatdetect` result | Meaning | Action |
|---|---|---|
| Maximum well under budget | Firmware is quiet; the remaining latency belongs to the kernel | Kernel tuning can help. Proceed with isolation; Xenomai stays a live fallback. |
| Maximum near or over budget | Firmware is stalling the CPU | **Xenomai would not help either.** Fix the platform (BIOS, C-states, thermal policy) or move the real-time core to another machine. |

Running `cyclictest` and drawing conclusions about kernels *before*
`hwlatdetect` is how people spend months moving to a dual-kernel that cannot
fix their problem.

### Tuning order, cheapest first

1. `hwlatdetect` — establish the firmware floor.
2. Baseline `cyclictest` **on the current stock kernel**, idle and under load.
3. `isolcpus`, `nohz_full`, and `rcu_nocbs` on the control cores.
4. **IRQ affinity, set explicitly:** GPU and network interrupts *away from* the
   isolated cores; the CAN interface's interrupt *onto* them. This is usually
   the single biggest win on a GPU machine. The problem is not that the driver
   exists; it is that its interrupts land on the control CPU.
5. Use `taskset` or a cgroup to keep the CUDA inference process off the
   isolated cores.
6. C-state and frequency-scaling policy; the `performance` governor on the
   control cores.
7. PREEMPT_RT, then re-measure every step above.

### The load under which numbers are taken

**Every timing number is measured with CUDA inference running.** A latency
figure from an idle laptop says nothing about this system. This is ADR-0001's
"representative load" clause made concrete. A benchmark run without the GPU
busy is not a baseline; it is a best case, and it must be labelled as one.

Provenance stamping ([ADR-0004](0004-system-decomposition.md) §3) therefore
also records: the NVIDIA driver version, whether an inference workload was
running, GPU IRQ affinity, and the `hwlatdetect` result for that machine.

## Consequences

- **Easier:** one machine to maintain and carry. Inference and control share a
  box, which removes a network hop from any perception-to-action loop.
- **Easier:** progress is unblocked. We stop debating kernels and start
  producing numbers, which is the only thing that can settle the debate.
- **Harder:** every real-time kernel update means a DKMS rebuild of an
  out-of-tree module that may need patching. Pin a working kernel and driver
  pair and record it.
- **Harder:** the jitter budget may simply not be achievable on this hardware.
  The honest fallbacks, in order: accept a lower control rate (250 Hz is
  probably fine for an outer loop on drives that close their own loops); move
  the real-time core to a small dedicated machine; or move the servo loop into
  a microcontroller (ADR-0001, Option C).
- **Revisit if:** `hwlatdetect` shows a firmware floor above budget; a second
  machine becomes available; or measured jitter under CUDA load fails a written
  budget. That budget does not exist yet (ADR-0002, action item 2) and is now
  the gating document.
