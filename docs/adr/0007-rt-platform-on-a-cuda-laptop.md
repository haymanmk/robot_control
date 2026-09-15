# ADR-0007: Real-Time Platform on a Single CUDA Laptop

**Status:** Accepted
**Date:** 2026-09-15
**Deciders:** haymanmk
**Amends:** [ADR-0002](0002-target-platform-rebot-b601-rs.md) §"Xenomai is off the table"

## Context

ADR-0002 closed the Xenomai question on the grounds that our loop is a 2 ms
*outer* setpoint loop and CAN bus load binds before kernel latency does. That
reasoning still holds for the *nominal* case. It assumed, however, a control PC
doing nothing but control.

The actual platform is one laptop, and it must also:

- run **local CUDA inference**, so the proprietary NVIDIA driver is mandatory —
  not a preference, a requirement of the application;
- be the developer's only machine, so it needs a working desktop (the AMD driver
  under PREEMPT_RT showed unusable fractional scaling and intermittent flicker).

GPU drivers are a well-known source of latency spikes under PREEMPT_RT: large
interrupt handlers, long non-preemptible sections, and an out-of-tree module that
must be rebuilt by DKMS against every RT kernel. So ADR-0002's dismissal was
reasoning about a machine we do not have. Xenomai returns as a **contingency** —
not because the loop got faster, but because the platform got noisier.

## Decision

1. **Single laptop, NVIDIA proprietary driver, accepted as a constraint.** CUDA
   inference is part of the application; the driver is not negotiable and the
   trade-off is understood.
2. **PREEMPT_RT plus CPU isolation first.** Measure. Escalate only on evidence.
   This is ADR-0001's own principle applied to a harder platform, not a new one.
3. **`hwlatdetect` runs before any conclusion about kernels** — see below. It is
   the measurement that decides whether escalating is even capable of helping.
4. **Xenomai stays a documented contingency**, reopened only if the isolation
   work fails its budget *and* `hwlatdetect` shows the latency source is
   something a dual-kernel can actually fix.
5. **Nothing in the RT path touches the GPU.** Inference is a non-RT tier
   activity ([ADR-0001](0001-rtos-and-middleware-selection.md) tier 3), and
   [ADR-0006](0006-process-topology-and-rt-client-transport.md)'s process
   separation makes that structural: inference is a bridge client like any other.

### The measurement that must come first

**If the latency source is firmware, no kernel fixes it.** Laptops are poor RT
platforms for a reason that has nothing to do with Linux: System Management
Interrupts. SMIs run firmware code in SMM with *interrupts disabled and the OS
unaware* — they stall PREEMPT_RT and Xenomai identically, because neither kernel
is running during one. Thermal management, fan control, and battery handling all
trigger them, and they are more aggressive on laptops than on servers.

So `hwlatdetect` (from `rt-tests`) runs first. It measures hardware-induced
latency with the kernel factored out. Its result partitions the decision:

| `hwlatdetect` result | Meaning | Action |
|---|---|---|
| Max well under budget | Firmware is quiet; remaining latency is the kernel's | Kernel tuning can help. Proceed with isolation; Xenomai remains a live contingency. |
| Max near or over budget | Firmware is stalling the CPU | **Xenomai would not help either.** Fix the platform (BIOS, C-states, thermal policy) or move the RT core off this machine. |

Running `cyclictest` and drawing kernel conclusions *before* `hwlatdetect` is how
people spend months migrating to a dual-kernel that cannot fix their problem.

### Tuning order, cheapest first

1. `hwlatdetect` — establish the firmware floor.
2. Baseline `cyclictest` **on the current stock kernel**, idle and under load.
3. `isolcpus` + `nohz_full` + `rcu_nocbs` on the control cores.
4. **IRQ affinity, explicitly**: GPU and NIC interrupts *away* from the isolated
   cores; the CAN interface's interrupt *onto* them. This is usually the single
   largest win on a GPU machine, because the problem is not the driver's
   existence but its interrupts landing on the control CPU.
5. `taskset`/cgroup the CUDA inference process away from the isolated cores.
6. C-state and frequency-scaling policy; `performance` governor on control cores.
7. PREEMPT_RT, and re-measure every step above.

### The load under which numbers are taken

**Every timing number is measured with CUDA inference running.** A latency figure
from an idle laptop is not a statement about this system — ADR-0001's
"representative load" clause, made concrete. A benchmark run without the GPU busy
is not a baseline, it is a best case, and it must be labelled as such.

Provenance stamping ([ADR-0004](0004-system-decomposition.md) §3) therefore also
records: NVIDIA driver version, whether an inference workload was running, GPU
IRQ affinity, and `hwlatdetect`'s result for that machine.

## Consequences

- **Easier:** one machine to maintain and carry; inference and control colocate,
  removing a network hop from any perception→action loop.
- **Easier:** progress is unblocked. We stop debating kernels and start producing
  numbers, which is the only thing that can settle the debate.
- **Harder:** every RT kernel update is a DKMS rebuild of an out-of-tree module
  that may need patching. Pin a working kernel/driver pair and record it.
- **Harder:** the jitter budget may simply not be met on this hardware. The
  honest fallbacks, in order: accept a lower control rate (250 Hz is likely fine
  for an outer loop on self-closing drives); move the RT core to a small
  dedicated machine; or move the servo loop into a microcontroller
  (ADR-0001 Option C).
- **Revisit if:** `hwlatdetect` exposes a firmware floor above budget; a second
  machine becomes available; or measured jitter under CUDA load fails a written
  budget that does not yet exist (ADR-0002 action item 2 — still open, and now
  the gating artifact).
