# ADR-0008: Enforce the Cyclic-Path Rules Mechanically

**Status:** Accepted
**Date:** 2026-09-23
**Deciders:** haymanmk
**Builds on:** [ADR-0003](0003-cpp-control-core-and-layering.md) (the rules), [ADR-0001](0001-rtos-and-middleware-selection.md) Amendment 2, [ADR-0007](0007-rt-platform-on-a-cuda-laptop.md) Amendment 1

## Context

ADR-0003 states the rules for code on the cyclic path: no allocation, no
locks a non-real-time thread could hold, no logging and no I/O except the
fieldbus, no exceptions across the cycle boundary. Until now they were
enforced by reading the code. That has already failed once: the demo's cycle
body called `printf` on watchdog events, in plain sight, through several
reviews.

A discussion of EVL's health monitoring showed a better way. An EVL thread
that sets `EVL_T_WOSS` is told about every demotion from the out-of-band
stage, with a diagnostic naming the cause — `EVL_HMDIAG_SYSDEMOTE` for a
stray in-band syscall, `EVL_HMDIAG_EXDEMOTE` for an exception such as a page
fault. The rule is enforced by the kernel, per thread, and every violation is
attributed.

PREEMPT_RT has no stages to switch between, but it has the two kernel hooks
needed for the same enforcement:

- **seccomp-BPF** filters the syscalls of one thread. With an allowlist of
  what the loop is meant to call, anything else is refused and reported with
  its syscall number (`SIGSYS`, `si_syscall`), or the thread is killed.
- **`getrusage(RUSAGE_THREAD)`** counts the thread's page faults. After
  `mlockall` and a stack prefault, a correct loop faults zero times.

## Decision

1. **`core/realtime` gains a cyclic guard**, armed by `CyclicTask` when
   `CyclicConfig::guard` is set, after the real-time options are applied and
   before the first cycle. It installs a seccomp allowlist on the loop's thread
   and reads fault counters around the run. The report goes into
   `CyclicReport::guard`.

2. **The allowlist is the statement of what the loop may be.** It may keep
   time (`clock_gettime`, `clock_nanosleep`, `restart_syscall`), talk to the
   fieldbus (the socket send and receive family), and, when done, read its own
   counters and exit. Absent on purpose: `write` and `read` (logging), `futex`
   (a contended lock), `mmap` and `brk` (allocation), `nanosleep` (the relative
   sleep), `poll` (waiting), `openat` and `ioctl` (device or file access).
   Each absence is a rule the guard catches.

3. **A refused syscall is not performed and returns `ENOSYS` to its caller.**
   The loop continues and keeps its deadline. This is the right failure mode:
   the forbidden thing does not happen, and the report names it. A kill mode
   exists for tests that must prove a violation is fatal and for a production
   loop where continuing after a refusal is not acceptable.

4. **A guarded loop runs on a thread that exits when the loop ends.** A
   seccomp filter cannot be removed from a thread, so `CyclicTask` gains
   `run_in_thread()` and `run_until_in_thread()`, and the guard is documented
   as usable only with them. The calling thread stays unfiltered and does the
   reporting.

5. **The guard is on in every test and lab that runs a cyclic loop**, and any
   violation or fault fails the test. The demo runs guarded.

## Consequences

- **Easier:** two rules move from "a reviewer must notice" to "the build
  refuses". The first thing the guard found was our own demo's `printf` in
  the cycle body, which is now an event log printed after the loop.
- **Easier:** the failure is attributed. `first was write (refused, returned
  ENOSYS)` is a smaller search than a 3 ms spike in a histogram.
- **Easier:** the same discipline transfers if Xenomai is ever adopted: the
  guard is the PREEMPT_RT counterpart of `EVL_T_WOSS`, and the habits are the
  same.
- **Harder:** the allowlist must be kept honest. A new transport that uses a
  different syscall must add it to `GuardOptions::extra_allowed_syscalls`
  deliberately, with a reason, rather than switching the guard off.
- **Harder:** the loop thread is now a separate thread by construction, so a
  program's structure is "spawn the guarded loop, join it, then report". That
  is also the structure ADR-0006 wanted for a long-running core.
- **Not covered:** a lock that happens to be uncontended never reaches
  `futex`, so the guard sees it only on the day it contends. The rule against
  locks stays a review rule; the guard is a backstop, not a proof.
- **Not covered:** `clock_gettime` through the vDSO never enters the kernel,
  so the guard cannot distinguish it from a syscall it would refuse. It is on
  the allowlist for the case where the vDSO is unavailable.
