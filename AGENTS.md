# Guide for coding agents

This is a real-time controller for a robot arm with no brakes. The rules below
are not style preferences; most exist because breaking them can stall the
control loop or leave the arm holding a bad setpoint with nobody watching. Read
the linked ADR before changing anything a rule covers.

## Build and check before you push

```bash
cmake -B build && cmake --build build -j          # warnings are on: -Wall -Wextra -Wpedantic -Wshadow -Wconversion
ctest --test-dir build --output-on-failure        # both suites must pass
cmake -B build-tsan -DRC_SANITIZE=thread && cmake --build build-tsan -j
ctest --test-dir build-tsan --output-on-failure   # required if you touched core/rt or core/bridge
cmake -B build -DRC_DOCS_WARN_AS_ERROR=ON && cmake --build build --target docs   # if Doxygen is installed
```

A change is not done until the build is warning-free and the tests pass. Fix
the cause of a warning; do not silence it. Never skip, disable or loosen a
test to get green.

## Rules for the cyclic path

Anything callable from inside the 500 Hz loop follows all of these
([ADR-0003](docs/adr/0003-cpp-control-core-and-layering.md), `core/README.md`):

- **No allocation.** No `new`, no growing containers, no `std::string`.
- **No locks a non-real-time thread could hold.** Lock-free only. A mutex
  here lets a client killed mid-critical-section block the loop forever.
- **No logging and no I/O**, except the fieldbus itself.
- **No exceptions across the cycle boundary.**
- **Every cyclic function is `noexcept`** and documents its worst-case cost.
- **Every loop runs on `CyclicTask`.** Nothing writes its own sleep loop, and
  deadlines come from a fixed origin, never from "now" (`core/rt/include/rc/rt/cyclic_task.hpp`).
- **Time is `CLOCK_MONOTONIC` in `int64` nanoseconds**, never `CLOCK_REALTIME`
  and never `double`.

If you are unsure whether a function is on the cyclic path, assume it is.

## Things that look like bugs and are not

- **`Seqlock` and `SpscRing` use fences and relaxed atomics on purpose.** Do
  not "fix" them with a mutex. Do not change a memory order without reading the
  file comment, which cites the paper the construction comes from.
- **`SpscRing::push()` drops when full.** That is the policy: a stalled reader
  must not stall control. Do not add a retry loop.
- **GCC's ThreadSanitizer warns that `atomic_thread_fence` is unsupported.**
  Expected; the seqlock is data-race-free by construction. See `core/README.md`.
- **`RtStatus` reports refused real-time setup instead of failing.** Keep it
  that way. A process that silently runs without SCHED_FIFO produces timing
  numbers nobody should believe.

## Shared-memory layout is versioned

`core/bridge/include/rc/bridge/layout.hpp` and `core/telemetry/include/rc/telemetry/record.hpp`
are memcpy'd across a process boundary and written to disk. Any change to a
field, its order, or its size requires bumping `kLayoutVersion`, and the
`static_assert`s on record sizes are budgets, not accidents. Records must stay
trivially copyable with no pointers.

## Safety behaviour is a design decision, not a code detail

[ADR-0005](docs/adr/0005-safe-state-and-stop-architecture.md) decides what the
arm does on a fault. Do not change any of this without an ADR:

- The default fault response is a **Category 2 stop**: decelerate, then hold
  compliantly. It stays powered because there is no safe unpowered state.
- **Return-to-home is never a fault response.** It is a graceful-shutdown and
  recovery feature only, and it is never automatic.
- **The watchdog lives inside the cyclic loop** and has the fewest possible
  dependencies. `take_control()` arms it; `attach()` alone never does.
- **Never send gains you would not want held forever.** The drives may hold the
  last MIT setpoint if our process dies, so `kp` and `kd` are limited to values
  safe to leave unattended.
- Signal handlers must be async-signal-safe: set an atomic flag and return.

## Repository conventions

- Layers depend downward only: `rt -> telemetry -> bridge -> can -> drive -> model -> control -> safety -> app`.
  `core/rt` depends on libc alone.
- `core/` is C++20 with plain CMake and **no ROS and no Python.** Python lives
  in `tools/`, `bench/` and `notebooks/` and is for analysis, never control.
- Notebooks are `.py` files with `# %%` cell markers. No `.ipynb` is committed.
- Telemetry is always on and every run file carries its provenance. A new
  measurement goes in `bench/` with a write-up in `notebooks/`.
- Public headers carry the reasoning in `///` comments, with a one-line summary
  directly above every type and member so editors show it on hover. Longer
  "why" prose goes in the `@file` block. Link to ADRs rather than repeating them.
- Design decisions go in `docs/adr/` as a numbered ADR. Add one; do not edit
  history in an accepted ADR except to mark it superseded.
- Formatting follows the existing files: 2-space indent, `snake_case`
  functions, `PascalCase` types, `kConstant`, `member_` with a trailing
  underscore. There is no formatter configured; match the file you are in.

## Before any motion on the real arm

Lab 00 in [docs/ROADMAP.md](docs/ROADMAP.md) is blocking. Nothing moves under
its own power until the drives' behaviour on lost communication has been
measured. Do not write code that assumes the answer either way.
