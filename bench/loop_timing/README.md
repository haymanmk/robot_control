# loop_timing — Tier 1 determinism benchmark

*(Lab 01 in [the roadmap](../../docs/ROADMAP.md).)*

**Question:** the reBot vendor stack runs its 500 Hz loop with
`time.sleep(dt - elapsed)`. How wrong is that, in microseconds?

**No hardware needed.** There are two implementations, one in pure-standard-
library Python and one in C++, so you can also answer a second question: how
much of the error is Python's fault?

## Before you run it: predict

Write your answers down first. Getting one wrong is what makes the idea stick.

1. At 500 Hz (a 2000 µs period) with about 400 µs of work per cycle, will
   `time.sleep(dt - elapsed)` run **fast**, **slow**, or **correct on average**?
2. After 10 seconds — 5000 cycles — how far from the ideal schedule will it be?
   Guess the order of magnitude: 10 µs? 1 ms? 100 ms? 1 s?
3. Does using an **absolute** deadline instead fix the **average** error, the
   **worst case**, or both?

Then:

```bash
python3 loop_timing.py --seconds 10
```

## What it compares

| Strategy | Pattern | Where it comes from |
|---|---|---|
| `relative-sleep` | `sleep(dt - elapsed)` | the vendor's `_control_loop_impl` |
| `absolute-sleep` | deadline = `origin + n*dt`, then `time.sleep(remaining)` | the smallest fix |
| `clock-nanosleep` | `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline)` | how real-time loops are actually written |

Useful flags: `--rate`, `--work-us`, `--only <strategy>`, `--json out.json`,
and `--rt` (asks for `SCHED_FIFO` and `mlockall`; needs `sudo` or `CAP_SYS_NICE`).

## Reference run

Ordinary (non-real-time) kernel, 4 s per strategy, 500 Hz, 400 µs of work:

```
strategy               mean       p99     p99.9       max       drift  overruns
-------------------------------------------------------------------------------
relative-sleep        94.2u    163.0u    209.9u    286.4u     188.37m         3
absolute-sleep         0.0u     64.8u    222.9u    241.1u       0.08m         4
clock-nanosleep        0.0u     73.4u   1178.2u   1713.4u       0.08m         4
```

### How to read it

**The main result is the `drift` column.** `relative-sleep` lost **188 ms in
4 seconds.** Every cycle came out about 94 µs long, and that time was never
given back. That is 4.7% slow, permanently. Your "500 Hz" loop is really a
477 Hz loop, and after an hour it is **170 seconds behind** where it thinks it is.

Why: `sleep(dt - elapsed)` asks to sleep for a *duration*. The kernel promises
*at least* that long, then adds its own wake-up delay on top. The next cycle
starts from wherever you actually woke, so the error accumulates. Both
absolute strategies compute deadlines from a fixed starting point, so a late
wake-up is absorbed by the next, shorter sleep instead of piling up. Drift:
**0.08 ms**, about 2300 times better, and it does not grow with run time.

**Drift and jitter are different failures.** Notice that `absolute-sleep` and
`clock-nanosleep` have almost zero drift but a `max` in the same range as
`relative-sleep`, or worse. Fixing drift did nothing for jitter. Jitter comes
from the scheduler, and it needs different tools: `SCHED_FIFO`, `mlockall`, CPU
isolation, PREEMPT_RT.

**Why `clock-nanosleep` looks worse here.** Its 1.7 ms `max` in this run is not
the system call's fault. This measurement came from a shared cloud virtual
machine with a non-real-time kernel and other tenants. On a tuned PREEMPT_RT
machine it should be the *best* of the three. This is exactly ADR-0001's point
about benchmark posts: **a latency number means nothing without the machine and
the load it was measured on.** Run this on your own control PC and get your own
numbers. Never inherit someone else's.

**Why percentiles, not averages.** The `mean` for both absolute strategies is
0.0 µs. A control loop is not harmed by its average cycle; it is harmed by the
worst one. Always quote p99.9 and max. A vendor who quotes a mean latency is
telling you nothing.

## Does this matter for our arm?

For the B601-RS specifically: **the jitter, mostly not; the drift, yes.**

- **Jitter:** ±200 µs on a 2000 µs period is about 10%. Our loop sends
  *setpoints* to drives that close their own current loops (ADR-0002), so a
  setpoint that arrives 200 µs late is smoothed by the drive. Compare a
  software current loop at 20 kHz, where 200 µs is four whole periods and the
  motor goes unstable. That is the case Xenomai exists for. It is not our case.
- **Drift:** it always matters. Anything you integrate — velocity estimated
  from position differences (Lab 06), a trajectory sampled by cycle count
  (Lab 10), a watchdog counting missed heartbeats (Lab 05) — is wrong by 4.7%
  if the loop is silently 4.7% slow, and you will spend a week debugging it as
  a "calibration problem".

**The lesson is not "the vendor code is bad."** It is honest, readable code
that gets an arm moving, and that has real value. The lesson is that *"it
works" and "it is deterministic" are different claims*, and now you have a
number that tells them apart.

## Things to try

1. `--work-us 2500` — work that takes longer than the period. Watch what each
   strategy does when it cannot keep up. Which one tells you the truth?
2. `--rate 50` versus `--rate 1000` — does drift scale with the rate, or with
   the run time?
3. `sudo python3 loop_timing.py --rt` — how much does `SCHED_FIFO` gain on your
   machine? (On a stock kernel: less than you would hope.)
4. Run it while compiling something large in another terminal. That is the
   "under representative load" clause in ADR-0001's action item 3, and it is
   where honest numbers come from.

## The C++ version, and what it shows

```bash
cmake -B build && cmake --build build -j
./build/bench/loop_timing/bench_loop_timing --seconds 10
sudo ./build/bench/loop_timing/bench_loop_timing --rt 80 --cpu 3
```

Same machine, same run length, both languages:

```
                    Python                    C++
strategy            mean    max    drift      mean    max    drift
relative-sleep      94.2u  286.4u  188.4ms    93.0u  330.3u  185.8ms
absolute-sleep       0.0u  241.1u    0.1ms     0.1u 2269.4u    0.1ms
clock-nanosleep      0.0u 1713.4u    0.1ms     0.0u  293.0u    0.1ms
```

**Predict before you look:** how much better is C++ at meeting a 2 ms deadline?

Answer: **not at all.** The numbers are the same within run-to-run noise. On a
non-real-time kernel the scheduler dominates, and the scheduler does not care
which language woke it up.

That is not an argument against C++. It is an argument against the *usual*
argument for it. C++ is worth choosing here for **control**, not speed: no
garbage-collector pause, no GIL, no allocation you did not write yourself.
Those properties do nothing for the median cycle and everything for the tail,
once the cycle body stops being a busy loop and becomes Pinocchio's RNEA plus
seven CAN frames. See [ADR-0003](../../docs/adr/0003-cpp-control-core-and-layering.md).

The C++ version is also a working demo of `core/rt`. Its `clock-nanosleep`
strategy *is* `rc::rt::CyclicTask`, the executive that every cyclic loop in
this project runs on. Read `core/rt/include/rc/rt/cyclic_task.hpp`: the
comments are the lesson, and the code is the proof.

## Where the fundamentals live

- **Monotonic versus wall clock.** `CLOCK_MONOTONIC` never jumps.
  `CLOCK_REALTIME` can go backwards when NTP corrects it. A control loop that
  uses the wall clock will, one day, try to sleep for a negative time.
- **`time.perf_counter()` versus `time.clock_gettime_ns(CLOCK_MONOTONIC)`.**
  This lab uses the second, because `clock_nanosleep` takes its deadlines on
  exactly that clock. Mixing time bases is a classic source of "impossible"
  timing bugs.
- **`mlockall`.** A page fault inside the control loop is a stall of several
  milliseconds. Real-time processes lock their memory in place before the loop
  starts.
- **`SCHED_FIFO`.** A fixed-priority scheduling class that runs until it
  yields. Powerful enough to hang a machine if your loop never sleeps, which is
  why it needs privileges.

**Next:** Lab 02 — look at the CAN bus and find out whether 500 Hz × 7 motors
even fits (see the bus-load arithmetic in ADR-0002).
