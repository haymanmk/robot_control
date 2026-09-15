# loop_timing — Tier 1 determinism benchmark

*(Lab 01 in [the roadmap](../../docs/ROADMAP.md).)*

**Question:** The reBot vendor stack runs its 500 Hz loop with
`time.sleep(dt - elapsed)`. How wrong is that, in microseconds?

**No hardware needed.** Two implementations — pure-stdlib Python and C++ —
so you can also answer: *how much of this is Python's fault?*

---

## Before you run it: predict

Write your answers down first. Getting one wrong is the part that makes the
concept stick.

1. At 500 Hz (2000 µs period) with ~400 µs of work per cycle, will
   `time.sleep(dt - elapsed)` run **fast**, **slow**, or **on average correct**?
2. After 10 seconds — 5000 cycles — how far from the ideal schedule will it be?
   Guess an order of magnitude: 10 µs? 1 ms? 100 ms? 1 s?
3. Does using an **absolute** deadline instead fix the **average** error, the
   **worst case**, or both?

Then:

```bash
python3 loop_timing.py --seconds 10
```

## What it compares

| Strategy | Pattern | Origin |
|---|---|---|
| `relative-sleep` | `sleep(dt - elapsed)` | the vendor's `_control_loop_impl` |
| `absolute-sleep` | deadline = `origin + n*dt`, then `time.sleep(remaining)` | the minimal fix |
| `clock-nanosleep` | `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, deadline)` | how RT loops are actually written |

Useful flags: `--rate`, `--work-us`, `--only <strategy>`, `--json out.json`,
and `--rt` (requests `SCHED_FIFO` + `mlockall`; needs `sudo` or `CAP_SYS_NICE`).

---

## Reference run

Ordinary (non-RT) kernel, 4 s per strategy, 500 Hz, 400 µs of work:

```
strategy               mean       p99     p99.9       max       drift  overruns
-------------------------------------------------------------------------------
relative-sleep        94.2u    163.0u    209.9u    286.4u     188.37m         3
absolute-sleep         0.0u     64.8u    222.9u    241.1u       0.08m         4
clock-nanosleep        0.0u     73.4u   1178.2u   1713.4u       0.08m         4
```

### Reading this

**The headline is the `drift` column.** `relative-sleep` lost **188 ms in 4
seconds** — every cycle came out ~94 µs long, and nothing ever gave that time
back. That is 4.7% slow, permanently. Your "500 Hz" loop is a 477 Hz loop, and
after an hour of running it is **170 seconds behind** where it thinks it is.

Why: `sleep(dt - elapsed)` asks to sleep for a *duration*. The kernel guarantees
*at least* that long, then adds its own wake-up latency on top. The next cycle
starts from wherever you actually woke, so the error compounds. Both absolute
strategies compute deadlines from a fixed origin, so a late wake-up is absorbed
by the next cycle's shorter sleep instead of accumulating. Drift: **0.08 ms** —
2300× better, and it does not grow with runtime.

**Drift and jitter are different failures.** Note that `absolute-sleep` and
`clock-nanosleep` have essentially zero drift but a `max` in the same range as
`relative-sleep`, or worse. Fixing drift did nothing for jitter — that is the
scheduler, and it takes a different tool (`SCHED_FIFO`, `mlockall`, CPU
isolation, PREEMPT_RT) to fix.

**Why `clock-nanosleep` looks worse here.** Its 1.7 ms `max` on this run is not
the syscall's fault — this measurement came from a shared cloud VM with a
non-RT kernel and noisy neighbours. On a tuned PREEMPT_RT box it should be the
*best* of the three. Which is exactly the point ADR-0001 makes about benchmark
posts: **a latency number is meaningless without the machine and the load it was
measured on.** Run this on your own control PC and get your own numbers. Never
inherit someone else's.

**Why percentiles, not averages.** `mean` for both absolute strategies is
0.0 µs. A control loop is not harmed by its average cycle; it is harmed by the
worst one. Always quote p99.9 and max. If a vendor quotes you a mean latency,
they are telling you nothing.

---

## Does this matter for our arm?

Honest answer for the B601-RS specifically: **the jitter, mostly not. The drift,
yes.**

- **Jitter**: ±200 µs on a 2000 µs period is ~10%. Our loop ships *setpoints* to
  drives that close their own current loops (ADR-0002), so a setpoint arriving
  200 µs late is smoothed by the drive. Compare this to a software current loop
  at 20 kHz, where 200 µs is four entire periods and the motor destabilises —
  that is the case Xenomai exists for, and it is not our case.
- **Drift**: it always matters. Any time you integrate — velocity estimated by
  finite-differencing (Lab 06), a trajectory sampled by cycle count (Lab 10),
  a watchdog counting missed heartbeats (Lab 05) — a loop that is silently 4.7%
  slow makes every one of those wrong by 4.7%, and you will debug it as a
  "calibration problem" for a week.

**So the lesson is not "the vendor code is bad."** It is honest, readable code
that gets an arm moving, and that has real value. The lesson is that
*"it works" and "it is deterministic" are different claims*, and you now have a
number that tells them apart.

---

## Things to try

1. `--work-us 2500` — work longer than the period. Watch what each strategy does
   when it cannot keep up. Which one tells you the truth about it?
2. `--rate 50` vs `--rate 1000` — does drift scale with rate, or with runtime?
3. `sudo python3 loop_timing.py --rt` — how much does `SCHED_FIFO` buy on your
   machine? (On a stock kernel: less than you would hope.)
4. Run it while compiling something large in another terminal. That is the
   "under representative load" clause in ADR-0001's action item 3, and it is
   where honest numbers come from.

## The C++ edition, and what it tells you

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

**Predict before you look:** how much faster is C++ at hitting a 2 ms deadline?

Answer: **not at all.** The numbers are the same to within run-to-run noise. On
a non-RT kernel the scheduler dominates, and the scheduler does not care what
language woke it up.

That is not an argument against C++ — it is an argument against the *usual*
argument for it. C++ is worth choosing here for **control**, not speed: no GC
pause, no GIL, no allocation you did not write. Those properties do nothing for
the median cycle and everything for the tail, once the cycle body stops being a
busy-spin and starts being Pinocchio's RNEA plus seven CAN frames. See
[ADR-0003](../../docs/adr/0003-cpp-control-core-and-layering.md).

The C++ version is also a working demo of `core/rt` — its `clock-nanosleep`
strategy *is* `rc::rt::CyclicTask`, the executive every cyclic loop in this
project will run on. Read `core/rt/include/rc/rt/cyclic_task.hpp`: the comments
there are the lesson, the code is the proof.

## Where the fundamentals live

- **Monotonic vs wall clock** — `CLOCK_MONOTONIC` never jumps; `CLOCK_REALTIME`
  can go backwards on an NTP step. A control loop that uses the wall clock will,
  one day, sleep for a negative duration.
- **`time.perf_counter()` vs `time.clock_gettime_ns(CLOCK_MONOTONIC)`** — this
  lab uses the latter because `clock_nanosleep` takes deadlines on that exact
  clock. Mixing time bases is a classic source of "impossible" timing bugs.
- **`mlockall`** — a page fault in your control loop is a multi-millisecond
  stall. RT processes lock their memory resident before the loop starts.
- **`SCHED_FIFO`** — a fixed-priority class that runs until it yields. Powerful
  enough to hang a machine if your loop never sleeps, which is why it is
  privileged.

**Next:** Lab 02 — put a scope on the CAN bus and find out whether 500 Hz × 7
motors even fits (see the bus-budget arithmetic in ADR-0002).
