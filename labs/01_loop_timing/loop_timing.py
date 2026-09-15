#!/usr/bin/env python3
"""Lab 01 — how much does a control loop's timing actually drift?

The reBot vendor stack runs its 500 Hz loop like this
(``reBotArm_control_py/actuator/rebotarm.py``)::

    t0 = time.perf_counter()
    control_fn(...)
    elapsed = time.perf_counter() - t0
    if dt - elapsed > 0:
        time.sleep(dt - elapsed)

That is a *relative* sleep. This lab measures what it costs, against two
alternatives, with no hardware and no third-party packages.

Run::

    python3 loop_timing.py                       # all strategies, 500 Hz, 10 s
    python3 loop_timing.py --rate 500 --work-us 600
    sudo python3 loop_timing.py --rt             # add SCHED_FIFO + mlockall

Vocabulary this lab is built around:

  drift   accumulated error in *when* cycle N happened vs when it should have.
          A relative sleep drifts without bound; an absolute deadline does not.
  jitter  spread of the per-cycle period around nominal. Drift can be zero
          while jitter is awful, and vice versa. They are different failures.
"""
from __future__ import annotations

import argparse
import ctypes
import ctypes.util
import json
import os
import sys
import time
from dataclasses import dataclass, field

NS_PER_S = 1_000_000_000
CLOCK_MONOTONIC = 1
TIMER_ABSTIME = 1


# ---------------------------------------------------------------------------
# libc bindings: clock_nanosleep and mlockall are not exposed by the stdlib
# ---------------------------------------------------------------------------

class _Timespec(ctypes.Structure):
    _fields_ = [("tv_sec", ctypes.c_long), ("tv_nsec", ctypes.c_long)]


_libc = ctypes.CDLL(ctypes.util.find_library("c") or "libc.so.6", use_errno=True)
_libc.clock_nanosleep.argtypes = [
    ctypes.c_int, ctypes.c_int, ctypes.POINTER(_Timespec), ctypes.POINTER(_Timespec)
]
_libc.clock_nanosleep.restype = ctypes.c_int

EINTR = 4
MCL_CURRENT, MCL_FUTURE = 1, 2


def sleep_until_ns(deadline_ns: int) -> None:
    """Block until CLOCK_MONOTONIC reaches deadline_ns.

    Unlike time.sleep(), this takes an *absolute* target, so the kernel's own
    wake-up latency is not added to our period -- it is absorbed. Note that
    clock_nanosleep returns the error number directly rather than setting errno.
    """
    ts = _Timespec(deadline_ns // NS_PER_S, deadline_ns % NS_PER_S)
    while True:
        rc = _libc.clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ctypes.byref(ts), None)
        if rc == 0:
            return
        if rc != EINTR:
            raise OSError(rc, f"clock_nanosleep failed: {os.strerror(rc)}")


def now_ns() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


# ---------------------------------------------------------------------------
# The fake control job: busy-spin for a fixed duration.
#
# Busy-spin rather than sleep, because a real control_fn burns CPU (Pinocchio
# RNEA, CAN frame packing) -- and a loop whose work *sleeps* hides exactly the
# scheduling effects we are trying to see.
# ---------------------------------------------------------------------------

def busy_for_us(duration_us: float) -> None:
    end = now_ns() + int(duration_us * 1000)
    while now_ns() < end:
        pass


# ---------------------------------------------------------------------------
# Loop strategies. Each returns the list of per-cycle wake timestamps (ns).
# ---------------------------------------------------------------------------

def loop_relative_sleep(cycles: int, period_ns: int, work_us: float) -> list[int]:
    """The vendor pattern: sleep(dt - elapsed).

    Every cycle pays the kernel's wake-up latency *on top of* the period, and
    never gets it back. Expect the loop to run slow, and the error to accumulate
    linearly for as long as it runs.
    """
    dt = period_ns / NS_PER_S
    stamps = []
    for _ in range(cycles):
        t0 = time.perf_counter()
        stamps.append(now_ns())
        busy_for_us(work_us)
        remaining = dt - (time.perf_counter() - t0)
        if remaining > 0:
            time.sleep(remaining)
    return stamps


def loop_absolute_sleep(cycles: int, period_ns: int, work_us: float) -> list[int]:
    """Phase-locked, but still using time.sleep().

    Deadlines come from a fixed origin (start + n*period), so drift is
    corrected every cycle instead of accumulating. time.sleep() still rounds up
    to the timer resolution, so jitter remains, but it no longer compounds.
    """
    stamps = []
    origin = now_ns()
    for n in range(cycles):
        stamps.append(now_ns())
        busy_for_us(work_us)
        deadline = origin + (n + 1) * period_ns
        remaining = (deadline - now_ns()) / NS_PER_S
        if remaining > 0:
            time.sleep(remaining)
    return stamps


def loop_clock_nanosleep(cycles: int, period_ns: int, work_us: float) -> list[int]:
    """Phase-locked with an absolute-deadline syscall -- the real-time idiom.

    This is what a PREEMPT_RT cyclic task looks like in any language: compute a
    deadline, clock_nanosleep(TIMER_ABSTIME) to it, repeat. No drift by
    construction, and jitter reduced to the scheduler's own.
    """
    stamps = []
    origin = now_ns()
    for n in range(cycles):
        stamps.append(now_ns())
        busy_for_us(work_us)
        sleep_until_ns(origin + (n + 1) * period_ns)
    return stamps


STRATEGIES = {
    "relative-sleep": loop_relative_sleep,
    "absolute-sleep": loop_absolute_sleep,
    "clock-nanosleep": loop_clock_nanosleep,
}


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

@dataclass
class Result:
    name: str
    period_ns: int
    cycles: int
    errors_us: list[float] = field(default_factory=list)  # period error per cycle
    drift_us: float = 0.0                                 # total accumulated
    overruns: int = 0                                     # cycles late by >10%

    @property
    def stats(self) -> dict:
        e = sorted(self.errors_us)
        n = len(e)
        if n == 0:
            return {}

        def pct(p: float) -> float:
            return e[min(n - 1, int(p * n))]

        return {
            "min_us": e[0],
            "mean_us": sum(e) / n,
            "p50_us": pct(0.50),
            "p99_us": pct(0.99),
            "p999_us": pct(0.999),
            "max_us": e[-1],
            "drift_us": self.drift_us,
            "overruns": self.overruns,
        }


def analyse(name: str, stamps: list[int], period_ns: int) -> Result:
    r = Result(name=name, period_ns=period_ns, cycles=len(stamps))
    overrun_threshold_us = period_ns / 1000 * 0.10
    for i in range(1, len(stamps)):
        err_us = ((stamps[i] - stamps[i - 1]) - period_ns) / 1000.0
        r.errors_us.append(err_us)
        if abs(err_us) > overrun_threshold_us:
            r.overruns += 1
    # Drift is the end-to-end error, not the sum of per-cycle errors: it answers
    # "after N cycles, how far from the ideal schedule are we?"
    ideal_ns = period_ns * (len(stamps) - 1)
    r.drift_us = ((stamps[-1] - stamps[0]) - ideal_ns) / 1000.0
    return r


def histogram(errors_us: list[float], width: int = 52, bins: int = 13) -> str:
    lo, hi = min(errors_us), max(errors_us)
    if hi - lo < 1e-9:
        return f"    all samples at {lo:+.1f} us\n"
    step = (hi - lo) / bins
    counts = [0] * bins
    for e in errors_us:
        counts[min(bins - 1, int((e - lo) / step))] += 1
    peak = max(counts) or 1
    out = []
    for i, c in enumerate(counts):
        edge = lo + i * step
        bar = "#" * int(width * c / peak)
        out.append(f"    {edge:+9.1f} us | {bar:<{width}} {c}")
    return "\n".join(out) + "\n"


# ---------------------------------------------------------------------------
# Real-time privileges (optional)
# ---------------------------------------------------------------------------

def try_realtime(priority: int = 80) -> list[str]:
    """Request SCHED_FIFO and lock memory. Returns human-readable notes.

    Both usually need privileges. Failing here is itself informative -- it is
    the same wall you hit deploying a control process as a normal user.
    """
    notes = []
    try:
        os.sched_setscheduler(0, os.SCHED_FIFO, os.sched_param(priority))
        notes.append(f"SCHED_FIFO priority {priority}: OK")
    except (OSError, PermissionError) as exc:
        notes.append(f"SCHED_FIFO: FAILED ({exc.strerror}) -- run with sudo, or "
                     f"grant CAP_SYS_NICE / raise RLIMIT_RTPRIO in limits.conf")
    if _libc.mlockall(MCL_CURRENT | MCL_FUTURE) == 0:
        notes.append("mlockall(MCL_CURRENT|MCL_FUTURE): OK -- no page faults in the loop")
    else:
        err = ctypes.get_errno()
        notes.append(f"mlockall: FAILED ({os.strerror(err)})")
    return notes


# ---------------------------------------------------------------------------

def main() -> int:
    p = argparse.ArgumentParser(
        description="Measure control-loop timing drift and jitter.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    p.add_argument("--rate", type=float, default=500.0,
                   help="loop rate in Hz (default: 500, the reBot B601-RS nominal)")
    p.add_argument("--seconds", type=float, default=10.0,
                   help="duration per strategy (default: 10)")
    p.add_argument("--work-us", type=float, default=400.0,
                   help="simulated control_fn cost in microseconds (default: 400)")
    p.add_argument("--rt", action="store_true",
                   help="request SCHED_FIFO and mlockall before measuring")
    p.add_argument("--only", choices=sorted(STRATEGIES), action="append",
                   help="run only these strategies (repeatable)")
    p.add_argument("--json", metavar="PATH", help="also write results as JSON")
    args = p.parse_args()

    period_ns = int(NS_PER_S / args.rate)
    cycles = int(args.seconds * args.rate)
    if args.work_us > period_ns / 1000:
        print(f"note: work ({args.work_us:.0f} us) exceeds the period "
              f"({period_ns/1000:.0f} us) -- the loop cannot keep up by design.\n")

    print(f"Lab 01 -- control loop timing")
    print(f"  rate      {args.rate:g} Hz  (period {period_ns/1000:.1f} us)")
    print(f"  cycles    {cycles} per strategy ({args.seconds:g} s)")
    print(f"  work      {args.work_us:g} us busy-spin per cycle")
    print(f"  kernel    {os.uname().release}")
    if args.rt:
        for note in try_realtime():
            print(f"  rt        {note}")
    print()

    chosen = args.only or list(STRATEGIES)
    results = []
    for name in chosen:
        print(f"running {name} ...", end=" ", flush=True)
        stamps = STRATEGIES[name](cycles, period_ns, args.work_us)
        r = analyse(name, stamps, period_ns)
        results.append(r)
        print("done")
    print()

    header = f"{'strategy':<17}{'mean':>10}{'p99':>10}{'p99.9':>10}{'max':>10}{'drift':>12}{'overruns':>10}"
    print(header)
    print("-" * len(header))
    for r in results:
        s = r.stats
        print(f"{r.name:<17}{s['mean_us']:>9.1f}u{s['p99_us']:>9.1f}u"
              f"{s['p999_us']:>9.1f}u{s['max_us']:>9.1f}u"
              f"{s['drift_us']/1000:>11.2f}m{s['overruns']:>10}")
    print("\n  mean/p99/max are per-cycle period error in microseconds (us).")
    print("  drift is total schedule error in milliseconds (ms) after the whole run.")
    print(f"  overruns are cycles off nominal by more than 10% ({period_ns/10000:.0f} us).\n")

    for r in results:
        print(f"  {r.name} -- per-cycle period error distribution")
        print(histogram(r.errors_us))

    if args.json:
        payload = {
            "rate_hz": args.rate, "period_ns": period_ns, "cycles": cycles,
            "work_us": args.work_us, "realtime_requested": args.rt,
            "kernel": os.uname().release,
            "results": {r.name: r.stats for r in results},
        }
        with open(args.json, "w") as f:
            json.dump(payload, f, indent=2)
        print(f"  wrote {args.json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
