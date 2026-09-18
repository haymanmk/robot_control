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

NANOSECONDS_PER_SECOND = 1_000_000_000
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


def sleep_until_nanoseconds(deadline_nanoseconds: int) -> None:
    """Block until CLOCK_MONOTONIC reaches deadline_nanoseconds.

    Unlike time.sleep(), this takes an *absolute* target, so the kernel's own
    wake-up latency is not added to our period -- it is absorbed. Note that
    clock_nanosleep returns the error number directly rather than setting errno.
    """
    spec = _Timespec(deadline_nanoseconds // NANOSECONDS_PER_SECOND,
                     deadline_nanoseconds % NANOSECONDS_PER_SECOND)
    while True:
        result = _libc.clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, ctypes.byref(spec), None)
        if result == 0:
            return
        if result != EINTR:
            raise OSError(result, f"clock_nanosleep failed: {os.strerror(result)}")


def now_nanoseconds() -> int:
    return time.clock_gettime_ns(time.CLOCK_MONOTONIC)


# ---------------------------------------------------------------------------
# The fake control job: busy-spin for a fixed duration.
#
# Busy-spin rather than sleep, because a real control_fn burns CPU (Pinocchio
# RNEA, CAN frame packing) -- and a loop whose work *sleeps* hides exactly the
# scheduling effects we are trying to see.
# ---------------------------------------------------------------------------

def busy_for_microseconds(duration_microseconds: float) -> None:
    end = now_nanoseconds() + int(duration_microseconds * 1000)
    while now_nanoseconds() < end:
        pass


# ---------------------------------------------------------------------------
# Loop strategies. Each returns the list of per-cycle wake timestamps (ns).
# ---------------------------------------------------------------------------

def loop_relative_sleep(cycles: int, period_nanoseconds: int, work_microseconds: float) -> list[int]:
    """The vendor pattern: sleep(dt - elapsed).

    Every cycle pays the kernel's wake-up latency *on top of* the period, and
    never gets it back. Expect the loop to run slow, and the error to accumulate
    linearly for as long as it runs.
    """
    period_seconds = period_nanoseconds / NANOSECONDS_PER_SECOND
    stamps = []
    for _ in range(cycles):
        start = time.perf_counter()
        stamps.append(now_nanoseconds())
        busy_for_microseconds(work_microseconds)
        remaining = period_seconds - (time.perf_counter() - start)
        if remaining > 0:
            time.sleep(remaining)
    return stamps


def loop_absolute_sleep(cycles: int, period_nanoseconds: int, work_microseconds: float) -> list[int]:
    """Phase-locked, but still using time.sleep().

    Deadlines come from a fixed origin (start + n*period), so drift is
    corrected every cycle instead of accumulating. time.sleep() still rounds up
    to the timer resolution, so jitter remains, but it no longer compounds.
    """
    stamps = []
    origin = now_nanoseconds()
    for cycle in range(cycles):
        stamps.append(now_nanoseconds())
        busy_for_microseconds(work_microseconds)
        deadline = origin + (cycle + 1) * period_nanoseconds
        remaining = (deadline - now_nanoseconds()) / NANOSECONDS_PER_SECOND
        if remaining > 0:
            time.sleep(remaining)
    return stamps


def loop_clock_nanosleep(cycles: int, period_nanoseconds: int, work_microseconds: float) -> list[int]:
    """Phase-locked with an absolute-deadline syscall -- the real-time idiom.

    This is what a PREEMPT_RT cyclic task looks like in any language: compute a
    deadline, clock_nanosleep(TIMER_ABSTIME) to it, repeat. No drift by
    construction, and jitter reduced to the scheduler's own.
    """
    stamps = []
    origin = now_nanoseconds()
    for cycle in range(cycles):
        stamps.append(now_nanoseconds())
        busy_for_microseconds(work_microseconds)
        sleep_until_nanoseconds(origin + (cycle + 1) * period_nanoseconds)
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
    period_nanoseconds: int
    cycles: int
    errors_microseconds: list[float] = field(default_factory=list)  # period error per cycle
    drift_microseconds: float = 0.0                                 # total accumulated
    overruns: int = 0                                     # cycles late by >10%

    @property
    def stats(self) -> dict:
        errors = sorted(self.errors_microseconds)
        count = len(errors)
        if count == 0:
            return {}

        def percentile(fraction: float) -> float:
            return errors[min(count - 1, int(fraction * count))]

        return {
            "minimum_microseconds": errors[0],
            "mean_microseconds": sum(errors) / count,
            "p50_microseconds": percentile(0.50),
            "p99_microseconds": percentile(0.99),
            "p999_microseconds": percentile(0.999),
            "maximum_microseconds": errors[-1],
            "drift_microseconds": self.drift_microseconds,
            "overruns": self.overruns,
        }


def analyse(name: str, stamps: list[int], period_nanoseconds: int) -> Result:
    result = Result(name=name, period_nanoseconds=period_nanoseconds, cycles=len(stamps))
    overrun_threshold_microseconds = period_nanoseconds / 1000 * 0.10
    for index in range(1, len(stamps)):
        error_microseconds = ((stamps[index] - stamps[index - 1]) - period_nanoseconds) / 1000.0
        result.errors_microseconds.append(error_microseconds)
        if abs(error_microseconds) > overrun_threshold_microseconds:
            result.overruns += 1
    # Drift is the end-to-end error, not the sum of per-cycle errors: it answers
    # "after N cycles, how far from the ideal schedule are we?"
    ideal_nanoseconds = period_nanoseconds * (len(stamps) - 1)
    result.drift_microseconds = ((stamps[-1] - stamps[0]) - ideal_nanoseconds) / 1000.0
    return result


def histogram(errors_microseconds: list[float], width: int = 52, bins: int = 13) -> str:
    low, high = min(errors_microseconds), max(errors_microseconds)
    if high - low < 1e-9:
        return f"    all samples at {low:+.1f} us\n"
    step = (high - low) / bins
    counts = [0] * bins
    for error in errors_microseconds:
        counts[min(bins - 1, int((error - low) / step))] += 1
    peak = max(counts) or 1
    out = []
    for index, count in enumerate(counts):
        edge = low + index * step
        bar = "#" * int(width * count / peak)
        out.append(f"    {edge:+9.1f} us | {bar:<{width}} {count}")
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
    except (OSError, PermissionError) as error:
        notes.append(f"SCHED_FIFO: FAILED ({error.strerror}) -- run with sudo, or "
                     f"grant CAP_SYS_NICE / raise RLIMIT_RTPRIO in limits.conf")
    if _libc.mlockall(MCL_CURRENT | MCL_FUTURE) == 0:
        notes.append("mlockall(MCL_CURRENT|MCL_FUTURE): OK -- no page faults in the loop")
    else:
        error_number = ctypes.get_errno()
        notes.append(f"mlockall: FAILED ({os.strerror(error_number)})")
    return notes


# ---------------------------------------------------------------------------

def main() -> int:
    parser = argparse.ArgumentParser(
        description="Measure control-loop timing drift and jitter.",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__,
    )
    parser.add_argument("--rate", type=float, default=500.0,
                   help="loop rate in Hz (default: 500, the reBot B601-RS nominal)")
    parser.add_argument("--seconds", type=float, default=10.0,
                   help="duration per strategy (default: 10)")
    parser.add_argument("--work-us", dest="work_microseconds", type=float, default=400.0,
                   help="simulated control_fn cost in microseconds (default: 400)")
    parser.add_argument("--rt", action="store_true",
                   help="request SCHED_FIFO and mlockall before measuring")
    parser.add_argument("--only", choices=sorted(STRATEGIES), action="append",
                   help="run only these strategies (repeatable)")
    parser.add_argument("--json", metavar="PATH", help="also write results as JSON")
    args = parser.parse_args()

    period_nanoseconds = int(NANOSECONDS_PER_SECOND / args.rate)
    cycles = int(args.seconds * args.rate)
    if args.work_microseconds > period_nanoseconds / 1000:
        print(f"note: work ({args.work_microseconds:.0f} us) exceeds the period "
              f"({period_nanoseconds/1000:.0f} us) -- the loop cannot keep up by design.\n")

    print(f"Lab 01 -- control loop timing")
    print(f"  rate      {args.rate:g} Hz  (period {period_nanoseconds/1000:.1f} us)")
    print(f"  cycles    {cycles} per strategy ({args.seconds:g} s)")
    print(f"  work      {args.work_microseconds:g} us busy-spin per cycle")
    print(f"  kernel    {os.uname().release}")
    if args.rt:
        for note in try_realtime():
            print(f"  rt        {note}")
    print()

    chosen = args.only or list(STRATEGIES)
    results = []
    for name in chosen:
        print(f"running {name} ...", end=" ", flush=True)
        stamps = STRATEGIES[name](cycles, period_nanoseconds, args.work_microseconds)
        result = analyse(name, stamps, period_nanoseconds)
        results.append(result)
        print("done")
    print()

    header = f"{'strategy':<17}{'mean':>10}{'p99':>10}{'p99.9':>10}{'max':>10}{'drift':>12}{'overruns':>10}"
    print(header)
    print("-" * len(header))
    for result in results:
        stats = result.stats
        print(f"{result.name:<17}{stats['mean_microseconds']:>9.1f}u"
              f"{stats['p99_microseconds']:>9.1f}u"
              f"{stats['p999_microseconds']:>9.1f}u{stats['maximum_microseconds']:>9.1f}u"
              f"{stats['drift_microseconds']/1000:>11.2f}m{stats['overruns']:>10}")
    print("\n  mean/p99/max are per-cycle period error in microseconds (us).")
    print("  drift is total schedule error in milliseconds (ms) after the whole run.")
    print(f"  overruns are cycles off nominal by more than 10% ({period_nanoseconds/10000:.0f} us).\n")

    for result in results:
        print(f"  {result.name} -- per-cycle period error distribution")
        print(histogram(result.errors_microseconds))

    if args.json:
        payload = {
            "rate_hertz": args.rate, "period_nanoseconds": period_nanoseconds, "cycles": cycles,
            "work_microseconds": args.work_microseconds, "realtime_requested": args.rt,
            "kernel": os.uname().release,
            "results": {result.name: result.stats for result in results},
        }
        with open(args.json, "w") as file:
            json.dump(payload, file, indent=2)
        print(f"  wrote {args.json}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
