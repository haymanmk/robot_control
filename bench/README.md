# bench — performance benchmarks

These are not test scaffolding. They are the measurements that let us say how
good the system is. Results are stored as JSON and compared with a baseline,
so that "I measured it once" becomes "it is still true." The four tiers of
measurement are described in [ADR-0004](../docs/adr/0004-system-decomposition.md).

| Benchmark | Tier | Measures | Hardware |
|---|---|---|---|
| [loop_timing](loop_timing/) | 1 — determinism | wake latency, period error, drift, overruns | none |

```bash
cmake -B build && cmake --build build -j
./build/bench/loop_timing/bench_loop_timing --seconds 10
```

Every result must record where it came from: git commit, kernel and whether it
is `PREEMPT_RT`, CPU model and governor, the real-time priority actually
granted, and the CAN bit rate. A number without its machine is not a
measurement.
