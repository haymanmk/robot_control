# bench — performance regression suite

Not test scaffolding. These are the measurements that let us say how good the
system is, stored as JSON and compared against a baseline so that "I measured it
once" becomes "it is still true." See [ADR-0004](../docs/adr/0004-system-decomposition.md)
for the four-tier measurement taxonomy.

| Benchmark | Tier | Measures | Hardware |
|---|---|---|---|
| [loop_timing](loop_timing/) | 1 — determinism | wake latency, period error, drift, overruns | none |

```bash
cmake -B build && cmake --build build -j
./build/bench/loop_timing/bench_loop_timing --seconds 10
```

Every result must carry its provenance — git SHA, kernel and whether it is
`PREEMPT_RT`, CPU model and governor, RT priority actually granted, CAN bitrate.
A number without its machine is not a measurement.
