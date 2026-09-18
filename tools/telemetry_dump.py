#!/usr/bin/env python3
"""Read a telemetry run written by rc::telemetry::FileSink.

The .bin file is a flat array of fixed-size records and the .json sidecar
carries both the provenance and the exact numpy dtype, so analysis code never
hand-transcribes the C++ struct:

    import json, numpy as np
    meta = json.load(open("run.json"))
    data = np.fromfile("run.bin", dtype=np.dtype(meta["record_dtype"]))
    data["meas_pos"][:, 0]        # joint 0 over the whole run

This script deliberately uses only the standard library, so it works on the
control machine with nothing installed. Usage:

    python3 tools/telemetry_dump.py /tmp/rc_demo_telemetry
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

# Mirrors rc::telemetry::TelemetryRecord. The sidecar's record_dtype is the
# authority; this is the stdlib equivalent, and load() checks the two agree.
RECORD = "<Q5q4I48f"
RECORD_SIZE = struct.calcsize(RECORD)
JOINTS = 8
ARRAYS = ("cmd_pos", "cmd_vel", "cmd_tau", "meas_pos", "meas_vel", "meas_tau")

FLAGS = [
    (1 << 0, "overrun"), (1 << 1, "missed_deadline"), (1 << 2, "stale_feedback"),
    (1 << 3, "command_dropped"), (1 << 4, "telemetry_lost"), (1 << 5, "client_lost"),
    (1 << 6, "limit_violation"), (1 << 7, "drive_fault"), (1 << 8, "stopping"),
    (1 << 9, "holding"),
]
MODES = {0: "idle", 1: "mit", 2: "pos_vel", 3: "velocity", 4: "stopping", 5: "holding"}


def load(prefix: str):
    meta_path, bin_path = Path(f"{prefix}.json"), Path(f"{prefix}.bin")
    meta = json.loads(meta_path.read_text())

    declared = meta.get("record_dtype", {}).get("itemsize")
    if declared is not None and declared != RECORD_SIZE:
        raise SystemExit(
            f"record size mismatch: the file says {declared} bytes, this script "
            f"expects {RECORD_SIZE}. The C++ struct changed -- update this script "
            f"and bump current_layout_version."
        )

    raw = bin_path.read_bytes()
    if len(raw) % RECORD_SIZE:
        # A partial tail means the writer was killed mid-record. Report it rather
        # than silently analysing a truncated run.
        print(f"warning: {len(raw) % RECORD_SIZE} trailing bytes -- run was truncated",
              file=sys.stderr)

    records = []
    for chunk in struct.iter_unpack(RECORD, raw[: len(raw) - len(raw) % RECORD_SIZE]):
        r = {
            "cycle": chunk[0], "deadline_ns": chunk[1], "wake_ns": chunk[2],
            "exec_ns": chunk[3], "can_tx_ns": chunk[4], "can_rx_ns": chunk[5],
            "joint_count": chunk[6], "flags": chunk[7],
            "fault_mask": chunk[8], "mode": chunk[9],
        }
        floats = chunk[10:]
        for i, name in enumerate(ARRAYS):
            r[name] = list(floats[i * JOINTS:(i + 1) * JOINTS])
        records.append(r)
    return meta, records


def describe_flags(value: int) -> str:
    names = [name for bit, name in FLAGS if value & bit]
    return ",".join(names) if names else "-"


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    meta, records = load(sys.argv[1])

    b, m, run = meta["build"], meta["machine"], meta["run"]
    print(f"{len(records)} records  ({len(records) * RECORD_SIZE / 1e6:.2f} MB)")
    print(f"  build     {b['git_sha']} ({b['git_state']}), {b['build_type']}")
    print(f"  kernel    {m['kernel_release']}  [{m['preempt_model']}]"
          f"{'  REALTIME' if m['realtime_kernel'] else ''}")
    print(f"  cpu       {m['cpu_model']} x{m['cpu_count']}"
          + (f", governor={m['cpu_governor']}" if m['cpu_governor'] else ""))
    if m.get("isolated_cpus"):
        print(f"  isolated  {m['isolated_cpus']}")
    if m.get("nvidia_driver"):
        state = "inference RUNNING" if m.get("gpu_workload_running") else "idle"
        print(f"  gpu       {m['nvidia_driver']} [{state}]")
    print(f"  run       {run['wall_clock']}  {run['label']!r}  @ {run['control_rate_hz']} Hz")

    if not records:
        return 0

    # Mode transitions are usually what you actually came to look at.
    print("\n  mode transitions")
    previous = None
    for r in records:
        key = (r["mode"], r["flags"])
        if key != previous:
            print(f"    cycle {r['cycle']:8d}  mode={MODES.get(r['mode'], r['mode']):<9}"
                  f" flags={describe_flags(r['flags'])}")
            previous = key

    joints = records[0]["joint_count"]
    print(f"\n  tracking error, |cmd - meas| over {joints} joints")
    for j in range(joints):
        errs = [abs(r["cmd_pos"][j] - r["meas_pos"][j]) for r in records]
        errs.sort()
        p99 = errs[min(len(errs) - 1, int(0.99 * len(errs)))]
        print(f"    joint{j}  mean {sum(errs)/len(errs):.5f}  p99 {p99:.5f}  max {errs[-1]:.5f} rad")
    return 0


if __name__ == "__main__":
    sys.exit(main())
