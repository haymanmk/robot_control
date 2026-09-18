#!/usr/bin/env python3
"""Read a telemetry run written by robot_control::telemetry::FileSink.

The .bin file is a flat array of fixed-size records and the .json sidecar
carries both the provenance and the exact numpy dtype, so analysis code never
hand-transcribes the C++ struct:

    import json, numpy as np
    meta = json.load(open("run.json"))
    data = np.fromfile("run.bin", dtype=np.dtype(meta["record_dtype"]))
    data["measured_position"][:, 0]        # joint 0 over the whole run

This script deliberately uses only the standard library, so it works on the
control machine with nothing installed. Usage:

    python3 tools/telemetry_dump.py /tmp/robot_control_demo_telemetry
"""
from __future__ import annotations

import json
import struct
import sys
from pathlib import Path

# Mirrors robot_control::telemetry::TelemetryRecord. The sidecar's record_dtype is the
# authority; this is the stdlib equivalent, and load() checks the two agree.
RECORD = "<Q5q4I48f"
RECORD_SIZE = struct.calcsize(RECORD)
JOINTS = 8
ARRAYS = ("commanded_position", "commanded_velocity", "commanded_torque",
          "measured_position", "measured_velocity", "measured_torque")

FLAGS = [
    (1 << 0, "overrun"), (1 << 1, "missed_deadline"), (1 << 2, "stale_feedback"),
    (1 << 3, "command_dropped"), (1 << 4, "telemetry_lost"), (1 << 5, "client_lost"),
    (1 << 6, "limit_violation"), (1 << 7, "drive_fault"), (1 << 8, "stopping"),
    (1 << 9, "holding"),
]
MODES = {0: "idle", 1: "mit", 2: "pos_vel", 3: "velocity", 4: "stopping", 5: "holding"}


def load(prefix: str):
    metadata_path, binary_path = Path(f"{prefix}.json"), Path(f"{prefix}.bin")
    metadata = json.loads(metadata_path.read_text())

    declared = metadata.get("record_dtype", {}).get("itemsize")
    if declared is not None and declared != RECORD_SIZE:
        raise SystemExit(
            f"record size mismatch: the file says {declared} bytes, this script "
            f"expects {RECORD_SIZE}. The C++ struct changed -- update this script "
            f"and bump current_layout_version."
        )

    raw = binary_path.read_bytes()
    if len(raw) % RECORD_SIZE:
        # A partial tail means the writer was killed mid-record. Report it rather
        # than silently analysing a truncated run.
        print(f"warning: {len(raw) % RECORD_SIZE} trailing bytes -- run was truncated",
              file=sys.stderr)

    records = []
    for chunk in struct.iter_unpack(RECORD, raw[: len(raw) - len(raw) % RECORD_SIZE]):
        record = {
            "cycle": chunk[0], "deadline_nanoseconds": chunk[1], "wake_nanoseconds": chunk[2],
            "execution_nanoseconds": chunk[3], "can_transmit_nanoseconds": chunk[4],
            "can_receive_nanoseconds": chunk[5],
            "joint_count": chunk[6], "flags": chunk[7],
            "fault_mask": chunk[8], "mode": chunk[9],
        }
        floats = chunk[10:]
        for index, name in enumerate(ARRAYS):
            record[name] = list(floats[index * JOINTS:(index + 1) * JOINTS])
        records.append(record)
    return metadata, records


def describe_flags(value: int) -> str:
    names = [name for bit, name in FLAGS if value & bit]
    return ",".join(names) if names else "-"


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    metadata, records = load(sys.argv[1])

    build, machine, run = metadata["build"], metadata["machine"], metadata["run"]
    print(f"{len(records)} records  ({len(records) * RECORD_SIZE / 1e6:.2f} MB)")
    print(f"  build     {build['git_sha']} ({build['git_state']}), {build['build_type']}")
    print(f"  kernel    {machine['kernel_release']}  [{machine['preempt_model']}]"
          f"{'  REALTIME' if machine['realtime_kernel'] else ''}")
    print(f"  cpu       {machine['cpu_model']} x{machine['cpu_count']}"
          + (f", governor={machine['cpu_governor']}" if machine['cpu_governor'] else ""))
    if machine.get("isolated_cpus"):
        print(f"  isolated  {machine['isolated_cpus']}")
    if machine.get("nvidia_driver"):
        state = "inference RUNNING" if machine.get("gpu_workload_running") else "idle"
        print(f"  gpu       {machine['nvidia_driver']} [{state}]")
    print(f"  run       {run['wall_clock']}  {run['label']!r}  @ {run['control_rate_hertz']} Hz")

    if not records:
        return 0

    # Mode transitions are usually what you actually came to look at.
    print("\n  mode transitions")
    previous = None
    for record in records:
        key = (record["mode"], record["flags"])
        if key != previous:
            print(f"    cycle {record['cycle']:8d}  mode={MODES.get(record['mode'], record['mode']):<9}"
                  f" flags={describe_flags(record['flags'])}")
            previous = key

    joints = records[0]["joint_count"]
    print(f"\n  tracking error, |cmd - meas| over {joints} joints")
    for joint in range(joints):
        errors = [abs(record["commanded_position"][joint] - record["measured_position"][joint])
                  for record in records]
        errors.sort()
        p99 = errors[min(len(errors) - 1, int(0.99 * len(errors)))]
        print(f"    joint{joint}  mean {sum(errors)/len(errors):.5f}  p99 {p99:.5f}  max {errors[-1]:.5f} rad")
    return 0


if __name__ == "__main__":
    sys.exit(main())
