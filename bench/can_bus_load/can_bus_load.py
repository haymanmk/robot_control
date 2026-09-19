#!/usr/bin/env python3
"""Lab 02 -- what is actually on the CAN bus?

Turns a ``candump`` log (or one from ``bench_can_capture``) into the numbers
ADR-0002 guessed at: frames per cycle, which identifiers, bytes per frame,
the spacing between a command frame and its feedback per joint, the loop rate
as seen on the wire, and bus utilisation as a range. It can also *synthesize*
a log for the case ADR-0002 assumed -- seven motors, one command and one
feedback each, 500 Hz, 1 Mbit/s -- so the prediction step has something
concrete to look at before the arm is plugged in.

Standard library only, so it runs on the control machine as it is.

    python3 can_bus_load.py analyze lab02.log --bitrate 1000000
    python3 can_bus_load.py analyze lab02.log --bitrate 1000000 \\
        --counters-before before.txt --counters-after after.txt --json lab02.json
    python3 can_bus_load.py synthesize predicted.log --rate 500 --joints 7

Bit-time arithmetic (mirrors robot_control::can::bits_on_wire in
core/can/include/robot_control/can/frame.hpp; the C++ test pins the numbers):

    standard, 8 bytes:  111 bits with no stuffing .. 135 worst case
    extended, 8 bytes:  131 .. 160

both including the 3-bit interframe space.
"""
from __future__ import annotations

import argparse
import json
import random
import re
import statistics
import sys
from collections import Counter, defaultdict
from dataclasses import asdict, dataclass, field
from pathlib import Path

NANOSECONDS_PER_SECOND = 1_000_000_000
ERROR_FLAG = 0x2000_0000
EXTENDED_MASK = 0x1FFF_FFFF
STANDARD_MASK = 0x7FF
INTERFRAME_SPACE_BITS = 3


# ---------------------------------------------------------------------------
# Frames and the bit-time arithmetic
# ---------------------------------------------------------------------------

@dataclass
class Frame:
    timestamp_nanoseconds: int
    interface: str
    identifier: int
    extended: bool
    remote: bool = False
    error: bool = False
    fd: bool = False
    data: bytes = b""

    @property
    def length(self) -> int:
        return len(self.data)


def frame_bits(length: int, extended: bool, remote: bool = False, error: bool = False) -> tuple[int, int]:
    """(minimum, maximum) bit times a classic frame occupies, interframe space included."""
    if error:
        return 0, 0
    data_bits = 0 if remote else 8 * length
    fixed_bits = 64 if extended else 44
    stuffed_region = (54 if extended else 34) + data_bits
    minimum = fixed_bits + data_bits + INTERFRAME_SPACE_BITS
    return minimum, minimum + (stuffed_region - 1) // 4


# ---------------------------------------------------------------------------
# Parsing candump output
# ---------------------------------------------------------------------------

# "(1700000000.123456) can0 1FD01#0011223344556677"   candump -l / bench_can_capture
LOG_LINE = re.compile(r"^\s*\((\d+)\.(\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)#(\S*)")
# " (1700000000.123456)  can0  1FD01   [8]  00 11 22 33 44 55 66 77"   candump -ta
TABLE_LINE = re.compile(r"^\s*\((\d+)\.(\d+)\)\s+(\S+)\s+([0-9A-Fa-f]+)\s+\[(\d+)\]\s*(.*)$")


def _timestamp(seconds: str, fraction: str) -> int:
    return int(seconds) * NANOSECONDS_PER_SECOND + int(fraction.ljust(9, "0")[:9])


def parse_line(line: str) -> Frame | None:
    match = LOG_LINE.match(line)
    if match:
        seconds, fraction, interface, identifier_text, payload = match.groups()
        raw_identifier = int(identifier_text, 16)
        # "#R" is a remote frame; "##<flags nibble><data>" is CAN FD; else data.
        fd = payload.startswith("#")
        remote = payload.startswith("R")
        hex_text = "" if remote else payload[2:] if fd else payload
        hex_text = "".join(character for character in hex_text if character in "0123456789ABCDEFabcdef")
        data = bytes.fromhex(hex_text[: len(hex_text) // 2 * 2])
    else:
        match = TABLE_LINE.match(line)
        if not match:
            return None
        seconds, fraction, interface, identifier_text, length_text, rest = match.groups()
        raw_identifier = int(identifier_text, 16)
        remote = "remote request" in rest
        fd = len(length_text) == 2   # candump prints FD lengths as two digits, classic as one
        hex_bytes = [token for token in rest.split() if len(token) == 2 and all(c in "0123456789ABCDEFabcdef" for c in token)]
        data = b"" if remote else bytes(int(token, 16) for token in hex_bytes[: int(length_text)])
    error = bool(raw_identifier & ERROR_FLAG)
    # candump prints 3 hex digits for an 11-bit id and 8 for a 29-bit one.
    extended = len(identifier_text) > 3 and not error
    identifier = raw_identifier & (EXTENDED_MASK if extended else STANDARD_MASK)
    return Frame(_timestamp(seconds, fraction), interface, identifier, extended, remote, error, fd, data)


def parse_log(path: Path) -> list[Frame]:
    frames: list[Frame] = []
    with path.open(encoding="utf-8", errors="replace") as handle:
        for line in handle:
            frame = parse_line(line)
            if frame is not None:
                frames.append(frame)
    return frames


# `ip -details -statistics link show can0`, saved before and after the run.
def parse_ip_link(path: Path) -> dict:
    text = path.read_text(encoding="utf-8", errors="replace")
    result: dict = {}
    bitrate = re.search(r"\bbitrate (\d+)", text)
    if bitrate:
        result["bitrate"] = int(bitrate.group(1))
    if "dbitrate" in text or re.search(r"<[^>]*\bFD\b[^>]*>", text):
        result["fd_capable"] = True
    for direction in ("RX", "TX"):
        header = re.search(rf"{direction}:\s+([a-z ]+)\n\s*([\d ]+)", text)
        if header:
            names = header.group(1).split()
            values = [int(value) for value in header.group(2).split()]
            result[direction.lower()] = dict(zip(names, values))
    return result


# ---------------------------------------------------------------------------
# Protocol: which frame is a command to which joint, and which is feedback
# ---------------------------------------------------------------------------

@dataclass
class Classified:
    kind: str          # "command", "feedback", "other"
    joint: int | None  # motor CAN id, 1..7 on the B601-RS


def classify_robstride(frame: Frame, host_id: int | None) -> Classified:
    """RobStride / CyberGear framing: a 29-bit id whose top 5 bits are the
    message type, bits 23..8 a data field, bits 7..0 the destination. Type 1
    is the MIT command (destination = motor); type 2 is the motor's feedback
    (bits 15..8 = motor, bits 7..0 = host). Types 3, 4, 6 are enable, disable
    and set-zero, sent to the motor like a command. Types 17 and 18 are
    parameter read and write, whose replies come back with the same type and
    the host id in the low byte."""
    if not frame.extended or frame.error:
        return Classified("other", None)
    message_type = (frame.identifier >> 24) & 0x1F
    low = frame.identifier & 0xFF
    middle = (frame.identifier >> 8) & 0xFF
    if message_type == 2:
        return Classified("feedback", middle)
    if message_type in (1, 3, 4, 6):
        return Classified("command", low)
    if message_type in (17, 18):
        if host_id is not None and low == host_id:
            return Classified("feedback", middle)
        return Classified("command", low)
    return Classified("other", None)


def classify_none(frame: Frame, host_id: int | None) -> Classified:
    return Classified("other", None)


PROTOCOLS = {"robstride": classify_robstride, "none": classify_none}


def infer_host_id(frames: list[Frame]) -> int | None:
    """The host id is the low byte every type-2 (feedback) frame is addressed to."""
    low_bytes = Counter(frame.identifier & 0xFF for frame in frames
                        if frame.extended and not frame.error and (frame.identifier >> 24) & 0x1F == 2)
    return low_bytes.most_common(1)[0][0] if low_bytes else None


# ---------------------------------------------------------------------------
# Analysis
# ---------------------------------------------------------------------------

def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return 0.0
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round(fraction * (len(ordered) - 1))))
    return ordered[index]


def summarize_microseconds(values_nanoseconds: list[int]) -> dict:
    microseconds = [value / 1000.0 for value in values_nanoseconds]
    if not microseconds:
        return {"count": 0}
    return {
        "count": len(microseconds),
        "minimum_microseconds": min(microseconds),
        "p50_microseconds": percentile(microseconds, 0.50),
        "p99_microseconds": percentile(microseconds, 0.99),
        "maximum_microseconds": max(microseconds),
    }


@dataclass
class Analysis:
    log: str
    frames: int = 0
    span_seconds: float = 0.0
    frames_per_second: float = 0.0
    bytes_per_second: float = 0.0
    standard_frames: int = 0
    extended_frames: int = 0
    fd_frames: int = 0
    remote_frames: int = 0
    error_frames: int = 0
    bitrate: int = 0
    minimum_bits: int = 0
    maximum_bits: int = 0
    utilisation_minimum: float = 0.0
    utilisation_maximum: float = 0.0
    host_id: int | None = None
    identifiers: list[dict] = field(default_factory=list)
    cycles: int = 0
    measured_rate_hertz: float = 0.0
    frames_per_cycle: float = 0.0
    cycle_period: dict = field(default_factory=dict)
    cycle_span: dict = field(default_factory=dict)
    joints: dict = field(default_factory=dict)
    counters: dict = field(default_factory=dict)
    verdict: dict = field(default_factory=dict)


def analyze(frames: list[Frame], bitrate: int, nominal_rate: float, protocol: str, host_id: int | None,
            counters_before: dict | None = None, counters_after: dict | None = None, log_name: str = "") -> Analysis:
    result = Analysis(log=log_name, bitrate=bitrate)
    if not frames:
        return result
    frames = sorted(frames, key=lambda frame: frame.timestamp_nanoseconds)
    span = frames[-1].timestamp_nanoseconds - frames[0].timestamp_nanoseconds
    result.frames = len(frames)
    result.span_seconds = span / NANOSECONDS_PER_SECOND
    total_bytes = sum(frame.length for frame in frames)
    if span > 0:
        result.frames_per_second = len(frames) / result.span_seconds
        result.bytes_per_second = total_bytes / result.span_seconds

    for frame in frames:
        if frame.error:
            result.error_frames += 1
            continue
        if frame.fd:
            result.fd_frames += 1
        if frame.remote:
            result.remote_frames += 1
        if frame.extended:
            result.extended_frames += 1
        else:
            result.standard_frames += 1
        minimum, maximum = frame_bits(frame.length, frame.extended, frame.remote)
        result.minimum_bits += minimum
        result.maximum_bits += maximum
    if span > 0 and bitrate > 0:
        capacity = bitrate * result.span_seconds
        result.utilisation_minimum = result.minimum_bits / capacity
        result.utilisation_maximum = result.maximum_bits / capacity

    classify = PROTOCOLS[protocol]
    if host_id is None and protocol == "robstride":
        host_id = infer_host_id(frames)
    result.host_id = host_id
    classified = [classify(frame, host_id) for frame in frames]

    # Per identifier: what it is, how often, how big.
    by_identifier: dict[tuple[int, bool], list[int]] = defaultdict(list)
    lengths: dict[tuple[int, bool], list[int]] = defaultdict(list)
    kinds: dict[tuple[int, bool], Classified] = {}
    for frame, tag in zip(frames, classified):
        if frame.error:
            continue
        key = (frame.identifier, frame.extended)
        by_identifier[key].append(frame.timestamp_nanoseconds)
        lengths[key].append(frame.length)
        kinds[key] = tag
    for key, stamps in sorted(by_identifier.items(), key=lambda item: -len(item[1])):
        intervals = [b - a for a, b in zip(stamps, stamps[1:])]
        result.identifiers.append({
            "identifier": f"{key[0]:08X}" if key[1] else f"{key[0]:03X}",
            "extended": key[1],
            "kind": kinds[key].kind,
            "joint": kinds[key].joint,
            "count": len(stamps),
            "mean_interval_microseconds": statistics.fmean(intervals) / 1000.0 if intervals else 0.0,
            "mean_length": statistics.fmean(lengths[key]),
        })

    # Cycles: each command to the lowest-numbered joint starts one.
    command_joints = sorted({tag.joint for tag in classified if tag.kind == "command" and tag.joint is not None})
    if command_joints:
        marker = command_joints[0]
        marker_stamps = [frame.timestamp_nanoseconds for frame, tag in zip(frames, classified)
                         if tag.kind == "command" and tag.joint == marker]
        result.cycles = len(marker_stamps)
        if len(marker_stamps) > 1:
            marker_span = marker_stamps[-1] - marker_stamps[0]
            result.measured_rate_hertz = (len(marker_stamps) - 1) / (marker_span / NANOSECONDS_PER_SECOND)
            result.frames_per_cycle = (len(frames) - result.error_frames) / len(marker_stamps)
            result.cycle_period = summarize_microseconds([b - a for a, b in zip(marker_stamps, marker_stamps[1:])])

        # Within a cycle: first command out to last feedback back. That is the
        # joint-to-joint skew ADR-0002 says must be measured, not assumed away.
        cycle_index = -1
        first_command: list[int] = []
        last_feedback: list[int] = []
        for frame, tag in zip(frames, classified):
            if tag.kind == "command" and tag.joint == marker:
                cycle_index += 1
                first_command.append(frame.timestamp_nanoseconds)
                last_feedback.append(frame.timestamp_nanoseconds)
            elif cycle_index >= 0 and tag.kind in ("command", "feedback"):
                last_feedback[cycle_index] = frame.timestamp_nanoseconds
        result.cycle_span = summarize_microseconds([b - a for a, b in zip(first_command, last_feedback)])

    # Per joint: commands, feedbacks, and command-to-feedback spacing.
    pending: dict[int, int] = {}
    delays: dict[int, list[int]] = defaultdict(list)
    counts: dict[int, Counter] = defaultdict(Counter)
    for frame, tag in zip(frames, classified):
        if tag.joint is None:
            continue
        counts[tag.joint][tag.kind] += 1
        if tag.kind == "command":
            pending[tag.joint] = frame.timestamp_nanoseconds   # a newer command supersedes an unanswered one
        elif tag.kind == "feedback" and tag.joint in pending:
            delays[tag.joint].append(frame.timestamp_nanoseconds - pending.pop(tag.joint))
    for joint in sorted(counts):
        commands = counts[joint]["command"]
        feedbacks = counts[joint]["feedback"]
        result.joints[str(joint)] = {
            "commands": commands,
            "feedbacks": feedbacks,
            "feedbacks_per_command": feedbacks / commands if commands else 0.0,
            "command_to_feedback": summarize_microseconds(delays[joint]),
        }

    if counters_before and counters_after and "rx" in counters_before and "rx" in counters_after:
        received = {name: counters_after["rx"][name] - counters_before["rx"].get(name, 0)
                    for name in counters_after["rx"]}
        transmitted = {name: counters_after["tx"][name] - counters_before["tx"].get(name, 0)
                       for name in counters_after.get("tx", {})}
        result.counters = {"received": received, "transmitted": transmitted,
                           "bitrate": counters_after.get("bitrate"),
                           "fd_capable": counters_after.get("fd_capable", False)}
        if counters_after.get("bitrate") and counters_after["bitrate"] != bitrate:
            result.counters["warning"] = (f"--bitrate {bitrate} disagrees with the interface's "
                                          f"{counters_after['bitrate']}; utilisation uses --bitrate")

    # The three possibilities ADR-0002 listed. Evidence, not opinion.
    feedback_ratios = [entry["feedbacks_per_command"] for entry in result.joints.values() if entry["commands"]]
    result.verdict = {
        "fits_at_worst_case_stuffing": result.utilisation_maximum <= 1.0,
        "hypothesis_faster_bus": {
            "fd_frames_seen": result.fd_frames,
            "interface_bitrate": (result.counters or {}).get("bitrate"),
            "true_if": "FD frames appear or the interface bit rate exceeds 1 Mbit/s",
        },
        "hypothesis_feedback_not_every_cycle": {
            "feedbacks_per_command": statistics.fmean(feedback_ratios) if feedback_ratios else None,
            "true_if": "feedbacks per command is well below 1",
        },
        "hypothesis_rate_below_nominal": {
            "nominal_rate_hertz": nominal_rate,
            "measured_rate_hertz": result.measured_rate_hertz,
            "true_if": f"the measured rate is clearly below {nominal_rate:g} Hz",
        },
    }
    return result


# ---------------------------------------------------------------------------
# Report
# ---------------------------------------------------------------------------

def _row(label: str, summary: dict) -> str:
    if summary.get("count", 0) == 0:
        return f"  {label:<28} (none)"
    return (f"  {label:<28} min {summary['minimum_microseconds']:8.1f}  p50 {summary['p50_microseconds']:8.1f}"
            f"  p99 {summary['p99_microseconds']:8.1f}  max {summary['maximum_microseconds']:8.1f}  us"
            f"  (n={summary['count']})")


def report(analysis: Analysis) -> str:
    lines = [f"Lab 02 -- CAN bus load from {analysis.log}", ""]
    if analysis.frames == 0:
        lines.append("  no frames parsed: is this a candump log?")
        return "\n".join(lines)
    lines += [
        f"  frames {analysis.frames} over {analysis.span_seconds:.3f} s = {analysis.frames_per_second:.1f} frames/s,"
        f" {analysis.bytes_per_second:.0f} bytes/s",
        f"  standard {analysis.standard_frames}  extended {analysis.extended_frames}  fd {analysis.fd_frames}"
        f"  remote {analysis.remote_frames}  error {analysis.error_frames}",
        f"  bus utilisation at {analysis.bitrate} bit/s: {analysis.utilisation_minimum * 100:.1f}% .."
        f" {analysis.utilisation_maximum * 100:.1f}%  (no stuffing .. worst-case stuffing)",
        "",
        "  identifier  kind      joint  count   interval(us)  bytes",
    ]
    for entry in analysis.identifiers[:24]:
        joint = "-" if entry["joint"] is None else str(entry["joint"])
        lines.append(f"  {entry['identifier']:<10}  {entry['kind']:<8}  {joint:>5}  {entry['count']:>6}"
                     f"  {entry['mean_interval_microseconds']:>12.1f}  {entry['mean_length']:>5.1f}")
    if len(analysis.identifiers) > 24:
        lines.append(f"  ... {len(analysis.identifiers) - 24} more identifiers")
    if analysis.cycles:
        lines += ["",
                  f"  cycles {analysis.cycles}: measured rate {analysis.measured_rate_hertz:.2f} Hz,"
                  f" {analysis.frames_per_cycle:.2f} frames per cycle",
                  _row("cycle period", analysis.cycle_period),
                  _row("first command -> last frame", analysis.cycle_span),
                  ""]
        for joint, entry in analysis.joints.items():
            lines.append(f"  joint {joint}: {entry['commands']} commands, {entry['feedbacks']} feedbacks"
                         f" ({entry['feedbacks_per_command']:.2f} per command)")
            lines.append(_row("    command -> feedback", entry["command_to_feedback"]))
    else:
        lines += ["", "  no command frames recognised; use --protocol robstride on a RobStride bus"]
    if analysis.counters:
        received = analysis.counters["received"]
        transmitted = analysis.counters["transmitted"]
        lines += ["", "  driver counters over the run:",
                  f"    rx {received.get('packets', 0)} frames, {received.get('bytes', 0)} bytes,"
                  f" {received.get('errors', 0)} errors, {received.get('dropped', 0)} dropped",
                  f"    tx {transmitted.get('packets', 0)} frames, {transmitted.get('bytes', 0)} bytes,"
                  f" {transmitted.get('errors', 0)} errors, {transmitted.get('dropped', 0)} dropped"]
        if analysis.counters.get("bitrate"):
            lines.append(f"    interface bit rate {analysis.counters['bitrate']}"
                         f"{' (CAN FD capable)' if analysis.counters.get('fd_capable') else ''}")
        if "warning" in analysis.counters:
            lines.append(f"    warning: {analysis.counters['warning']}")
    verdict = analysis.verdict
    lines += ["", "  ADR-0002's three possibilities:",
              f"    fits at worst-case stuffing?        {'yes' if verdict['fits_at_worst_case_stuffing'] else 'NO'}",
              f"    faster bus (CAN FD)?                fd frames {verdict['hypothesis_faster_bus']['fd_frames_seen']},"
              f" interface bit rate {verdict['hypothesis_faster_bus']['interface_bitrate']}",
              f"    feedback not every cycle?           feedbacks per command"
              f" {verdict['hypothesis_feedback_not_every_cycle']['feedbacks_per_command']}",
              f"    rate below nominal?                 measured {verdict['hypothesis_rate_below_nominal']['measured_rate_hertz']:.2f} Hz"
              f" vs {verdict['hypothesis_rate_below_nominal']['nominal_rate_hertz']:g} Hz nominal"]
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# Synthesizer: the bus ADR-0002 assumed, so the prediction has a picture
# ---------------------------------------------------------------------------

def synthesize(path: Path, rate: float, joints: int, seconds: float, bitrate: int, extended: bool,
               feedback_every: int, response_microseconds: float, host_id: int, seed: int) -> int:
    """Writes a candump-style log of an idealised vendor loop: every cycle the
    host sends one MIT command per joint, spaced by a few microseconds of Python,
    and each motor answers with one feedback frame after a fixed response time.
    Frames queue for the bus and win it by identifier, lowest first, the way
    CAN arbitration works. Stuffing is drawn uniformly between none and worst
    case, because the real count depends on data this model does not have.

    The queue is unbounded, so an overloaded bus shows up as delays that grow
    without limit. On real hardware the driver's transmit queue fills instead
    and send() fails with ENOBUFS -- a different symptom of the same cause."""
    generator = random.Random(seed)
    period = int(NANOSECONDS_PER_SECOND / rate)
    origin = 1_700_000_000 * NANOSECONDS_PER_SECOND
    pending: list[tuple[int, int, int, bool, int]] = []   # (ready_at, identifier, length, extended, kind)
    written: list[tuple[int, int, int, bool]] = []
    bus_free = origin

    def command_identifier(joint: int) -> int:
        return ((1 << 24) | (host_id << 8) | joint) if extended else joint

    def feedback_identifier(joint: int) -> int:
        return ((2 << 24) | (joint << 8) | host_id) if extended else (0x100 | joint)

    cycles = int(seconds * rate)
    for cycle in range(cycles):
        cycle_start = origin + cycle * period + int(generator.gauss(0, 20_000))
        for joint in range(1, joints + 1):
            pending.append((cycle_start + joint * 8_000, command_identifier(joint), 8, extended, 1))
        # Serve the bus until the next cycle starts (or the queue empties).
        next_cycle = origin + (cycle + 1) * period
        while pending:
            earliest = min(candidate[0] for candidate in pending)
            ready = [entry for entry in pending if entry[0] <= max(bus_free, earliest)]
            ready.sort(key=lambda entry: (entry[1], entry[0]))
            entry = ready[0]
            pending.remove(entry)
            start = max(bus_free, entry[0])
            if start >= next_cycle and cycle + 1 < cycles:
                pending.append(entry)
                break
            minimum, maximum = frame_bits(entry[2], entry[3])
            bits = generator.randint(minimum, maximum)
            end = start + bits * NANOSECONDS_PER_SECOND // bitrate
            bus_free = end
            written.append((end, entry[1], entry[2], entry[3]))
            if entry[4] == 1 and (cycle % feedback_every == 0):
                joint = entry[1] & 0xFF if extended else entry[1]
                pending.append((end + int(response_microseconds * 1000), feedback_identifier(joint), 8, extended, 2))
    written.sort()
    with path.open("w", encoding="utf-8") as handle:
        for stamp, identifier, length, is_extended in written:
            payload = "".join(f"{generator.randrange(256):02X}" for _ in range(length))
            identifier_text = f"{identifier:08X}" if is_extended else f"{identifier:03X}"
            handle.write(f"({stamp // NANOSECONDS_PER_SECOND}.{stamp % NANOSECONDS_PER_SECOND:09d}) can0 {identifier_text}#{payload}\n")
    return len(written)


# ---------------------------------------------------------------------------
# Self-test: the parser against every candump notation, and the bit arithmetic
# against the numbers the C++ test pins. Run by CTest as can_bus_load_selftest.
# ---------------------------------------------------------------------------

def selftest() -> int:
    failures = 0

    def check(condition: bool, message: str) -> None:
        nonlocal failures
        if not condition:
            failures += 1
            print(f"FAIL {message}")

    check(frame_bits(8, False) == (111, 135), "standard 8-byte frame bits")
    check(frame_bits(8, True) == (131, 160), "extended 8-byte frame bits")
    check(frame_bits(0, False) == (47, 55), "empty standard frame bits")
    check(frame_bits(8, False, remote=True) == (47, 55), "remote frame carries no data bits")
    check(frame_bits(8, True, error=True) == (0, 0), "error frame costs nothing we can count")

    samples = {
        "log classic": ("(1700000000.123456) can0 123#DEADBEEF", 0x123, False, b"\xde\xad\xbe\xef", False, False, False),
        "log extended": ("(1700000000.123456) can0 0100FD01#0011223344556677", 0x0100FD01, True, bytes(range(0, 0x78, 0x11)), False, False, False),
        "log remote": ("(1.5) can0 7FF#R", 0x7FF, False, b"", True, False, False),
        "log fd": ("(1.5) can0 123##1DEADBEEF00112233445566778899AABBCCDDEEFF", 0x123, False, bytes.fromhex("DEADBEEF00112233445566778899AABBCCDDEEFF"), False, True, False),
        "log error": ("(1.5) can0 20000004#0000000000000000", 0x4, False, bytes(8), False, False, True),
        "capture hw": ("(12.000000001) can0 020001FD#00112233 hw=99", 0x020001FD, True, bytes.fromhex("00112233"), False, False, False),
        "table classic": (" (1700000000.123456)  can0  123   [4]  DE AD BE EF", 0x123, False, b"\xde\xad\xbe\xef", False, False, False),
        "table extended": (" (1700000000.123456)  can0  0100FD01   [8]  00 11 22 33 44 55 66 77", 0x0100FD01, True, bytes(range(0, 0x78, 0x11)), False, False, False),
        "table remote": (" (1.0)  can0  123   [0]  remote request", 0x123, False, b"", True, False, False),
        "table error": (" (1.0)  can0  20000004   [8]  00 00 00 00 00 00 00 00   ERRORFRAME", 0x4, False, bytes(8), False, False, True),
    }
    for name, (line, identifier, extended, data, remote, fd, error) in samples.items():
        frame = parse_line(line)
        check(frame is not None, f"{name}: parsed")
        if frame is None:
            continue
        check(frame.identifier == identifier, f"{name}: identifier {frame.identifier:X} != {identifier:X}")
        check(frame.extended == extended, f"{name}: extended {frame.extended}")
        check(frame.data == data, f"{name}: data {frame.data.hex()} != {data.hex()}")
        check(frame.remote == remote, f"{name}: remote {frame.remote}")
        check(frame.fd == fd, f"{name}: fd {frame.fd}")
        check(frame.error == error, f"{name}: error {frame.error}")
    first = parse_line("(1700000000.123456) can0 123#00")
    check(first is not None and first.timestamp_nanoseconds == 1_700_000_000_123_456_000, "microsecond timestamp scaled to ns")
    check(parse_line("  can0  123   [1]  00") is None, "a line without a timestamp is not a frame")

    ip_text = ("3: can0: <NOARP,UP,LOWER_UP,ECHO> mtu 16 qdisc pfifo_fast state UP mode DEFAULT group default qlen 10\n"
               "    link/can  promiscuity 0 minmtu 0 maxmtu 0\n"
               "    can state ERROR-ACTIVE (berr-counter tx 0 rx 0) restart-ms 0\n"
               "\t  bitrate 1000000 sample-point 0.750\n"
               "    RX:  bytes packets errors dropped  missed   mcast\n"
               "       1234    5678      1       2       0       0\n"
               "    TX:  bytes packets errors dropped carrier collsns\n"
               "       4321    8765      0       0       0       0\n")
    import tempfile
    with tempfile.NamedTemporaryFile("w", suffix=".txt", delete=False) as handle:
        handle.write(ip_text)
        ip_path = Path(handle.name)
    parsed = parse_ip_link(ip_path)
    ip_path.unlink()
    check(parsed.get("bitrate") == 1_000_000, "ip link: bitrate")
    check(parsed.get("rx", {}).get("packets") == 5678 and parsed["rx"]["dropped"] == 2, "ip link: RX counters")
    check(parsed.get("tx", {}).get("bytes") == 4321, "ip link: TX counters")

    # A synthesized bus must come back with the shape it was written with.
    with tempfile.TemporaryDirectory() as directory:
        log = Path(directory) / "synthetic.log"
        synthesize(log, 500.0, 7, 0.2, 1_000_000, True, 1, 150.0, 0xFD, 1)
        analysis = analyze(parse_log(log), 1_000_000, 500.0, "robstride", None, log_name=str(log))
    check(analysis.host_id == 0xFD, "synthetic: host id inferred")
    check(analysis.cycles == 100, f"synthetic: {analysis.cycles} cycles")
    check(abs(analysis.frames_per_cycle - 14.0) < 0.01, f"synthetic: {analysis.frames_per_cycle} frames per cycle")
    check(len(analysis.joints) == 7, "synthetic: seven joints")
    check(0.85 < analysis.utilisation_minimum < 0.95 and analysis.utilisation_maximum > 1.0,
          f"synthetic: utilisation {analysis.utilisation_minimum:.2f}..{analysis.utilisation_maximum:.2f}")

    if failures:
        print(f"FAILED can_bus_load selftest: {failures} checks failed")
        return 1
    print("PASS can_bus_load selftest")
    return 0


# ---------------------------------------------------------------------------
# Command line
# ---------------------------------------------------------------------------

def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    commands = parser.add_subparsers(dest="command", required=True)

    analyze_parser = commands.add_parser("analyze", help="report on a candump log")
    analyze_parser.add_argument("log", type=Path)
    analyze_parser.add_argument("--bitrate", type=int, default=1_000_000, help="from `ip -details link show can0`")
    analyze_parser.add_argument("--rate", type=float, default=500.0, help="nominal loop rate, Hz")
    analyze_parser.add_argument("--protocol", choices=sorted(PROTOCOLS), default="robstride")
    analyze_parser.add_argument("--host-id", type=lambda text: int(text, 0), default=None,
                                help="host CAN id (default: inferred from feedback frames)")
    analyze_parser.add_argument("--counters-before", type=Path, help="saved `ip -details -statistics link show can0`")
    analyze_parser.add_argument("--counters-after", type=Path)
    analyze_parser.add_argument("--json", type=Path, help="write the full analysis here")

    synthesize_parser = commands.add_parser("synthesize", help="write a log of the bus ADR-0002 assumed")
    synthesize_parser.add_argument("output", type=Path)
    synthesize_parser.add_argument("--rate", type=float, default=500.0)
    synthesize_parser.add_argument("--joints", type=int, default=7)
    synthesize_parser.add_argument("--seconds", type=float, default=2.0)
    synthesize_parser.add_argument("--bitrate", type=int, default=1_000_000)
    synthesize_parser.add_argument("--standard-ids", action="store_true", help="11-bit identifiers instead of 29-bit")
    synthesize_parser.add_argument("--feedback-every", type=int, default=1, help="feedback only every N cycles")
    synthesize_parser.add_argument("--response-us", type=float, default=150.0, dest="response_microseconds",
                                   help="motor response time after a command, microseconds")
    synthesize_parser.add_argument("--host-id", type=lambda text: int(text, 0), default=0xFD)
    synthesize_parser.add_argument("--seed", type=int, default=1)

    commands.add_parser("selftest", help="check the parser and the bit arithmetic")

    arguments = parser.parse_args(argv)
    if arguments.command == "selftest":
        return selftest()
    if arguments.command == "synthesize":
        count = synthesize(arguments.output, arguments.rate, arguments.joints, arguments.seconds, arguments.bitrate,
                           not arguments.standard_ids, arguments.feedback_every, arguments.response_microseconds,
                           arguments.host_id, arguments.seed)
        print(f"wrote {count} frames to {arguments.output}")
        return 0

    frames = parse_log(arguments.log)
    before = parse_ip_link(arguments.counters_before) if arguments.counters_before else None
    after = parse_ip_link(arguments.counters_after) if arguments.counters_after else None
    analysis = analyze(frames, arguments.bitrate, arguments.rate, arguments.protocol, arguments.host_id,
                       before, after, str(arguments.log))
    print(report(analysis))
    if arguments.json:
        arguments.json.write_text(json.dumps(asdict(analysis), indent=2) + "\n", encoding="utf-8")
        print(f"\nwritten to {arguments.json}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
