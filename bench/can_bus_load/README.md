# can_bus_load — Tier 2 fieldbus benchmark

*(Lab 02 in [the roadmap](../../docs/ROADMAP.md).)*

**Question:** at 500 Hz commanding all seven joints, how loaded is the CAN bus,
and what is the worst-case frame latency?

**Hardware needed:** the arm's CAN adapter, with the motors powered but
**disabled** (no torque). Nothing here moves the arm, and Lab 00 still blocks
anything that would ([ADR-0005](../../docs/adr/0005-safe-state-and-stop-architecture.md)).

## Why this lab exists

[ADR-0002](../../docs/adr/0002-target-platform-rebot-b601-rs.md) did the
arithmetic from memory: 14 frames per cycle, about 130 bits each, 500 Hz, on
1 Mbit/s is roughly 90% of the bus. That is far above the 30–50% where
latency stays predictable, so one of three things must be true:

1. the bus is faster than 1 Mbit/s (CAN FD),
2. feedback is not requested from every motor every cycle, or
3. the real rate is below 500 Hz.

This lab finds out which. The answer sets the timing budget for the whole
project, because on CAN it is bus load, not the kernel, that sets the
worst-case command latency.

## Before you run it: predict

Write these down first.

1. A RobStride command frame has a 29-bit identifier and 8 data bytes. How
   many bit times is that on the wire, with the worst-case bit stuffing?
   (Hint: the C++ test pins the answer.)
2. Fourteen of those at 500 Hz on 1 Mbit/s: does it fit? Compute the
   utilisation for no stuffing and for worst-case stuffing.
3. Command frames are message type 1, feedback frames type 2. Which wins
   arbitration when both are waiting? What does that do to the time from a
   command going out to its feedback coming back?
4. Will the loop rate seen on the wire be 500 Hz, or the 477 Hz Lab 01
   predicts for a relative-sleep loop?

Then look at what the model says:

```bash
python3 bench/can_bus_load/can_bus_load.py synthesize predicted.log --rate 500 --joints 7
python3 bench/can_bus_load/can_bus_load.py analyze predicted.log --bitrate 1000000
```

The synthesizer writes the bus ADR-0002 *assumed*: seven motors, one command
and one feedback each, every cycle, 29-bit identifiers, a 150 µs motor
response. It is a model, not a measurement. It exists so that your
prediction has a picture, and so the analysis is tested before the real log
exists.

## The measurement

All steps in order. Save every output; the analysis wants the counter
snapshots as files.

```bash
# 1. what the interface is really configured for
ip -details link show can0

# 2. driver counters before
ip -details -statistics link show can0 > before.txt

# 3. capture with kernel timestamps while the vendor stack runs its normal loop
#    with the motors disabled. -t a: absolute timestamps; -H: hardware
#    timestamps if the adapter has them.
candump -ta -H can0 > lab02.log        # let it run 10 s, then Ctrl-C

# 4. driver counters after
ip -details -statistics link show can0 > after.txt

# 5. the numbers
python3 bench/can_bus_load/can_bus_load.py analyze lab02.log --bitrate <from step 1> \
    --counters-before before.txt --counters-after after.txt --json lab02.json
```

Then the same capture through our own transport, to check it against
`candump` on the real adapter: same frames, same order, kernel timestamps,
and whether the adapter offered hardware timestamps.

```bash
cmake -B build && cmake --build build -j
./build/bench/can_bus_load/bench_can_capture --interface can0 --seconds 10 \
    --bitrate <from step 1> --output lab02_capture.log --json lab02_capture.json
python3 bench/can_bus_load/can_bus_load.py analyze lab02_capture.log --bitrate <from step 1>
```

The two analyses should agree on frame counts, identifiers and the
command-to-feedback spacing to within the kernel's timestamp resolution. If
`bench_can_capture` reports frames without a kernel timestamp, or the driver
counted more frames than the socket received, say so in the write-up: it
means the transport, not the bus, needs work.

## What the report gives you

| Line | What it answers |
|---|---|
| `bus utilisation ... % .. %` | how full the bus is. A range, because bit stuffing depends on the data. Above 100% at the top of the range means frames were queued past the cycle. |
| `standard / extended / fd` | 29-bit identifiers cost 20 more bits per frame than 11-bit ones. An FD frame means possibility 1. |
| `identifier / kind / joint / count / interval` | what is on the bus and how often. Anything `other` is a frame the RobStride decoder does not know; look at it. |
| `measured rate` and `cycle period` | the loop rate as seen on the wire, with its jitter. Compare with Lab 01. |
| `feedbacks per command` | 1.0 means every command is answered every cycle. Well below 1 is possibility 2. |
| `command -> feedback` per joint | the round trip the drive layer waits for, and the starting point of Lab 03. |
| `first command -> last frame` | the joint-to-joint skew inside one cycle, which ADR-0002 says must be measured, not assumed. |
| driver counters | the kernel's own count. `dropped` above zero means the kernel had nowhere to put a frame before we drained it. |

## What feeds the timing budget

ADR-0002's action item 2 needs: the period, the jitter budget, the allowed
overrun rate, the watchdog timeout and the joint-to-joint skew tolerance.
This lab supplies the measured rate and its jitter, the round trip per joint
and the skew. Lab 03 refines the round trip. Write the numbers into the
budget with the machine and the bit rate they came from; a number without
its machine is not a measurement.

## Where the fundamentals live

- **Arbitration.** Every node starts sending at once; a dominant (0) bit beats
  a recessive (1), so the lowest identifier wins and the losers retry after
  the frame. Nothing is lost, but a low-priority frame can wait behind every
  higher one, without limit, if the bus is full.
- **Bit stuffing.** After five identical bits the sender inserts one
  opposite bit so receivers can stay in sync. It adds up to one bit in four
  over the stuffed part of the frame, and it depends on the data. That is why
  bus load is a range.
- **Why utilisation sets the latency.** Queueing: at 90% load the wait for
  the bus is dominated by the frames ahead of you, and it grows without bound
  as load approaches 100%. Kernel latency is tens of microseconds; one frame
  is 130 µs; seven frames ahead of you is a millisecond.
- **`SO_TIMESTAMPING`.** A timestamp taken when our thread reads the socket
  measures our scheduler. The kernel stamps the frame when the driver hands
  it over; an adapter with a clock stamps it at the end of the frame. Use the
  best one you have.
- **`candump`, `cangen`, `cansend`.** The `can-utils` package. `cangen` at a
  chosen rate is the tool for the "things to try" below.

## Things to try

1. `cangen can0 -g 0 -I 0x7FF -L 8` in a second terminal: a flood of
   lowest-priority frames. Does the vendor loop's feedback still arrive? Now
   `-I 0x001`. What changed, and why is that worse?
2. Re-run with the motors enabled and holding position (after Lab 00), and
   compare frame counts. Does the drive send anything unprompted?
3. `--feedback-every 2` on the synthesizer: what the bus would look like if
   the vendor asked for feedback every other cycle. Compare with what you
   measured.

**Next:** Lab 03 — the round trip per joint, from command out to feedback
back, as a distribution.
