# %% [markdown]
# # Lab 02 — what is actually on the CAN bus?
#
# **Question.** At 500 Hz, commanding all seven joints of the B601-RS, how
# loaded is the bus, and what is the worst-case frame latency?
#
# **Why it matters.** ADR-0002 computed, from memory, that 14 frames per cycle
# at 500 Hz is about 90% of a 1 Mbit/s bus. On CAN, bus load sets the
# worst-case latency, not the kernel, so this single number decides the
# timing budget for everything above it. This notebook does the arithmetic
# properly, shows what the assumed bus *would* look like, and then reads the
# real log once it exists.
#
# Runs with the standard library only. `matplotlib` is used for a plot if it
# is installed, and skipped if not.

# %%
import sys
from pathlib import Path

REPOSITORY = Path(__file__).resolve().parent.parent if "__file__" in globals() else Path.cwd().parent
sys.path.insert(0, str(REPOSITORY / "bench" / "can_bus_load"))
import can_bus_load  # noqa: E402  (the bench is the analysis; the notebook is the write-up)

# %% [markdown]
# ## 1. The arithmetic, done properly
#
# A classic CAN frame is fixed fields plus data plus a data-dependent number
# of *stuff bits*: after five identical bits the sender inserts one opposite
# bit so receivers keep their clocks locked. The stuffed region runs from
# start-of-frame through the CRC, so the worst case is one extra bit per
# four. Nobody knows the exact count without the data and the CRC, which is
# why every bus-load number in this lab is a **range**.
#
# The 3-bit interframe space is included: no other frame can start during it,
# so it is bus time the frame consumed.

# %%
print(f"{'frame':<28}{'no stuffing':>14}{'worst case':>14}")
for label, length, extended in [("standard (11-bit id), 8 bytes", 8, False),
                                ("extended (29-bit id), 8 bytes", 8, True),
                                ("standard, 0 bytes", 0, False),
                                ("extended, 0 bytes", 0, True)]:
    minimum, maximum = can_bus_load.frame_bits(length, extended)
    print(f"{label:<28}{minimum:>10} bits{maximum:>10} bits")

# %% [markdown]
# RobStride drives use **29-bit identifiers** (the message type sits in the
# top five bits). So each of the 14 frames is 131–160 bits, not the
# "108–130" ADR-0002 guessed for a standard frame.

# %%
BITRATE = 1_000_000
RATE = 500.0
JOINTS = 7
minimum_bits, maximum_bits = can_bus_load.frame_bits(8, True)
frames_per_second = 2 * JOINTS * RATE
print(f"{frames_per_second:.0f} frames/s x {minimum_bits}..{maximum_bits} bits "
      f"= {frames_per_second * minimum_bits / BITRATE * 100:.0f}% .. "
      f"{frames_per_second * maximum_bits / BITRATE * 100:.0f}% of {BITRATE / 1e6:.0f} Mbit/s")
for rate in (500, 400, 250, 200, 100):
    load_maximum = 2 * JOINTS * rate * maximum_bits / BITRATE
    print(f"  at {rate:>4} Hz: up to {load_maximum * 100:5.0f}%"
          f"{'   <- under the 50% where latency stays predictable' if load_maximum <= 0.5 else ''}")

# %% [markdown]
# So at the worst case, **14 extended frames at 500 Hz do not fit on 1 Mbit/s
# at all**, and even with no stuffing the bus is over 90% full. One of
# ADR-0002's three possibilities has to be true:
#
# 1. the bus is faster (CAN FD, or a 29-bit bus at more than 1 Mbit/s),
# 2. feedback is not requested every cycle,
# 3. the real rate is below 500 Hz.
#
# ## 2. Predict
#
# Before running the model or the measurement, write down:
#
# - Which of the three possibilities do you expect? Why?
# - Command frames are type 1 (`0x01xxxxxx`), feedback frames type 2
#   (`0x02xxxxxx`). CAN arbitration lets the lowest identifier win. When the
#   host sends seven commands in a burst, when does the *first* feedback get
#   onto the bus?
# - The vendor loop uses a relative sleep (Lab 01 measured 4.7% slow). Will
#   the wire show 500 Hz or 477 Hz?

# %% [markdown]
# ## 3. What the assumed bus would look like
#
# The synthesizer plays the bus ADR-0002 assumed: seven motors, one command
# and one feedback each, every cycle, with the arbitration rule and a 150 µs
# motor response time. Stuffing is drawn at random between none and worst
# case. This is a **model**, not data: it shows what the numbers below mean,
# and it tested the analysis before the arm was plugged in.

# %%
import tempfile

with tempfile.TemporaryDirectory() as directory:
    predicted_log = Path(directory) / "predicted.log"
    can_bus_load.synthesize(predicted_log, RATE, JOINTS, seconds=2.0, bitrate=BITRATE, extended=True,
                            feedback_every=1, response_microseconds=150.0, host_id=0xFD, seed=1)
    predicted = can_bus_load.analyze(can_bus_load.parse_log(predicted_log), BITRATE, RATE, "robstride", None,
                                     log_name="predicted.log (model)")
print(can_bus_load.report(predicted))

# %% [markdown]
# Two things to notice in the model before looking at real data:
#
# - **Command-to-feedback is about a millisecond, not 150 µs.** All seven
#   type-1 commands have lower identifiers than any type-2 feedback, so the
#   feedback for joint 1 waits until the whole command burst is out. That is
#   arbitration doing exactly what it is designed to do, and it means the
#   round trip is set by the *burst*, not the motor.
# - **The bus does not keep up.** Above 100% at the top of the range, and the
#   `first command -> last frame` maximum is many cycles long: frames queue
#   across cycle boundaries. On real hardware this shows up differently, as
#   `ENOBUFS` from the driver's transmit queue, but the cause is the same.

# %% [markdown]
# ## 4. The measurement
#
# Follow `bench/can_bus_load/README.md`: `ip -details link show can0` for the
# real bit rate, `candump -ta -H can0 > lab02.log` while the vendor stack runs
# with the motors **disabled**, and `ip -details -statistics link show can0`
# before and after into `before.txt` and `after.txt`. Put the files next to
# this notebook, set the bit rate, and run the cell.

# %%
LOG = Path("lab02.log")
COUNTERS_BEFORE = Path("before.txt")
COUNTERS_AFTER = Path("after.txt")
MEASURED_BITRATE = 1_000_000   # replace with the value from `ip -details link show can0`

measured = None
if LOG.exists():
    before = can_bus_load.parse_ip_link(COUNTERS_BEFORE) if COUNTERS_BEFORE.exists() else None
    after = can_bus_load.parse_ip_link(COUNTERS_AFTER) if COUNTERS_AFTER.exists() else None
    measured = can_bus_load.analyze(can_bus_load.parse_log(LOG), MEASURED_BITRATE, RATE, "robstride", None,
                                    before, after, log_name=str(LOG))
    print(can_bus_load.report(measured))
else:
    print(f"{LOG} not found: record it first (see bench/can_bus_load/README.md). "
          "Nothing below this cell has data yet.")

# %% [markdown]
# ## 5. Reading the result
#
# Fill this in from the report above, in this order, and keep the wrong
# predictions next to the right numbers.
#
# 1. **Bit rate and frame type.** From step 1 and the `standard / extended /
#    fd` line. If FD frames or a bit rate above 1 Mbit/s: possibility 1.
# 2. **Feedbacks per command.** 1.0 means the vendor stack asks every motor
#    every cycle. Well below 1: possibility 2.
# 3. **Measured rate.** Against 500 Hz nominal and against Lab 01's relative
#    sleep drift. Below nominal: possibility 3, and now you know by how much.
# 4. **Utilisation.** The range, at the real bit rate. Compare with ADR-0002's
#    90% and say which side of 50% the arm actually runs on.
# 5. **Round trip and skew.** `command -> feedback` per joint and `first
#    command -> last frame` per cycle. These go straight into the timing
#    budget (ADR-0002, action item 2) as the dead time and the joint-to-joint
#    skew; Lab 03 refines the dead time.
#
# ## 6. The theory the numbers just showed
#
# - **Bus utilisation is a queueing problem.** Waiting time for the bus grows
#   like *load / (1 − load)*: at 50% you wait one frame on average, at 90% nine,
#   and at 100% the queue never empties. A CAN frame is 130 µs, so "nine
#   frames" is over a millisecond, half the cycle. No kernel tuning touches
#   this. That is why ADR-0002 says the fieldbus budget comes before the
#   real-time tier.
# - **Arbitration is priority scheduling with no preemption.** The lowest
#   identifier always wins, and a frame already on the bus is never
#   interrupted. So the *identifier layout* is a design decision: the vendor
#   protocol puts all commands ahead of all feedback, which minimises command
#   latency and maximises feedback latency. Whether that is the right choice
#   for a 500 Hz outer loop is a question the timing budget has to answer.
# - **Bit stuffing is why nothing here is a single number.** Up to 25% extra
#   bits, decided by the data. Report the range, or report the measurement.
# - **Timestamps from the kernel.** A timestamp taken in user space after
#   `read()` measures how late our thread was, not when the frame arrived.
#   `SO_TIMESTAMPING` gives the kernel's stamp at delivery, and a hardware
#   stamp at end-of-frame if the adapter has a clock. `core/can` uses both;
#   `bench_can_capture` reports which it got.

# %%
try:
    import matplotlib.pyplot as plt
except ImportError:
    plt = None
    print("matplotlib not installed; skipping the plot")

if plt is not None:
    for title, analysis in (("model", predicted), ("measured", measured)):
        if analysis is None or not analysis.joints:
            continue
        joints = sorted(analysis.joints, key=int)
        p50 = [analysis.joints[j]["command_to_feedback"].get("p50_microseconds", 0) for j in joints]
        p99 = [analysis.joints[j]["command_to_feedback"].get("p99_microseconds", 0) for j in joints]
        maximum = [analysis.joints[j]["command_to_feedback"].get("maximum_microseconds", 0) for j in joints]
        figure, axis = plt.subplots(figsize=(7, 3.5))
        axis.plot(joints, p50, "o-", label="p50")
        axis.plot(joints, p99, "s-", label="p99")
        axis.plot(joints, maximum, "^-", label="max")
        axis.set_title(f"command -> feedback per joint ({title}), "
                       f"bus {analysis.utilisation_minimum * 100:.0f}..{analysis.utilisation_maximum * 100:.0f}%")
        axis.set_xlabel("joint (motor CAN id)")
        axis.set_ylabel("microseconds")
        axis.legend()
        figure.tight_layout()
    plt.show()
