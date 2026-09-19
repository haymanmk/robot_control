# Handoff: `core/can` and Lab 02

A note for the session that starts `core/can`. It says what exists, what was
decided, and what the first steps are, so that nothing has to be rediscovered.
Read [AGENTS.md](../AGENTS.md) and [coding-style.md](coding-style.md) first;
they are binding.

## Where things stand

Done, tested natively and under ThreadSanitizer, on branch
`claude/elegant-davinci-1tn9hu`:

| Module | What it gives `core/can` |
|---|---|
| `core/rt` (`robot_control::realtime`) | the clock, `CyclicTask`, lock-free `SpscRing` and `Seqlock`, real-time setup with limit inspection |
| `core/telemetry` | the 256-byte per-cycle record, which already has `can_tx_ns` and `can_rx_ns` fields waiting to be filled |
| `core/bridge` | the shared-memory transport and watchdog; not needed by `core/can` directly |

Layers depend downward only: `rt -> telemetry -> bridge -> can -> drive -> ...`.
`core/can` may use `core/rt` and `core/telemetry`; nothing above it.

## What was decided (do not re-decide)

- **The bus is the binding constraint, not the kernel.** At 500 Hz with 7
  motors, one command and one feedback frame each, classic CAN at 1 Mbit/s is
  about 90% utilised ([ADR-0002](adr/0002-target-platform-rebot-b601-rs.md)).
  Lab 02 exists to find out whether that is really what is on the wire.
- **Timestamps come from the kernel, not user space.** Use `SO_TIMESTAMPING`
  (and hardware timestamps if the adapter supports them). A timestamp taken in
  user space measures our scheduler, not the bus
  ([ADR-0004](adr/0004-system-decomposition.md) §4).
- **The cyclic path rules apply** to anything the 500 Hz loop calls: no
  allocation, no blocking, `noexcept`, worst-case cost documented. Opening the
  socket, configuring filters and reading statistics are not cyclic; sending a
  frame and draining received frames are.
- **Nothing moves under its own power until Lab 00 is done**
  ([ADR-0005](adr/0005-safe-state-and-stop-architecture.md)). `core/can` can
  be built and tested entirely on `vcan0` without the arm.

## Suggested shape

Keep it small. Names follow the style guide: whole words, `snake_case` values,
`CamelCase` types, namespace `robot_control::can`.

```
core/can/include/robot_control/can/
  frame.hpp        CanFrame: identifier, flags (extended, remote, error), length,
                   8 data bytes, receive timestamp in nanoseconds. Trivially
                   copyable; goes into rings and telemetry unchanged.
  transport.hpp    CanTransport interface: open/close (non-cyclic);
                   send(const CanFrame&) and receive(CanFrame&) (cyclic,
                   non-blocking, noexcept); statistics() (non-cyclic).
  socketcan.hpp    SocketCanTransport: AF_CAN raw socket, non-blocking,
                   SO_TIMESTAMPING with software and, if available, hardware
                   receive timestamps read from recvmsg's control data.
  statistics.hpp   BusStatistics: frames and bytes sent and received, receive
                   errors, and an estimate of bus utilisation from bit counts
                   and the configured bit rate. Read from
                   /sys/class/net/<interface>/ for the driver's own counters.
```

Test against the virtual interface, which needs no hardware:

```bash
sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
```

A test can open two `SocketCanTransport`s on `vcan0`, send from one, receive
on the other, and check the identifier, data and that the receive timestamp
is a kernel timestamp (non-zero, monotonic). `vcan0` cannot report bus load or
hardware timestamps; those are checked in Lab 02 on the real adapter.

## Lab 02 — what is actually on the wire?

Question: at 500 Hz commanding all seven joints, how loaded is the bus, and
what is the worst-case frame latency? Hardware needed: the arm's CAN adapter,
motors powered but **disabled** (no torque). Steps, in order:

1. Record the bit rate the interface is really configured for:
   `ip -details link show can0`.
2. With the vendor's Python stack driving the arm in its normal loop, capture
   with kernel timestamps: `candump -ta -H can0 > lab02.log`.
3. From the log: frames per cycle, which identifiers, bytes per frame, and the
   spacing between a command frame and the matching feedback frame per joint.
4. Bus utilisation from the driver's counters over a fixed interval:
   `ip -statistics link show can0` before and after 10 s.
5. Compare with the 90% estimate in ADR-0002 and write down which of the three
   possibilities is true: faster bus (CAN FD), feedback not requested every
   cycle, or a real rate below 500 Hz.

Deliverable: a benchmark under `bench/can_bus_load/` that repeats steps 1, 3
and 4 from a `candump` log, and a write-up in `notebooks/02_can_bus.py`. The
result feeds the timing budget (ADR-0002, action item 2).

## Starting the session

```bash
git pull
cmake -B build && cmake --build build -j && ctest --test-dir build --output-on-failure
./build/app/realtime_check/realtime_check
```

Then: "Start `core/can` following `docs/handoff-core-can.md`."
