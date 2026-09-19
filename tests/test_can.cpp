/// Unit tests for core/can that need no CAN interface: the frame layout, the
/// bit-time arithmetic Lab 02 rests on, the utilisation formula, the sysfs
/// counter reader (against `lo`, which every Linux machine has), and the
/// transport's failure path. The loopback test on vcan0 is test_can_loopback.

#include <cstring>

#include "robot_control/can/frame.hpp"
#include "robot_control/can/socketcan.hpp"
#include "robot_control/can/statistics.hpp"
#include "test_support.hpp"

using robot_control::can::bits_on_wire;
using robot_control::can::bus_utilisation;
using robot_control::can::CanFrame;
using robot_control::can::frame_error;
using robot_control::can::frame_extended;
using robot_control::can::frame_remote;
using robot_control::can::InterfaceCounters;
using robot_control::can::make_data_frame;
using robot_control::can::read_interface_counters;
using robot_control::can::SocketCanTransport;
using robot_control::can::Utilisation;
using robot_control::can::WireBits;

namespace {

void test_frame_layout_and_factory() {
  const std::uint8_t payload[] = {0xDE, 0xAD, 0xBE, 0xEF, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06};
  const CanFrame frame = make_data_frame(0x1FFFFFFFu | 0x80000000u, payload, 10, true);
  CHECK_EQ(frame.identifier, 0x1FFFFFFFu);      // flag bits stripped
  CHECK(frame.extended());
  CHECK(!frame.remote());
  CHECK(!frame.error());
  CHECK_EQ(frame.length, 8u);                    // clamped to a classic frame
  CHECK_EQ(frame.data[0], 0xDEu);
  CHECK_EQ(frame.data[7], 0x04u);
  CHECK_EQ(frame.receive_timestamp_nanoseconds, 0);
  CHECK_EQ(frame.hardware_timestamp_nanoseconds, 0);

  const CanFrame standard = make_data_frame(0xFFFu, payload, 0, false);
  CHECK_EQ(standard.identifier, 0x7FFu);         // 11 bits only
  CHECK(!standard.extended());
  CHECK_EQ(standard.length, 0u);

  // A frame built with nothing is all zeros, so records can be compared bytewise.
  const CanFrame empty{};
  unsigned char zeros[sizeof(CanFrame)] = {};
  CHECK(std::memcmp(&empty, zeros, sizeof(CanFrame)) == 0);
}

/// The numbers ADR-0002 quoted from memory, now derived: a standard frame with
/// 8 data bytes is 108 bits before stuffing and 132 after worst-case stuffing;
/// an extended one is 128 and 157. Plus 3 bits of interframe space each.
void test_bits_on_wire() {
  const std::uint8_t payload[8] = {};
  const WireBits standard8 = bits_on_wire(make_data_frame(0x123, payload, 8, false));
  CHECK_EQ(standard8.minimum, 111u);
  CHECK_EQ(standard8.maximum, 135u);

  const WireBits extended8 = bits_on_wire(make_data_frame(0x123, payload, 8, true));
  CHECK_EQ(extended8.minimum, 131u);
  CHECK_EQ(extended8.maximum, 160u);

  const WireBits standard0 = bits_on_wire(make_data_frame(0x123, payload, 0, false));
  CHECK_EQ(standard0.minimum, 47u);
  CHECK_EQ(standard0.maximum, 47u + 8u);         // (34 - 1) / 4 = 8 stuff bits at most

  CanFrame remote = make_data_frame(0x123, payload, 8, false);
  remote.flags |= frame_remote;
  const WireBits remote_bits = bits_on_wire(remote);
  CHECK_MSG(remote_bits.minimum == standard0.minimum, "a remote frame carries no data bits");

  CanFrame error{};
  error.flags = frame_error;
  CHECK_EQ(bits_on_wire(error).maximum, 0u);

  // The Lab 02 arithmetic: 14 extended 8-byte frames per cycle at 500 Hz on
  // 1 Mbit/s. This is the number the lab exists to check against the wire.
  const std::uint64_t per_second_minimum = 14ull * extended8.minimum * 500ull;
  const std::uint64_t per_second_maximum = 14ull * extended8.maximum * 500ull;
  const Utilisation load = bus_utilisation(per_second_minimum, per_second_maximum, 1'000'000'000, 1'000'000);
  CHECK_MSG(load.minimum > 0.91 && load.minimum < 0.92, "minimum ~0.917");
  CHECK_MSG(load.maximum > 1.11 && load.maximum < 1.13, "maximum ~1.12: does not fit");
}

void test_utilisation_edge_cases() {
  CHECK_EQ(bus_utilisation(100, 200, 0, 1'000'000).maximum, 0.0);
  CHECK_EQ(bus_utilisation(100, 200, 1'000'000'000, 0).maximum, 0.0);
  const Utilisation half = bus_utilisation(500'000, 500'000, 1'000'000'000, 1'000'000);
  CHECK_MSG(half.minimum == 0.5 && half.maximum == 0.5, "exact when min == max");
}

void test_interface_counters() {
  // `lo` exists everywhere; its numbers are arbitrary but the reader must
  // find all eight files and stamp the read with the monotonic clock.
  const InterfaceCounters loopback = read_interface_counters("lo");
  CHECK_MSG(loopback.valid, "could not read /sys/class/net/lo/statistics");
  CHECK(loopback.read_at_nanoseconds > 0);

  const InterfaceCounters missing = read_interface_counters("no_such_interface_0");
  CHECK(!missing.valid);

  InterfaceCounters earlier = loopback;
  InterfaceCounters later = loopback;
  later.received_frames += 10;
  later.received_bytes += 640;
  later.read_at_nanoseconds += 2'000'000;
  const InterfaceCounters delta = InterfaceCounters::difference(earlier, later);
  CHECK(delta.valid);
  CHECK_EQ(delta.received_frames, 10u);
  CHECK_EQ(delta.received_bytes, 640u);
  CHECK_EQ(delta.read_at_nanoseconds, 2'000'000);
  CHECK_EQ(delta.transmitted_frames, 0u);
  earlier.valid = false;
  CHECK(!InterfaceCounters::difference(earlier, later).valid);
}

/// open() must fail cleanly, with a reason, on a machine without the
/// interface or without CAN support at all, and never leave a socket behind.
void test_transport_open_failure() {
  SocketCanTransport transport;
  CHECK(!transport.is_open());
  CHECK(!transport.open("nocan0"));
  CHECK(!transport.is_open());
  CHECK_MSG(!transport.last_error().empty(), "a failed open must say why");
  std::printf("  open(nocan0): %s\n", transport.last_error().c_str());

  CHECK(!transport.open(""));
  CHECK(!transport.open("this_name_is_far_too_long_for_ifnamsiz"));

  // The cyclic calls on a closed transport are defined: they fail, they do not crash.
  CanFrame frame{};
  CHECK(!transport.send(frame));
  CHECK(!transport.receive(frame));
  CHECK(!transport.wait_readable(0));
  CHECK_EQ(transport.statistics().send_failures, 1u);
  CHECK_EQ(transport.statistics().frames_received, 0u);
  transport.close();  // idempotent
}

}  // namespace

int main() {
  test_frame_layout_and_factory();
  test_bits_on_wire();
  test_utilisation_edge_cases();
  test_interface_counters();
  test_transport_open_failure();
  return robot_control::test::finish("can");
}
