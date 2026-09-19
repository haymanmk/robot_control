/// Loopback test for SocketCanTransport on the virtual interface:
///
///   sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0
///
/// Two transports on vcan0; what one sends the other receives, with the
/// identifier and data intact and a kernel receive timestamp that is on
/// CLOCK_MONOTONIC and monotonic across frames. Filters are checked too.
///
/// vcan0 cannot report bus load or hardware timestamps; those are Lab 02's
/// job on the real adapter. Without vcan0 this test exits with code 77 and
/// CTest reports it as skipped, not passed. That is deliberate: a machine
/// without CAN support cannot vouch for this code, and must not look as if it
/// had.

#include <cstdio>
#include <cstdlib>
#include <cstring>

#include "robot_control/can/socketcan.hpp"
#include "robot_control/realtime/clock.hpp"
#include "test_support.hpp"

using robot_control::can::CanFilter;
using robot_control::can::CanFrame;
using robot_control::can::make_data_frame;
using robot_control::can::SocketCanTransport;
using robot_control::realtime::monotonic_now;

namespace {

constexpr int skip_exit_code = 77;
constexpr const char* interface = "vcan0";

/// Waits up to a second for one frame; a loopback delivery takes microseconds.
bool receive_one(SocketCanTransport& transport, CanFrame& frame) {
  for (int attempt = 0; attempt < 1000; ++attempt) {
    if (transport.receive(frame)) return true;
    if (!transport.wait_readable(1)) continue;
  }
  return false;
}

void test_send_receive(SocketCanTransport& sender, SocketCanTransport& receiver) {
  const std::uint8_t payload[] = {1, 2, 3, 4, 5, 6, 7, 8};
  const std::int64_t before = monotonic_now().count();

  CHECK(sender.send(make_data_frame(0x123, payload, 8, false)));
  CanFrame frame{};
  CHECK_MSG(receive_one(receiver, frame), "nothing arrived on vcan0");
  CHECK_EQ(frame.identifier, 0x123u);
  CHECK(!frame.extended());
  CHECK_EQ(frame.length, 8u);
  CHECK(std::memcmp(frame.data, payload, 8) == 0);

  const std::int64_t after = monotonic_now().count();
  CHECK_MSG(frame.receive_timestamp_nanoseconds > 0, "kernel timestamp missing");
  CHECK_MSG(frame.receive_timestamp_nanoseconds >= before - 1'000'000 &&
                frame.receive_timestamp_nanoseconds <= after + 1'000'000,
            "receive timestamp is not on CLOCK_MONOTONIC (off by more than 1 ms)");
  CHECK_EQ(frame.hardware_timestamp_nanoseconds, 0);   // vcan has no clock
  CHECK_EQ(receiver.statistics().frames_without_kernel_timestamp, 0u);

  // Extended identifier, short payload, and the sender must not hear itself.
  CHECK(sender.send(make_data_frame(0x1ABCDEF, payload, 3, true)));
  CHECK_MSG(receive_one(receiver, frame), "extended frame did not arrive");
  CHECK_EQ(frame.identifier, 0x1ABCDEFu);
  CHECK(frame.extended());
  CHECK_EQ(frame.length, 3u);
  CHECK_EQ(frame.data[3], 0u);
  CanFrame echo{};
  CHECK_MSG(!sender.receive(echo), "a socket must not receive its own frames");

  CHECK_EQ(sender.statistics().frames_sent, 2u);
  CHECK_EQ(sender.statistics().bytes_sent, 11u);
  CHECK_EQ(receiver.statistics().frames_received, 2u);
  CHECK_EQ(receiver.statistics().bytes_received, 11u);
  CHECK_EQ(receiver.statistics().minimum_bits_on_wire, 111u + 91u);   // 131 - 5 * 8
}

/// Timestamps of a burst must never go backwards, and the spacing must be
/// small: the kernel stamps at delivery, not when we get around to reading.
void test_timestamps_monotonic(SocketCanTransport& sender, SocketCanTransport& receiver) {
  constexpr int burst = 100;   // well inside the default socket receive buffer
  std::uint8_t payload[8] = {};
  for (int index = 0; index < burst; ++index) {
    payload[0] = static_cast<std::uint8_t>(index);
    CHECK(sender.send(make_data_frame(0x200u + static_cast<std::uint32_t>(index % 8), payload, 8, false)));
  }
  std::int64_t previous = 0;
  int received = 0;
  bool ordered = true;
  CanFrame frame{};
  while (received < burst && receive_one(receiver, frame)) {
    if (frame.receive_timestamp_nanoseconds < previous) ordered = false;
    previous = frame.receive_timestamp_nanoseconds;
    ++received;
  }
  CHECK_EQ(received, burst);
  CHECK_MSG(ordered, "kernel receive timestamps went backwards");
}

void test_filters(SocketCanTransport& sender, SocketCanTransport& receiver) {
  const CanFilter only_0x300[] = {{0x300u, 0x7FFu}};
  CHECK(receiver.set_filters(only_0x300, 1));
  std::uint8_t payload[1] = {9};
  CHECK(sender.send(make_data_frame(0x301, payload, 1, false)));
  CHECK(sender.send(make_data_frame(0x300, payload, 1, false)));
  CanFrame frame{};
  CHECK_MSG(receive_one(receiver, frame), "the filtered-in frame did not arrive");
  CHECK_EQ(frame.identifier, 0x300u);
  CHECK_MSG(!receive_one(receiver, frame), "the filtered-out frame arrived");
  CHECK(receiver.set_filters(nullptr, 0));   // back to everything
  CHECK(sender.send(make_data_frame(0x301, payload, 1, false)));
  CHECK(receive_one(receiver, frame));
  CHECK_EQ(frame.identifier, 0x301u);
}

}  // namespace

int main() {
  SocketCanTransport sender;
  SocketCanTransport receiver;
  if (!sender.open(interface) || !receiver.open(interface)) {
    std::printf("SKIP can_loopback: cannot open %s (%s)\n"
                "  sudo modprobe vcan && sudo ip link add dev vcan0 type vcan && sudo ip link set up vcan0\n",
                interface, sender.last_error().empty() ? receiver.last_error().c_str()
                                                       : sender.last_error().c_str());
    return skip_exit_code;
  }
  std::printf("  %s open; hardware timestamps: %s\n", interface,
              receiver.hardware_timestamps_enabled() ? "yes" : "no (expected on vcan)");

  test_send_receive(sender, receiver);
  test_timestamps_monotonic(sender, receiver);
  test_filters(sender, receiver);
  return robot_control::test::finish("can_loopback");
}
