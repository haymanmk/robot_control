/// Lab 02 capture tool: record what is on the CAN bus with kernel timestamps,
/// through the same SocketCanTransport the control loop will use.
///
/// `candump -ta -H can0` already does this job, and the README says to run it
/// first. This tool exists to check our own transport against it on the real
/// adapter: same frames, same order, timestamps from the kernel (and from the
/// adapter's clock, if it has one), plus the driver's counters before and
/// after so the bus utilisation can be cross-checked two ways.
///
/// Output is the `candump -l` log format, one frame per line,
///
///     (seconds.nanoseconds) can0 1FD01#00112233445566778
///
/// with the timestamp on CLOCK_MONOTONIC rather than the wall clock, so it
/// feeds can_bus_load.py exactly like a candump log. Only differences between
/// timestamps matter to the analysis, so the time base does not.
///
/// Build:  cmake -B build && cmake --build build -j
/// Run:    ./build/bench/can_bus_load/bench_can_capture --interface can0 --seconds 10
///             --bitrate 1000000 --output lab02_capture.log

#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

#include "robot_control/can/socketcan.hpp"
#include "robot_control/can/statistics.hpp"
#include "robot_control/realtime/clock.hpp"
#include "robot_control/telemetry/provenance.hpp"

using robot_control::can::bus_utilisation;
using robot_control::can::BusStatistics;
using robot_control::can::CanFrame;
using robot_control::can::format_statistics;
using robot_control::can::InterfaceCounters;
using robot_control::can::read_interface_counters;
using robot_control::can::SocketCanTransport;
using robot_control::can::Utilisation;
using robot_control::realtime::monotonic_now;
using robot_control::telemetry::Provenance;

namespace {

[[noreturn]] void usage(const char* program, int code) {
  std::fprintf(
      stderr,
      "usage: %s [--interface IF] [--seconds S] [--bitrate BPS] [--output FILE] [--json FILE]\n"
      "  --interface  SocketCAN interface (default can0; vcan0 works for a dry run)\n"
      "  --seconds    capture length (default 10)\n"
      "  --bitrate    configured bit rate, from `ip -details link show can0` (default 1000000)\n"
      "  --output     write the frame log here (default: standard output)\n"
      "  --json       write the summary and provenance here\n",
      program);
  std::exit(code);
}

/// One frame in candump's log notation. Error frames keep the 0x20000000 flag
/// in the identifier, as candump prints them, so the analysis can tell them apart.
void write_frame(std::FILE* out, const char* interface, const CanFrame& frame) {
  const std::int64_t stamp = frame.receive_timestamp_nanoseconds;
  std::fprintf(out, "(%lld.%09lld) %s ", static_cast<long long>(stamp / 1'000'000'000),
               static_cast<long long>(stamp % 1'000'000'000), interface);
  if (frame.error()) {
    std::fprintf(out, "%08X#", frame.identifier | 0x20000000u);
  } else if (frame.extended()) {
    std::fprintf(out, "%08X#", frame.identifier);
  } else {
    std::fprintf(out, "%03X#", frame.identifier);
  }
  if (frame.remote()) {
    std::fputc('R', out);
  } else {
    for (unsigned index = 0; index < frame.length; ++index) {
      std::fprintf(out, "%02X", frame.data[index]);
    }
  }
  if (frame.hardware_timestamp_nanoseconds != 0) {
    // Not candump syntax; a trailing field the parser ignores unless asked.
    std::fprintf(out, " hw=%lld", static_cast<long long>(frame.hardware_timestamp_nanoseconds));
  }
  std::fputc('\n', out);
}

}  // namespace

int main(int argc, char** argv) {
  std::string interface = "can0";
  double seconds = 10.0;
  unsigned bitrate = 1'000'000;
  std::string output_path;
  std::string json_path;

  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    auto next = [&]() -> const char* {
      if (index + 1 >= argc) usage(argv[0], 2);
      return argv[++index];
    };
    if (argument == "--interface") interface = next();
    else if (argument == "--seconds") seconds = std::atof(next());
    else if (argument == "--bitrate") bitrate = static_cast<unsigned>(std::atol(next()));
    else if (argument == "--output") output_path = next();
    else if (argument == "--json") json_path = next();
    else if (argument == "-h" || argument == "--help") usage(argv[0], 0);
    else usage(argv[0], 2);
  }

  Provenance provenance = Provenance::collect();
  provenance.can_interface = interface;
  provenance.can_bitrate = bitrate;
  provenance.label = "lab02 can capture";
  std::fprintf(stderr, "Lab 02 -- CAN capture on %s for %.1f s at %u bit/s\n%s\n", interface.c_str(),
               seconds, bitrate, provenance.to_summary().c_str());

  SocketCanTransport transport;
  if (!transport.open(interface)) {
    std::fprintf(stderr, "cannot open %s: %s\n", interface.c_str(), transport.last_error().c_str());
    return 1;
  }
  std::fprintf(stderr, "  hardware timestamps: %s\n",
               transport.hardware_timestamps_enabled() ? "enabled by the adapter" : "not offered by the adapter");

  std::FILE* out = stdout;
  if (!output_path.empty()) {
    out = std::fopen(output_path.c_str(), "w");
    if (out == nullptr) {
      std::perror(output_path.c_str());
      return 1;
    }
  }

  const InterfaceCounters before = read_interface_counters(interface);
  const std::int64_t start = monotonic_now().count();
  const std::int64_t end = start + static_cast<std::int64_t>(seconds * 1e9);
  std::int64_t first_stamp = 0;
  std::int64_t last_stamp = 0;
  std::uint64_t hardware_stamped = 0;
  CanFrame frame{};
  while (monotonic_now().count() < end) {
    if (!transport.receive(frame)) {
      (void)transport.wait_readable(10);
      continue;
    }
    if (first_stamp == 0) first_stamp = frame.receive_timestamp_nanoseconds;
    last_stamp = frame.receive_timestamp_nanoseconds;
    if (frame.hardware_timestamp_nanoseconds != 0) ++hardware_stamped;
    write_frame(out, interface.c_str(), frame);
  }
  const InterfaceCounters after = read_interface_counters(interface);
  if (out != stdout) std::fclose(out);

  const BusStatistics statistics = transport.statistics();
  const std::int64_t span = last_stamp - first_stamp;
  const Utilisation load = bus_utilisation(statistics.minimum_bits_on_wire, statistics.maximum_bits_on_wire, span, bitrate);
  const InterfaceCounters delta = InterfaceCounters::difference(before, after);

  std::fprintf(stderr, "\nsocket saw, over %.3f s of frames:\n%s", static_cast<double>(span) / 1e9,
               format_statistics(statistics).c_str());
  std::fprintf(stderr, "  frames with a hardware timestamp   %12llu\n", static_cast<unsigned long long>(hardware_stamped));
  std::fprintf(stderr, "  bus utilisation from bit counts    %6.1f%% .. %.1f%%\n", load.minimum * 100.0, load.maximum * 100.0);
  if (delta.valid) {
    std::fprintf(stderr, "\ndriver counters over %.3f s:\n", static_cast<double>(delta.read_at_nanoseconds) / 1e9);
    std::fprintf(stderr, "  rx frames %llu  rx bytes %llu  rx errors %llu  rx dropped %llu\n",
                 static_cast<unsigned long long>(delta.received_frames), static_cast<unsigned long long>(delta.received_bytes),
                 static_cast<unsigned long long>(delta.receive_errors), static_cast<unsigned long long>(delta.receive_dropped));
    std::fprintf(stderr, "  tx frames %llu  tx bytes %llu  tx errors %llu  tx dropped %llu\n",
                 static_cast<unsigned long long>(delta.transmitted_frames), static_cast<unsigned long long>(delta.transmitted_bytes),
                 static_cast<unsigned long long>(delta.transmit_errors), static_cast<unsigned long long>(delta.transmit_dropped));
    if (delta.received_frames != statistics.frames_received) {
      std::fprintf(stderr, "  note: the driver counted %llu received frames, the socket %llu; "
                   "the difference is frames before open, after close, or dropped\n",
                   static_cast<unsigned long long>(delta.received_frames),
                   static_cast<unsigned long long>(statistics.frames_received));
    }
  } else {
    std::fprintf(stderr, "\ndriver counters: not readable for %s\n", interface.c_str());
  }

  if (!json_path.empty()) {
    std::FILE* json = std::fopen(json_path.c_str(), "w");
    if (json == nullptr) {
      std::perror(json_path.c_str());
      return 1;
    }
    std::string provenance_json = provenance.to_json();
    const auto last_brace = provenance_json.rfind('}');
    if (last_brace != std::string::npos) provenance_json.erase(last_brace);
    std::fprintf(json,
                 "%s,\n  \"capture\": {\n"
                 "    \"span_seconds\": %.6f,\n"
                 "    \"frames_received\": %llu,\n"
                 "    \"bytes_received\": %llu,\n"
                 "    \"error_frames\": %llu,\n"
                 "    \"frames_without_kernel_timestamp\": %llu,\n"
                 "    \"frames_with_hardware_timestamp\": %llu,\n"
                 "    \"hardware_timestamps_enabled\": %s,\n"
                 "    \"minimum_bits_on_wire\": %llu,\n"
                 "    \"maximum_bits_on_wire\": %llu,\n"
                 "    \"utilisation_minimum\": %.6f,\n"
                 "    \"utilisation_maximum\": %.6f,\n"
                 "    \"driver_received_frames\": %llu,\n"
                 "    \"driver_received_bytes\": %llu,\n"
                 "    \"driver_receive_dropped\": %llu,\n"
                 "    \"driver_receive_errors\": %llu\n"
                 "  }\n}\n",
                 provenance_json.c_str(), static_cast<double>(span) / 1e9,
                 static_cast<unsigned long long>(statistics.frames_received),
                 static_cast<unsigned long long>(statistics.bytes_received),
                 static_cast<unsigned long long>(statistics.receive_errors),
                 static_cast<unsigned long long>(statistics.frames_without_kernel_timestamp),
                 static_cast<unsigned long long>(hardware_stamped),
                 transport.hardware_timestamps_enabled() ? "true" : "false",
                 static_cast<unsigned long long>(statistics.minimum_bits_on_wire),
                 static_cast<unsigned long long>(statistics.maximum_bits_on_wire), load.minimum, load.maximum,
                 static_cast<unsigned long long>(delta.received_frames),
                 static_cast<unsigned long long>(delta.received_bytes),
                 static_cast<unsigned long long>(delta.receive_dropped),
                 static_cast<unsigned long long>(delta.receive_errors));
    std::fclose(json);
    std::fprintf(stderr, "summary written to %s\n", json_path.c_str());
  }
  return 0;
}
