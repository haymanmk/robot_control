#include "robot_control/can/socketcan.hpp"

#include <cerrno>
#include <cstring>
#include <ctime>

#include <fcntl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <linux/errqueue.h>
#include <linux/net_tstamp.h>
#include <linux/sockios.h>
#include <net/if.h>
#include <poll.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <unistd.h>

namespace robot_control::can {
namespace {

/// Single-writer increment: the cyclic thread is the only writer, so a plain
/// load-add-store is race-free and cheaper than fetch_add's locked instruction.
inline void add_relaxed(std::atomic<std::uint64_t>& counter, std::uint64_t amount) noexcept {
  counter.store(counter.load(std::memory_order_relaxed) + amount, std::memory_order_relaxed);
}

[[nodiscard]] inline std::int64_t to_nanoseconds(const timespec& time) noexcept {
  return static_cast<std::int64_t>(time.tv_sec) * 1'000'000'000LL + static_cast<std::int64_t>(time.tv_nsec);
}

/// CLOCK_MONOTONIC minus CLOCK_REALTIME, read back to back. Two vDSO reads, no
/// system call. The only place in the core that touches the wall clock; see
/// the header for why it is allowed here.
[[nodiscard]] inline std::int64_t monotonic_minus_realtime() noexcept {
  timespec realtime{};
  timespec monotonic{};
  clock_gettime(CLOCK_REALTIME, &realtime);
  clock_gettime(CLOCK_MONOTONIC, &monotonic);
  return to_nanoseconds(monotonic) - to_nanoseconds(realtime);
}

[[nodiscard]] inline std::int64_t monotonic_now_nanoseconds() noexcept {
  timespec monotonic{};
  clock_gettime(CLOCK_MONOTONIC, &monotonic);
  return to_nanoseconds(monotonic);
}

}  // namespace

SocketCanTransport::~SocketCanTransport() { close(); }

bool SocketCanTransport::fail(const char* what) {
  const int saved_errno = errno;
  error = std::string(what) + ": " + std::strerror(saved_errno);
  close();
  return false;
}

bool SocketCanTransport::open(const std::string& name) {
  close();
  error.clear();
  interface = name;
  hardware_timestamps = false;

  if (interface.empty() || interface.size() >= IFNAMSIZ) {
    error = "interface name must be 1.." + std::to_string(IFNAMSIZ - 1) + " characters";
    return false;
  }

  descriptor = ::socket(PF_CAN, SOCK_RAW | SOCK_CLOEXEC, CAN_RAW);
  if (descriptor < 0) {
    return fail("socket(PF_CAN, SOCK_RAW, CAN_RAW)");
  }

  ifreq request{};
  std::strncpy(request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  if (::ioctl(descriptor, SIOCGIFINDEX, &request) < 0) {
    return fail("SIOCGIFINDEX (no such interface?)");
  }
  const int interface_index = request.ifr_ifindex;

  // Kernel receive timestamps, software always, hardware if the adapter has a
  // clock. SOF_TIMESTAMPING_SOFTWARE / RAW_HARDWARE ask for them to be
  // *reported* in the control message; the RX_ flags ask for them to be
  // *generated*.
  const int timestamp_flags = SOF_TIMESTAMPING_RX_SOFTWARE | SOF_TIMESTAMPING_SOFTWARE |
                              SOF_TIMESTAMPING_RX_HARDWARE | SOF_TIMESTAMPING_RAW_HARDWARE;
  if (::setsockopt(descriptor, SOL_SOCKET, SO_TIMESTAMPING, &timestamp_flags,
                   sizeof(timestamp_flags)) < 0) {
    return fail("setsockopt(SO_TIMESTAMPING)");
  }

  // Ask the adapter to stamp received frames with its own clock. Most USB
  // adapters and vcan refuse (EOPNOTSUPP, or EINVAL from older drivers), and
  // that is not an error: the software timestamp is still the kernel's.
  hwtstamp_config hardware_config{};
  hardware_config.tx_type = HWTSTAMP_TX_OFF;
  hardware_config.rx_filter = HWTSTAMP_FILTER_ALL;
  ifreq hardware_request{};
  std::strncpy(hardware_request.ifr_name, interface.c_str(), IFNAMSIZ - 1);
  hardware_request.ifr_data = reinterpret_cast<char*>(&hardware_config);
  hardware_timestamps = ::ioctl(descriptor, SIOCSHWTSTAMP, &hardware_request) == 0;

  // Error frames are information. Without this mask the driver drops them and
  // a bus-off looks exactly like a silent motor.
  const can_err_mask_t error_mask = CAN_ERR_MASK;
  if (::setsockopt(descriptor, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &error_mask, sizeof(error_mask)) < 0) {
    return fail("setsockopt(CAN_RAW_ERR_FILTER)");
  }

  const int flags = ::fcntl(descriptor, F_GETFL, 0);
  if (flags < 0 || ::fcntl(descriptor, F_SETFL, flags | O_NONBLOCK) < 0) {
    return fail("fcntl(O_NONBLOCK)");
  }

  sockaddr_can address{};
  address.can_family = AF_CAN;
  address.can_ifindex = interface_index;
  if (::bind(descriptor, reinterpret_cast<const sockaddr*>(&address), sizeof(address)) < 0) {
    return fail("bind(AF_CAN)");
  }
  return true;
}

void SocketCanTransport::close() noexcept {
  if (descriptor >= 0) {
    ::close(descriptor);
    descriptor = -1;
  }
}

bool SocketCanTransport::set_filters(const CanFilter* filters, std::size_t count) {
  if (descriptor < 0) {
    error = "set_filters: not open";
    return false;
  }
  // A bounded stack array: this is non-cyclic, but there is still no reason to
  // allocate for a handful of filters. Seven motors need seven at most.
  constexpr std::size_t max_filters = 32;
  if (count > max_filters) {
    error = "set_filters: at most " + std::to_string(max_filters) + " filters";
    return false;
  }
  // Zero filters is a trap: to the kernel it means "receive nothing", which
  // is how a raw socket is muted. "Accept everything" is one filter with a
  // zero mask, the same filter a new socket starts with.
  can_filter raw[max_filters] = {};
  std::size_t installed = count;
  if (count == 0) {
    installed = 1;
  }
  for (std::size_t index = 0; index < count; ++index) {
    raw[index].can_id = filters[index].identifier & extended_identifier_mask;
    raw[index].can_mask = filters[index].mask & extended_identifier_mask;
  }
  const socklen_t size = static_cast<socklen_t>(installed * sizeof(can_filter));
  if (::setsockopt(descriptor, SOL_CAN_RAW, CAN_RAW_FILTER, raw, size) < 0) {
    const int saved_errno = errno;
    error = std::string("setsockopt(CAN_RAW_FILTER): ") + std::strerror(saved_errno);
    return false;
  }
  return true;
}

bool SocketCanTransport::send(const CanFrame& frame) noexcept {
  can_frame raw{};
  raw.can_id = frame.identifier & (frame.extended() ? extended_identifier_mask : standard_identifier_mask);
  if (frame.extended()) raw.can_id |= CAN_EFF_FLAG;
  if (frame.remote()) raw.can_id |= CAN_RTR_FLAG;
  raw.len = frame.length > max_data_length ? max_data_length : frame.length;
  std::memcpy(raw.data, frame.data, raw.len);

  // MSG_DONTWAIT belt and braces with O_NONBLOCK. A full driver queue returns
  // ENOBUFS, immediately: SocketCAN never blocks a writer on the queue.
  const ssize_t written = ::send(descriptor, &raw, sizeof(raw), MSG_DONTWAIT);
  if (written != static_cast<ssize_t>(sizeof(raw))) {
    add_relaxed(send_failures, 1);
    return false;
  }
  const WireBits bits = bits_on_wire(frame);
  add_relaxed(frames_sent, 1);
  add_relaxed(bytes_sent, raw.len);
  add_relaxed(minimum_bits_on_wire, bits.minimum);
  add_relaxed(maximum_bits_on_wire, bits.maximum);
  return true;
}

bool SocketCanTransport::receive(CanFrame& frame) noexcept {
  can_frame raw{};
  // Room for the timestamping message; anything else the kernel might add is
  // truncated (MSG_CTRUNC) and ignored, which is harmless here.
  alignas(cmsghdr) unsigned char control[CMSG_SPACE(sizeof(scm_timestamping))] = {};
  iovec vector{&raw, sizeof(raw)};
  msghdr message{};
  message.msg_iov = &vector;
  message.msg_iovlen = 1;
  message.msg_control = control;
  message.msg_controllen = sizeof(control);

  const ssize_t got = ::recvmsg(descriptor, &message, MSG_DONTWAIT);
  if (got < static_cast<ssize_t>(sizeof(raw))) {
    // EAGAIN is the normal "queue empty". Anything else is also "no frame";
    // a persistent failure shows up as the drive layer's stale-feedback flag.
    return false;
  }

  frame = CanFrame{};
  if (raw.can_id & CAN_ERR_FLAG) {
    frame.flags = frame_error;
    frame.identifier = raw.can_id & CAN_ERR_MASK;
    add_relaxed(receive_errors, 1);
  } else {
    const bool extended = (raw.can_id & CAN_EFF_FLAG) != 0;
    frame.identifier = raw.can_id & (extended ? extended_identifier_mask : standard_identifier_mask);
    frame.flags = static_cast<std::uint8_t>((extended ? frame_extended : 0) |
                                            ((raw.can_id & CAN_RTR_FLAG) ? frame_remote : 0));
  }
  frame.length = raw.len > max_data_length ? max_data_length : raw.len;
  std::memcpy(frame.data, raw.data, frame.length);

  std::int64_t software_realtime = 0;
  for (cmsghdr* header = CMSG_FIRSTHDR(&message); header != nullptr; header = CMSG_NXTHDR(&message, header)) {
    if (header->cmsg_level == SOL_SOCKET && header->cmsg_type == SO_TIMESTAMPING) {
      scm_timestamping stamps{};
      std::memcpy(&stamps, CMSG_DATA(header), sizeof(stamps));
      software_realtime = to_nanoseconds(stamps.ts[0]);            // [0] software
      frame.hardware_timestamp_nanoseconds = to_nanoseconds(stamps.ts[2]);  // [2] raw hardware
    }
  }
  if (software_realtime != 0) {
    frame.receive_timestamp_nanoseconds = software_realtime + monotonic_minus_realtime();
  } else {
    // Should not happen with SO_TIMESTAMPING set; counted so it cannot pass
    // unnoticed, because a user-space stamp measures the wrong thing.
    frame.receive_timestamp_nanoseconds = monotonic_now_nanoseconds();
    add_relaxed(frames_without_kernel_timestamp, 1);
  }

  if (!frame.error()) {
    const WireBits bits = bits_on_wire(frame);
    add_relaxed(frames_received, 1);
    add_relaxed(bytes_received, frame.length);
    add_relaxed(minimum_bits_on_wire, bits.minimum);
    add_relaxed(maximum_bits_on_wire, bits.maximum);
  }
  return true;
}

BusStatistics SocketCanTransport::statistics() const noexcept {
  BusStatistics snapshot;
  snapshot.frames_sent = frames_sent.load(std::memory_order_relaxed);
  snapshot.bytes_sent = bytes_sent.load(std::memory_order_relaxed);
  snapshot.frames_received = frames_received.load(std::memory_order_relaxed);
  snapshot.bytes_received = bytes_received.load(std::memory_order_relaxed);
  snapshot.send_failures = send_failures.load(std::memory_order_relaxed);
  snapshot.receive_errors = receive_errors.load(std::memory_order_relaxed);
  snapshot.frames_without_kernel_timestamp = frames_without_kernel_timestamp.load(std::memory_order_relaxed);
  snapshot.minimum_bits_on_wire = minimum_bits_on_wire.load(std::memory_order_relaxed);
  snapshot.maximum_bits_on_wire = maximum_bits_on_wire.load(std::memory_order_relaxed);
  return snapshot;
}

bool SocketCanTransport::wait_readable(int timeout_milliseconds) noexcept {
  if (descriptor < 0) {
    return false;
  }
  pollfd poller{};
  poller.fd = descriptor;
  poller.events = POLLIN;
  for (;;) {
    const int ready = ::poll(&poller, 1, timeout_milliseconds);
    if (ready < 0 && errno == EINTR) {
      continue;
    }
    return ready > 0 && (poller.revents & POLLIN) != 0;
  }
}

}  // namespace robot_control::can
