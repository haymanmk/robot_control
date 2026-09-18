#include "rc/bridge/shared_region.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <new>
#include <utility>

#include "rc/rt/clock.hpp"

namespace rc::bridge {
namespace {
constexpr std::size_t region_bytes = sizeof(BridgeRegion);
}  // namespace

const char* to_string(RegionError error) noexcept {
  switch (error) {
    case RegionError::ok: return "ok";
    case RegionError::shm_open_failed: return "shm_open failed";
    case RegionError::truncate_failed: return "ftruncate failed";
    case RegionError::map_failed: return "mmap failed";
    case RegionError::not_found: return "region does not exist (is the RT core running?)";
    case RegionError::not_ready: return "region exists but is still initialising (retry)";
    case RegionError::bad_magic: return "bad magic (not a bridge region, or corrupt)";
    case RegionError::version_mismatch: return "layout version mismatch (rebuild the client)";
    case RegionError::size_mismatch: return "record size mismatch (rebuild the client)";
    case RegionError::mlock_failed: return "mlock failed (raise RLIMIT_MEMLOCK)";
  }
  return "unknown";
}

SharedRegion::~SharedRegion() { close(); }

SharedRegion::SharedRegion(SharedRegion&& other) noexcept
    : region(other.region),
      mapped_bytes(other.mapped_bytes),
      region_name(std::move(other.region_name)),
      owner(other.owner),
      locked(other.locked) {
  other.region = nullptr;
  other.mapped_bytes = 0;
  other.owner = false;
  other.locked = false;
}

SharedRegion& SharedRegion::operator=(SharedRegion&& other) noexcept {
  if (this != &other) {
    close();
    region = other.region;
    mapped_bytes = other.mapped_bytes;
    region_name = std::move(other.region_name);
    owner = other.owner;
    locked = other.locked;
    other.region = nullptr;
    other.mapped_bytes = 0;
    other.owner = false;
    other.locked = false;
  }
  return *this;
}

void SharedRegion::close() noexcept {
  if (region != nullptr) {
    if (locked) {
      ::munlock(region, mapped_bytes);
    }
    ::munmap(region, mapped_bytes);
    region = nullptr;
  }
  mapped_bytes = 0;
  locked = false;
  // Deliberately NOT unlinking here: a server restart should be able to hand
  // over, and a client destructor must never remove the server's region.
}

RegionError SharedRegion::create(const std::string& name, SharedRegion& out,
                                 std::uint32_t control_period_nanoseconds,
                                 std::uint32_t watchdog_timeout_cycles, bool lock_memory) {
  // Replace any stale region from a previous run. A crashed server leaves its
  // shm behind; reusing it would inherit whatever indices it died with.
  ::shm_unlink(name.c_str());

  const int descriptor = ::shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
  if (descriptor < 0) {
    return RegionError::shm_open_failed;
  }
  if (::ftruncate(descriptor, static_cast<off_t>(region_bytes)) != 0) {
    ::close(descriptor);
    ::shm_unlink(name.c_str());
    return RegionError::truncate_failed;
  }
  void* address = ::mmap(nullptr, region_bytes, PROT_READ | PROT_WRITE, MAP_SHARED, descriptor, 0);
  ::close(descriptor);  // the mapping keeps the object alive; the descriptor is no longer needed
  if (address == MAP_FAILED) {
    ::shm_unlink(name.c_str());
    return RegionError::map_failed;
  }

  // ftruncate zero-fills, so magic is already 0 == "not ready" and any client
  // that races us here gets not_ready rather than garbage.
  auto* mapped = new (address) BridgeRegion{};
  mapped->snapshot.reset();
  mapped->telemetry.reset();
  mapped->commands.reset();

  BridgeHeader& header = mapped->header;
  header.layout_version = current_layout_version;
  header.header_size = static_cast<std::uint32_t>(sizeof(BridgeHeader));
  header.telemetry_record_size = static_cast<std::uint32_t>(sizeof(rc::telemetry::TelemetryRecord));
  header.command_record_size = static_cast<std::uint32_t>(sizeof(CommandRecord));
  header.snapshot_size = static_cast<std::uint32_t>(sizeof(rc::telemetry::StateSnapshot));
  header.telemetry_capacity = static_cast<std::uint32_t>(telemetry_ring_capacity);
  header.command_capacity = static_cast<std::uint32_t>(command_ring_capacity);
  header.control_period_nanoseconds = control_period_nanoseconds;
  header.server_start_nanoseconds = rc::rt::monotonic_now().count();
  header.server_process_id = static_cast<std::uint64_t>(::getpid());
  header.watchdog_timeout_cycles.store(watchdog_timeout_cycles, std::memory_order_relaxed);
  header.server_state.store(static_cast<std::uint32_t>(ServerState::starting),
                       std::memory_order_relaxed);

  out.close();
  out.region = mapped;
  out.mapped_bytes = region_bytes;
  out.region_name = name;
  out.owner = true;

  RegionError result = RegionError::ok;
  if (lock_memory) {
    if (::mlock(address, region_bytes) == 0) {
      out.locked = true;
    } else {
      // Not fatal: the region is still usable. Reported so the caller can decide
      // whether a pageable bridge is acceptable, rather than silently running an
      // RT loop over memory that can fault.
      result = RegionError::mlock_failed;
    }
  }

  // Publish. Everything a client validates or reads is complete before this
  // store; the release pairs with the acquire load in attach().
  header.magic.store(region_magic, std::memory_order_release);
  return result;
}

RegionError SharedRegion::attach(const std::string& name, SharedRegion& out, bool read_only) {
  const int flags = read_only ? O_RDONLY : O_RDWR;
  const int descriptor = ::shm_open(name.c_str(), flags, 0);
  if (descriptor < 0) {
    return (errno == ENOENT) ? RegionError::not_found : RegionError::shm_open_failed;
  }

  struct stat file_info {};
  if (::fstat(descriptor, &file_info) != 0) {
    ::close(descriptor);
    return RegionError::shm_open_failed;
  }
  if (file_info.st_size == 0) {
    ::close(descriptor);
    return RegionError::not_ready;  // created, not yet ftruncated
  }
  if (static_cast<std::size_t>(file_info.st_size) < region_bytes) {
    ::close(descriptor);
    return RegionError::size_mismatch;
  }

  const int protection = read_only ? PROT_READ : (PROT_READ | PROT_WRITE);
  void* address = ::mmap(nullptr, region_bytes, protection, MAP_SHARED, descriptor, 0);
  ::close(descriptor);
  if (address == MAP_FAILED) {
    return RegionError::map_failed;
  }

  auto* mapped = static_cast<BridgeRegion*>(address);
  const BridgeHeader& header = mapped->header;

  auto reject = [&](RegionError error) {
    ::munmap(address, region_bytes);
    return error;
  };

  // Magic first, with acquire: it is the server's publish gate, and nothing
  // else in the header is trustworthy until it reads back as ours.
  const std::uint64_t observed_magic = header.magic.load(std::memory_order_acquire);
  if (observed_magic == 0) {
    return reject(RegionError::not_ready);
  }
  if (observed_magic != region_magic) {
    return reject(RegionError::bad_magic);
  }
  if (header.layout_version != current_layout_version) {
    return reject(RegionError::version_mismatch);
  }
  if (header.header_size != sizeof(BridgeHeader) ||
      header.telemetry_record_size != sizeof(rc::telemetry::TelemetryRecord) ||
      header.command_record_size != sizeof(CommandRecord) ||
      header.snapshot_size != sizeof(rc::telemetry::StateSnapshot) ||
      header.telemetry_capacity != telemetry_ring_capacity ||
      header.command_capacity != command_ring_capacity) {
    return reject(RegionError::size_mismatch);
  }

  out.close();
  out.region = mapped;
  out.mapped_bytes = region_bytes;
  out.region_name = name;
  out.owner = false;
  return RegionError::ok;
}

void SharedRegion::unlink(const std::string& name) noexcept { ::shm_unlink(name.c_str()); }

}  // namespace rc::bridge
