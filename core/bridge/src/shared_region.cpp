#include "rc/bridge/shared_region.hpp"

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cerrno>
#include <ctime>
#include <new>
#include <utility>

namespace rc::bridge {
namespace {

constexpr std::size_t kRegionSize = sizeof(BridgeRegion);

std::int64_t monotonic_ns() noexcept {
  timespec ts{};
  ::clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<std::int64_t>(ts.tv_sec) * 1'000'000'000 + ts.tv_nsec;
}

}  // namespace

const char* to_string(RegionError e) noexcept {
  switch (e) {
    case RegionError::kOk: return "ok";
    case RegionError::kShmOpenFailed: return "shm_open failed";
    case RegionError::kTruncateFailed: return "ftruncate failed";
    case RegionError::kMapFailed: return "mmap failed";
    case RegionError::kNotFound: return "region does not exist (is the RT core running?)";
    case RegionError::kBadMagic: return "bad magic (not a bridge region, or corrupt)";
    case RegionError::kVersionMismatch: return "layout version mismatch (rebuild the client)";
    case RegionError::kSizeMismatch: return "record size mismatch (rebuild the client)";
    case RegionError::kMlockFailed: return "mlock failed (raise RLIMIT_MEMLOCK)";
  }
  return "unknown";
}

SharedRegion::~SharedRegion() { close(); }

SharedRegion::SharedRegion(SharedRegion&& other) noexcept
    : region_(other.region_),
      mapped_size_(other.mapped_size_),
      name_(std::move(other.name_)),
      owner_(other.owner_),
      locked_(other.locked_) {
  other.region_ = nullptr;
  other.mapped_size_ = 0;
  other.owner_ = false;
  other.locked_ = false;
}

SharedRegion& SharedRegion::operator=(SharedRegion&& other) noexcept {
  if (this != &other) {
    close();
    region_ = other.region_;
    mapped_size_ = other.mapped_size_;
    name_ = std::move(other.name_);
    owner_ = other.owner_;
    locked_ = other.locked_;
    other.region_ = nullptr;
    other.mapped_size_ = 0;
    other.owner_ = false;
    other.locked_ = false;
  }
  return *this;
}

void SharedRegion::close() noexcept {
  if (region_ != nullptr) {
    if (locked_) {
      ::munlock(region_, mapped_size_);
    }
    ::munmap(region_, mapped_size_);
    region_ = nullptr;
  }
  mapped_size_ = 0;
  locked_ = false;
  // Deliberately NOT unlinking here: a server restart should be able to hand
  // over, and a client destructor must never remove the server's region.
}

RegionError SharedRegion::create(const std::string& name, SharedRegion& out, bool lock_memory) {
  // Replace any stale region from a previous run. A crashed server leaves its
  // shm behind; reusing it would inherit whatever indices it died with.
  ::shm_unlink(name.c_str());

  const int fd = ::shm_open(name.c_str(), O_CREAT | O_EXCL | O_RDWR, S_IRUSR | S_IWUSR);
  if (fd < 0) {
    return RegionError::kShmOpenFailed;
  }
  if (::ftruncate(fd, static_cast<off_t>(kRegionSize)) != 0) {
    ::close(fd);
    ::shm_unlink(name.c_str());
    return RegionError::kTruncateFailed;
  }
  void* addr = ::mmap(nullptr, kRegionSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0);
  ::close(fd);  // the mapping keeps the object alive; the fd is no longer needed
  if (addr == MAP_FAILED) {
    ::shm_unlink(name.c_str());
    return RegionError::kMapFailed;
  }

  // ftruncate zero-fills, so the atomics are already at their zero value; the
  // placement-new makes that explicit and runs the members' constructors.
  auto* region = new (addr) BridgeRegion{};
  region->snapshot.reset();
  region->telemetry.reset();
  region->commands.reset();

  BridgeHeader& h = region->header;
  h.magic = kMagic;
  h.layout_version = kLayoutVersion;
  h.header_size = static_cast<std::uint32_t>(sizeof(BridgeHeader));
  h.telemetry_record_size = static_cast<std::uint32_t>(sizeof(rc::telemetry::TelemetryRecord));
  h.command_record_size = static_cast<std::uint32_t>(sizeof(CommandRecord));
  h.snapshot_size = static_cast<std::uint32_t>(sizeof(rc::telemetry::StateSnapshot));
  h.telemetry_capacity = static_cast<std::uint32_t>(kTelemetryCapacity);
  h.command_capacity = static_cast<std::uint32_t>(kCommandCapacity);
  h.server_start_ns = monotonic_ns();
  h.server_pid = static_cast<std::uint64_t>(::getpid());
  h.server_state.store(static_cast<std::uint32_t>(ServerState::kStarting),
                       std::memory_order_release);

  out.close();
  out.region_ = region;
  out.mapped_size_ = kRegionSize;
  out.name_ = name;
  out.owner_ = true;

  if (lock_memory) {
    if (::mlock(addr, kRegionSize) == 0) {
      out.locked_ = true;
    } else {
      // Not fatal: report it and let the caller decide. Silently running an RT
      // loop over pageable shared memory is the failure we are avoiding.
      return RegionError::kMlockFailed;
    }
  }
  return RegionError::kOk;
}

RegionError SharedRegion::attach(const std::string& name, SharedRegion& out, bool read_only) {
  const int flags = read_only ? O_RDONLY : O_RDWR;
  const int fd = ::shm_open(name.c_str(), flags, 0);
  if (fd < 0) {
    return (errno == ENOENT) ? RegionError::kNotFound : RegionError::kShmOpenFailed;
  }

  struct stat st {};
  if (::fstat(fd, &st) != 0 || static_cast<std::size_t>(st.st_size) < kRegionSize) {
    ::close(fd);
    return RegionError::kSizeMismatch;
  }

  const int prot = read_only ? PROT_READ : (PROT_READ | PROT_WRITE);
  void* addr = ::mmap(nullptr, kRegionSize, prot, MAP_SHARED, fd, 0);
  ::close(fd);
  if (addr == MAP_FAILED) {
    return RegionError::kMapFailed;
  }

  auto* region = static_cast<BridgeRegion*>(addr);
  const BridgeHeader& h = region->header;

  // Validate before trusting anything else in the mapping. Order matters: magic
  // first (is this even ours?), then version, then the sizes that would cause
  // silent misinterpretation rather than a crash.
  auto reject = [&](RegionError e) {
    ::munmap(addr, kRegionSize);
    return e;
  };
  if (h.magic != kMagic) {
    return reject(RegionError::kBadMagic);
  }
  if (h.layout_version != kLayoutVersion) {
    return reject(RegionError::kVersionMismatch);
  }
  if (h.header_size != sizeof(BridgeHeader) ||
      h.telemetry_record_size != sizeof(rc::telemetry::TelemetryRecord) ||
      h.command_record_size != sizeof(CommandRecord) ||
      h.snapshot_size != sizeof(rc::telemetry::StateSnapshot) ||
      h.telemetry_capacity != kTelemetryCapacity ||
      h.command_capacity != kCommandCapacity) {
    return reject(RegionError::kSizeMismatch);
  }

  out.close();
  out.region_ = region;
  out.mapped_size_ = kRegionSize;
  out.name_ = name;
  out.owner_ = false;
  return RegionError::kOk;
}

void SharedRegion::unlink(const std::string& name) noexcept { ::shm_unlink(name.c_str()); }

}  // namespace rc::bridge
