#pragma once

/// @file shared_region.hpp
/// Create, attach to, and tear down the POSIX shared-memory region.
///
/// Ownership is asymmetric on purpose. The **server creates** the region and
/// initialises it; a **client attaches** to one that already exists and
/// validates every size in the header before touching a byte. A client must
/// never create the region: if it did, starting the client first would produce
/// a zero-filled region that looks valid and is not.

#include <cstddef>
#include <string>

#include "rc/bridge/layout.hpp"

namespace rc::bridge {

enum class RegionError {
  kOk = 0,
  kShmOpenFailed,
  kTruncateFailed,
  kMapFailed,
  kNotFound,
  kBadMagic,
  kVersionMismatch,
  kSizeMismatch,
  kMlockFailed,
};

[[nodiscard]] const char* to_string(RegionError e) noexcept;

/// RAII wrapper over one mapping. Move-only.
class SharedRegion {
 public:
  SharedRegion() = default;
  ~SharedRegion();
  SharedRegion(const SharedRegion&) = delete;
  SharedRegion& operator=(const SharedRegion&) = delete;
  SharedRegion(SharedRegion&& other) noexcept;
  SharedRegion& operator=(SharedRegion&& other) noexcept;

  /// Server side: create (or replace) the region and construct the rings in it.
  /// @param lock_memory mlock the mapping. The RT side touches it every cycle,
  ///        so a major fault here would be a multi-millisecond stall.
  [[nodiscard]] static RegionError create(const std::string& name, SharedRegion& out,
                                          bool lock_memory = true);

  /// Client side: attach to an existing region, validating magic, version and
  /// every struct size. Refuses rather than reinterpreting.
  [[nodiscard]] static RegionError attach(const std::string& name, SharedRegion& out,
                                          bool read_only = false);

  /// Remove the name so a later create() starts clean. Existing mappings stay
  /// valid until unmapped, exactly like unlink(2) on a file.
  static void unlink(const std::string& name) noexcept;

  [[nodiscard]] bool valid() const noexcept { return region_ != nullptr; }
  [[nodiscard]] BridgeRegion* get() noexcept { return region_; }
  [[nodiscard]] const BridgeRegion* get() const noexcept { return region_; }
  [[nodiscard]] const std::string& name() const noexcept { return name_; }
  [[nodiscard]] bool owns() const noexcept { return owner_; }

 private:
  void close() noexcept;

  BridgeRegion* region_ = nullptr;
  std::size_t mapped_size_ = 0;
  std::string name_;
  bool owner_ = false;
  bool locked_ = false;
};

}  // namespace rc::bridge
