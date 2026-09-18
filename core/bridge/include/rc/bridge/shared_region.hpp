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

/// Why create() or attach() failed. ok is success.
enum class RegionError {
  ok = 0,
  shm_open_failed,
  truncate_failed,
  map_failed,
  not_found,
  not_ready,     ///< region exists but the server has not finished initialising it: retry
  bad_magic,
  version_mismatch,
  size_mismatch,
  mlock_failed,
};

/// Short human-readable name for a RegionError, for logs.
[[nodiscard]] const char* to_string(RegionError error) noexcept;

/// RAII wrapper over one mapping. Move-only.
class SharedRegion {
 public:
  SharedRegion() = default;
  ~SharedRegion();
  SharedRegion(const SharedRegion&) = delete;
  SharedRegion& operator=(const SharedRegion&) = delete;
  /// Transfers the mapping; @p other is left invalid.
  SharedRegion(SharedRegion&& other) noexcept;
  /// Closes this mapping, then takes over @p other's.
  SharedRegion& operator=(SharedRegion&& other) noexcept;

  /// Server side: create (or replace) the region and construct the rings in it.
  /// Every header field is populated before the magic is published, so a
  /// client can never observe a partially initialised region.
  /// @param lock_memory mlock the mapping. The RT side touches it every cycle,
  ///        so a major fault here would be a multi-millisecond stall.
  [[nodiscard]] static RegionError create(const std::string& name, SharedRegion& out,
                                          std::uint32_t control_period_ns,
                                          std::uint32_t watchdog_timeout_cycles,
                                          bool lock_memory = true);

  /// Client side: attach to an existing region, validating magic, version and
  /// every struct size. Refuses rather than reinterpreting.
  [[nodiscard]] static RegionError attach(const std::string& name, SharedRegion& out,
                                          bool read_only = false);

  /// Remove the name so a later create() starts clean. Existing mappings stay
  /// valid until unmapped, exactly like unlink(2) on a file.
  static void unlink(const std::string& name) noexcept;

  /// True if a region is mapped.
  [[nodiscard]] bool valid() const noexcept { return region != nullptr; }
  /// The mapped region, or nullptr if not valid().
  [[nodiscard]] BridgeRegion* get() noexcept { return region; }
  /// @copydoc get()
  [[nodiscard]] const BridgeRegion* get() const noexcept { return region; }
  /// The shm name this was created with or attached to.
  [[nodiscard]] const std::string& name() const noexcept { return region_name; }
  /// True if this object created the region and will unlink it on close.
  [[nodiscard]] bool owns() const noexcept { return owner; }

 private:
  void close() noexcept;

  BridgeRegion* region = nullptr;
  std::size_t mapped_bytes = 0;
  std::string region_name;
  bool owner = false;
  bool locked = false;
};

}  // namespace rc::bridge
