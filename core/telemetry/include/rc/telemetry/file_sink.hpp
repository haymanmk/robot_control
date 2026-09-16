#pragma once

/// @file file_sink.hpp
/// Drains the telemetry ring to disk on a non-RT thread.
///
/// Format: a JSON provenance sidecar (`<name>.json`) beside a flat array of
/// raw TelemetryRecords (`<name>.bin`). Deliberately not CSV and not a database:
///
///   * the record is already a fixed-size POD, so the file *is* the array —
///     numpy reads it with one `np.fromfile(..., dtype=...)`, zero parsing;
///   * a text format at 500 Hz costs formatting work per record for no benefit
///     anyone reads;
///   * provenance lives in a separate readable file so a human can answer
///     "what machine was this?" without a decoder.
///
/// Nothing here is callable from the cyclic path — it allocates, it writes, it
/// blocks. That is the point of it being a different thread.

#include <atomic>
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

#include "rc/telemetry/provenance.hpp"
#include "rc/telemetry/record.hpp"

namespace rc::bridge {
class BridgeServer;
}

namespace rc::telemetry {

/// Drains telemetry from a BridgeServer to a binary file on a non-RT thread.
/// Writes a JSON sidecar with the Provenance and the numpy dtype so a run is
/// reproducible and loadable without this code.
class FileSink {
 public:
  FileSink() = default;
  ~FileSink();
  FileSink(const FileSink&) = delete;
  FileSink& operator=(const FileSink&) = delete;

  /// Opens `<path_prefix>.bin` and writes `<path_prefix>.json`.
  [[nodiscard]] bool open(const std::string& path_prefix, const Provenance& provenance);

  /// Starts a background thread draining @p server every @p poll_interval_ms.
  /// The thread is ordinary (non-RT) and deliberately low priority: it must
  /// never compete with the control loop.
  void start(rc::bridge::BridgeServer& server, unsigned poll_interval_ms = 10);

  /// Stops the thread and flushes. Safe to call twice.
  void stop();

  [[nodiscard]] std::uint64_t records_written() const noexcept {
    return written_.load(std::memory_order_relaxed);
  }
  [[nodiscard]] bool running() const noexcept { return running_.load(std::memory_order_acquire); }
  [[nodiscard]] const std::string& bin_path() const noexcept { return bin_path_; }

  /// Synchronous drain, for tests and for a final flush after stop().
  /// The telemetry ring is single-consumer: this refuses (returns 0) while the
  /// background thread owns the ring, because two poppers on an SPSC ring
  /// corrupt the head index. Call stop() first.
  std::size_t drain_once(rc::bridge::BridgeServer& server);

 private:
  void run(rc::bridge::BridgeServer& server, unsigned poll_interval_ms);
  /// The actual drain, used by both entry points. No ownership check.
  std::size_t drain_impl(rc::bridge::BridgeServer& server);

  std::FILE* file_ = nullptr;
  std::string bin_path_;
  std::vector<TelemetryRecord> buffer_;  ///< allocated once, in open()
  std::thread thread_;
  std::atomic<bool> running_{false};
  std::atomic<bool> stop_requested_{false};
  std::atomic<std::uint64_t> written_{0};
};

/// numpy dtype matching TelemetryRecord, so analysis code cannot drift from the
/// C++ struct by hand-transcription. Emitted into the JSON sidecar.
[[nodiscard]] std::string numpy_dtype_json();

}  // namespace rc::telemetry
