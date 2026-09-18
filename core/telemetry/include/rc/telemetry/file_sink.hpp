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

  /// Starts a background thread draining @p server every @p poll_interval_milliseconds.
  /// The thread is ordinary (non-RT) and deliberately low priority: it must
  /// never compete with the control loop.
  void start(rc::bridge::BridgeServer& server, unsigned poll_interval_milliseconds = 10);

  /// Stops the thread and flushes. Safe to call twice.
  void stop();

  /// Records written to the .bin file so far.
  [[nodiscard]] std::uint64_t records_written() const noexcept {
    return written.load(std::memory_order_relaxed);
  }
  /// True while the drain thread started by start() is alive.
  [[nodiscard]] bool running() const noexcept { return worker_running.load(std::memory_order_acquire); }
  /// Path of the .bin file opened by open().
  [[nodiscard]] const std::string& binary_path() const noexcept { return output_path; }

  /// Synchronous drain, for tests and for a final flush after stop().
  /// The telemetry ring is single-consumer: this refuses (returns 0) while the
  /// background thread owns the ring, because two poppers on a single-consumer ring
  /// corrupt the head index. Call stop() first.
  std::size_t drain_once(rc::bridge::BridgeServer& server);

 private:
  void run(rc::bridge::BridgeServer& server, unsigned poll_interval_milliseconds);
  /// The actual drain, used by both entry points. No ownership check.
  std::size_t drain_unchecked(rc::bridge::BridgeServer& server);

  std::FILE* file = nullptr;
  std::string output_path;
  std::vector<TelemetryRecord> buffer;  ///< allocated once, in open()
  std::thread worker;
  std::atomic<bool> worker_running{false};
  std::atomic<bool> stop_requested{false};
  std::atomic<std::uint64_t> written{0};
};

/// numpy dtype matching TelemetryRecord, so analysis code cannot drift from the
/// C++ struct by hand-transcription. Emitted into the JSON sidecar.
[[nodiscard]] std::string numpy_dtype_json();

}  // namespace rc::telemetry
