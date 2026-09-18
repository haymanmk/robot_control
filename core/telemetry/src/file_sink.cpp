#include "rc/telemetry/file_sink.hpp"

#include <chrono>
#include <cstdio>
#include <fstream>

#include "rc/bridge/server.hpp"

namespace rc::telemetry {
namespace {
/// One drain pulls at most this many records. Bounds the time spent holding the
/// consumer side, and bounds the buffer we allocate once up front.
constexpr std::size_t drain_batch = 1024;
}  // namespace

FileSink::~FileSink() {
  stop();
  if (file != nullptr) {
    std::fclose(file);
    file = nullptr;
  }
}

std::string numpy_dtype_json() {
  // Field order and sizes must mirror TelemetryRecord exactly. The static_assert
  // on sizeof(TelemetryRecord) in record.hpp is what keeps this honest: change
  // the struct without changing this and the assert fires first.
  return
      R"({"names": ["cycle","deadline_ns","wake_ns","exec_ns","can_tx_ns","can_rx_ns",)"
      R"("joint_count","flags","fault_mask","mode",)"
      R"("cmd_pos","cmd_vel","cmd_tau","meas_pos","meas_vel","meas_tau"],)"
      R"("formats": ["<u8","<i8","<i8","<i8","<i8","<i8","<u4","<u4","<u4","<u4",)"
      R"("(8,)<f4","(8,)<f4","(8,)<f4","(8,)<f4","(8,)<f4","(8,)<f4"],)"
      R"("itemsize": 256})";
}

bool FileSink::open(const std::string& path_prefix, const Provenance& provenance) {
  output_path = path_prefix + ".bin";
  file = std::fopen(output_path.c_str(), "wb");
  if (file == nullptr) {
    return false;
  }

  std::ofstream meta(path_prefix + ".json");
  if (!meta) {
    std::fclose(file);
    file = nullptr;
    return false;
  }
  // Splice the dtype into the provenance object so one file fully describes how
  // to read the other.
  std::string json = provenance.to_json();
  const auto last_brace = json.rfind('}');
  if (last_brace != std::string::npos) {
    json.insert(last_brace, ",\n  \"record_dtype\": " + numpy_dtype_json() + "\n");
  }
  meta << json;

  // Allocate the drain buffer once, here, so the draining thread never does.
  buffer.resize(drain_batch);
  written.store(0, std::memory_order_relaxed);
  return true;
}

std::size_t FileSink::drain_once(rc::bridge::BridgeServer& server) {
  if (worker_running.load(std::memory_order_acquire)) {
    return 0;  // the worker owns the consumer side; see header
  }
  return drain_impl(server);
}

std::size_t FileSink::drain_impl(rc::bridge::BridgeServer& server) {
  if (file == nullptr || buffer.empty()) {
    return 0;
  }
  std::size_t total = 0;
  for (;;) {
    const std::size_t drained = server.drain(buffer.data(), buffer.size());
    if (drained == 0) {
      break;
    }
    std::fwrite(buffer.data(), sizeof(TelemetryRecord), drained, file);
    total += drained;
    if (drained < buffer.size()) {
      break;  // ring is drained
    }
  }
  if (total > 0) {
    written.fetch_add(total, std::memory_order_relaxed);
  }
  return total;
}

void FileSink::run(rc::bridge::BridgeServer& server, unsigned poll_interval_milliseconds) {
  while (!stop_requested.load(std::memory_order_acquire)) {
    drain_impl(server);
    std::this_thread::sleep_for(std::chrono::milliseconds(poll_interval_milliseconds));
  }
  drain_impl(server);  // final sweep: whatever the loop published on its way out
  if (file != nullptr) {
    std::fflush(file);
  }
}

void FileSink::start(rc::bridge::BridgeServer& server, unsigned poll_interval_milliseconds) {
  if (worker_running.load(std::memory_order_acquire)) {
    return;
  }
  stop_requested.store(false, std::memory_order_release);
  // Ownership is handed to the worker *before* it exists, on this thread, so
  // there is no window in which both a caller and the worker believe they may
  // pop. The worker thread never touches the `worker` handle -- it is being
  // move-assigned here while the thread is already running, and reading it
  // from the thread is a race ThreadSanitizer caught in an earlier version.
  worker_running.store(true, std::memory_order_release);
  worker = std::thread(&FileSink::run, this, std::ref(server), poll_interval_milliseconds);
}

void FileSink::stop() {
  stop_requested.store(true, std::memory_order_release);
  if (worker.joinable()) {
    worker.join();
  }
  worker_running.store(false, std::memory_order_release);
}

}  // namespace rc::telemetry
