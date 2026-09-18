#include "rc/telemetry/provenance.hpp"

#include <sys/resource.h>
#include <sys/utsname.h>
#include <unistd.h>

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <ctime>
#include <fstream>
#include <sstream>

#ifndef RC_GIT_SHA
#define RC_GIT_SHA "unknown"
#endif
#ifndef RC_GIT_DIRTY
#define RC_GIT_DIRTY "unknown"
#endif
#ifndef RC_BUILD_TYPE
#define RC_BUILD_TYPE "unknown"
#endif

namespace rc::telemetry {
namespace {

std::string trim(std::string text) {
  const auto not_space = [](unsigned char character) { return std::isspace(character) == 0; };
  text.erase(text.begin(), std::find_if(text.begin(), text.end(), not_space));
  text.erase(std::find_if(text.rbegin(), text.rend(), not_space).base(), text.end());
  return text;
}

/// First line of a sysfs/procfs file, or "" if unreadable. Absence is normal
/// (no cpufreq on a VM, no NVIDIA driver) and must not be an error.
std::string read_first_line(const char* path) {
  std::ifstream file(path);
  if (!file) {
    return {};
  }
  std::string line;
  std::getline(file, line);
  return trim(line);
}

std::string read_cpu_model() {
  std::ifstream file("/proc/cpuinfo");
  std::string line;
  while (std::getline(file, line)) {
    const auto colon = line.find(':');
    if (colon == std::string::npos) {
      continue;
    }
    const std::string key = trim(line.substr(0, colon));
    if (key == "model name" || key == "Model" || key == "Processor") {
      return trim(line.substr(colon + 1));
    }
  }
  return "unknown";
}

std::string read_nvidia_driver() {
  // e.g. "NVRM version: NVIDIA UNIX x86_64 Kernel Module  550.xx  ..."
  const std::string line = read_first_line("/proc/driver/nvidia/version");
  if (line.empty()) {
    return {};
  }
  return line;
}

std::string detect_preempt_model(const std::string& kernel_version) {
  // Newer kernels expose the model directly; fall back to the uname version
  // string, which carries PREEMPT_RT on an RT build.
  const std::string sysfs = read_first_line("/sys/kernel/preempt_model");
  if (!sysfs.empty()) {
    return sysfs;
  }
  if (kernel_version.find("PREEMPT_RT") != std::string::npos) {
    return "PREEMPT_RT";
  }
  if (kernel_version.find("PREEMPT_DYNAMIC") != std::string::npos) {
    return "PREEMPT_DYNAMIC";
  }
  if (kernel_version.find("PREEMPT") != std::string::npos) {
    return "PREEMPT";
  }
  return "none";
}

std::string iso8601_utc_now() {
  const std::time_t now = std::time(nullptr);
  std::tm calendar{};
  gmtime_r(&now, &calendar);
  char stamp[32];
  std::strftime(stamp, sizeof(stamp), "%Y-%m-%dT%H:%M:%SZ", &calendar);
  return stamp;
}

std::string json_escape(const std::string& text) {
  std::string out;
  out.reserve(text.size() + 8);
  for (const char character : text) {
    switch (character) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(character) < 0x20) {
          char escaped[8];
          std::snprintf(escaped, sizeof(escaped), "\\u%04x", static_cast<unsigned>(character));
          out += escaped;
        } else {
          out += character;
        }
    }
  }
  return out;
}

}  // namespace

Provenance Provenance::collect() {
  Provenance provenance;

  provenance.git_sha = RC_GIT_SHA;
  provenance.git_dirty = RC_GIT_DIRTY;
  provenance.build_type = RC_BUILD_TYPE;
  provenance.compiler = __VERSION__;
  provenance.build_time = __DATE__ " " __TIME__;

  utsname system{};
  if (::uname(&system) == 0) {
    provenance.hostname = system.nodename;
    provenance.kernel_release = system.release;
    provenance.kernel_version = system.version;
  }
  provenance.preempt_model = detect_preempt_model(provenance.kernel_version);
  // /sys/kernel/realtime is the authoritative marker where it exists.
  provenance.realtime_kernel =
      (read_first_line("/sys/kernel/realtime") == "1") || (provenance.preempt_model == "PREEMPT_RT");

  provenance.cpu_model = read_cpu_model();
  const long processors = ::sysconf(_SC_NPROCESSORS_ONLN);
  provenance.cpu_count = processors > 0 ? static_cast<unsigned>(processors) : 0;
  provenance.cpu_governor = read_first_line("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
  provenance.isolated_cpus = read_first_line("/sys/devices/system/cpu/isolated");
  provenance.nohz_full = read_first_line("/sys/devices/system/cpu/nohz_full");

  provenance.nvidia_driver = read_nvidia_driver();

  rlimit limit{};
  auto describe = [](rlim_t value) -> std::string {
    if (value == RLIM_INFINITY) return "unlimited";
    char text[32];
    std::snprintf(text, sizeof(text), "%.1fMiB", static_cast<double>(value) / 1048576.0);
    return text;
  };
  if (::getrlimit(RLIMIT_MEMLOCK, &limit) == 0) {
    provenance.memlock_limit = describe(limit.rlim_cur) + "/" + describe(limit.rlim_max);
  }
  if (::getrlimit(RLIMIT_RTPRIO, &limit) == 0) {
    provenance.rtprio_limit = limit.rlim_max == RLIM_INFINITY ? "99" : std::to_string(limit.rlim_max);
  }
  provenance.wall_clock = iso8601_utc_now();
  return provenance;
}

std::string Provenance::to_json() const {
  std::ostringstream out;
  const auto quoted = [](const std::string& value) { return '"' + json_escape(value) + '"'; };
  out << "{\n";
  out << "  \"build\": {\n";
  out << "    \"git_sha\": " << quoted(git_sha) << ",\n";
  out << "    \"git_state\": " << quoted(git_dirty) << ",\n";
  out << "    \"build_type\": " << quoted(build_type) << ",\n";
  out << "    \"compiler\": " << quoted(compiler) << ",\n";
  out << "    \"built_at\": " << quoted(build_time) << "\n";
  out << "  },\n";
  out << "  \"machine\": {\n";
  out << "    \"hostname\": " << quoted(hostname) << ",\n";
  out << "    \"kernel_release\": " << quoted(kernel_release) << ",\n";
  out << "    \"preempt_model\": " << quoted(preempt_model) << ",\n";
  out << "    \"realtime_kernel\": " << (realtime_kernel ? "true" : "false") << ",\n";
  out << "    \"cpu_model\": " << quoted(cpu_model) << ",\n";
  out << "    \"cpu_count\": " << cpu_count << ",\n";
  out << "    \"cpu_governor\": " << quoted(cpu_governor) << ",\n";
  out << "    \"isolated_cpus\": " << quoted(isolated_cpus) << ",\n";
  out << "    \"nohz_full\": " << quoted(nohz_full) << ",\n";
  out << "    \"memlock_limit\": " << quoted(memlock_limit) << ",\n";
  out << "    \"rtprio_limit\": " << quoted(rtprio_limit) << ",\n";
  out << "    \"nvidia_driver\": " << quoted(nvidia_driver) << ",\n";
  out << "    \"gpu_workload_running\": " << (gpu_workload_running ? "true" : "false") << "\n";
  out << "  },\n";
  out << "  \"run\": {\n";
  out << "    \"wall_clock\": " << quoted(wall_clock) << ",\n";
  out << "    \"label\": " << quoted(label) << ",\n";
  out << "    \"rt_notes\": " << quoted(rt_notes) << ",\n";
  out << "    \"can_interface\": " << quoted(can_interface) << ",\n";
  out << "    \"can_bitrate\": " << can_bitrate << ",\n";
  out << "    \"control_rate_hz\": " << control_rate_hz << "\n";
  out << "  }\n";
  out << "}\n";
  return out.str();
}

std::string Provenance::to_summary() const {
  std::ostringstream out;
  out << "  build     " << git_sha << " (" << git_dirty << "), " << build_type << '\n';
  out << "  kernel    " << kernel_release << "  [" << preempt_model << "]\n";
  out << "  cpu       " << cpu_model << " x" << cpu_count;
  if (!cpu_governor.empty()) {
    out << ", governor=" << cpu_governor;
  }
  out << '\n';
  if (!isolated_cpus.empty()) {
    out << "  isolated  " << isolated_cpus << '\n';
  }
  out << "  limits    memlock " << memlock_limit << ", rtprio " << rtprio_limit << '\n';
  if (!nvidia_driver.empty()) {
    out << "  gpu       " << nvidia_driver
       << (gpu_workload_running ? "  [inference RUNNING]" : "  [idle]") << '\n';
  }
  return out.str();
}

bool Provenance::suitable_as_baseline(std::string& why_not) const {
  std::ostringstream out;
  bool ok = true;
  if (git_dirty != "clean") {
    out << "working tree is " << git_dirty << " (baseline would not be reproducible); ";
    ok = false;
  }
  if (git_sha == "unknown") {
    out << "git SHA unknown; ";
    ok = false;
  }
  if (build_type != "RelWithDebInfo" && build_type != "Release") {
    out << "build type is " << build_type << " (timing from a debug build is fiction); ";
    ok = false;
  }
  why_not = out.str();
  return ok;
}

}  // namespace rc::telemetry
