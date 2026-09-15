#include "rc/telemetry/provenance.hpp"

#include <sys/utsname.h>
#include <unistd.h>

#include <algorithm>
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

std::string trim(std::string s) {
  const auto not_space = [](unsigned char c) { return std::isspace(c) == 0; };
  s.erase(s.begin(), std::find_if(s.begin(), s.end(), not_space));
  s.erase(std::find_if(s.rbegin(), s.rend(), not_space).base(), s.end());
  return s;
}

/// First line of a sysfs/procfs file, or "" if unreadable. Absence is normal
/// (no cpufreq on a VM, no NVIDIA driver) and must not be an error.
std::string read_first_line(const char* path) {
  std::ifstream f(path);
  if (!f) {
    return {};
  }
  std::string line;
  std::getline(f, line);
  return trim(line);
}

std::string read_cpu_model() {
  std::ifstream f("/proc/cpuinfo");
  std::string line;
  while (std::getline(f, line)) {
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
  const std::time_t t = std::time(nullptr);
  std::tm tm{};
  gmtime_r(&t, &tm);
  char buf[32];
  std::strftime(buf, sizeof(buf), "%Y-%m-%dT%H:%M:%SZ", &tm);
  return buf;
}

std::string json_escape(const std::string& s) {
  std::string out;
  out.reserve(s.size() + 8);
  for (const char c : s) {
    switch (c) {
      case '"': out += "\\\""; break;
      case '\\': out += "\\\\"; break;
      case '\n': out += "\\n"; break;
      case '\r': out += "\\r"; break;
      case '\t': out += "\\t"; break;
      default:
        if (static_cast<unsigned char>(c) < 0x20) {
          char buf[8];
          std::snprintf(buf, sizeof(buf), "\\u%04x", static_cast<unsigned>(c));
          out += buf;
        } else {
          out += c;
        }
    }
  }
  return out;
}

}  // namespace

Provenance Provenance::collect() {
  Provenance p;

  p.git_sha = RC_GIT_SHA;
  p.git_dirty = RC_GIT_DIRTY;
  p.build_type = RC_BUILD_TYPE;
  p.compiler = __VERSION__;
  p.build_time = __DATE__ " " __TIME__;

  utsname u{};
  if (::uname(&u) == 0) {
    p.hostname = u.nodename;
    p.kernel_release = u.release;
    p.kernel_version = u.version;
  }
  p.preempt_model = detect_preempt_model(p.kernel_version);
  // /sys/kernel/realtime is the authoritative marker where it exists.
  p.realtime_kernel =
      (read_first_line("/sys/kernel/realtime") == "1") || (p.preempt_model == "PREEMPT_RT");

  p.cpu_model = read_cpu_model();
  const long n = ::sysconf(_SC_NPROCESSORS_ONLN);
  p.cpu_count = n > 0 ? static_cast<unsigned>(n) : 0;
  p.cpu_governor = read_first_line("/sys/devices/system/cpu/cpu0/cpufreq/scaling_governor");
  p.isolated_cpus = read_first_line("/sys/devices/system/cpu/isolated");
  p.nohz_full = read_first_line("/sys/devices/system/cpu/nohz_full");

  p.nvidia_driver = read_nvidia_driver();
  p.wall_clock = iso8601_utc_now();
  return p;
}

std::string Provenance::to_json() const {
  std::ostringstream os;
  const auto s = [](const std::string& v) { return '"' + json_escape(v) + '"'; };
  os << "{\n";
  os << "  \"build\": {\n";
  os << "    \"git_sha\": " << s(git_sha) << ",\n";
  os << "    \"git_state\": " << s(git_dirty) << ",\n";
  os << "    \"build_type\": " << s(build_type) << ",\n";
  os << "    \"compiler\": " << s(compiler) << ",\n";
  os << "    \"built_at\": " << s(build_time) << "\n";
  os << "  },\n";
  os << "  \"machine\": {\n";
  os << "    \"hostname\": " << s(hostname) << ",\n";
  os << "    \"kernel_release\": " << s(kernel_release) << ",\n";
  os << "    \"preempt_model\": " << s(preempt_model) << ",\n";
  os << "    \"realtime_kernel\": " << (realtime_kernel ? "true" : "false") << ",\n";
  os << "    \"cpu_model\": " << s(cpu_model) << ",\n";
  os << "    \"cpu_count\": " << cpu_count << ",\n";
  os << "    \"cpu_governor\": " << s(cpu_governor) << ",\n";
  os << "    \"isolated_cpus\": " << s(isolated_cpus) << ",\n";
  os << "    \"nohz_full\": " << s(nohz_full) << ",\n";
  os << "    \"nvidia_driver\": " << s(nvidia_driver) << ",\n";
  os << "    \"gpu_workload_running\": " << (gpu_workload_running ? "true" : "false") << "\n";
  os << "  },\n";
  os << "  \"run\": {\n";
  os << "    \"wall_clock\": " << s(wall_clock) << ",\n";
  os << "    \"label\": " << s(label) << ",\n";
  os << "    \"rt_notes\": " << s(rt_notes) << ",\n";
  os << "    \"can_interface\": " << s(can_interface) << ",\n";
  os << "    \"can_bitrate\": " << can_bitrate << ",\n";
  os << "    \"control_rate_hz\": " << control_rate_hz << "\n";
  os << "  }\n";
  os << "}\n";
  return os.str();
}

std::string Provenance::to_summary() const {
  std::ostringstream os;
  os << "  build     " << git_sha << " (" << git_dirty << "), " << build_type << '\n';
  os << "  kernel    " << kernel_release << "  [" << preempt_model << "]\n";
  os << "  cpu       " << cpu_model << " x" << cpu_count;
  if (!cpu_governor.empty()) {
    os << ", governor=" << cpu_governor;
  }
  os << '\n';
  if (!isolated_cpus.empty()) {
    os << "  isolated  " << isolated_cpus << '\n';
  }
  if (!nvidia_driver.empty()) {
    os << "  gpu       " << nvidia_driver
       << (gpu_workload_running ? "  [inference RUNNING]" : "  [idle]") << '\n';
  }
  return os.str();
}

bool Provenance::suitable_as_baseline(std::string& why_not) const {
  std::ostringstream os;
  bool ok = true;
  if (git_dirty != "clean") {
    os << "working tree is " << git_dirty << " (baseline would not be reproducible); ";
    ok = false;
  }
  if (git_sha == "unknown") {
    os << "git SHA unknown; ";
    ok = false;
  }
  if (build_type != "RelWithDebInfo" && build_type != "Release") {
    os << "build type is " << build_type << " (timing from a debug build is fiction); ";
    ok = false;
  }
  why_not = os.str();
  return ok;
}

}  // namespace rc::telemetry
