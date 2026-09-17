#include "rc/rt/rt_setup.hpp"

#include <alloca.h>
#include <malloc.h>
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/resource.h>
#include <unistd.h>

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <sstream>

namespace rc::rt {
namespace {

constexpr std::uint64_t kUnlimited = UINT64_MAX;

std::uint64_t rlim_to_u64(rlim_t v) noexcept {
  return v == RLIM_INFINITY ? kUnlimited : static_cast<std::uint64_t>(v);
}

std::string mib(std::uint64_t bytes) {
  if (bytes == kUnlimited) {
    return "unlimited";
  }
  char buf[32];
  std::snprintf(buf, sizeof(buf), "%.1f MiB", static_cast<double>(bytes) / 1048576.0);
  return buf;
}

/// Parse "VmXxx:  1234 kB" lines from /proc/self/status.
std::uint64_t proc_status_kb(const char* key) {
  std::ifstream f("/proc/self/status");
  std::string line;
  const std::size_t klen = std::strlen(key);
  while (std::getline(f, line)) {
    if (line.compare(0, klen, key) == 0) {
      return static_cast<std::uint64_t>(std::strtoull(line.c_str() + klen, nullptr, 10)) * 1024;
    }
  }
  return 0;
}

/// CAP_IPC_LOCK lets a process ignore RLIMIT_MEMLOCK. Root has it; a service
/// can be granted it. We only need a yes/no, so read the effective set.
bool detect_cap_ipc_lock() {
  if (::geteuid() == 0) {
    return true;
  }
  std::ifstream f("/proc/self/status");
  std::string line;
  while (std::getline(f, line)) {
    if (line.compare(0, 7, "CapEff:") == 0) {
      const unsigned long long caps = std::strtoull(line.c_str() + 7, nullptr, 16);
      constexpr int kCapIpcLock = 14;
      return (caps >> kCapIpcLock) & 1ULL;
    }
  }
  return false;
}

}  // namespace

MemoryFigures MemoryFigures::read() {
  MemoryFigures m;
  rlimit rl{};
  if (::getrlimit(RLIMIT_MEMLOCK, &rl) == 0) {
    m.memlock_soft = rlim_to_u64(rl.rlim_cur);
    m.memlock_hard = rlim_to_u64(rl.rlim_max);
  }
  if (::getrlimit(RLIMIT_RTPRIO, &rl) == 0) {
    m.rtprio_hard = rlim_to_u64(rl.rlim_max);
  }
  m.vm_size = proc_status_kb("VmSize:");
  m.vm_rss = proc_status_kb("VmRSS:");
  m.vm_locked = proc_status_kb("VmLck:");
  m.has_cap_ipc_lock = detect_cap_ipc_lock();
  return m;
}

std::string MemoryFigures::format() const {
  std::ostringstream os;
  os << "  memlock   limit soft " << mib(memlock_soft) << ", hard " << mib(memlock_hard);
  if (has_cap_ipc_lock) {
    os << "  (CAP_IPC_LOCK: limit does not apply)";
  }
  os << '\n';
  os << "  memory    mapped " << mib(vm_size) << ", resident " << mib(vm_rss) << ", locked "
     << mib(vm_locked) << '\n';
  os << "  rtprio    hard limit " << (rtprio_hard == kUnlimited ? 99 : rtprio_hard) << '\n';
  return os.str();
}

std::vector<std::string> prepare_process(const ProcessTuning& tuning) noexcept {
  std::vector<std::string> notes;
  if (tuning.single_malloc_arena) {
    // One arena for the whole process. Otherwise each new thread that
    // allocates gets its own 64 MiB reservation, and MCL_FUTURE locks it all.
    ::mallopt(M_ARENA_MAX, 1);
    notes.emplace_back("malloc: single arena");
  }
  if (tuning.keep_freed_memory) {
    // Never trim the heap and never satisfy large requests with a fresh mmap:
    // freed memory stays mapped and locked, so a later allocation of the same
    // size reuses pages that cannot fault.
    ::mallopt(M_TRIM_THRESHOLD, -1);
    ::mallopt(M_MMAP_MAX, 0);
    notes.emplace_back("malloc: no trim, no mmap (freed memory stays locked)");
  }
  if (tuning.default_thread_stack > 0) {
    // glibc's default thread stack is RLIMIT_STACK, usually 8 MiB, fully locked
    // under MCL_FUTURE. Set a process-wide default for every thread created
    // after this point, std::thread included.
    pthread_attr_t attr;
    if (::pthread_attr_init(&attr) == 0) {
      if (::pthread_attr_setstacksize(&attr, tuning.default_thread_stack) == 0 &&
          ::pthread_setattr_default_np(&attr) == 0) {
        notes.emplace_back("threads: default stack " + mib(tuning.default_thread_stack));
      } else {
        notes.emplace_back("threads: could not set default stack size");
      }
      ::pthread_attr_destroy(&attr);
    }
  }
  return notes;
}

bool RtStatus::fully_applied(const RtOptions& opts) const noexcept {
  if (opts.priority > 0 && !scheduler_applied) return false;
  if (opts.lock_memory && !memory_locked) return false;
  if (opts.cpu >= 0 && !affinity_applied) return false;
  return true;
}

std::string RtStatus::format() const {
  std::ostringstream os;
  for (const auto& n : notes) {
    os << "  rt        " << n << '\n';
  }
  return os.str();
}

void prefault_stack(std::size_t bytes) noexcept {
  if (bytes == 0) {
    return;
  }
  // volatile so the compiler cannot conclude this buffer is dead and delete the
  // very page touches we are here to perform.
  volatile unsigned char* buf = static_cast<volatile unsigned char*>(alloca(bytes));
  for (std::size_t i = 0; i < bytes; i += 4096) {
    buf[i] = 0;
  }
}

RtStatus apply_realtime(const RtOptions& opts) noexcept {
  RtStatus st;
  MemoryFigures before = MemoryFigures::read();

  if (opts.priority > 0) {
    if (before.rtprio_hard != kUnlimited && static_cast<std::uint64_t>(opts.priority) > before.rtprio_hard) {
      st.notes.emplace_back("SCHED_FIFO: priority " + std::to_string(opts.priority) +
                            " exceeds RLIMIT_RTPRIO hard limit " +
                            std::to_string(before.rtprio_hard) +
                            " -- add '<user> - rtprio 99' to /etc/security/limits.conf and log in again");
    } else {
      sched_param param{};
      param.sched_priority = opts.priority;
      if (::sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
        st.scheduler_applied = true;
        st.notes.emplace_back("SCHED_FIFO priority " + std::to_string(opts.priority) + ": OK");
      } else {
        st.notes.emplace_back(std::string("SCHED_FIFO: FAILED (") + std::strerror(errno) +
                              ") -- run as root, grant CAP_SYS_NICE, or raise RLIMIT_RTPRIO");
      }
    }
  }

  if (opts.lock_memory) {
    if (opts.raise_soft_memlock && before.memlock_soft < before.memlock_hard) {
      rlimit rl{};
      rl.rlim_cur = before.memlock_hard == kUnlimited ? RLIM_INFINITY
                                                      : static_cast<rlim_t>(before.memlock_hard);
      rl.rlim_max = rl.rlim_cur;
      if (::setrlimit(RLIMIT_MEMLOCK, &rl) == 0) {
        st.notes.emplace_back("memlock: raised soft limit " + mib(before.memlock_soft) +
                              " -> " + mib(before.memlock_hard) + " (the hard limit)");
        before = MemoryFigures::read();
      }
    }
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      st.memory_locked = true;
      const MemoryFigures after = MemoryFigures::read();
      st.notes.emplace_back("mlockall(MCL_CURRENT|MCL_FUTURE): OK, " + mib(after.vm_locked) +
                            " locked");
    } else {
      const int err = errno;
      // Say exactly what is needed: the limit, and how much this process maps.
      // "raise RLIMIT_MEMLOCK" alone sends people to guess at a number.
      st.notes.emplace_back(std::string("mlockall: FAILED (") + std::strerror(err) +
                            ") -- this process maps " + mib(before.vm_size) +
                            " but RLIMIT_MEMLOCK is " + mib(before.memlock_soft) + " soft / " +
                            mib(before.memlock_hard) +
                            " hard. Run rc_rtcheck, then see docs/rt-setup.md");
    }
  }

  if (opts.cpu >= 0) {
    cpu_set_t set;
    CPU_ZERO(&set);
    CPU_SET(opts.cpu, &set);
    if (::sched_setaffinity(0, sizeof(set), &set) == 0) {
      st.affinity_applied = true;
      st.notes.emplace_back("affinity -> CPU " + std::to_string(opts.cpu) +
                            ": OK (is it in isolcpus?)");
    } else {
      st.notes.emplace_back(std::string("affinity: FAILED (") + std::strerror(errno) + ")");
    }
  }

  prefault_stack(opts.prefault_bytes);
  st.memory = MemoryFigures::read();
  return st;
}

}  // namespace rc::rt
