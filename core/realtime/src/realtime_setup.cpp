#include "robot_control/realtime/realtime_setup.hpp"

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

namespace robot_control::realtime {
namespace {

constexpr std::uint64_t unlimited = UINT64_MAX;

std::uint64_t limit_to_bytes(rlim_t value) noexcept {
  return value == RLIM_INFINITY ? unlimited : static_cast<std::uint64_t>(value);
}

std::string mebibytes(std::uint64_t bytes) {
  if (bytes == unlimited) {
    return "unlimited";
  }
  char text[32];
  std::snprintf(text, sizeof(text), "%.1f MiB", static_cast<double>(bytes) / 1048576.0);
  return text;
}

/// Parse "VmXxx:  1234 kB" lines from /proc/self/status.
std::uint64_t read_process_status_bytes(const char* key) {
  std::ifstream status_file("/proc/self/status");
  std::string line;
  const std::size_t key_length = std::strlen(key);
  while (std::getline(status_file, line)) {
    if (line.compare(0, key_length, key) == 0) {
      return static_cast<std::uint64_t>(std::strtoull(line.c_str() + key_length, nullptr, 10)) * 1024;
    }
  }
  return 0;
}

/// CAP_IPC_LOCK lets a process ignore RLIMIT_MEMLOCK. Read the effective
/// capability set rather than assuming uid 0 has it: a root process inside a
/// container or a capability-dropped service can lack it, and then the limit
/// applies -- and the headroom check must run.
bool detect_cap_ipc_lock() {
  std::ifstream status_file("/proc/self/status");
  std::string line;
  while (std::getline(status_file, line)) {
    if (line.compare(0, 7, "CapEff:") == 0) {
      const unsigned long long capabilities = std::strtoull(line.c_str() + 7, nullptr, 16);
      constexpr int cap_ipc_lock_bit = 14;
      return (capabilities >> cap_ipc_lock_bit) & 1ULL;
    }
  }
  return false;
}

}  // namespace

MemoryFigures MemoryFigures::read() {
  MemoryFigures figures;
  rlimit limit{};
  if (::getrlimit(RLIMIT_MEMLOCK, &limit) == 0) {
    figures.memlock_soft_limit = limit_to_bytes(limit.rlim_cur);
    figures.memlock_hard_limit = limit_to_bytes(limit.rlim_max);
  }
  if (::getrlimit(RLIMIT_RTPRIO, &limit) == 0) {
    figures.realtime_priority_limit = limit_to_bytes(limit.rlim_max);
  }
  figures.mapped_bytes = read_process_status_bytes("VmSize:");
  figures.resident_bytes = read_process_status_bytes("VmRSS:");
  figures.locked_bytes = read_process_status_bytes("VmLck:");
  figures.has_cap_ipc_lock = detect_cap_ipc_lock();
  return figures;
}

std::string MemoryFigures::format() const {
  std::ostringstream out;
  out << "  memlock   limit soft " << mebibytes(memlock_soft_limit) << ", hard " << mebibytes(memlock_hard_limit);
  if (has_cap_ipc_lock) {
    out << "  (CAP_IPC_LOCK: limit does not apply)";
  }
  out << '\n';
  out << "  memory    mapped " << mebibytes(mapped_bytes) << ", resident " << mebibytes(resident_bytes) << ", locked "
     << mebibytes(locked_bytes) << '\n';
  out << "  rtprio    hard limit " << (realtime_priority_limit == unlimited ? 99 : realtime_priority_limit) << '\n';
  return out.str();
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
    pthread_attr_t attributes;
    if (::pthread_attr_init(&attributes) == 0) {
      if (::pthread_attr_setstacksize(&attributes, tuning.default_thread_stack) == 0 &&
          ::pthread_setattr_default_np(&attributes) == 0) {
        notes.emplace_back("threads: default stack " + mebibytes(tuning.default_thread_stack));
      } else {
        notes.emplace_back("threads: could not set default stack size");
      }
      ::pthread_attr_destroy(&attributes);
    }
  }
  return notes;
}

bool RealtimeStatus::fully_applied(const RealtimeOptions& options) const noexcept {
  if (options.priority > 0 && !scheduler_applied) return false;
  if (options.lock_memory && !memory_locked) return false;
  if (options.cpu >= 0 && !affinity_applied) return false;
  return true;
}

std::string RealtimeStatus::format() const {
  std::ostringstream out;
  for (const auto& note : notes) {
    out << "  rt        " << note << '\n';
  }
  return out.str();
}

void prefault_stack(std::size_t bytes) noexcept {
  if (bytes == 0) {
    return;
  }
  // volatile so the compiler cannot conclude this buffer is dead and delete the
  // very page touches we are here to perform.
  //
  // Under -fstack-clash-protection (GCC's default on Ubuntu) the alloca itself
  // already probes every page it allocates, top to bottom, so the loop below is
  // redundant there -- but it is what makes this correct on a toolchain without
  // that protection, and it costs nothing. Either way the growth happens here,
  // which is why this must run BEFORE mlockall(): on the main thread the stack
  // is mapped on demand, and growing a locked stack past RLIMIT_MEMLOCK is
  // SIGSEGV, not an error (docs/rt-setup.md).
  volatile unsigned char* stack = static_cast<volatile unsigned char*>(alloca(bytes));
  for (std::size_t offset = 0; offset < bytes; offset += 4096) {
    stack[offset] = 0;
  }
}

RealtimeStatus apply_realtime(const RealtimeOptions& options) noexcept {
  RealtimeStatus status;
  MemoryFigures before = MemoryFigures::read();

  if (options.priority > 0) {
    if (before.realtime_priority_limit != unlimited && static_cast<std::uint64_t>(options.priority) > before.realtime_priority_limit) {
      status.notes.emplace_back("SCHED_FIFO: priority " + std::to_string(options.priority) +
                            " exceeds RLIMIT_RTPRIO hard limit " +
                            std::to_string(before.realtime_priority_limit) +
                            " -- add '<user> - rtprio 99' to /etc/security/limits.conf and log in again");
    } else {
      sched_param scheduling{};
      scheduling.sched_priority = options.priority;
      if (::sched_setscheduler(0, SCHED_FIFO, &scheduling) == 0) {
        status.scheduler_applied = true;
        status.notes.emplace_back("SCHED_FIFO priority " + std::to_string(options.priority) + ": OK");
      } else {
        status.notes.emplace_back(std::string("SCHED_FIFO: FAILED (") + std::strerror(errno) +
                              ") -- run as root, grant CAP_SYS_NICE, or raise RLIMIT_RTPRIO");
      }
    }
  }

  // Touch the stack *before* locking: the pages are then part of VmSize when
  // we decide whether locking is safe, and locking never has to grow the stack
  // afterwards. (Growing a VM_LOCKED stack past RLIMIT_MEMLOCK is SIGSEGV.)
  prefault_stack(options.prefault_bytes);
  before = MemoryFigures::read();

  if (options.lock_memory) {
    if (options.raise_soft_memlock && before.memlock_soft_limit < before.memlock_hard_limit) {
      rlimit limit{};
      limit.rlim_cur = before.memlock_hard_limit == unlimited ? RLIM_INFINITY
                                                      : static_cast<rlim_t>(before.memlock_hard_limit);
      limit.rlim_max = limit.rlim_cur;
      if (::setrlimit(RLIMIT_MEMLOCK, &limit) == 0) {
        status.notes.emplace_back("memlock: raised soft limit " + mebibytes(before.memlock_soft_limit) +
                              " -> " + mebibytes(before.memlock_hard_limit) + " (the hard limit)");
        before = MemoryFigures::read();
      }
    }

    // Would locking leave room to grow? If not, do not lock. Past this point a
    // page fault that exceeds the limit is fatal, not an error, and a process
    // that dies on its first heap or stack growth is worse than one that
    // reports honestly that it is running unlocked.
    const std::uint64_t needed = before.mapped_bytes + options.headroom_bytes;
    const bool limited = !before.has_cap_ipc_lock && before.memlock_soft_limit != unlimited;
    if (limited && needed > before.memlock_soft_limit) {
      status.notes.emplace_back(
          "mlockall: REFUSED -- this process maps " + mebibytes(before.mapped_bytes) + " and needs " +
          mebibytes(options.headroom_bytes) + " of headroom to grow, but RLIMIT_MEMLOCK is " +
          mebibytes(before.memlock_soft_limit) + " soft / " + mebibytes(before.memlock_hard_limit) +
          " hard. Locking anyway would succeed and then SIGSEGV on the next page fault. "
          "Raise the limit to at least " + mebibytes(needed) + " (docs/rt-setup.md)");
    } else if (::mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      status.memory_locked = true;
      const MemoryFigures after = MemoryFigures::read();
      std::string note = "mlockall(MCL_CURRENT|MCL_FUTURE): OK, " + mebibytes(after.locked_bytes) + " locked";
      if (limited) {
        note += ", " + mebibytes(before.memlock_soft_limit - after.locked_bytes) + " headroom left";
      }
      status.notes.emplace_back(note);
    } else {
      const int error_number = errno;
      status.notes.emplace_back(std::string("mlockall: FAILED (") + std::strerror(error_number) +
                            ") -- this process maps " + mebibytes(before.mapped_bytes) +
                            " but RLIMIT_MEMLOCK is " + mebibytes(before.memlock_soft_limit) + " soft / " +
                            mebibytes(before.memlock_hard_limit) +
                            " hard. Run realtime_check, then see docs/rt-setup.md");
    }
  }

  if (options.cpu >= 0) {
    cpu_set_t cpus;
    CPU_ZERO(&cpus);
    CPU_SET(options.cpu, &cpus);
    if (::sched_setaffinity(0, sizeof(cpus), &cpus) == 0) {
      status.affinity_applied = true;
      status.notes.emplace_back("affinity -> CPU " + std::to_string(options.cpu) +
                            ": OK (is it in isolcpus?)");
    } else {
      status.notes.emplace_back(std::string("affinity: FAILED (") + std::strerror(errno) + ")");
    }
  }

  status.memory = MemoryFigures::read();
  return status;
}

}  // namespace robot_control::realtime
