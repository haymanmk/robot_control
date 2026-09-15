#include "rc/rt/rt_setup.hpp"

#include <sched.h>
#include <sys/mman.h>
#include <alloca.h>

#include <cerrno>
#include <cstring>
#include <sstream>

namespace rc::rt {

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

  if (opts.priority > 0) {
    sched_param param{};
    param.sched_priority = opts.priority;
    if (::sched_setscheduler(0, SCHED_FIFO, &param) == 0) {
      st.scheduler_applied = true;
      st.notes.emplace_back("SCHED_FIFO priority " + std::to_string(opts.priority) + ": OK");
    } else {
      st.notes.emplace_back(
          std::string("SCHED_FIFO: FAILED (") + std::strerror(errno) +
          ") -- run as root, grant CAP_SYS_NICE, or raise RLIMIT_RTPRIO in "
          "/etc/security/limits.conf");
    }
  }

  if (opts.lock_memory) {
    if (::mlockall(MCL_CURRENT | MCL_FUTURE) == 0) {
      st.memory_locked = true;
      st.notes.emplace_back("mlockall(MCL_CURRENT|MCL_FUTURE): OK");
    } else {
      st.notes.emplace_back(std::string("mlockall: FAILED (") + std::strerror(errno) +
                            ") -- raise RLIMIT_MEMLOCK");
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
  return st;
}

}  // namespace rc::rt
