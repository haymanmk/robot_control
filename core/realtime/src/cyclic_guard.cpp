#include "robot_control/realtime/cyclic_guard.hpp"

#include <linux/audit.h>
#include <linux/filter.h>
#include <linux/seccomp.h>
#include <signal.h>
#include <sys/prctl.h>
#include <sys/resource.h>
#include <sys/syscall.h>
#include <ucontext.h>
#include <unistd.h>

#include <atomic>
#include <cerrno>
#include <cstddef>
#include <cstring>
#include <sstream>

// si_code of a SIGSYS raised by seccomp. ABI-stable; not every libc exposes it.
#ifndef SYS_SECCOMP
#define SYS_SECCOMP 1
#endif

// A sanitizer runtime (ThreadSanitizer, AddressSanitizer) makes syscalls of
// its own from every instrumented thread -- mmap for shadow memory, futex,
// sched_yield, write for its reports. An allowlist would refuse them and the
// runtime would abort. So under a sanitizer the syscall filter is not armed,
// and the guard says so; the fault counters still work.
#if defined(__SANITIZE_THREAD__) || defined(__SANITIZE_ADDRESS__)
#define ROBOT_CONTROL_SANITIZER_BUILD 1
#elif defined(__has_feature)
#if __has_feature(thread_sanitizer) || __has_feature(address_sanitizer)
#define ROBOT_CONTROL_SANITIZER_BUILD 1
#endif
#endif
#ifndef ROBOT_CONTROL_SANITIZER_BUILD
#define ROBOT_CONTROL_SANITIZER_BUILD 0
#endif

namespace robot_control::realtime {
namespace {

// ── per-thread violation state, written from the SIGSYS handler ─────────────
//
// The handler runs on the thread that made the refused call, so thread-local
// storage gives each guarded thread its own counters with no locking. Only
// async-signal-safe operations happen in the handler: atomic stores and a
// register write into the interrupted context.
thread_local std::atomic<std::uint64_t> violations{0};
thread_local std::atomic<int> first_violation{-1};

void on_sigsys(int, siginfo_t* info, void* context_pointer) {
  if (info == nullptr || info->si_code != SYS_SECCOMP) {
    return;
  }
  violations.fetch_add(1, std::memory_order_relaxed);
  int expected = -1;
  first_violation.compare_exchange_strong(expected, info->si_syscall, std::memory_order_relaxed);

  // The kernel did not run the syscall. Make that visible to the caller as an
  // ordinary failure rather than leaving the result register undefined.
#if defined(__x86_64__)
  auto* context = static_cast<ucontext_t*>(context_pointer);
  if (context != nullptr) {
    context->uc_mcontext.gregs[REG_RAX] = -ENOSYS;
  }
#else
  (void)context_pointer;
#endif
}

// ── the allowlist ──────────────────────────────────────────────────────────
//
// Everything a correct cyclic loop needs, and nothing else. Read it as a
// statement of what the loop is allowed to be: it may keep time, it may talk
// to the fieldbus, and when it is done it may read its own counters and exit.
//
// Deliberately absent: write/read (logging), futex (any contended lock),
// mmap/brk (allocation), nanosleep (the relative sleep the vendor uses),
// poll/epoll (waiting), openat/ioctl (device or file access). Each of those is
// a rule violation the guard exists to catch.
constexpr int built_in_allowed[] = {
    SYS_clock_gettime,     // the clock (usually the vDSO; allowed in case it is not)
    SYS_clock_nanosleep,   // the phase-locked sleep
    SYS_restart_syscall,   // how the kernel resumes a sleep interrupted by a signal
    // The fieldbus. On x86-64 and AArch64 there are no send/recv syscalls:
    // glibc's send() is sendto() with no address, recv() is recvfrom(). Those
    // two plus the msg variants cover every socket transport we would write.
    SYS_sendto,            // fieldbus: SocketCanTransport::send (via ::send)
    SYS_sendmsg,           // fieldbus, if a transport uses it
    SYS_recvfrom,          // fieldbus, if a transport uses it (via ::recv)
    SYS_recvmsg,           // fieldbus: SocketCanTransport::receive
    SYS_getrusage,         // read_cyclic_guard()
    SYS_rt_sigreturn,      // returning from the SIGSYS handler itself
    SYS_exit,              // the guarded thread ending
    SYS_madvise,           // glibc advises the stack away on thread exit
    SYS_munmap,            // and may unmap it
    SYS_sched_yield,       // harmless; a yield is not a violation, only a smell
};

#if defined(__x86_64__)
constexpr std::uint32_t audit_arch = AUDIT_ARCH_X86_64;
#elif defined(__aarch64__)
constexpr std::uint32_t audit_arch = AUDIT_ARCH_AARCH64;
#else
#error "cyclic_guard: add the AUDIT_ARCH_* value for this architecture"
#endif

constexpr std::uint32_t nr_offset = static_cast<std::uint32_t>(offsetof(seccomp_data, nr));
constexpr std::uint32_t arch_offset = static_cast<std::uint32_t>(offsetof(seccomp_data, arch));

std::vector<sock_filter> build_program(const std::vector<int>& allowed, bool kill) {
  std::vector<sock_filter> program;
  // Kill mode ends the whole process, not just the loop thread. A control core
  // whose loop is dead but whose process still holds the bridge and the CAN
  // socket is the worst state available; the process dying is at least the
  // state ADR-0005 planned for.
  const std::uint32_t refuse = kill ? SECCOMP_RET_KILL_PROCESS : SECCOMP_RET_TRAP;

  // 1. Refuse anything not from our own ABI: a foreign-ABI syscall would be
  //    matched against the wrong number table and could slip through.
  program.push_back(BPF_STMT(BPF_LD | BPF_W | BPF_ABS, arch_offset));
  program.push_back(BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, audit_arch, 1, 0));
  program.push_back(BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_KILL_PROCESS));

  // 2. Load the syscall number and compare against each allowed value.
  program.push_back(BPF_STMT(BPF_LD | BPF_W | BPF_ABS, nr_offset));
  for (const int number : allowed) {
    program.push_back(BPF_JUMP(BPF_JMP | BPF_JEQ | BPF_K, static_cast<std::uint32_t>(number), 0, 1));
    program.push_back(BPF_STMT(BPF_RET | BPF_K, SECCOMP_RET_ALLOW));
  }

  // 3. Everything else.
  program.push_back(BPF_STMT(BPF_RET | BPF_K, refuse));
  return program;
}

void read_faults(std::uint64_t& minor, std::uint64_t& major) noexcept {
  rusage usage{};
  if (::getrusage(RUSAGE_THREAD, &usage) == 0) {
    minor = static_cast<std::uint64_t>(usage.ru_minflt);
    major = static_cast<std::uint64_t>(usage.ru_majflt);
  }
}

thread_local std::uint64_t minor_at_arm = 0;
thread_local std::uint64_t major_at_arm = 0;
thread_local bool faults_armed = false;

}  // namespace

bool cyclic_guard_supported() noexcept {
  if (ROBOT_CONTROL_SANITIZER_BUILD) {
    return false;  // see the note at the top of the file
  }
  // PR_GET_SECCOMP fails with EINVAL on a kernel built without seccomp; with
  // it, the call succeeds (returning the current mode) or, if a strict filter
  // is already active, kills us -- which cannot happen here because we never
  // install strict mode.
  return ::prctl(PR_GET_SECCOMP, 0, 0, 0, 0) >= 0;
}

std::string syscall_name(int number) {
  switch (number) {
    case SYS_write: return "write";
    case SYS_read: return "read";
    case SYS_futex: return "futex";
    case SYS_mmap: return "mmap";
    case SYS_brk: return "brk";
    case SYS_nanosleep: return "nanosleep";
    case SYS_openat: return "openat";
    case SYS_ioctl: return "ioctl";
    case SYS_poll: return "poll";
    case SYS_mprotect: return "mprotect";
    case SYS_writev: return "writev";
    case SYS_fsync: return "fsync";
    case SYS_close: return "close";
    default: break;
  }
  return "nr " + std::to_string(number);
}

GuardReport arm_cyclic_guard(const GuardOptions& options) {
  GuardReport report;

  if (options.count_page_faults) {
    read_faults(minor_at_arm, major_at_arm);
    faults_armed = true;
    report.faults_counted = true;
  }

  if (!options.filter_syscalls) {
    return report;
  }
  if (ROBOT_CONTROL_SANITIZER_BUILD) {
    report.notes.emplace_back(
        "syscall filter: not armed in a sanitizer build (the sanitizer runtime "
        "makes syscalls of its own); page faults are still counted");
    return report;
  }
  if (!cyclic_guard_supported()) {
    report.notes.emplace_back("syscall filter: seccomp not available in this kernel");
    return report;
  }

  violations.store(0, std::memory_order_relaxed);
  first_violation.store(-1, std::memory_order_relaxed);

  if (!options.kill_on_violation) {
    struct sigaction action {};
    action.sa_sigaction = on_sigsys;
    action.sa_flags = SA_SIGINFO;
    ::sigemptyset(&action.sa_mask);
    if (::sigaction(SIGSYS, &action, nullptr) != 0) {
      report.notes.emplace_back(std::string("syscall filter: sigaction(SIGSYS) failed (") +
                                std::strerror(errno) + ")");
      return report;
    }
  }

  // Installing a filter as an unprivileged thread requires promising never to
  // gain privileges by exec. A control thread never execs, so this costs nothing.
  if (::prctl(PR_SET_NO_NEW_PRIVS, 1, 0, 0, 0) != 0) {
    report.notes.emplace_back(std::string("syscall filter: PR_SET_NO_NEW_PRIVS failed (") +
                              std::strerror(errno) + ")");
    return report;
  }

  std::vector<int> allowed(std::begin(built_in_allowed), std::end(built_in_allowed));
  allowed.insert(allowed.end(), options.extra_allowed_syscalls.begin(),
                 options.extra_allowed_syscalls.end());
  std::vector<sock_filter> program = build_program(allowed, options.kill_on_violation);

  sock_fprog filter{};
  filter.len = static_cast<unsigned short>(program.size());
  filter.filter = program.data();
  // prctl(PR_SET_SECCOMP) applies to the calling thread only, which is the
  // point: the drain thread, the bridge clients and main() stay unfiltered.
  if (::prctl(PR_SET_SECCOMP, SECCOMP_MODE_FILTER, &filter, 0, 0) != 0) {
    report.notes.emplace_back(std::string("syscall filter: PR_SET_SECCOMP failed (") +
                              std::strerror(errno) + ")");
    return report;
  }
  report.filter_installed = true;
  return report;
}

void read_cyclic_guard(GuardReport& report) noexcept {
  if (faults_armed) {
    std::uint64_t minor = 0;
    std::uint64_t major = 0;
    read_faults(minor, major);
    report.minor_faults = minor - minor_at_arm;
    report.major_faults = major - major_at_arm;
  }
  report.syscall_violations = violations.load(std::memory_order_relaxed);
  report.first_violation_syscall = first_violation.load(std::memory_order_relaxed);
}

std::string GuardReport::format() const {
  std::ostringstream out;
  for (const auto& note : notes) {
    out << "  guard     " << note << '\n';
  }
  if (filter_installed) {
    if (syscall_violations == 0) {
      out << "  guard     syscalls: filtered, no violations\n";
    } else {
      out << "  guard     syscalls: " << syscall_violations << " VIOLATION"
          << (syscall_violations == 1 ? "" : "S") << ", first was "
          << syscall_name(first_violation_syscall) << " (refused, returned ENOSYS)\n";
    }
  }
  if (faults_counted) {
    out << "  guard     page faults: " << minor_faults << " minor, " << major_faults << " major"
        << ((minor_faults + major_faults) == 0 ? "\n" : "  <-- the cyclic path must not fault\n");
  }
  return out.str();
}

}  // namespace robot_control::realtime
