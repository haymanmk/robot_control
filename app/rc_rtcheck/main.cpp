/// rc_rtcheck — examine the real-time environment before changing anything.
///
/// Answers, in order: what are the limits, what would this process need, what
/// happens when it asks, and what is left to fix. Change nothing on the system.
///
///     ./build/app/rc_rtcheck/rc_rtcheck            # as the user who will run the core
///     ./build/app/rc_rtcheck/rc_rtcheck --priority 80 --cpu 3
///
/// Exit status 0 if every requested step was granted, 1 otherwise, so it can
/// gate a launch script.

#include <cstdio>
#include <cstdlib>
#include <string>
#include <thread>

#include "rc/rt/rt_setup.hpp"
#include "rc/telemetry/provenance.hpp"

int main(int argc, char** argv) {
  int priority = 80;
  int cpu = -1;
  bool no_tuning = false;
  for (int index = 1; index < argc; ++index) {
    const std::string argument = argv[index];
    if (argument == "--priority" && index + 1 < argc) priority = std::atoi(argv[++index]);
    else if (argument == "--cpu" && index + 1 < argc) cpu = std::atoi(argv[++index]);
    else if (argument == "--no-tuning") no_tuning = true;
    else {
      std::printf("usage: %s [--priority N] [--cpu N] [--no-tuning]\n", argv[0]);
      return 2;
    }
  }

  const auto provenance = rc::telemetry::Provenance::collect();
  std::printf("rc_rtcheck\n%s", provenance.to_summary().c_str());

  std::printf("\n1. Limits and footprint before anything is asked for\n");
  std::printf("%s", rc::rt::MemoryFigures::read().format().c_str());

  if (!no_tuning) {
    std::printf("\n2. prepare_process(): shrink what mlockall would lock\n");
    for (const auto& note : rc::rt::prepare_process()) {
      std::printf("  rt        %s\n", note.c_str());
    }
  } else {
    std::printf("\n2. prepare_process() skipped (--no-tuning)\n");
  }

  // The demo has one background thread (FileSink). Create one here too, so
  // the footprint we report includes what a thread costs under this tuning.
  std::thread probe([] { volatile int sink = 0; (void)sink; });
  probe.join();
  std::printf("\n   footprint with one extra thread created:\n");
  std::printf("%s", rc::rt::MemoryFigures::read().format().c_str());

  std::printf("\n3. apply_realtime(priority=%d, lock_memory=true%s)\n", priority,
              cpu >= 0 ? ", affinity" : "");
  rc::rt::RtOptions options;
  options.priority = priority;
  options.lock_memory = true;
  options.cpu = cpu;
  const auto status = rc::rt::apply_realtime(options);
  std::printf("%s", status.format().c_str());

  std::printf("\n4. After\n%s", status.memory.format().c_str());

  const bool ok = status.fully_applied(options);
  std::printf("\n%s\n", ok ? "READY: every requested step was granted."
                           : "NOT READY: see the FAILED lines above and docs/rt-setup.md.");
  return ok ? 0 : 1;
}
