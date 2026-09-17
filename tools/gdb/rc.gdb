# gdb settings for this project. Load with:
#     gdb -x tools/gdb/rc.gdb ./build/app/rc_rtcheck/rc_rtcheck
# or add `source /path/to/robot_control/tools/gdb/rc.gdb` to ~/.gdbinit.
# See docs/debugging.md.

set pagination off
set print pretty on
set print object on
set confirm off
# Show the file and line in every frame of a backtrace.
set print frame-info source-and-location

# If the sources are not where they were when the binary was built, tell gdb
# how to rewrite the recorded prefix. Uncomment and edit:
#     set substitute-path /path/recorded/at/build /path/where/sources/are/now
# Find the recorded path with:
#     readelf --debug-dump=info <binary> | grep -m1 DW_AT_comp_dir

# Real-time processes raise SIGSEGV on purpose in one documented case (see
# docs/rt-setup.md); make gdb stop and show it rather than passing it through.
handle SIGSEGV stop print

# Convenience: print what the real-time setup granted.
define rc-rtstatus
  printf "scheduler_applied=%d memory_locked=%d affinity_applied=%d\n", $arg0.scheduler_applied, $arg0.memory_locked, $arg0.affinity_applied
end
document rc-rtstatus
Print the three grant flags of an rc::rt::RtStatus: rc-rtstatus st
end
