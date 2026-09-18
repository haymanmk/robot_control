# Debugging with gdb: getting source lines

gdb does not "import" source files. It **finds** them, using two things:
the debug information inside the binary, and a search path you can adjust.
When source lines are missing, one of those two is wrong. This page shows how
to check each one, and how to fix it.

## 1. The binary must contain debug information

Source lines come from DWARF debug information, which the compiler adds with
`-g`. Our default build type is `RelWithDebInfo`, which compiles with
`-O2 -g`, so every binary already has it. You can confirm:

```bash
readelf --debug-dump=info build/app/realtime_check/realtime_check | grep -m2 -E 'DW_AT_(name|comp_dir)'
```

```
DW_AT_name     : /home/user/robot_control/app/realtime_check/main.cpp
DW_AT_comp_dir : /home/user/robot_control/build/app/realtime_check
```

Those two fields are the whole story. `DW_AT_name` is the **absolute path of
each source file as it was when the binary was built**. gdb opens exactly that
path. If it exists, source lines just work — no configuration at all:

```
(gdb) list robot_control::realtime::prefault_stack
146	void prefault_stack(std::size_t bytes) noexcept {
147	  if (bytes == 0) {
```

If you see `<optimized out>` for variables, or stepping jumps around, that is
`-O2`, not missing sources. For stepping through logic, use a debug build in a
separate directory:

```bash
cmake -B build-debug -DCMAKE_BUILD_TYPE=Debug && cmake --build build-debug -j
```

Keep timing measurements on `RelWithDebInfo`; numbers from a `-O0` build are
not meaningful (ADR-0003).

## 2. If the sources are not where the binary says they are

This happens when the binary was built somewhere else — on another machine, in
a container, in CI — or the tree was moved. gdb then prints:

```
/home/user/robot_control/core/realtime/src/realtime_setup.cpp: No such file or directory.
```

gdb always tries the recorded absolute path **first**. Only when that file
does not exist does it fall back to the search path. So the two fixes below
do nothing while the old path still exists — which is fine, because then you
do not need them.

Pick the first when the whole tree moved; the second for one-off directories.

**Rewrite the recorded prefix** (preferred):

```
(gdb) set substitute-path /home/user/robot_control /path/where/the/tree/is/now
(gdb) list robot_control::realtime::prefault_stack
```

**Add search directories.** gdb looks for the file's *basename* in each:

```
(gdb) directory /path/to/robot_control/core/realtime/src
```

Check what gdb is doing with `info source` (the file it is showing and where
it found it), `show substitute-path`, and `show directories`.

Both settings can go in a gdb script so you do not retype them. This project
ships one:

```bash
gdb -x tools/gdb/robot_control.gdb ./build/app/realtime_check/realtime_check
```

Edit the `substitute-path` line in that file if your tree has moved. (You can
also `source` it from `~/.gdbinit`. A `.gdbinit` inside the project directory
is *not* loaded by default; gdb refuses it unless the directory is added to
`set auto-load safe-path`, so the explicit `-x` is simpler.)

## 3. Source lines inside libc and libstdc++

A backtrace like this one, from the segfault we fixed, has file and line for
our code *and* for glibc:

```
#0  __GI___clock_nanosleep (...) at ../sysdeps/unix/sysv/linux/clock_nanosleep.c:78
#1  robot_control::realtime::sleep_until (...) at /home/user/robot_control/core/realtime/src/clock.cpp:31
```

The glibc line comes from the **debug symbols package** for the library, not
from anything in our build. On Ubuntu and Debian:

```bash
sudo apt install libc6-dbg          # glibc symbols: file and line in backtraces
sudo apt install libstdc++6-13-dbg  # libstdc++ symbols (match your GCC version)
```

gdb finds these automatically under `/usr/lib/debug`. Note that symbols give
you the **file name and line number**; they do not include the **source
text**. `list` inside a glibc function will still say the `.c` file is
missing unless you also have the source, which is a separate package
(`glibc-source`) or comes from the next section.

## 4. debuginfod: symbols and sources fetched on demand

Modern gdb can fetch both debug symbols *and* source files for system
libraries from a server, when it needs them. On Ubuntu:

```bash
export DEBUGINFOD_URLS="https://debuginfod.ubuntu.com"
gdb ./build/app/realtime_check/realtime_check
```

The first time, gdb asks whether to enable it; answer yes, or put
`set debuginfod enabled on` in your gdb script. It caches what it downloads
under `~/.cache/debuginfod_client`. This is the easiest way to get library
source text, and it covers packages you never thought to install. It needs
network access, so it will not help on an isolated control machine; install
the `-dbg` packages there instead.

## 5. Checking which of the two is the problem

| Symptom | Cause | Fix |
|---|---|---|
| Backtrace shows only addresses, no function names | No debug info in the binary | Build `RelWithDebInfo` or `Debug`; check with `readelf` |
| Function names and `file.cpp:123`, but `list` says "No such file" | Sources moved | `set substitute-path` or `directory` |
| Our frames have file:line; library frames show `?? ()` | No symbols for that library | Install the `-dbg` package or enable debuginfod |
| Library frames have file:line, but `list` fails inside them | Symbols without source text | `glibc-source`, or debuginfod |
| Variables print as `<optimized out>` | `-O2` | Use a `Debug` build for stepping |

## Two things specific to this project

**A real-time process can be killed by SIGSEGV on purpose.** With
`mlockall(MCL_FUTURE)` active, a page fault that would exceed
`RLIMIT_MEMLOCK` is fatal rather than an error (see `docs/rt-setup.md`). If
you are debugging a crash in `apply_realtime()` or right after it, check
`VmLck` against `ulimit -l` before you suspect the code.

**gdb and `SCHED_FIFO` do not mix on one core.** A real-time thread stopped
at a breakpoint is fine; a real-time thread *spinning* at high priority on the
same CPU as gdb can starve gdb itself. Debug with `priority = 0`, or pin gdb
to a different core (`taskset -c 0 gdb ...`).
