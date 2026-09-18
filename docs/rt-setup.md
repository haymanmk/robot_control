# Setting up a machine for the real-time core

This page is about the two resource limits that stop an ordinary user from
running real-time code: **locked memory** (`RLIMIT_MEMLOCK`) and **real-time
priority** (`RLIMIT_RTPRIO`). It explains what to look at before you change
anything, what to change, and why the numbers are what they are.

## First, examine; do not guess

Build the project and run the check tool **as the user who will run the
control core**, not as root:

```bash
./build/app/rc_rtcheck/rc_rtcheck
```

It prints the current limits, how much memory this process maps, how much
headroom locking would leave, what happens when it asks for `SCHED_FIFO` and
`mlockall`, and what is still missing. It
changes nothing on the system. Its exit status is 0 only if every step was
granted, so a launch script can use it as a gate.

You can also look by hand:

```bash
ulimit -l          # locked-memory soft limit, in KiB
ulimit -Hl         # the hard limit
ulimit -r          # real-time priority limit
prlimit --pid $$   # everything, for the current shell
grep -E 'VmSize|VmRSS|VmLck' /proc/<pid>/status   # for a running process
```

## What the numbers mean

`mlockall(MCL_CURRENT | MCL_FUTURE)` locks **everything the process has
mapped**, now and in the future — not only what it is using. `VmSize` (mapped)
is the number that has to fit under the limit, not `VmRSS` (resident).

On a stock Ubuntu the default limit is **8 MiB**. The first version of the demo
mapped **80 MiB**, and only about 1 MiB of that was doing real work:

| Where the 80 MiB went | Size |
|---|---|
| Thread stack and malloc arena for **one** background thread | 72 MiB |
| Shared libraries (libc, libstdc++, libm) | 6 MiB |
| The bridge shared-memory region | 1 MiB |
| Main stack, heap, our own binary | 1 MiB |

The 72 MiB is glibc's doing. Every non-main thread gets an 8 MiB stack by
default, and the first time it allocates, glibc reserves a private 64 MiB
malloc arena for it. Normally that reservation costs nothing, because untouched
pages are not real. Under `MCL_FUTURE`, every mapped page is faulted in and
locked, so the reservation becomes real memory — and it counts against the limit.

## Shrink the footprint first

`rc::rt::prepare_process()` fixes this, and the demo now calls it before it
creates any thread. It does three things:

- `mallopt(M_ARENA_MAX, 1)` — one malloc arena for the whole process.
- `mallopt(M_TRIM_THRESHOLD, -1)` and `mallopt(M_MMAP_MAX, 0)` — freed memory
  stays mapped and locked, so a later allocation of the same size reuses pages
  that cannot fault.
- `pthread_setattr_default_np` with a 1 MiB stack — every thread created after
  this call, `std::thread` included, gets a small stack.

**Order matters.** These only affect threads created *afterwards*. Call it at
the top of `main()`.

With that in place the demo locks **9.1 MiB** (measured), most of it shared
libraries. That is still just over the 8 MiB default, so the limit does need
raising — but by a small amount, not by ten times. Any real-time process should
be measured the same way; do not raise the limit to cover a footprint you have
not looked at.

## Why a tight limit crashes instead of failing

This is the part that is not obvious, and it produced a segmentation fault in
`rc_rtcheck` on a machine where the limit was just above the footprint.

`mlockall(MCL_CURRENT | MCL_FUTURE)` returns an error only if the memory
mapped *right now* does not fit under the limit. If it fits, it succeeds — and
from then on every new page the process touches must also be locked. When the
stack grows or the heap grows past the limit, the kernel does not return
`ENOMEM`. **The page fault fails, and the process gets `SIGSEGV`.** There is no
error to check and nothing to catch: the process is simply killed at the next
allocation or the next deep function call.

### Why the main thread's stack is the usual trigger

Thread stacks come in two kinds, and they behave differently under a lock:

- A **pthread stack** (every `std::thread`) is created with `mmap` at its full
  size. It is in `VmSize` from the moment the thread exists, whether or not it
  is ever touched. `mlockall(MCL_CURRENT)` locks all of it at once, and nothing
  the thread does later can grow it.
- The **main thread's stack** is different. The kernel maps only the part that
  has been used so far — a few hundred KiB — and **grows it on demand**, one
  page fault at a time, up to `RLIMIT_STACK`. `VmStk` in `/proc/<pid>/status`
  shows its current size.

So a 512 KiB `alloca` on the main thread, *after* locking, is not "512 KiB out
of an 8 MiB stack". It is 512 KiB of **new mapping** that did not exist when
`mlockall` counted, and under `MCL_FUTURE` every byte of it must be locked as
it appears. If `VmSize` was 7.8 MiB against an 8 MiB limit, the first touch
asks for 8.3 MiB and is refused with `SIGSEGV`.

The accounting is page-exact: with 8 pages of room left under the limit, the
ninth new stack page is the one that dies (measured; 19 pages of room gave
death on the twentieth). It looks like it dies "on the first touch" only
because of the compiler. Ubuntu's GCC enables `-fstack-clash-protection` by
default, and under it an `alloca` **probes every page it allocates, top to
bottom, before returning** — that is what the protection is. So the stack
growth, and the death, happen inside the `alloca` itself, before the first
line of the loop that follows it. A debugger points at the function, the loop
counter reads zero, and the pages that would have fit are used up by the
probe. Either way the outcome is the same: dead, with no error to handle.

So "the limit is bigger than `VmSize`" is not enough. The limit must be bigger
than everything the process will ever map. `apply_realtime()` therefore:

1. touches its stack **before** locking, so those pages are counted;
2. checks that `VmSize` plus a headroom (default 4 MiB) fits under the limit;
3. **refuses to lock** if it does not, and prints the number to raise the limit to.

The first version locked first and prefaulted afterwards, and on a machine
with an 8 MiB limit and a 7.8 MiB footprint that was a crash. A lock that
succeeds and kills you a moment later is worse than no lock, because the timing
numbers up to that moment look fine.

## Then raise the limits, as far as needed and no further

A process may raise its own **soft** limit up to the **hard** limit without any
privilege, and `apply_realtime()` does that automatically. Raising the **hard**
limit needs root, and it is done in one of three places.

**For an interactive user** — add to `/etc/security/limits.conf`:

```
<user>   -   memlock   unlimited
<user>   -   rtprio    99
```

Then log out and back in. (The `-` sets both soft and hard.) `unlimited` is
the usual choice for a dedicated control machine; on a shared laptop, a value
of a few hundred MiB is enough and safer. Check the result with `ulimit -l`.

**For a service** — in the systemd unit:

```
[Service]
LimitMEMLOCK=infinity
LimitRTPRIO=99
```

**For a running shell, without logging out** — `prlimit` can raise the hard
limit if run as root, and the change applies to processes started from that
shell:

```bash
sudo prlimit --pid $$ --memlock=unlimited:unlimited --rtprio=99:99
```

## What not to do

- **Do not run the control core as root** to get around the limits. Root has
  `CAP_IPC_LOCK` and `CAP_SYS_NICE`, so the limits do not apply, and you will
  not notice a footprint problem until you deploy as a normal user. It also
  means a bug in the control loop runs with full privilege.
- **Do not skip `mlockall` because it failed.** A control loop over pageable
  memory works until the first page fault, which arrives as a multi-millisecond
  stall at a time of the kernel's choosing.
- **Do not use a `SCHED_FIFO` priority above 50** without a reason. The
  kernel's own real-time threads (interrupt handlers, `ksoftirqd`) run at 50;
  going above them can starve the very interrupts your CAN bus depends on.
  `rc_rtcheck` uses 80 to match `cyclictest` and common practice on isolated
  cores; on a shared laptop, 40 is a better starting point.

## Related

- [ADR-0007](adr/0007-rt-platform-on-a-cuda-laptop.md) — the full tuning order
  for this laptop, and why `hwlatdetect` comes before any of it.
- `core/rt/include/rc/rt/realtime_setup.hpp` — what `apply_realtime()` asks for and
  how it reports what it got.
