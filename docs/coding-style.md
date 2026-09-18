# Coding style

This is the naming rule for all C++ in `core/` and `app/`, and for Python in
`tools/`, `bench/` and `notebooks/` where the same idea applies. It exists
because the code base has drifted into a mix of styles (`kMaxJoints`,
`member_`, `cfg`, `Nanos`, `rt_status`) and a reader should never have to guess
which convention a name follows.

The rule is short. Everything else in this file is examples and the answers to
the questions that come up when applying it.

## The rule

| What                                   | Style                     | Example                                   |
|----------------------------------------|---------------------------|-------------------------------------------|
| Variables, function parameters         | `snake_case`              | `period`, `overrun_threshold`             |
| Attributes (data members, fields)      | `snake_case`              | `missed_deadlines`, `wake_latency`        |
| Constants and enumerators              | `snake_case`              | `max_joints`, `layout_version`            |
| Type aliases (`using`, `typedef`)      | `snake_case`              | `nanoseconds`, `word`, `joint_index`      |
| Classes, structs, enums                | `CamelCase`               | `CyclicTask`, `CyclicConfig`, `ServerState` |
| Functions and methods                  | `snake_case`              | `take_control()`, `out_of_range()`        |
| Namespaces, files, directories         | `snake_case`              | `robot_control::real_time`, `cyclic_task.hpp` |

And one rule that cuts across all of the rows:

**No abbreviations. Write the whole word.** `configuration`, not `cfg`;
`buffer`, not `buf`; `position` and `velocity`, not `pos` and `vel`;
`proportional_gain` and `derivative_gain`, not `kp` and `kd`; `options`, not
`opts`; `error`, not `err`; `priority`, not `prio`; `index`, not `idx`;
`timestamp`, not `ts`; `command`, not `cmd`; `provenance`, not `prov`.

Length is not a problem. An editor completes long names; nobody can complete a
name they cannot decode.

## Why

- A name's shape tells you what it is. `CamelCase` is a class, struct or enum
  you declared; everything else is a value, a function or an alias. No prefix
  such as `k` is needed to carry that information, so it is not used.
- Abbreviations are only readable to the person who wrote them, on the day they
  wrote them. `dt` means "delta time" in a control loop and "date" in a log
  writer. `ts` is a timestamp or a ThreadSanitizer build. The full word costs
  nothing at the call site and removes the guess.
- One convention means a reviewer checks names against a single table instead
  of asking "which of the three styles was this file written in?".

## Details and edge cases

**Type aliases are plain `snake_case`, classes and structs are `CamelCase`.**
An alias is a name for some other type, and it reads like the values it
describes. A class or struct is its own type and the `CamelCase` says so. No
`_t` suffix: it is an abbreviation for "type", and names ending in `_t` are
reserved by POSIX at global scope anyway.

```cpp
using nanoseconds = std::chrono::nanoseconds;      // alias: snake_case
using joint_index = std::uint8_t;

struct CyclicConfig {                              // struct: CamelCase
  nanoseconds period{std::chrono::milliseconds(2)};     // attribute: snake_case
  RealTimeOptions real_time{};
};
```

When an alias and a variable would otherwise take the same name, rename the
variable for its role (`cycle_period`), or use an underscore as described
below.

**Enums are `CamelCase`, their enumerators are `snake_case`.** An `enum class`
is a type you declare like a struct, and its values are constants.

```cpp
enum class ServerState { idle, starting, holding, stopping, shutdown };
```

**Constants are `snake_case` like any other variable.** The `k` prefix is an
abbreviation for "constant" and goes away with the others. `constexpr` at the
declaration already says it is a constant.

```cpp
inline constexpr std::size_t max_joints = 6;
inline constexpr std::uint32_t layout_version = 3;
```

**A leading or trailing underscore is allowed when it removes an ambiguity,
and only then.** The usual case is a constructor or setter parameter that
would otherwise shadow the member it initialises. Prefer a name that says what
the parameter is (`initial_head`), and reach for the underscore when no such
name exists:

```cpp
class SpscRing {
 public:
  explicit SpscRing(std::size_t capacity_) : capacity(capacity_) {}
 private:
  std::size_t capacity;
  std::size_t head = 0;
};
```

The underscore is not a marker for "this is a member" or "this is private".
Do not add `_` to every member of a class; a class where every field ends in
`_` is back to the old style. Never use a leading underscore followed by an
uppercase letter or a double underscore: those spellings are reserved by the
C++ standard.

**Namespaces are words too.** `rc::rt` reads as nothing; `robot_control::real_time`
reads as what it is. Namespace and directory renames are a repository-wide
change, so they are done as a single dedicated commit, not opportunistically
inside a feature change (see "Existing code" below).

**Units are spelled out and go at the end of the name.** `span_ns` becomes
`span_nanoseconds`, `rate_hz` becomes `rate_hertz`. Better still, carry the
unit in the type (`nanoseconds`) so the name does not have to.

**Standard-library and OS names are not ours to rename.** `size_t`, `time_t`,
`rlim_t`, `mlock()`, `sched_setscheduler()` stay as they are. Wrapping them in
a project alias or function is fine, and that wrapper follows this rule.

**Acronyms that are the name of a thing are not abbreviations.** `CAN`, `CPU`,
`MIT` (the drive's MIT control mode), `CUDA`, `POSIX`. They are written as
words in identifiers: `CanFrame`, `can_frame`, `cpu_affinity`, `MitMode`. The
test is "would spelling it out make it *less* recognisable to someone who
knows the domain?". `Controller Area Network` fails that test; `configuration`
passes it. When in doubt, spell it out.

**Loop counters and lambda parameters are not exempt.** `for (std::size_t joint = 0; ...)`
instead of `for (std::size_t i = 0; ...)` whenever the index means something.
A bare `i` is tolerable only when the loop is three lines long and the index
means nothing more than "the next one".

## Existing code

Most of the code written before this rule does not follow it. The rule applies
in three steps:

1. **New code follows the rule from the first line.** No exceptions, including
   code that sits next to old-style code in the same file.
2. **Code you touch gets renamed as you touch it**, when the rename stays
   inside the files your change already modifies. A renamed identifier that
   is part of a public header or a shared-memory layout counts as an API change
   and gets its own commit so the diff is reviewable.
3. **Cross-cutting renames** (namespaces, directory names, the `rc/` include
   prefix, the `kLayoutVersion` family) happen in dedicated, mechanical commits
   that change nothing else. Do not mix them into a behavioural change.

Do not rename anything in [`core/bridge/include/rc/bridge/layout.hpp`](../core/bridge/include/rc/bridge/layout.hpp)
or [`core/telemetry/include/rc/telemetry/record.hpp`](../core/telemetry/include/rc/telemetry/record.hpp)
without reading the shared-memory section of [AGENTS.md](../AGENTS.md) first.
A rename changes no bytes, but the same commit is a tempting place to change
a field, and that needs a layout-version bump.

## Formatting

Unchanged from before: 2-space indent, and match the surrounding file. There
is no formatter configured.
