# board-app-ridden

The binary that must run under a rider: **zero motion authority, and the
build proves it** (ICD §6.3, DR-MODE-1).

It links `hal` (`BoardObserve`) only — never `hal-actuate`, `sim-backend`, or
any wire-encoding crate. `crates/xtask`'s gate walks `cargo metadata`'s
resolve graph from this crate over normal edges (dev- and build-dependencies
excluded — they don't ship) and fails the build if `hal-actuate` is
reachable. That is a stronger guarantee than "no one calls `apply()`": the
binary is not linked against anything that *could*.

There is no rider, no hardware and no real ridden observe backend yet, so
this binary currently runs its loop against `ShadowBackend`, a local
placeholder `BoardObserve` implementation that produces synthetic
observations (see `src/main.rs`). It exists to prove the shadow-mode shape —
observe, compute, discard — compiles and runs end to end, with no path to
`apply()` even available to call. Swapping in a real observe-only hardware
backend is later work; it does not change this crate's dependency shape.

Its driverless counterpart is `board-app-driverless`.
