# hal

The seam between `control-core` and whatever actually drives the wheel.

Observing and actuating are separate **crates**, not just separate traits
(BoardIo ICD §6.1, §6.3):

- **`hal`** (this crate) — `BoardObserve`: `open` / `close` / `wait_observe` /
  `run_metadata`. The ridden binary links exactly this and nothing more.
- **`hal-actuate`** — `BoardActuate`: `arm` / `apply`, plus the `Disarm`
  handle. The ridden binary must not depend on this crate at all. A trait
  split alone survives a careful reviewer; it does not survive
  `cargo build --workspace --all-features` or a transitive dev-dependency,
  because cargo features unify across the whole graph. Absence of a
  dependency does not unify — that's the property `crates/xtask`'s gate
  asserts by walking `cargo metadata`'s resolve graph from `board-app-ridden`
  and failing if `hal-actuate` is reachable over a normal edge.

`wait_observe()` is the **sole time-advancing call**: in sim it steps physics to
the next control instant, on hardware it blocks on the IMU. `apply()` enqueues
and never advances time, so actuation delay stays additive on top of the
structural loop delay instead of being folded into it. Zero or one `apply()` per
`wait_observe()` — zero is the shadow-mode shape. `CallSequence` (in this
crate) enforces those rules for both backends so they cannot drift apart.

This replaced a single `cycle(cmd)` call that returned an `Observation`
(DR-BOARDIO-1, amended). That signature conflated observing with actuating,
which is what made pre/post-command state ambiguous — and on a balancer, phase
error is indistinguishable from negative damping.
