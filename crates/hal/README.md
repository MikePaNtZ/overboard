# hal

The seam between `control-core` and whatever actually drives the wheel.

Two traits, deliberately separate (BoardIo ICD §6.1):

- **`BoardObserve`** — `open` / `close` / `wait_observe` / `run_metadata`. The
  ridden binary links exactly this and nothing more.
- **`BoardActuate`** — `arm` / `apply`, plus the `Disarm` handle. A binary that
  never links it *cannot* actuate, so shadow mode is enforced by the compiler
  rather than by a runtime check.

`wait_observe()` is the **sole time-advancing call**: in sim it steps physics to
the next control instant, on hardware it blocks on the IMU. `apply()` enqueues
and never advances time, so actuation delay stays additive on top of the
structural loop delay instead of being folded into it. Zero or one `apply()` per
`wait_observe()` — zero is the shadow-mode shape. `CallSequence` enforces those
rules for both backends so they cannot drift apart.

This replaced a single `cycle(cmd)` call that returned an `Observation`
(DR-BOARDIO-1, amended). That signature conflated observing with actuating,
which is what made pre/post-command state ambiguous — and on a balancer, phase
error is indistinguishable from negative damping.
