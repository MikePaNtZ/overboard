# Issue #121: `sim-backend` populates `erpm`, and the Rust-hosted seam runs the default estimator config

`#120`'s module doc claimed `hal::Observation` "carries raw IMU and `motor_current_a` (DR-OBS-1)
but no wheel-rate/ERPM channel yet." **That was wrong** — `Observation` has carried `erpm`,
`erpm_effective_age_ns` and `tacho_raw` since DR-OBS-1; `sim-backend` simply never populated them.
The distinction mattered because the wrong reason pointed at the most expensive possible fix (an
ICD field-set change, a cross-role Promise) when the real one was local and entirely Senior
Controls' — populating a field inside a crate this role owns.

**`crates/sim-backend`** (AC1–AC3): resolves the model's existing `wheel_vel` `jointvel` sensor
(already declared in `overboard_onewheel.xml`, unread until now — a `<sensor>` element, already
Senior Controls turf, no model edit needed) in `open()`, alongside `frame_gyro`/`frame_accel`.
`wait_observe()` reads it after stepping (same ordering rule as the IMU sensors) and converts to
ERPM through one new constant, `board_types::RAD_S_PER_ERPM` (0.00698 rad/s per ERPM, ICD §10.5 —
the same ratio `sim/scenarios/imperfections.py` calls `wheel_rate_quantum_rad_s`). **Not derived
from an independently-known pole-pair count**: back-solving the ratio lands near 15 pole pairs, a
plausible figure for a hoverboard-class hub motor, but that back-derivation is not confirmed
against the real motor and is not asserted as fact — the ICD's own ratio is used directly instead
of guessing an independent pole-pair count, per the project's rule against fabricating a protocol
constant. Not quantised to the nearest integer ERPM: this backend has no imperfection-profile
plumbing yet (its own header already says so for the actuation-delay stub), so like
`motor_current_a` this is the idealised, noiseless reading — integer ERPM quantisation is
`imperfections.py`'s row, which has no Rust-side home yet.

`erpm_effective_age_ns` is set to `0` explicitly with a comment stating it is honestly fresh (the
sim has no ERPM transport lag to model), not a leftover default. `tacho_raw` and `duty` are left
at `COLD_START`'s zero **on purpose, documented in place** (AC2): `tacho_raw`'s real-VESC
wrap/reset convention is not documented anywhere this project can check, and `duty` depends on bus
voltage and back-EMF this backend does not model at all — populating either would be presenting a
fabricated protocol quantity as data.

**`crates/board-app-driverless/src/bin/impulse-response-rust.rs`** (AC4–AC5): now runs
`estimator_accel_aiding` mode 1 (wheel odometry, via `control_core::WheelAccelEstimator`) instead
of mode 2 (command feedforward) — the mode `RustController()` defaults to and `test_closed_loop.py`
gates on, no longer avoided only because a field was empty. Round-trips `obs.erpm` back through
`RAD_S_PER_ERPM` into wheel rate, then `DEFAULT_R_EFF_M` into ground speed, matching
`control-ffi::ob_controller_update`'s own `(None, wheel_accel.as_mut())` branch. The module doc's
wrong rationale is replaced with the corrected one.

**`tests/test_rust_hosted_impulse_response.py`**: Python-hosted comparator switched to
`estimator_accel_aiding=1` to match; the tolerance section now accounts for the ERPM round-trip's
extra `f32` rounding (algebraically cancels through the same ratio, smaller than the pre-existing
summation-order noise budget) — same bounds, not loosened.

## Testing

- `python3 -m pytest tests/` — 266 passed, 2 xfailed (new `board-types` unit test included via
  `cargo test`, not pytest).
- `cargo test --workspace` — all green, including a new regression pin
  (`rad_s_per_erpm_matches_the_icd_quantum_sim_scenarios_imperfections_uses`) so
  `board_types::RAD_S_PER_ERPM` cannot silently drift from `imperfections.py`'s
  `wheel_rate_quantum_rad_s`.
- `cargo clippy --workspace --all-targets -- -D warnings` — clean.
- `cargo fmt --check` — clean.
- `cargo run -p xtask -- gate` — passes.
- `python3 .github/policy_check.py` — all hard checks pass (advisories unrelated to this change).

## Deliberately left out

- Any change to `Observation`'s field set — the issue's whole point is that none was needed.
- Integer ERPM quantisation / update-rate modelling — that is `imperfections.py`'s
  `wheel_rate_quantum_rad_s` row, and `sim-backend` has no imperfection-profile plumbing to hang it
  on yet (a separate, larger increment, out of scope here).
- The ridden/cascade plant's Rust-hosted seam — this issue and its harness are driverless-only, per
  `sim-backend`'s own scope.

Closes #121.
