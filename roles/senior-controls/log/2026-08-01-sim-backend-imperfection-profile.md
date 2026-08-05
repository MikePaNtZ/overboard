# Issue #129 — `sim-backend` gets a real imperfection profile (PR TBD)

`crates/sim-backend` stamped `imperfection_profile_id: None` unconditionally and modelled
actuation delay as a one-whole-cycle buffer — the gap Mechanical's #128 supplied a contract
for (generated conformance vectors) but that this crate had not yet wired up. SR-SIM-3's "no
ideal-only mode in CI" did not hold on the Rust-hosted path.

## What changed

New `crates/sim-backend/src/imperfections.rs`: an `ImperfectionProfile`/`ImperfectionState`
pair mirroring `sim/scenarios/imperfections.py` field-for-field and method-for-method —
`IDEAL`/`STAGE0_PLACEHOLDER`/`STAGE0_CUTBACK` constants, the cutback → saturate → transport
delay → current-loop lag chain (fractional, interpolated delay — not rounded to whole cycles),
wheel-rate quantisation (round-half-to-even, via `f64::round_ties_even`, not Rust's default
round-half-away-from-zero) and hold, and gyro/accel noise from a small seeded SplitMix64 +
Box-Muller generator (deliberately not NumPy's PCG64 — distributional/reproducibility
conformance only, per the Python module's own conformance-contract section).

Wired into `SimBackend`:
- `apply()`'s existing structural one-cycle defer is unchanged (protocol-level, ICD §5.2's
  loop delay). The profile's own chain now runs on top of it in `wait_observe()`, so
  `motor_current_a` reports the current the plant actually sees, not an echo.
- Reported ERPM goes through quantise+hold; the raw truth (not the reported value) feeds the
  *next* cycle's cutback decision, one control period stale by construction — this backend
  reads wheel truth only after `Plant::step`, one line too late for the same cycle's cutback,
  unlike the batch-structured Python scenarios. Documented in `last_wheel_rate_rad_s`'s field
  comment rather than hidden.
- `run_metadata().imperfection_profile_id` stamps the real id (UTF-8 bytes, zero-padded to 32
  — not hashed, since a hash would need this crate to reproduce a Python-side serialisation
  scheme that doesn't exist yet) and stays `None` only when the profile is genuinely `IDEAL`.
- Default profile is `IDEAL` (`SimBackend::default()`/`new()`/`with_params()` unchanged;
  `with_profile()` opts in), so every pre-#129 caller's behaviour is bit-for-bit unchanged —
  verified by running `tests/test_rust_hosted_impulse_response.py` and
  `tests/test_rust_python_plant_replay_equivalence.py` locally against the built binaries, not
  just asserted from reading the code.

New `crates/sim-backend/tests/imperfection_conformance.rs`: shells out to
`python -m sim.scenarios.imperfections --emit-conformance-vectors -` (generated, not
committed, per that module's own instruction) and checks every deterministic row bit-for-bit.

## A real bug the conformance test caught in its own harness, not in the code under test

`serde_json`'s default `f64` parser is fast but **not correctly rounded** — it returned a
value 1 ULP off Python's own for a real vector during development here
(`9.753086419753087` vs `...089`, bits `...cd6f` vs `...cd70`). Diagnosed by writing a
throwaway example binary and comparing bit patterns at each stage (JSON text → parsed f64 →
computed f64) against a hand-rolled Rust repro that matched Python exactly, until the one step
that didn't match turned out to be serde_json's own parse, not anything in `imperfections.rs`.
Fixed by enabling serde_json's `float_roundtrip` feature (dev-dependency only). Without it the
conformance test would have been checking Rust's output against numbers Python never actually
produced — a false-negative machine dressed as a bit-for-bit gate.

## CI

`rust` job now also installs pinned `numpy` (from `requirements-sim.txt`, same pin the `sim`
job uses) alongside the existing `mujoco` install, rather than relying on it being pulled in
transitively by mujoco's own dependency resolution — that transitive pull happened to satisfy
the import during local testing, but which numpy version it resolves to is an accident of
mujoco's requirements, not a guarantee against the same pin `sim` gates on.

## Acceptance criteria (issue #129)

All eight checked: chain order, fractional delay, round-half-to-even quantisation + correct
hold-refresh semantics, bit-for-bit Rust test against the generated vectors, distributional +
reproducible gyro/accel noise, `imperfection_profile_id` stamping, generator invoked rather
than a committed fixture, `test_rust_hosted_impulse_response.py`'s `profile=IDEAL` untouched
(and re-verified green, not just left alone).

## Deliberately left out, flagged rather than fixed

- **`erpm_effective_age_ns` stays hardcoded `0`.** True staleness tracking needs its own state
  (ns since the wheel-rate hold last actually refreshed), and for the one model this backend
  steps (`overboard_onewheel.xml`, 2 ms) every STAGE0 profile's `wheel_rate_update_hz` (500 Hz)
  matches the control rate exactly, so the hold never actually goes stale — `0` stays honest
  for every profile this crate can currently be run with. Would need revisiting if this crate
  ever steps the bench-rig model (0.5 ms) or a profile with a slower update rate.
- **No physically-realistic end-to-end test of the cutback cap actually binding.** Driving the
  simulated wheel to `derate_onset_rad_s` (27 rad/s) from rest inside a bounded test needs
  knowing how the board's real inertia and current respond, which isn't this crate's call to
  assume. The cutback *math* is proven bit-for-bit by the conformance test and unit-tested in
  isolation; the *backend wiring* is proven by a rest-state test (cutback correctly does NOT
  engage at true wheel rate ≈ 0) rather than a claim that it engages under load.
- **`SimBackend::commanded_current_a`/`applied_current_a` naming, not touched further.** Now
  carries real meaning (pre-chain vs. post-chain), documented on the fields; did not rename
  the public `applied_current_a()` accessor, which callers already depend on.
