# Issue #250 — a near-zero accelerometer reading no longer resolves to a confident 180°

`ComplementaryFilter::accel_pitch` computes `atan2(f_x - a, -f_z)`. Fed a (near) zero
specific-force vector — any brief unloading: cresting a rise, a kerb, a drop, a jump — this
still returns a confident angle, most often 180° exactly (`atan2(0, -0)`), and the old
estimator trusted it immediately. The controller then saturates corrective current at the
full rail against an attitude that was never measured.

## What changed

`crates/control-core/src/lib.rs`: `MIN_TRUSTED_ACCEL_MAG_M_S2` (half of g, derived not
picked — see the doc comment) gates `accel_trusted` **unconditionally**, independent of the
existing opt-in `accel_trust_band_m_s2` disturbance-rejection knob (which stays exactly as
documented, off by default, caller's choice). Below the floor the estimator coasts on the
gyro instead of fusing a directionless vector — both in steady-state fusion and, separately,
on the very first sample (the `!initialised` snap-to-accelerometer path), so a board that
spawns or resumes out of ground contact no longer initialises on the degenerate angle either.

Two new unit tests in `control-core` pin this: a first-sample free-fall reading does not
snap to a degenerate angle and initialises cleanly on the next trusted sample; sustained
free-fall for 5 s (several τ) never converges toward 180°, with every sample counted in
`rejected_samples()`.

This is unconditional and applies to every existing caller (`sim-host`'s two
`ComplementaryFilter::with_trust_band(ESTIMATOR_TAU_S, 0.0)` call sites needed no change —
the floor applies regardless of the trust-band argument — and every `control-ffi` consumer,
including the Python scenario harness via `rust_controller.py`).

## The consequence nobody asked for, found by actually running the suite

That last point is not free. Re-running the full suite (as this org's convention requires
before opening a PR, not just the crate under direct edit) found `tests/test_terrain.py`
red: 4 of its GATE assertions, including the file's own stated headline ("a rolling profile
is harder than a steady grade of the same steepness"), no longer held. Root-caused rather
than reverted: the terrain scenario's dip and transitions produce genuine near-unloading
moments that the OLD estimator mishandled via this exact defect, which was (at least
partly) what these assertions were measuring instead of what they claimed to measure.

Re-swept grade rather than guessed a replacement number:
- Steady grade's own survival ceiling (10.0%) is **unchanged** by this fix — verified
  against unfixed code directly (`git stash`), reproduces identically. Not something this
  fix touches.
- Rolling terrain's ceiling moved from 10.0% to 10.5%, i.e. it now has MORE margin than
  steady, not less. There is a real, repeatable 10.1–10.5% window where the relationship
  is the exact opposite of the file's original headline.
- The truth-vs-estimate comparison still holds in the original shape, just at a re-derived
  grade: 11.0% (from a measured 10.6–11.4% window; 11.6% already flips back — the same
  non-monotonicity issue #24 AC2's disturbance sweep already documented for this codebase).
- The dip/descent estimator-error-RMS ordering did not shift, it inverted categorically —
  true at every grade swept, 2–9%. **Not root-caused** — flagged with a hypothesis (the dip
  is where curvature is most active) but not asserted as fact, matching this same file's
  own established precedent from issue #228 for not chasing a mechanism outside a pass's
  scope.

`tests/test_terrain.py` is re-pinned to the newly measured numbers, each with the sweep
that produced it recorded in the test docstring and the module docstring's new
"RE-MEASURED AFTER ISSUE #250" section, not just the new pass/fail direction.

## Deliberately left out

- **Issue #250's asks 2 and 3** (defined recovery behaviour for a sustained low-g drop where
  gyro-only integration drifts; whether this deserves a `Faults` bit). Both are real design
  questions with no numeric acceptance criterion given in the issue — "decide" and
  "consider," not a stated target — and are exactly the kind of judgement call this session's
  own scope-discipline guidance says to flag rather than invent an answer for under an
  unattended run. Left for a deliberate pass.
- **Whether the rolling-vs-steady headline is true at some OTHER grade or parametrisation,
  independent of this bug.** Not re-examined; the original "transitions cost margin"
  hypothesis is neither confirmed nor refuted here, only shown to have been entangled with
  a defect at the specific 10% test point it was pinned at.
- **Root-causing why the dip specifically inverts.** Flagged as a hypothesis, not chased.
- **ADR-0011 itself and issue #227.** Not edited, not closed. This finding bears directly on
  both (issue #227 is literally about whether the freeze-and-pin generalises past flat
  ground; this session found a second, independent reason the terrain-scenario numbers
  around it were unreliable) — flagged prominently in the PR for the COO, not resolved here.
  `docs/` is COO turf regardless.
- **`crates/control-ffi`'s own default `Params` and the FFI struct's field defaults were
  left untouched** — the new floor is unconditional at the `ComplementaryFilter` level, so
  it already protects every FFI consumer without needing a default changed at that
  boundary too.

## Verification

- `cargo test -p control-core` — 37/37 (35 pre-existing + 2 new).
- `cargo test --workspace` — clean, no regressions.
- `cargo clippy --workspace --all-targets -- -D warnings`, `cargo fmt --all -- --check`,
  `cargo run -p xtask -- gate` — all clean.
- `python3 -m pytest tests/` — 333 passed, 5 xfailed (same counts as pre-fix baseline;
  the 4 that flipped are re-pinned, not skipped or deleted).
- `python3 .github/policy_check.py` — all hard checks pass.
