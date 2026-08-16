# Issue #229 — ADR-0011 criterion (g), a runtime damping override and the sweep itself

**PR:** #TBD (`fix/controls/damping-sweep-knob`)

## What

`wheel_hinge`'s `damping="0.08"` (both MJCF models) could previously only be varied by
editing `sim/models/` (Sr. Mechanical & Systems' fidelity contract) and rebuilding for
every sweep point — so ADR-0011 criterion (g) ("every pass must hold across a damping
sweep of 0.5x-2x") had never actually been taken, at either the 40 A or 60 A operating
point. Added a runtime knob mirroring `--incline-deg`'s own pattern exactly:

- `plant-mujoco`: `plant_mujoco_get_dof_damping`/`plant_mujoco_set_dof_damping` (shim.c),
  `Plant::dof_damping`/`Plant::set_dof_damping` (safe wrapper, same "before the first
  step" panic contract `set_gravity` carries).
- `sim-backend`: `SimBackend::set_damping_scale(Option<f64>)`, applied at `open()`
  against `wheel_hinge`'s dofadr. Deliberately `Option<f64>` rather than an `f64`
  defaulting to `1.0` — `#[derive(Default)]` would silently default to `0.0`, a real
  (and destructive) multiplier, not an identity value. Not re-applied in `reset()` the
  way the incline is: `dof_damping` is a model field `mj_resetData` does not touch, and
  unlike `set_gravity`'s reapply (which is idempotent because rotation preserves
  magnitude), reapplying `base * scale` from an already-scaled value would compound on
  every reset.
- `sim-host`: `--damping-scale SCALE` CLI flag, threaded through `HostConfig`.

## The measurement (Ask 2)

Took the sweep at the current 40 A operating point (the 60 A packet is withdrawn per
ADR-0011's second ratification). **Criterion (g), taken literally, fails.** Both
deployed-path entries of the criterion (a) matrix hold cleanly through 1.4x
(peak lean 9.0°) and invert at 1.5x (8.04 s) and every point above it — a real cliff,
not numerical noise, reaching full inversion by 5.64 s at 2.0x. At the model's own
declared damping (1.0x) both scenarios hold with real margin; this is not a claim that
today's board is unsafe, it is a measured, false robustness claim in the exact place
the criterion exists to check for. Full sweep table in
`tests/test_damping_sweep.py::test_the_acceptance_matrix_does_not_hold_across_the_full_0_5x_2x_sweep`,
written `xfail(strict=True)` per this repo's own convention for a measured, not-yet-fixed
criterion — so it turns XPASS the day someone lands a fix, rather than staying a silent
green.

## Deliberately left out (Ask 3)

Whether this blocks the launch-hold exit or is re-homed to the hardware gate (the way
criterion (a)-3, the kerb strike, already was) is a decision, not a measurement, and not
mine to make alone — flagged in the issue and in the PR rather than decided here.

## Verification

`cargo build`/`clippy -D warnings`/`test --workspace` clean on the touched crates (no
regressions: 17+37+39 pre-existing tests still pass, plus 5 new ones — 2 Rust unit tests,
3 Python). `cargo fmt` clean. `python3 -m pytest tests/test_damping_sweep.py` — 3 passed,
1 xfailed as designed. `python3 .github/policy_check.py` — all hard checks pass, every
touched path resolves to Senior Controls turf. `cargo run -p xtask -- gate` — unrelated
pre-existing finding only (`hal-actuate`/`plant-mujoco` unreachable from
`board-app-ridden`).
