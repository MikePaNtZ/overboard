# Issue #194 — disturbance forces are world-frame, and that is now heading-dependent

**PR:** fix/controls/disturbance-force-world-frame-194 (TBD number)

## What was decided

Every disturbance this repo fires today — the startup kick, the on-demand fall
kick, and the ADR-0011 scheduled kerb impulse — is a physically world-anchored
event (a kerb does not rotate with the board; neither does an exogenous
shove). **World frame is the correct and already-intended convention for all
of them.** No call site needs body-frame semantics today, so no rotation
support was added. `apply_external_force`'s doc comment and the `Disturbance`
struct's field docs already stated "world frame" explicitly before this PR —
that part of the issue was already satisfied on master.

## What changed

- Extracted the inline force/torque selection in `host.rs::run()` into
  `select_disturbance_force_torque`, a pure function that takes no
  heading/yaw parameter at all — the contract is now enforced by the type
  signature, not just a comment (matches this repo's general "encode it,
  don't just document it" pattern, e.g. issue #208).
- Two new unit tests: one pins that the function is heading-independent by
  construction (same inputs -> same output, nothing to rotate even if a
  future edit wanted to), one is a refactor-safety net for window precedence
  (scheduled disturbance > startup kick > fall kick > none), unchanged from
  the inline form it replaced.
- `STARTUP_KICK_FORCE_N` / `FALL_KICK_FORCE_N` doc comments now state the
  frame explicitly per-constant, not just at the mechanism. Also noted:
  the startup kick has never been able to land at non-zero heading (fires at
  t=1.0s, before any steer); the fall kick is operator-triggered and CAN land
  mid-carve, which is the one live path the original issue flagged.
- `sim/scenarios/impulse_response.py`'s `ImpulseParams.direction` docstring
  now states the same world-frame convention and cross-references the Rust
  side, so the contract reads the same on both languages' disturbance APIs.

## Deliberately left out

No body-frame rotation was added to `SimBackend`/`apply_external_force` —
nothing on hand needs it. If a future scenario wants a "push the nose" that
tracks the board mid-carve, that is a body-frame push and belongs at the call
site (rotate the vector by current heading before calling
`select_disturbance_force_torque`), not a hidden frame flag added here.

## Verification

`cargo test --workspace` clean (all crates), `cargo clippy -p sim-host
--all-targets -- -D warnings` clean, `cargo fmt -p sim-host -- --check`
clean, `pytest tests/test_impulse_response.py` (16 tests) clean,
`python3 .github/policy_check.py` — all hard checks pass (10 roles, 42
ownership rules); only pre-existing advisories, none introduced by this
change.

## Also noticed, out of scope

Issue #271 ("Allow steering during a tail brake") describes code that does
not exist anywhere on current `master` — no `EngageState` enum, no
`TailBraking` state, nothing matching `tail.brak`/`TailBrak` in the whole
tracked tree or in `git log -S` history on this branch. `grep`/`git grep`
across `crates/`, `docs/decisions/ADR-0011-*.md`, and the full repo turned up
nothing. Flagging rather than picking it up: either the issue is stale (the
described feature was never actually landed, contrary to its "CEO decision
2026-08-12" framing), or it references a branch this shallow clone cannot
see. Worth a `verify-stale`-style check before anyone starts building the
described `EngageState::TailBraking` match arm.
