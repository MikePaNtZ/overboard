# 2026-08-13 — Issue #161: W1/W2's four deliverables are already on master

Cron dispatch pass. Open `role:senior-controls` issues with no PR against them, after excluding
issues already worked by other dispatches (#168/#169/#133/#142 all have open PRs now — #267/#245/
#275/#274), #182 (the Stage-0B epic — most of its increments are already tracked and dispatched as
their own separate issues, so it is not itself a single well-scoped unit), #132 (blocked on a bench
`kt` measurement that Sr. Mechanical & Systems has not produced yet — not executable from this
role alone), and #61 (`Dispatch: COO only`, reserved): #161 was the one workable and unclaimed.

## What #161 asked for

Launch-weekend issue (2026-08-01/02, design gate waived by the CEO for that weekend). One
non-negotiable — "the real control law runs the board, or we do not ship the game" — split into
two done-when gates:

- **W1 (tonight):** a `sim-host` running `plant-mujoco` + `control-ffi`'s real cascade at a fixed
  500 Hz on its own dedicated thread, UDP out (pose/wheel angle/sim time) and UDP in (player
  input). Done when the board is visible and moving in Unreal under the real control law.
- **W2 (Sunday morning):** a ballasted rider model in a *separate* model file (not
  `overboard_onewheel.xml`, to protect the bit-identity replay gate), a widened wheel geom labelled
  as a playability divergence, and a roll-shaped yaw limiter as the roll stopgap. Done when a
  person can drive forward, stop, reverse and turn without face-planting every ten seconds.

## Where each landed

All four pieces are on master today, and have been for some time — the issue was simply never
closed:

- **500 Hz dedicated-thread host, UDP in/out** — `crates/sim-host/src/host.rs` line 1 states it
  outright: `//! The 500 Hz control loop, dedicated thread, UDP in/out (issue #161).` The file is
  5,472 lines and has grown far past the launch-weekend scope into everything ADR-0011 and the
  Stage-0B work since have needed from it — `wire.rs`/`pacer.rs`/`scenario.rs` are the same
  lineage.
- **Ballasted rider model, separate file** — `sim/models/overboard_rider.xml` exists alongside
  (not merged into) `sim/models/overboard_onewheel.xml`.
- **Widened wheel geom, labelled** — `overboard_rider.xml`'s header states it explicitly: "The
  wheel geom is WIDENED for playability: half-width 0.075 m -> 0.15", with a second note that this
  is "a playability widening, not a claim about" the hardware geometry, and the geom itself is
  commented "WIDENED for playability, see this file's header" at the point of use — exactly the
  "must not silently propagate into a controls result" requirement in the issue.
- **Roll-shaped yaw limiter** — `yaw_rate_rad_s(steer, forward_speed_m_s, roll_authority)` in
  `host.rs`, gated by `roll_authority`/`YAW_AUTHORITY_FLOOR`, called from the main loop to produce
  `dyaw_rad`.

## Verification performed here

Fresh `python3.12 -m venv` + `pip install -r requirements-sim.txt`, `cargo build --workspace
--release` against it, then:

- `cargo test -p sim-host --release` — 39 passed, 0 failed.
- `cargo test --workspace --release` — every crate green, 0 failed across the workspace.
- `pytest tests/` (full suite) — 333 passed, 5 xfailed, 0 failed.
- `python3 .github/policy_check.py` — all hard checks pass (pre-existing advisory notices only,
  unrelated to this change).

## Deliberately left out

- Did not touch `crates/sim-host` itself — nothing here needed fixing, this is a stale-issue
  closure like #142's and #230's before it.
- Did not chase the "Explicitly NOT this weekend" list (terrain conformance, record/replay,
  lean-steer roll controller, disturbance observer, bench rig, reconstruction, tyre carving, MCAP
  telemetry, ABI v2) — those were deliberately deferred by the issue itself and have their own
  tracking elsewhere (several already landed, several still open under their own issue numbers).
- Did not verify the Unreal-side "done when" wording literally (a person driving in-engine) —
  that half is `overboard-game` turf; this issue's four ACs were framed for the controls/sim side,
  and I only own the artifacts this repo can produce.
