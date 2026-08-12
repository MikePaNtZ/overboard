# 2026-08-12 — Issue #201: turn radius was too wide, doubled the curvature gain

CEO's own report: "turn radius is too wide by maybe 2x to turn around completely," against
an ask of roughly 3.5 m. `YAW_CURVATURE_PER_STEER_RAD_PER_M` (0.15) had only ever been
quoted from `1 / k` ≈ 6.7 m — never actually driven through a turn and measured. PR TBD.

## What was measured, and why the formula alone wasn't enough

`host.rs`'s full-sim yaw law also passes through `roll_authority`
(`YAW_AUTHORITY_FLOOR`/`ROLL_FULL_YAW_AUTHORITY_RAD`) and the low-speed tighten ramp,
neither of which the plain `1 / k` formula accounts for, and both were added after this
constant was first picked. So this needed a real measurement, not a re-quote.

Added `--scripted-scenario turnaround` (`scenario.rs`) — full steer/lateral and 0.6 lean
(issue #201's own acceptance criterion) from a standing start — and measured the ground
track's LOCAL radius (ground speed / heading rate, `tests/test_turn_radius.py`), not a
whole-run circle fit. This plant has no outer velocity loop, so a sustained lean keeps
accelerating the board the whole time; radius is only well-defined instant-to-instant, not
as a run-wide constant, so a global circle fit assumes something that isn't true here.

**BEFORE (0.15):** measured 6.16 m at 93% of the reference speed, climbing toward the
formula's 6.67 m asymptote as speed approached it — i.e. the formula was right,
`roll_authority` saturates to ~1.0 in practice, and the radius really was that wide.

**AFTER (0.30):** median 3.284 m (10th-90th percentile spread 0.096 m) across the
near-reference-speed tail of the hold — comfortably inside the 3.6 m acceptance ceiling,
and comfortably inside the drivable corridor throughout (max lateral excursion 5.05 m
against an 8.6 m half-width) — where the *pre-fix* radius would not have fit.

## What salvaged from the stale branch, and what didn't

`feat/controls/turn-radius-and-reset` (issue #201's own prior WIP, NEEDS-REWORK, do not
merge) is many commits behind `master` and its `host.rs` diff deletes several ADR-0012 and
ADR-0011-matrix features that master has since grown — not usable as-is. Salvaged only the
idea of a dedicated `turnaround` schedule; the actual schedule shape is new (this one holds
0.6 lean straight through the turn instead of coasting through it, because coasting lets
speed — and therefore curvature — drift the whole way round, which is what made the stale
branch's own circle fit unfittable in the first place).

## Deliberately left out

- `feat/controls/turn-radius-and-reset`'s "reset" half is untouched — issue #201 doesn't
  ask for it, and the issue's own note says it's tracked separately.
- No change to `roll_authority`, the low-speed tighten ramp, or anything else in the yaw
  law besides the one constant issue #201 names.

## A local false alarm, corrected before reporting anything

`cargo run -p xtask -- gate` failed in this session's sandbox on an unmodified `master`
checkout (`hal-actuate, plant-mujoco are unreachable from board-app-ridden`), which looked
like a real regression on master. Checked against the actual GitHub Actions run for that
exact commit (2e41fc73) before reporting it: the `crate-exclusion boundary gate` step
passed there, and every other job did too. So this was an artefact of this sandbox's own
incremental-build state (crates built piecemeal, one at a time, ahead of the gate check,
which can change what `cargo metadata`'s feature unification sees), not a real defect on
master. Recorded here so a future session doesn't rediscover the same false alarm.
