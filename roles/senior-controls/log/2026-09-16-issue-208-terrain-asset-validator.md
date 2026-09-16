# 2026-09-16 — Issue #208: the world-authoring asset rule (ADR-0011 condition 2)

Cron dispatch pass. `#151`'s comment thread records the CEO's "Turn it on" (2026-08-01),
confirmed by reading the thread directly rather than trusting a prior session's summary of it.

## Why this issue

Read all 23 currently-open issues' bodies for an `Owner: Senior Controls` / `Dispatch: cron OK`
tag; as every prior cron pass in this directory has also found, neither literal tag exists
anywhere in the queue except `Dispatch: COO only` on #61. Went by turf + open-PR-check instead,
the established convention.

Of the turf-mine, not-already-PR'd, not-blocked candidates, **#208** (the world-authoring asset
rule) was the clear pick:

- **#286, #285, #284, #261, #202, #216, #182, #168, #132** — each already has an open PR.
- **#204, #235** — sequencing / open-design-question blocked, per PR #295's log entry from
  yesterday's pass (re-checked, still true today: #239 for #202 is still open).
- **#265, #217, #203, #159, #156, #160, #162, #163** — Digital Content Production / Game
  Engineer / COO / Sr. Mechanical & Systems turf.
- **#151, #33** — resolved / not dispatchable (a conversation).
- **#208** — turf-mine (verified myself, not assumed — see below), has written acceptance
  criteria, no open PR against it, and PR #295 (still open, unmerged) explicitly names it "the
  next Senior Controls pickup."

## Turf, verified independently

```
$ python3 .github/policy_check.py --who sim/scenarios/terrain_validate.py
  owner : Senior Controls  [Ratified]
  rule  : /sim/  (CODEOWNERS:78)
$ python3 .github/policy_check.py --who sim/assets/terrain
  owner : Senior Controls  [Ratified]
  rule  : /sim/  (CODEOWNERS:78)
```

Did not just trust PR #295's log claiming the same thing — re-ran the command myself against
this checkout.

## What I built

`sim/scenarios/terrain_validate.py`: a validator for a one-dimensional heightfield asset (the
same shape `sim/scenarios/terrain.py` tiles across a MuJoCo `<hfield>`'s rows) against two
independent limits:

- **Step**: an adjacent-sample height delta above `max_step_m` (default 0.5 mm). Only checked
  when the asset's sample spacing is finer than `STEP_CHECK_MAX_DX_M` (2 cm) — see "a real design
  bug, caught by my own test" below for why.
- **Slope**: local grade above `max_slope_deg` (default 3.0°), dx-independent by construction.

Both defaults cite where the numbers they sit inside of come from, in code comments, rather than
inventing new ones:

- Step: half of "rides out roughly a 1 mm lip at a calm point... nothing at its worst point"
  (`tests/test_cmd_envelope_reserve.py`'s xfail reason on criterion (a)-3). Half, not the full
  figure, because the same sentence says worst-phase survival is zero.
- Slope: comfortably inside both the 6.5° corridor-brake arrest boundary
  (`tests/test_incline_tolerance.py`) and the 8%-grade (4.57°) estimator-completion ceiling
  (`sim/scenarios/terrain.py`'s `TerrainParams` docstring) — two independent measurements landing
  in the same few degrees.

Neither is a restatement of a threshold: `test_incline_tolerance.py` says explicitly that none of
its numbers "may be promoted to one," and that the margin is "the world-authoring role's to
write." I judged the margin above; I did not invent the measurements it sits inside of.

`sim/assets/terrain/example_gentle_ramp.json`: one reference asset (a 1% linear ramp) so there is
something for AC1's "shipped terrain assets" language to mean today, and so the validator has a
real file to exercise in CI, not just synthetic in-memory cases. Explicitly documented as a
demo/reference, not a claim about the actual game world (Game Engineer's `overboard-game` role
per ADR-0009 and the repo boundary in `CLAUDE.md`).

`tests/test_terrain_validate.py`: seven tests. Two build a deliberately-violating asset on disk
and run the real file-loading path against it (AC1 — "reading the validator is not evidence it
works"): one isolates a step violation, one isolates a slope violation from a step by sampling
finely enough that the per-sample delta stays under the step limit while the slope still exceeds
it. One asserts every shipped asset validates clean, which is the actual CI gate (AC3) — it runs
inside `pytest tests/ -v`, which the `sim` CI job already runs unconditionally on every push and
PR, so no separate path-triggered workflow was needed for AC3.

## A real design bug, caught by my own test, not by inspection

First draft made the step check unconditional (dx-independent absolute delta, always). A test
for "a gentle 1° ramp at 0.5 m spacing should pass" failed: at that spacing a 1% grade rises
~8.7 mm per sample, which is (correctly, on the numbers) larger than the 0.5 mm step limit —
except it is not a step at all, it is the same smooth grade the slope check already accepts,
sampled coarsely. The interpolated surface between those two grid points is still a 1% ramp.

Fixed by making the step check apply only below `STEP_CHECK_MAX_DX_M` (2 cm, chosen to sit well
under the ~20 mm kerb feature scale `sim/scenarios/plant.py`'s `KERB_STRIKE_VALIDITY` already
uses) — a heightfield sampled coarser than that cannot represent a genuine discontinuity any
finer than its own spacing, so there is nothing meaningful to flag, and flagging it anyway would
be a false positive on every ordinary authored grade. Added
`test_a_coarse_ramp_is_not_falsely_flagged_as_a_step` to pin this regression explicitly rather
than leaving it only implicit in the passing case that first caught it.

## Verified, not assumed

- `pytest tests/test_terrain_validate.py -v`: 7 passed (both this sandbox's bare `python3.11`,
  which has no numpy/mujoco and doesn't need them for this file, and the pinned
  `mujoco==3.10.0`/`numpy==2.5.1`/`pytest==8.4.2` venv the rest of the suite needs).
- `pytest tests/ --collect-only`: 362 tests collect cleanly (up from 355 before this pass) — no
  import errors introduced.
- `python3 .github/policy_check.py`: passes.
- Full `cargo build --release --workspace --all-targets` + `pytest tests/ -v`: kicked off:
  <build/test result filled in before PR is opened — see the PR body for the actual numbers, not
  restated here in case it finishes after this file is written>.

## Deliberately left out

- **Run-out length.** `test_incline_tolerance.py` measures that the rule "needs an angle AND a
  run-out length" — a slope that is individually harmless produces unbounded speed if it runs
  long enough, because there is no speed loop. This file checks slope pointwise only. Adding a
  cumulative-run-length check is a distinct, well-scoped follow-up, not bundled in here.
- **A 2D height grid.** Nothing in this repo authors one yet; the 1D profile matches what
  `sim/scenarios/terrain.py` actually tiles into MuJoCo today. Inventing a richer format nobody
  produces would be speculative.
- **Re-deriving the KD-channel control-authority figure** (the "~76°/s at zero lean" the kerb
  xfail reason cites) to compute per-height survivability dynamically, instead of a fixed step
  limit. That number is derived in Rust from a torque constant and current limit I did not want
  to reconstruct from memory into a value that gates CI — a wrong reconstruction would be exactly
  the fabricated-constant failure mode this project rules out explicitly. Used the plainer, safer
  reading of the issue's own "actual ask" instead: fixed, cited, margined limits.
- Did **not** re-litigate PR #295's ADR-0011-third-ratification claim (see the CONTEXT.md note
  above) — out of scope for this pass, which only needed the turf finding.
