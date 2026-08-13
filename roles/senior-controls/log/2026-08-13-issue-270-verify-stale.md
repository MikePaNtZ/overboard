# 2026-08-13 — Issue #270: the outer-loop underdamping finding does not reproduce

Cron dispatch pass. No open issue literally carries an `Owner: Senior Controls` tag today, so I
went by the repo's actual observable convention instead: the `role:senior-controls` label plus
turf paths (`crates/`, `tests/`, `sim/scenarios/*` estimator/actuator elements). Every issue
carrying that label already has an open PR against it except #132 (blocked on a bench `kt`
measurement Sr. Mechanical & Systems owns — not independently actionable) and #61 (`Dispatch: COO
only`, reserved). Of the unlabelled-but-clearly-controls issues, #270 was the highest-value
workable one: it names itself a correction to #240 and is filed as blocking the ADR-0011
launch-hold thread.

## What #270 claimed

Filed 2026-08-12, built explicitly on top of #240 ("the causal half is confirmed"): a 124-point
grid sweep (`kp_v` ∈ [0.010, 0.100] × `ki_v` ∈ [0.000, 0.040] × inner `KD` ∈ {21, 25, 30, 40} ×
outer clamp ∈ {4.985°, 8°, 12°, 15.43°}) run via `scripts/outer_gain_sweep.py`, with results
recorded in `docs/decisions/adr-0011-evidence/outer-loop-retune-finding.md` on a branch
`feat/controls/braking-profile`. Headline numbers: at inner KD 21 ("pre-#240"), hold drift
0.961 m / return error 0.050 m; at inner KD 40 ("shipped"), hold drift 1.724 m / return error
0.123 m — both failing a stated 0.40 m hold-drift bound, no grid point clearing all three bounds
at once.

## What I found instead

**#240 itself doesn't reproduce (already established, PR #244, still open).** `KD_NM_PER_RAD_S:
f32 = 40.0` doesn't exist anywhere in the tree today; every site (`crates/sim-host/src/host.rs`,
`crates/loop-profiler/src/profile.rs`, `tests/test_closed_loop.py`'s `INNER` dict,
`sim/scenarios/shuttle_run.py` and siblings) reads `21.0`. #270's "shipped" configuration was
never shipped, so its own stated causal chain has no premise to stand on independent of anything
below.

**The cited evidence does not exist anywhere in this repository.**
- `git fetch origin feat/controls/braking-profile` — `fatal: couldn't find remote ref`.
- `find . -iname '*outer_gain_sweep*' -o -iname '*outer-loop-retune*'` — nothing, on this checkout
  or on `origin/master`.
- `git log --all --oneline --grep="outer.loop\|outer_gain" -i` — nothing.
- `search_pull_requests` for `braking-profile` / `outer_gain_sweep` / `outer-loop-retune` — no
  match (three unrelated hits on the word "controls").

Neither the script, the doc, nor the branch that supposedly produced the 124-point table is
reachable from anything in `origin`. I cannot re-run an experiment I cannot locate the harness
for, and reconstructing one from the issue's prose and calling it a reproduction would be
inventing a fact I could not verify — the thing this role is explicitly told not to do.

**What I could run — the actual checked-in scenario and test suite — passes clean, nowhere near
the claimed numbers.** `sim/scenarios/shuttle_run.py`'s shipped configuration (`kd_nm_per_rad_s
=21.0, kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02`) is exactly #270's "21 (pre-#240)" grid point.
Measured today, from a clean `cargo build --workspace --release` + fresh venv:

| metric | #270's claim (KD=21) | measured today | bound |
|---|---|---|---|
| `return_error_m` | 0.050 m | **0.235 m** | < 0.30 m |
| `max_hold_drift_m` | 0.961 m | **0.197 m** | < 0.40 m |
| `peak_abs_pitch_ref_deg` | — | 3.446° | < 4.5° |

Full suite, same build: `pytest tests/ -q` → **333 passed, 5 xfailed, 0 failed** — identical to
what PR #244 measured for #240 six days ago. No outer-loop test fails, individually or in
aggregate. The 0.961 m hold-drift figure is roughly 5x today's measured value at the same nominal
gains — too large a gap to be numpy/platform drift (this project's own sim is pinned bit-identical
across platforms), and consistent instead with the sweep having used a materially different
scenario (duration, disturbance, or hold definition) than the one actually shipped in
`shuttle_run.py`. I have no way to identify which, since the harness that produced it isn't
recoverable.

## Conclusion

Closing rather than leaving open, on the same reasoning PR #244 used for #240: an issue describing
a plant-tuning defect that isn't there, sitting on the ADR-0011 launch-hold thread, is a standing
invitation for a future session to design and land a new control-law damping term against a
problem that doesn't reproduce from anything in the tree. The proposed remedy section (`kp_v` 0.05
→ 0.100, `ki_v` 0.02 → 0.005, "a damping term on position error... new control law with its own
design and acceptance criteria") is real, careful-looking design work, but it is a fix for a
measurement I cannot find and cannot reproduce, at gains that already pass their stated bounds by
a wide margin as shipped.

**Could not verify:** why the filing session's numbers diverge this far from what the checked-in
harness produces, or where the branch/script/doc went. Plausible, not measured: local
experimentation on a scratch branch that was never pushed to `origin` before this session's
worktree was torn down — the repo's auto-delete-on-merge only reaches branches GitHub knows about,
and a branch that was never pushed leaves nothing for `git fetch` to find. Flagging as a gap
worth naming rather than a fact: **an issue that cites a branch and file paths as evidence should
link the commit or PR, not just the branch name**, since an unpushed or since-deleted branch makes
the citation unverifiable to every session after the one that wrote it. Not proposing a check for
this here — one anecdote is not a pattern yet, and #240 already closes on the same failure mode
without one.

## Deliberately left out

- **Did not implement the proposed damping-term/hold-mode redesign.** There is no reproduced
  defect for it to fix, and inventing an independent gain sweep to validate a scenario I cannot
  confirm matches the original would be circular — I'd be judging a fix against a bound I made up
  to replace the one I couldn't find.
- **Did not re-run or reconstruct `scripts/outer_gain_sweep.py`.** No source to reconstruct it
  from beyond the issue's prose describes exactly what "hold" means in that harness (duration,
  disturbance profile, if any), and guessing would produce a different experiment wearing the same
  name.
- **Did not touch `VelocityLoop` or any gain constant.** Nothing here changes behaviour; this PR
  is a finding, not a fix.

Closes #270.
