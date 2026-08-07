# 2026-08-07 — Issue #240: the reported 24 test failures do not reproduce

Cron dispatch pass. #240 has no `Owner:` tag, but the domain (cascade/outer-loop gains,
`crates/`/`tests/`) is unambiguously Controls turf and nothing else in the open-issue queue was
both workable and unclaimed, so I picked it up.

## What #240 claimed

Filed 2026-08-06 14:26 UTC against master, reporting **24 failed / 343 passed / 5 xfailed,
byte-identical across three consecutive runs**, all three failures in the outer velocity/position
loop (`test_shuttle_run.py::test_it_holds_station_during_the_pauses`,
`test_closed_loop.py::test_the_cascade_brings_the_ridden_board_back_to_rest`,
`test_hill.py::test_the_board_holds_a_medium_descent[1.0]`), with a stated hypothesis: the inner
loop's `KD_NM_PER_RAD_S` went 21 → 40 during the real-motor-constant retune and the outer loop's
gains were never re-derived against the new inner plant.

## What I measured instead

1. **Current master (`a2c21ff`), full suite, clean venv + `cargo build --workspace --release`:**
   `333 passed, 5 xfailed, 0 failed`. The three named tests pass individually.
2. **No `KD_NM_PER_RAD_S: f32 = 40.0` anywhere in the tree.** Every site that sets it
   (`crates/sim-host/src/host.rs`, `crates/loop-profiler/src/profile.rs`,
   `tests/test_closed_loop.py`'s `INNER` dict, `test_delay_budget_stage0b.py`, `test_estimator.py`,
   `hill.py`, `shuttle_run.py`, `terrain.py`, `analyse_deadline_bursts.py`,
   `test_imperfections.py`) reads `21.0`, and `git log -p` on `test_closed_loop.py` shows the
   `INNER` dict has read `kd_nm_per_rad_s=21.0` since the line was first added — never edited. The
   stated hypothesis's premise (a `21 → 40` retune landed in the tree) isn't there to test.
3. **The commit that was master at filing time, isolated in its own worktree (`c85375c`, the ADR-0011
   third-ratification/drag-model commit — `a2c21ff` that followed it is docs-only, ADR-0012, no
   code diff):** clean `cargo build --workspace --release` (needed — the worktree had no build
   artifacts of its own), same venv, same suite: **`333 passed, 5 xfailed, 0 failed`**. The exact
   commit #240 says produced 24 deterministic failures produces zero, from a clean build.

## Conclusion

The described defect does not reproduce, at the commit it was filed against, from a from-scratch
build. Closing rather than leaving open — an open issue describing a plant-tuning regression that
isn't there is a standing invitation for a future session to retune gains against a problem that
doesn't exist, which is worse than no issue at all.

**Could not verify, and not claiming it:** *why* the filing session saw 24 failures. The likeliest
mechanism — a stale `libcontrol_ffi.so` linked against something other than what
`crates/control-ffi/src/lib.rs` currently says, e.g. left over from unrelated local
experimentation — fits (the FFI boundary is exactly what `test_the_control_library_is_actually_built`
checks for *existence* but not *freshness* of), but I did not reproduce a stale-library state to
confirm the mechanism, so this is a plausible explanation, not a measured one. Flagging the
freshness gap as a finding, not fixing it here — it's a `tests/` scope decision (what should
invalidate the "already built" check: mtime vs. source hash) that deserves its own increment
rather than riding in on an issue-closure PR.

## Deliberately left out

- Did not implement the outer-loop gain sweep #240 proposed as its confirming experiment — there
  is nothing here for it to confirm against.
- Did not touch the FFI build-freshness gap noted above.
