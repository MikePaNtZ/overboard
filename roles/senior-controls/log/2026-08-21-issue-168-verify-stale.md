# 2026-08-21 — Issue #168: the Pacer bug and jitter-percentiles ask are already closed

Cron dispatch pass. Open issues labeled `role:senior-controls`: #182 (Stage-0B umbrella, active,
increments I6–I8 are hardware/architecture work well past a single scoped increment), #168, #132
(blocked — its own default action requires Sr. Mechanical's bench-fitted `kt` first;
`sim/scenarios/plant.py`'s `KT_NM_PER_A` is still the unfitted 0.7 placeholder, so the
precondition for "Senior Controls owns the retune" does not exist yet), #61 (`Dispatch: COO
only`, reserved). Of that set, #168 had no PR open against it and turned out to already be
resolved.

## What #168 asked for

Filed after the COO's independent verification of the 500 Hz host's pacing: `Pacer`'s own doc
comment claimed a single slow cycle costs one miss, "not a cascading pile-up of catch-up
iterations" — measured data (macOS timer coalescing, ~15 ms stalls followed by ~8-tick bursts)
showed exactly that pile-up, so the comment described the opposite of the real behaviour. Four
asks: (1) fix the comment, (2) consider a real-time thread policy on macOS, (3) report jitter
percentiles instead of a bare miss count, (4) promote the miss count to a wire field. The issue
states its own deferrals in its own text — ask 2 is "Tuesday work, not weekend work," ask 4 is
"fine for W1."

## What I verified, not assumed

- `crates/sim-host/src/pacer.rs`'s doc comment on `wait_for_next` already carries a
  `CORRECTION (issue #168)` block describing the real behaviour (bounded catch-up bursts, correct
  on average, deliberately not reset-to-now) — ask 1, done.
- `Pacer::jitter_percentiles()` (nearest-rank p50/p99/max over a bounded 2,000-sample window) is
  implemented and covered by dedicated tests (`jitter_percentiles_are_empty_before_any_call`,
  `on_time_ticks_report_zero_jitter`, `one_late_tick_among_on_time_ones_shows_up_in_max_but_not_p50`,
  `the_jitter_window_forgets_samples_older_than_its_capacity`), and `host.rs`'s stats file /
  `wire-probe`'s report line both carry the percentiles alongside the existing miss count — ask 3,
  done. Landed in `120c3dc` (PR #267, 2026-08-08), which itself records "asks 2 and 4 are
  explicitly deferred by the issue itself" — but the PR's own commit message and its
  `roles/senior-controls/log/2026-08-08-issue-168-jitter-percentiles.md` entry both state
  up front that the PR "references #168, does not close it," and no later PR added a `Closes
  #168`. So the issue sat open with both in-scope asks already landed, same shape as #230
  (`roles/senior-controls/log/2026-08-09-issue-230-verify-stale.md`): a PR that satisfied an
  issue's stated acceptance without the magic-word trailer to auto-close it.
- Re-ran `cargo test -p sim-host --lib pacer` fresh on this checkout: 9/9 passing, unchanged.

Asks 2 and 4 are not silently dropped — they are exactly what the issue itself already named as
out of scope for now, not a gap this pass is choosing to ignore.

## Conclusion

Closing rather than leaving open. Both asks #168 actually required (the wrong comment, the
illegible bare miss count) are landed and tested on master; the two remaining asks are the
issue's own stated deferrals, not unfinished acceptance criteria. An open issue here is a standing
invitation for a future session to re-litigate a comment fix and a percentile feature that already
shipped.

## Deliberately left out

- Did not implement ask 2 (real-time thread policy on macOS) or ask 4 (wire field for the miss
  count) — both are the issue's own explicit deferrals, not something this pass is newly deciding
  to skip. Either is a fine future increment, filed fresh against its own acceptance criterion if
  someone wants it, rather than reopening this one.
- Did not touch `crates/loop-profiler` or any other crate — this is a docs-only, no-code-change
  PR, same shape as the #230 and #230-adjacent verify-stale entries.

## Also found, out of scope, flagged rather than fixed

- **Issue #132's dependency is still unmet.** Its own default action reads "Fit `kt` on the bench
  before any ridden test... Sr. Mechanical & Systems owns the bench measurement; Senior Controls
  owns the retune." `sim/scenarios/plant.py:57`'s `KT_NM_PER_A = 0.7` is still the documented
  unfitted placeholder (`sim/scenarios/bench_spinup.py`'s module docstring says the same). Nothing
  in this checkout suggests a bench-fitted value exists yet. Not a defect — just recording that
  #132 is not yet actionable by this role, so a future pass doesn't have to re-derive that.
- **Issue #182's remaining increments (I6 platform-contract manifest, I7 sim-in-the-loop
  self-test, I8 motor-in-the-loop) are all real architecture work**, each larger than a single
  unattended increment and several requiring hardware in hand. Flagging rather than starting one
  speculatively and blowing the ~10-file scope guideline on a design decision nobody asked this
  pass to make.
