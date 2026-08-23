# 2026-08-21 — Issue #132: the (kp, kd) feasibility map (PR #293, not closing #132)

Cron dispatch pass. Same routing convention as prior passes (PR #245, #276, #283): no open
issue currently carries a literal `Owner: Senior Controls` body marker except #261 (already has
an open PR, #263 — skipped) and #61 (`Dispatch: COO only`, reserved). Went by `role:senior-controls`
labels plus turf paths instead. #182 and #168 both already have open PRs (#290, #291). That left
#132 as the only `role:senior-controls`-labelled issue with no PR in flight.

**Correcting the 2026-08-16 pass's read of #132.** That entry (above) called #132 "not
independently actionable without hardware that does not exist yet," on the grounds that AC1
needs a bench-fitted `kt`. True for AC1 — but #132's own comment thread (posted the same day the
issue was filed) reframes the finding and lays out a five-step revised order, and steps 1
(#137, torque re-denomination) and 2 (#133/#142, reference disturbance) are both already merged.
**Step 3 — sweep `(kp, kd)` as a plane against gain margin and delay margin — is explicitly called
out in that same comment as "all of it kt-independent."** The prior pass's summary of #132 didn't
distinguish AC1 (blocked, Mechanical's) from the revised order's other steps (not blocked, mine).
This pass did that step: `scripts/gain_margin_sweep.py` + `tests/test_gain_margin_sweep.py`,
repo-only, no hardware needed.

**Finding, not fixed:** at today's model state, the comment's own two guardrails no longer
overlap. `p` (the ridden RHP pole) has moved from 5.24 to 5.564 rad/s since the comment was
written — plausibly from an intervening Mechanical mass/inertia change, not investigated further
here since that's their turf, not a defect to chase. That shifts the recommended crossover floor
(`1.5p`) to ~8.35 rad/s, past the same comment's explicit "treat >~8 rad/s as unvalidated"
ceiling. The script reports this explicitly (`recommended_band_is_empty`) rather than silently
widening the ceiling to make the band non-empty. No gain changed as a result — this is a finding
for the eventual retune, not a decision.

**Deliberately left out, flagged rather than fixed:** the nonlinear disturbance-capacity axis
(step 3's third dimension — needs a full closed-loop impulse run per grid point, real separate
work), step 4's robustness-over-p sweep (a different independent variable), and AC1's bench `kt`
fit (still Mechanical's, still blocked on hardware). Does not close #132.

PR #293 opened against `master`, full python (356 passed, 7 xfailed) and rust suites green,
`policy_check.py` hard checks pass. Per role instructions, opened and left for review — not
merged, auto-merge not enabled.
