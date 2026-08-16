# Issue #228 — hill.py's estimator mechanism claim was never the active config (PR #281)

Issue #228 names `tests/test_terrain.py`, but that file makes no mechanism claim at all —
checked its full git history (`git log --follow`), `CommandFeedforward` has never appeared
there. The real misattribution is in `sim/scenarios/hill.py`'s module docstring and
`tests/test_hill.py`'s `test_the_estimator_absorbs_the_slope_into_its_attitude_error`: both
claimed the hill scenario runs on `CommandFeedforward` and derived the ≈slope-angle RMS error
from that mechanism. `hill.py`'s `run()` never actually passes `estimator_accel_aiding` to
`RustController`, so the scenario runs on `RustController`'s own default — mode 1
(`WheelAccelEstimator`, wheel odometry) — not mode 2 as claimed.

**Forced mode 2 on directly (real shipped gain, `ACCEL_FF_GAIN_M_S2_PER_A = 0.0584`) and
measured `est_error_over_slope` ≈ 0.036 at 5% and 10% grade — nowhere near the ≈0.81/0.84 mode
1 actually produces** (which matches the existing 0.6–1.3x test band and the docstring's
"≈0.87x across 5–20%" line unchanged). So the retracted mechanism doesn't reproduce even under
the estimator it names — not just misattributed to the wrong file, falsified as an explanation.

Retracted the claim in both docstrings, kept the numbers (still real, still measured), stated
which estimator is actually configured, and **flagged rather than guessed** at what in mode 1
actually produces the near-slope-angle error. No control code, gain, or estimator config
changed — documentation-only. Verified: full `pytest tests/` (333 passed, 5 xfailed, unchanged
before/after), clean `policy_check.py`.

**Deliberately left out:** issue #228's second ask (confirm a "steady/rolling inversion at
12–14% under a 60A envelope" claim). Searched the full git history of `test_terrain.py`,
`hill.py` and `terrain.py` and could not find that claim anywhere, past or present. Not closed
as "does not reproduce" — could not locate it to test at all, which is weaker and different,
so flagged rather than asserted either way.
