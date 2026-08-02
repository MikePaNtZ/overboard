# 2026-08-02 — Issue #207: pin the estimator trim and the reserve's derivation

ADR-0011 second ratification, criteria (f1) and (f2), plus the incline-tolerance
measurement the world-authoring constraint was blocked on. PR #215.

## What was pinned, and the instrument that had to be replaced to pin it

The obvious instrument for (f1) — least squares of the `est − truth` residual against
specific force across the acceleration ramp — is **badly conditioned**, and measuring it
said so before anything was asserted: on one run the fitted slope moves **4.9 → 7.1 °/(m/s²)**
across reasonable choices of fit window, because the complementary filter's 2 s time constant
makes the residual lag the specific force throughout the ramp. A band tight enough to be a pin
would have chattered on the window choice; a band loose enough not to would have detected
nothing. The shipped test's ±25% band was the second of those.

The identity is a **steady-state** statement, so it is now measured in steady state: residual
against `atan(a_unaided/g)` over a fixed late window. Ratio **1.028**, unexplained static part
**2.8%**. Asserted as the identity (ratio ≈ 1, ±5%) rather than as a fitted slope.

Trim pinned at **−2.501 ± 0.10°**.

## Where the band came from — this is the part worth remembering

Not from what passed. Measured in the (f2) work: **shifting the trim 0.10° moves the
peak-demand slope 5.19–5.26%**, against the **5%** band that slope's own provenance check
allows. So 0.10° is the trim movement at which `CMD_ENVELOPE_RESERVE` stops being derived from
anything true. The pin fires exactly where the derivation goes stale, which is the only place
it has a principled reason to fire. (f1) and (f2) are therefore one pin seen twice.

The ±0.25° static-error characterisation was used **only** as a sanity check in the safe
direction — the band the derivation argument produced happens to sit well inside it. Had the
derivation produced a wider band the answer would have been to look harder, not to borrow
0.25°. Promoting 0.25° to a bar is the exact move criterion (f) exists to forbid.

## The acceptance criterion was met by breaking CI on purpose, not by reading the test

Spike PR #214: `ACCEL_FF_GAIN_M_S2_PER_A` 0.0584 → 0.0650 (a realistic re-derivation — the
constant is `kt / (r_eff · total mass)`). `sim` went red on the trim pin.
[Run](https://github.com/MikePaNtZ/overboard/actions/runs/30762664302/job/91535879744). Closed
without merging.

Two things fell out of it:

- **The sim is bit-identical across platforms.** Ubuntu runner `−2.6896194189755955°`; macOS
  arm64 `−2.6896°`. That was the open risk in a ±0.10° band on a 2.5° quantity, and it is
  answered — the band cannot produce a platform-dependent false positive. Useful for anyone
  pinning anything else here.
- **A trim move of that size takes four other assertions with it** — peak-demand slope fell to
  29.20 A/unit against 42.03, the unshaped positive control stopped inverting the board, and the
  loss-of-authority warning stopped firing at all. The trim is load-bearing for most of what
  ADR-0011 measures.

## Incline tolerance — the prediction was wrong, and the premise was wrong

Issue #207 predicted "under 0.5°", reasoning that a slope is an effective static pitch
disturbance. **The middle step fails.** A static pitch *estimate error* is a signal the
controller cannot see — it regulates a lie forever. A slope is one it **can** see: the IMU still
measures true gravity, so attitude is correct on a hill and the slope arrives as an ordinary
along-track force the balance loop already leans against. Different disturbances, different
tolerances.

Measured (`tests/test_incline_tolerance.py`; `--incline-deg` rotates `mjModel::opt.gravity` at
open, which is the same rigid-body problem as a tilted ground plane and keeps the change out of
`sim/models/`):

| | measured |
|---|---|
| ADR-0011 matrix inverts | **nowhere** in ±12° (hold) / ±8° (reversal) — ≥24× the prediction |
| station-keeping | **zero at every incline.** Released, free-rolls at 0.70·g·sin φ, 43 m/s on a 15° grade inside one 18.5 s schedule, nothing in the host intervening |
| self-arrest at 0.6 lean | arrested to **6.5°**, outrun at **7.0°** |

So the binding constraint is **not inversion, it is that there is no speed loop** —
`MAX_GROUND_SPEED_M_S` shapes the *stick*, and a stick cap cannot brake against gravity. The
asset rule needs an angle **and a run-out length**: a 0.25° grade is fine for a metre and not for
a kilometre (~1159 m to speed-cap onset from rest).

None of these is an acceptance threshold and the file says so at length. They are inputs the
world-authoring role writes the rule *from*.

## Trap worth knowing about: the corridor brake

The first version of the self-arrest measurement attributed the braking to the speed cap. It is
the **corridor brake** — roll backwards past `CORRIDOR_X_MAX_M` = 50 m and the host applies
`CORRIDOR_BRAKE_LEAN` = 0.6 for the rest of the run. Caught by looking at `applied_fore_aft`,
which goes to exactly 0.60. Any released-board measurement that rolls *backwards* hits this
within ~17 s at 3°. The free-roll test therefore runs **downhill-forward** (700 m of corridor)
and asserts `applied_fore_aft == 0` throughout.

## Deliberately not delivered, flagged rather than fudged

- **The steepest grade the board can climb at full stick.** Between 4° and 7°, and not reported:
  the 18.5 s schedule cannot distinguish a board climbing slowly from one about to slide back.
  Needs a longer scripted schedule — its own increment. Also the least safety-relevant of the
  four: a grade the board cannot climb is a stuck player, not a fall.
- **The incline numbers are not in `docs/`.** A new file under `docs/` is COO turf; the
  measurement lives in the test file's docstring on the same precedent as issue #24 AC5. If the
  world-authoring role wants it mirrored into `docs/`, that is a COO call, not mine to take.
- **(f3), `θ_ref = atan(a_des/g)` as an explicit feedforward**, is untouched — ADR-0011 defers it
  and bundles it with the headroom fix. This PR is what makes the accident safe to stand on
  until then, not a substitute for it.
