"""Bench-rig flywheel/rotor inertia ratio: how much it costs, and what to do
about it.

Issue #65: the two goBILDA 82 mm flywheels are ordered and shipping, and
their inertia is now a manufacturer figure rather than a guess -- 1651 g*cm^2
each, 3302 g*cm^2 (3.302e-4 kg*m^2) together (see #66/#72's `bench_rig.xml`
update). Against the estimated bare-rotor inertia band, 2000-3000 g*cm^2 (no
manufacturer publishes this for an e-skate 6374, and the two sources found
during sourcing disagreed 16x -- #65), that gives a ratio `J_disc/J_bare` of
roughly 1.1-1.65. The sim originally validated the two-run method at 6.69,
using a heavier custom aluminium disc that was never bought (`bench_spinup.
NAMEPLATE_KT_NM_PER_A`'s sibling constant, the old 75 mm/572 g custom-disc
estimate this rig has since moved off of).

WHY THE RATIO MATTERS: THE ERROR-AMPLIFICATION DERIVATION
-----------------------------------------------------------
`identify()` (see `bench_spinup.py`) solves

    kt = J_disc / (i * (1/alpha2 - 1/alpha1))

from two fitted slopes, bare (`alpha1`) and loaded (`alpha2`). Let
`rho = alpha1/alpha2 = 1 + J_disc/J_bare` -- the same slope-separation ratio
`identify()`'s own docstring calls out (7.4x at the design point). Standard
first-order error propagation, treating `alpha1` and `alpha2` as independent
measurements each with relative error `eps` (from finite R^2, sensor noise,
whatever the bench actually measures that error to be), gives a relative
error in `kt` of

    sigma_kt / kt  =  eps * sqrt(1 + rho^2) / (rho - 1)

`kt_error_amplification()` is that multiplier, `sqrt(1+rho^2)/(rho-1)`,
independent of `eps` -- it says how much WORSE (or better) a given per-alpha
measurement error becomes once it passes through the two-run algebra, as a
function of the ratio alone. It is cross-checked in
`tests/test_bench_inertia_ratio.py` against a second, independent
derivation: numerical differentiation of the actual `kt` expression above
(not a restatement of the same formula), the same convention #64's
`bench_riser.py` used for its own closed-form cross-check.

THE NUMBERS, MEASURED FROM THE FORMULA RATHER THAN ASSERTED
--------------------------------------------------------------
    design point (ratio 6.69, never-bought disc)         amplification 1.159x
    confirmed range, worst end   (ratio 1.101)            amplification 2.113x
    confirmed range, best end    (ratio 1.651)             amplification 1.716x

So accepting the confirmed flywheels costs a **1.48x-1.82x** increase in how
much any given alpha-fit error shows up in `kt` -- "roughly doubles" (the
issue's own estimate) is accurate at the pessimistic end and a bit
conservative at the optimistic end. It is a real cost, but a bounded,
finite one: nothing here blows up or goes non-invertible, because the
confirmed ratio (>=1.1) still clears the "added inertia comparable to or
larger than the unknown" floor the two-run method actually needs.

THE CHEAP FIX, AND WHY IT WORKS
---------------------------------
`kt_error_amplification` falls fast as `ratio` grows past ~2 and flattens out
above ~5 (`d(amplification)/d(ratio)` shrinks as `ratio` grows), so a single
inexpensive disc that recovers most of the way to the design point buys back
nearly all of the loss. A single ~150 mm-diameter, ~600 g steel disc
(`STEEL_DISC_INERTIA_KG_M2`) stacked alongside the two confirmed flywheels
gives a combined `J_disc` of ~2.018e-3 kg*m^2 -- against the SAME 2000-3000
g*cm^2 bare-rotor band, that is a ratio of 6.7-10.1, i.e. amplification
1.16x at the worst case: back to the design point, for the ~$20-30 the issue
itself named. `RECOMMENDED_STACK_DESCRIPTION` states this as the pick.

THE RECOMMENDATION (issue #65's four acceptance criteria)
-------------------------------------------------------------
1. **Add inertia, not "accept and carry the wider error bar."** The
   confirmed ratio is workable (it clears the required floor) but the
   amplification cost (1.5-1.8x) is not free, and the fix costs $20-30
   against torque figures every later stage inherits. That asymmetry is
   the whole justification.
2. **Sequence it behind the bare-rotor measurement, not before it.** The
   16x sourcing disagreement on `J_bare` is resolved for free by the first
   bench session's own bare run (mode 1 of `identify()` already measures
   it) -- no purchase is needed to get that number, and ordering before
   measuring risks buying for the wrong end of a 16x-wide guess. Rerunning
   `recommend()` with the measured `J_bare` in place of the estimated band
   turns this from a range into a single, confirmed pick.
3. **The error bar is the multiplier above, not an absolute number.** No
   bench current-noise or R^2 figure exists yet for THIS rig (that is a
   Stage-0B measurement, not a sim one), so `kt_error_amplification` is
   reported as a multiplier on whatever that measured `eps` turns out to
   be, not a fabricated absolute percentage.
4. **Sequencing already answered under point 2** -- measure `J_bare` first.

WHAT THIS MODULE DOES NOT DO
-------------------------------
It does not place an order. Platform selection is this role's remit
(ROLES.md), but committing money is the CEO's -- this module states the
arithmetic and the pick so that decision can be made in one read, not zero.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

#: goBILDA 3628-0032-0082, manufacturer-published (#65/#66): 1651 g*cm^2
#: each, converted to SI (1 g*cm^2 = 1e-7 kg*m^2).
FLYWHEEL_INERTIA_EACH_KG_M2 = 1651e-7
FLYWHEEL_COUNT = 2
J_DISC_CONFIRMED_KG_M2 = FLYWHEEL_INERTIA_EACH_KG_M2 * FLYWHEEL_COUNT

#: Estimated bare-rotor inertia band for a 6374-class outrunner. No
#: manufacturer publishes this figure and the two sources found during
#: sourcing disagreed 16x (#65) -- this is a working range, not a
#: measurement. `recommend()` should be re-run with the bench's own
#: bare-run figure once it exists; see the module docstring.
J_BARE_ESTIMATE_LOW_KG_M2 = 2000e-7
J_BARE_ESTIMATE_HIGH_KG_M2 = 3000e-7

#: The ratio the two-run method was originally sim-validated at, using a
#: heavier custom aluminium disc (75 mm radius, 12 mm thick, 6061) that was
#: never bought -- see `bench_spinup.py`'s FLYWHEEL_* constants, which still
#: describe that never-built part, not the goBILDA discs actually shipped.
DESIGN_POINT_RATIO = 6.69

#: A single steel disc sized to recover most of the design-point margin
#: cheaply, per the issue's own costing ("~150 mm steel disc (~600 g)
#: reaches the design point for roughly $20-30"). 75 mm radius, 0.6 kg,
#: solid disc about its own axis: J = 0.5*m*r^2.
STEEL_DISC_MASS_KG = 0.6
STEEL_DISC_RADIUS_M = 0.075
STEEL_DISC_INERTIA_KG_M2 = 0.5 * STEEL_DISC_MASS_KG * STEEL_DISC_RADIUS_M**2

RECOMMENDED_STACK_DESCRIPTION = (
    "Both confirmed goBILDA flywheels plus one ~150 mm/600 g steel disc "
    "(~$20-30, per the issue's own costing) -- ordered only after the "
    "bare-rotor run measures J_bare, not before."
)


def slope_ratio(j_disc_kg_m2: float, j_bare_kg_m2: float) -> float:
    """`alpha1/alpha2` in `identify()`'s own terms: `1 + J_disc/J_bare`."""
    return 1.0 + j_disc_kg_m2 / j_bare_kg_m2


def kt_error_amplification(j_disc_over_j_bare: float) -> float:
    """How much a given relative error in each fitted slope (`alpha1`,
    `alpha2`) is amplified in the resulting `kt`, as a function of the
    disc/bare inertia ratio alone. See the module docstring for the
    derivation and its independent numerical cross-check
    (`tests/test_bench_inertia_ratio.py`).

    `j_disc_over_j_bare` must be > 0 -- `identify()` itself divides by
    `(1/alpha2 - 1/alpha1)`, which is exactly this constraint restated in
    slope terms, since a ratio of 0 means no added inertia to measure
    against at all.
    """
    if j_disc_over_j_bare <= 0.0:
        raise ValueError(
            f"J_disc/J_bare must be positive, got {j_disc_over_j_bare} -- "
            "a non-positive ratio means no added inertia to fit against."
        )
    rho = 1.0 + j_disc_over_j_bare
    return math.sqrt(1.0 + rho**2) / (rho - 1.0)


@dataclass(frozen=True)
class InertiaRatioAssessment:
    """The #65 verdict, both ends of the bare-rotor estimate band."""

    ratio_low: float
    ratio_high: float
    amplification_low: float
    """Amplification at the LOWER ratio (larger assumed J_bare) -- the
    worse, pessimistic case."""
    amplification_high: float
    """Amplification at the HIGHER ratio (smaller assumed J_bare) -- the
    better, optimistic case."""
    amplification_at_design_point: float
    worst_case_relative_increase: float
    """`amplification_low / amplification_at_design_point` -- how much worse
    the worst confirmed case is than what the method was validated at."""


def assess_confirmed_flywheels(
    j_disc_kg_m2: float = J_DISC_CONFIRMED_KG_M2,
    j_bare_low_kg_m2: float = J_BARE_ESTIMATE_LOW_KG_M2,
    j_bare_high_kg_m2: float = J_BARE_ESTIMATE_HIGH_KG_M2,
) -> InertiaRatioAssessment:
    """Assess the confirmed goBILDA flywheels against the estimated bare-rotor
    band. A larger assumed `J_bare` gives a SMALLER ratio (worse case) --
    `ratio_low` therefore pairs with `j_bare_high_kg_m2` and vice versa.
    """
    ratio_low = j_disc_kg_m2 / j_bare_high_kg_m2
    ratio_high = j_disc_kg_m2 / j_bare_low_kg_m2
    amp_low = kt_error_amplification(ratio_low)
    amp_high = kt_error_amplification(ratio_high)
    amp_design = kt_error_amplification(DESIGN_POINT_RATIO)
    return InertiaRatioAssessment(
        ratio_low=ratio_low,
        ratio_high=ratio_high,
        amplification_low=amp_low,
        amplification_high=amp_high,
        amplification_at_design_point=amp_design,
        worst_case_relative_increase=amp_low / amp_design,
    )


def assess_with_steel_disc(
    j_bare_low_kg_m2: float = J_BARE_ESTIMATE_LOW_KG_M2,
    j_bare_high_kg_m2: float = J_BARE_ESTIMATE_HIGH_KG_M2,
) -> InertiaRatioAssessment:
    """Same assessment, with `STEEL_DISC_INERTIA_KG_M2` stacked onto the
    confirmed flywheels -- the recommended fix."""
    return assess_confirmed_flywheels(
        j_disc_kg_m2=J_DISC_CONFIRMED_KG_M2 + STEEL_DISC_INERTIA_KG_M2,
        j_bare_low_kg_m2=j_bare_low_kg_m2,
        j_bare_high_kg_m2=j_bare_high_kg_m2,
    )


def recommend(
    j_bare_measured_kg_m2: float | None = None,
) -> str:
    """The #65 recommendation, in one string. Pass the bench's own measured
    `J_bare` (from `identify()`'s bare run) once it exists to turn this from
    a range into a single confirmed pick, per the module docstring's point 2.
    """
    if j_bare_measured_kg_m2 is not None:
        ratio_as_shipped = J_DISC_CONFIRMED_KG_M2 / j_bare_measured_kg_m2
        amp_as_shipped = kt_error_amplification(ratio_as_shipped)
        ratio_with_disc = (
            J_DISC_CONFIRMED_KG_M2 + STEEL_DISC_INERTIA_KG_M2
        ) / j_bare_measured_kg_m2
        amp_with_disc = kt_error_amplification(ratio_with_disc)
        return (
            f"Measured J_bare={j_bare_measured_kg_m2:.4e} kg*m^2: as-shipped ratio "
            f"{ratio_as_shipped:.2f} (amplification {amp_as_shipped:.3f}x); with the "
            f"steel disc, ratio {ratio_with_disc:.2f} (amplification "
            f"{amp_with_disc:.3f}x). Add the disc if the as-shipped amplification "
            "is not already acceptable against the bench's measured per-alpha error."
        )
    a = assess_confirmed_flywheels()
    return (
        f"Confirmed flywheels alone: ratio {a.ratio_low:.2f}-{a.ratio_high:.2f}, "
        f"kt-error amplification {a.amplification_low:.3f}x-{a.amplification_high:.3f}x "
        f"({a.worst_case_relative_increase:.2f}x worse than the "
        f"{a.amplification_at_design_point:.3f}x design point). Recommendation: "
        f"{RECOMMENDED_STACK_DESCRIPTION}"
    )
