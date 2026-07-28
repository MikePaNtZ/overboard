"""Acceptance gate for issue #65: the flywheel/rotor inertia ratio decision.

Pins the arithmetic (an independent numerical cross-check of the closed-form
error-amplification formula, so a coding slip doesn't silently move the
verdict) and the current recommendation against the confirmed goBILDA
figures (#65/#66) -- a regression gate, so a change to the confirmed
flywheel or bare-rotor estimate quietly moving the verdict is caught rather
than assumed.
"""

import math

import pytest

from sim.scenarios.bench_inertia_ratio import (
    DESIGN_POINT_RATIO,
    FLYWHEEL_COUNT,
    FLYWHEEL_INERTIA_EACH_KG_M2,
    J_BARE_ESTIMATE_HIGH_KG_M2,
    J_BARE_ESTIMATE_LOW_KG_M2,
    J_DISC_CONFIRMED_KG_M2,
    STEEL_DISC_INERTIA_KG_M2,
    assess_confirmed_flywheels,
    assess_with_steel_disc,
    kt_error_amplification,
    recommend,
    slope_ratio,
)


def _numeric_amplification(ratio: float, eps: float = 1e-6) -> float:
    """Independent cross-check: numerically differentiate `identify()`'s own
    `kt = J_disc/(i*(1/alpha2 - 1/alpha1))` expression (dropping the
    constants `J_disc` and `i`, which cancel in a *relative* sensitivity)
    with respect to `alpha1` and `alpha2` directly, rather than restating
    the closed-form `sqrt(1+rho^2)/(rho-1)` a second time. `alpha1=1` is a
    free choice of units; only the ratio `alpha1/alpha2 = 1+ratio` matters.
    """
    alpha1 = 1.0
    alpha2 = alpha1 / (1.0 + ratio)

    def ln_kt(d_alpha1: float, d_alpha2: float) -> float:
        a1 = alpha1 * (1.0 + d_alpha1)
        a2 = alpha2 * (1.0 + d_alpha2)
        return math.log(1.0 / (1.0 / a2 - 1.0 / a1))

    d_ln_d_alpha1 = (ln_kt(eps, 0.0) - ln_kt(-eps, 0.0)) / (2 * eps)
    d_ln_d_alpha2 = (ln_kt(0.0, eps) - ln_kt(0.0, -eps)) / (2 * eps)
    return math.sqrt(d_ln_d_alpha1**2 + d_ln_d_alpha2**2)


@pytest.mark.parametrize("ratio", [0.05, 0.5, 1.101, 1.37, 1.651, 2.201, 6.69, 20.0])
def test_amplification_matches_independent_numeric_derivative(ratio):
    closed_form = kt_error_amplification(ratio)
    numeric = _numeric_amplification(ratio)
    assert closed_form == pytest.approx(numeric, rel=1e-6)


def test_amplification_rejects_nonpositive_ratio():
    with pytest.raises(ValueError):
        kt_error_amplification(0.0)
    with pytest.raises(ValueError):
        kt_error_amplification(-1.0)


def test_amplification_decreases_as_ratio_grows():
    # More added inertia relative to the unknown always helps -- monotonic,
    # not just numerically close, across the whole confirmed-to-design range.
    ratios = [1.0, 1.101, 1.37, 1.651, 2.201, 6.69, 6.726, 20.0]
    amplifications = [kt_error_amplification(r) for r in ratios]
    assert amplifications == sorted(amplifications, reverse=True)


def test_slope_ratio_matches_identifys_own_two_run_algebra():
    # identify()'s docstring: alpha1/alpha2 = 1 + J_disc/J_bare. The
    # never-bought custom disc (1.6103e-3 kg*m^2, #72's before/after table)
    # against the same bare-rotor estimate this module uses is what produces
    # the design point 6.69 in the first place.
    j_disc_old_custom_estimate = 1.6103e-3
    j_bare = 2.4071e-4
    rho = slope_ratio(j_disc_old_custom_estimate, j_bare)
    assert rho == pytest.approx(1.0 + j_disc_old_custom_estimate / j_bare, rel=1e-9)
    assert rho == pytest.approx(DESIGN_POINT_RATIO + 1.0, rel=1e-3)


def test_confirmed_flywheel_inertia_matches_manufacturer_figure():
    # goBILDA 3628-0032-0082: 1651 g*cm^2 each, two on the shaft (#65/#66).
    assert FLYWHEEL_COUNT == 2
    assert FLYWHEEL_INERTIA_EACH_KG_M2 == pytest.approx(1651e-7, rel=1e-9)
    assert J_DISC_CONFIRMED_KG_M2 == pytest.approx(3.302e-4, rel=1e-6)


def test_confirmed_flywheels_land_in_the_issues_stated_ratio_band():
    a = assess_confirmed_flywheels()
    # Issue #65: "against an estimated bare-rotor inertia of 2000-3000
    # g*cm^2 ... a ratio of roughly 1.1-1.65."
    assert a.ratio_low == pytest.approx(1.101, abs=0.01)
    assert a.ratio_high == pytest.approx(1.651, abs=0.01)


def test_confirmed_flywheels_cost_roughly_a_doubling_vs_design_point():
    a = assess_confirmed_flywheels()
    # Issue #65: "error amplification on kt roughly doubles." The exact
    # figure this module derives is 1.48x-1.82x -- accurate at the
    # pessimistic end of "roughly doubles" and a firm number where the
    # issue only had an estimate.
    assert 1.4 < a.worst_case_relative_increase < 1.9
    assert a.amplification_at_design_point == pytest.approx(1.159, abs=0.005)
    assert a.amplification_low == pytest.approx(2.113, abs=0.005)
    assert a.amplification_high == pytest.approx(1.716, abs=0.005)


def test_steel_disc_recovers_the_design_point_amplification():
    a = assess_with_steel_disc()
    # The whole justification for the recommended fix: worst-case
    # amplification with the disc added should land back near (not just
    # "better than") the original design-point figure.
    assert a.amplification_low == pytest.approx(
        a.amplification_at_design_point, rel=0.01
    )
    # And it should clear the design-point ratio outright, not just approach
    # it from below.
    assert a.ratio_low >= DESIGN_POINT_RATIO


def test_steel_disc_costs_less_inertia_than_a_third_flywheel_would_need():
    # Sanity bound on the recommended part: cheaper, lower-part-count than
    # reaching the same ratio by adding more 82 mm flywheels alone (~$16
    # each per the issue, but it would take several -- see the module
    # docstring's arithmetic).
    j_bare_high = J_BARE_ESTIMATE_HIGH_KG_M2
    j_disc_needed = DESIGN_POINT_RATIO * j_bare_high
    flywheels_needed = j_disc_needed / FLYWHEEL_INERTIA_EACH_KG_M2
    assert flywheels_needed > 12  # far more than the two already bought
    assert STEEL_DISC_INERTIA_KG_M2 > 0.0


def test_recommend_without_measurement_reports_the_range_and_the_pick():
    text = recommend()
    assert "1.10" in text or "1.11" in text  # ratio_low, formatted to 2dp
    assert "steel disc" in text.lower()


def test_recommend_with_measured_j_bare_uses_it_instead_of_the_band():
    # Once the bench's own bare run exists, the recommendation should react
    # to it rather than staying pinned to the estimated band -- this is the
    # point of sequencing the purchase behind the measurement.
    text_low_bare = recommend(j_bare_measured_kg_m2=J_BARE_ESTIMATE_LOW_KG_M2)
    text_high_bare = recommend(j_bare_measured_kg_m2=J_BARE_ESTIMATE_HIGH_KG_M2)
    assert text_low_bare != text_high_bare
    assert "Measured J_bare" in text_low_bare
