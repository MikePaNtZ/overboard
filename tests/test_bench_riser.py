"""Acceptance gate for issue #64: is the 1/4" riser stiff enough?

`bench_riser.py`'s first-mode ROM is a hand-derivable cantilever-with-tip-mass
calculation; these tests pin both the ARITHMETIC (independent scaling checks
against the analytic formula, so a coding slip doesn't silently move the
verdict) and the CURRENT VERDICT against the rig as designed and as purchased
(#65/#66's confirmed flywheel figures) -- a regression gate, so a geometry
change to bench_rig.xml that quietly moves the riser back into "adequate" or
further into "inadequate" is caught rather than assumed.
"""

import math

import pytest

from sim.scenarios.bench_riser import (
    ALUMINUM_DENSITY_KG_M3,
    ALUMINUM_E_PA,
    DEFAULT_BRACE_HEIGHT_M,
    FLYWHEEL_COUNT,
    FLYWHEEL_MASS_EACH_KG,
    MOTOR_MASS_HIGH_KG,
    MOTOR_MASS_LOW_KG,
    TARGET_HZ,
    RiserGeometry,
    assess_riser,
    cantilever_first_mode_hz,
    describe_bench_signature,
    gusset_brace_first_mode_hz,
    hub_mass_from_model,
    riser_geometry_from_model,
    tip_mass_bounds_kg,
    verdict,
)

# A convenient, round test fixture -- not the real rig -- for the arithmetic
# checks below, so the numbers are easy to hand-verify.
_TEST_GEOM = RiserGeometry(length_m=0.2, width_m=0.12, thickness_m=0.00635)


def test_riser_geometry_from_model_matches_confirmed_purchase():
    geom = riser_geometry_from_model()
    assert geom.length_m == pytest.approx(0.20, abs=1e-9)
    assert geom.width_m == pytest.approx(0.12, abs=1e-9)
    # Confirmed purchased dimension (#66), not whatever the compiled model
    # currently rounds it to -- see the module docstring.
    assert geom.thickness_m == pytest.approx(0.00635, abs=1e-9)


def test_hub_mass_matches_geometry_times_density():
    mass = hub_mass_from_model()
    # bench_rig.xml: hub is a cylinder, r=0.011, half-length=0.010, density 2700.
    expected = 2700.0 * math.pi * 0.011**2 * (2 * 0.010)
    assert mass == pytest.approx(expected, rel=1e-6)


def test_cantilever_first_mode_matches_closed_form_no_beam_mass():
    """With a massless beam (density 0), `cantilever_first_mode_hz` must
    reduce exactly to f = (1/2pi)*sqrt(3EI/(L^3*m)) -- the textbook formula
    with no Rayleigh correction to get wrong."""
    geom = RiserGeometry(length_m=0.2, width_m=0.12, thickness_m=0.00635)
    tip_mass = 0.9

    moment_of_inertia = geom.width_m * geom.thickness_m**3 / 12.0
    stiffness = 3.0 * ALUMINUM_E_PA * moment_of_inertia / geom.length_m**3
    expected = math.sqrt(stiffness / tip_mass) / (2.0 * math.pi)

    # Zero out the Rayleigh beam-mass term by using a vanishingly light beam:
    # same closed form, computed independently of the module's own beam-mass
    # constant so this is a real cross-check rather than the same formula
    # copy-pasted.
    import sim.scenarios.bench_riser as br

    original_density = br.ALUMINUM_DENSITY_KG_M3
    try:
        br.ALUMINUM_DENSITY_KG_M3 = 1e-12
        result = cantilever_first_mode_hz(tip_mass, geom)
    finally:
        br.ALUMINUM_DENSITY_KG_M3 = original_density

    assert result == pytest.approx(expected, rel=1e-6)


def test_first_mode_scales_as_thickness_to_the_1p5():
    """f ~ sqrt(I) ~ sqrt(t^3) = t^1.5, holding tip mass and beam mass fixed
    (beam mass is a small correction here, so this is checked loosely)."""
    thin = RiserGeometry(length_m=0.2, width_m=0.12, thickness_m=0.00635)
    thick = RiserGeometry(length_m=0.2, width_m=0.12, thickness_m=0.0127)  # 2x
    tip_mass = 0.9

    f_thin = cantilever_first_mode_hz(tip_mass, thin)
    f_thick = cantilever_first_mode_hz(tip_mass, thick)

    assert f_thick / f_thin == pytest.approx(2.0**1.5, rel=0.05)


def test_first_mode_scales_inversely_as_length_to_the_1p5():
    """f ~ sqrt(1/L^3) = L^-1.5, same loose tolerance rationale as above."""
    long_geom = RiserGeometry(length_m=0.2, width_m=0.12, thickness_m=0.00635)
    short_geom = RiserGeometry(length_m=0.1, width_m=0.12, thickness_m=0.00635)  # half
    tip_mass = 0.9

    f_long = cantilever_first_mode_hz(tip_mass, long_geom)
    f_short = cantilever_first_mode_hz(tip_mass, short_geom)

    assert f_short / f_long == pytest.approx(2.0**1.5, rel=0.05)


def test_heavier_tip_mass_lowers_frequency():
    f_light = cantilever_first_mode_hz(0.5, _TEST_GEOM)
    f_heavy = cantilever_first_mode_hz(1.5, _TEST_GEOM)
    assert f_heavy < f_light


def test_tip_mass_bounds_include_confirmed_flywheels_and_exact_hub():
    hub = 0.0205
    low, high = tip_mass_bounds_kg(hub)
    flywheels = FLYWHEEL_MASS_EACH_KG * FLYWHEEL_COUNT
    assert low == pytest.approx(MOTOR_MASS_LOW_KG + hub + flywheels)
    assert high == pytest.approx(MOTOR_MASS_HIGH_KG + hub + flywheels)
    assert flywheels == pytest.approx(0.304)  # 2 x goBILDA 3628-0032-0082, 152 g each


@pytest.mark.parametrize(
    "low,high,expected",
    [
        (TARGET_HZ * 1.5, TARGET_HZ * 1.5, "adequate"),
        (TARGET_HZ * 1.5 - 0.01, TARGET_HZ * 2.0, "marginal"),  # worst end just misses
        (TARGET_HZ, TARGET_HZ, "marginal"),
        (TARGET_HZ - 0.01, TARGET_HZ * 10, "inadequate"),  # worst end just misses target
    ],
)
def test_verdict_bands_are_evaluated_at_the_pessimistic_end(low, high, expected):
    assert verdict(low, high) == expected


def test_gusset_at_zero_height_is_the_unbraced_case():
    unbraced = cantilever_first_mode_hz(0.9, _TEST_GEOM)
    braced_at_base = gusset_brace_first_mode_hz(0.9, _TEST_GEOM, brace_height_m=0.0)
    assert braced_at_base == pytest.approx(unbraced)


def test_gusset_brace_raises_the_first_mode():
    unbraced = cantilever_first_mode_hz(0.9, _TEST_GEOM)
    braced = gusset_brace_first_mode_hz(0.9, _TEST_GEOM, brace_height_m=0.08)
    assert braced > unbraced


def test_as_designed_riser_is_inadequate_against_target():
    """The regression gate: as purchased (200 mm x 120 mm x 6.35 mm 6061,
    carrying two confirmed 152 g goBILDA flywheels + an assumed-range motor),
    the riser's first mode sits BELOW the 50 Hz target, at both ends of the
    assumed motor-mass range. If this ever flips to "adequate" because
    someone changed the riser geometry, that is real news for #64 and this
    test should be updated to say so explicitly -- not silently pass."""
    assessment = assess_riser()
    assert assessment.verdict == "inadequate"
    assert assessment.first_mode_high_hz < TARGET_HZ


def test_default_gusset_brace_reaches_adequate_with_margin():
    assessment = assess_riser(brace_height_m=DEFAULT_BRACE_HEIGHT_M)
    assert assessment.braced_verdict == "adequate"
    assert assessment.braced_first_mode_low_hz > TARGET_HZ * 1.5


def test_adequate_as_designed_riser_skips_the_braced_recheck():
    """If a hypothetically stiffer riser clears the target on its own,
    `assess_riser` should not report a braced result nobody asked for."""
    stiff_geometry = RiserGeometry(length_m=0.05, width_m=0.12, thickness_m=0.00635)
    assessment = assess_riser(geometry=stiff_geometry, hub_mass_kg=0.0205)
    assert assessment.verdict == "adequate"
    assert assessment.braced_verdict is None
    assert assessment.braced_first_mode_low_hz is None
    assert assessment.braced_first_mode_high_hz is None
    assert assessment.brace_height_m is None


def test_bench_signature_names_the_target_frequency_and_the_tap_test():
    description = describe_bench_signature(40.0)
    assert "40" in description
    assert "tap" in description.lower()
