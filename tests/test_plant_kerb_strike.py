"""What an obstacle is worth — issue #142 AC2.

The #132 gain retune needs a target, and the 20 N·s it currently has is
inherited from a scenario nominal rather than derived. This is the derivation
side of that: geometry and speed in, impulse out.

**These tests exist mostly to stop the model being wrong in ways that look
right.** The first version of `kerb_strike_impulse` charged 0.25·v of lost
speed for a step of height ZERO, and every number it produced looked entirely
plausible — alarming, but plausible. What caught it was checking a limit whose
answer is known independently of the model. So the limits are tested first and
hardest here, before any of the numbers are.
"""

import pytest

from sim.scenarios.plant import (
    KERB_STRIKE_VALIDITY,
    kerb_strike_impulse,
    kerb_strike_vs_com_impulse,
)

R_WHEEL = 0.1454
MASS = 82.5


# --------------------------------------------------------------------------
# Limits with known answers — the checks that caught the broken model
# --------------------------------------------------------------------------

def test_a_zero_height_obstacle_costs_nothing():
    """THE TEST THAT CAUGHT THE FIRST MODEL. A step of no height is not an
    obstacle; a model that charges for it is wrong no matter how reasonable its
    other numbers look. The textbook rotate-about-the-edge formulation charges
    0.25·v here, because it forces rotation about a contact point that is not
    blocking anything."""
    r = kerb_strike_impulse(0.0, 6.0)
    assert r["impulse_ns"][1] == pytest.approx(0.0, abs=1e-6)
    assert r["pitch_rate_deg_s"][1] == pytest.approx(0.0, abs=1e-6)


def test_standing_still_costs_nothing():
    """The other free limit: an obstacle you are not moving toward."""
    r = kerb_strike_impulse(0.030, 0.0)
    assert r["impulse_ns"][1] == pytest.approx(0.0, abs=1e-9)


def test_the_impulse_is_linear_in_speed():
    """Both models are impulsive and gravity does nothing during the strike, so
    nothing in either is quadratic in v. If this ever fails, an energy term has
    crept into a momentum argument."""
    a = kerb_strike_impulse(0.020, 3.0)["impulse_ns"][0]
    b = kerb_strike_impulse(0.020, 6.0)["impulse_ns"][0]
    assert b == pytest.approx(2.0 * a, rel=1e-9)


def test_a_taller_obstacle_always_costs_more():
    """Monotonicity in height. Cheap, and it would catch a sign error in the
    edge geometry that the absolute numbers would not."""
    js = [kerb_strike_impulse(h, 5.0)["impulse_ns"][1]
          for h in (0.005, 0.010, 0.020, 0.040, 0.070)]
    assert js == sorted(js)


def test_an_obstacle_at_or_above_axle_height_is_refused():
    """At h ≥ r the wheel strikes at or above its own centre. There is no
    climbing geometry left and no impulse model applies — the board stops and
    the rider does not. Refusing is honest; returning a number would invite
    someone to quote it."""
    with pytest.raises(ValueError, match="wall, not a step"):
        kerb_strike_impulse(R_WHEEL, 5.0)
    with pytest.raises(ValueError, match="wall, not a step"):
        kerb_strike_impulse(0.20, 5.0)


# --------------------------------------------------------------------------
# The bracket
# --------------------------------------------------------------------------

def test_the_two_models_converge_on_a_real_kerb():
    """The structural result worth having: for a tall step it does not matter
    how much the edge grips, because a gripping pivot and a frictionless normal
    give the same answer. Disagreement is confined to small lips."""
    assert kerb_strike_impulse(0.100, 6.0)["models_agree_within_pct"] < 5.0


def test_the_pivot_is_unreachable_on_a_small_lip_and_is_withheld():
    """The pivot does not merely get harsh at small heights, it becomes
    unphysical — arresting the roll would need μ = 2.34 at 5 mm and 5.43 at
    1 mm, against rubber-on-asphalt's ~1.0 (the model's own `<geom friction>`).
    The edge slips and the wheel rolls through.

    So the bracket collapses to the frictionless value there rather than
    quoting an upper bound that cannot happen. This is the friction gate doing
    its job; before it existed this height reported a 3× range whose top end
    was an artefact."""
    r = kerb_strike_impulse(0.005, 6.0)
    assert not r["pivot_reachable"]
    assert r["pivot_friction_required"] > 2.0
    assert r["impulse_ns"][0] == r["impulse_ns"][1]


def test_the_friction_gate_opens_around_the_tyre_deflection_height():
    """Two independent criteria land on the same ~20 mm boundary: the friction
    the pivot needs drops to ~1.0 there, and it is also where a pneumatic tyre
    stops swallowing the obstacle. Neither was tuned to the other, which is the
    main reason to believe either."""
    assert not kerb_strike_impulse(0.010, 6.0)["pivot_reachable"]
    assert kerb_strike_impulse(0.030, 6.0)["pivot_reachable"]
    mu = kerb_strike_impulse(0.020, 6.0)["pivot_friction_required"]
    assert 0.9 < mu < 1.1, f"the gate moved off the tyre-deflection height: μ={mu:.2f}"


def test_the_bracket_is_ordered_even_where_the_models_cross():
    """They cross near h/r ≈ 0.6: as the step gets tall the normal tilts and
    the frictionless case stops being the gentler one. An `assert lo < hi`
    here would be a wrong assumption dressed as a sanity check."""
    for h in (0.005, 0.030, 0.080, 0.110, 0.140):
        lo, hi = kerb_strike_impulse(h, 6.0)["impulse_ns"]
        assert lo <= hi


def test_validity_closes_at_both_ends():
    """Below the tyre's own deflection a pneumatic tyre swallows the obstacle
    and the rigid model describes a different event; above h/r = 0.6 the event
    is a crash rather than a disturbance. A result outside that band is a
    bound, not an answer, and says so."""
    floor_mm = KERB_STRIKE_VALIDITY["tyre_deflection_mm"][1]
    assert not kerb_strike_impulse(floor_mm / 2000.0, 6.0)["within_validity"]
    assert kerb_strike_impulse(0.030, 6.0)["within_validity"]
    assert not kerb_strike_impulse(0.6 * R_WHEEL + 0.001, 6.0)["within_validity"]


# --------------------------------------------------------------------------
# The finding that actually matters to #132
# --------------------------------------------------------------------------

def test_a_kerb_strike_is_not_a_com_impulse_of_the_same_size():
    """THE POINT OF THE WHOLE EXERCISE. `impulse_response.py` applies its
    20 N·s through the CoM, so the disturbance is purely linear and the initial
    pitch rate is zero by construction — its own docstring says so, and says a
    kerb strike is the follow-on case it is not modelling.

    A kerb strike lands ~0.7 m below the CoM and imparts hundreds of deg/s
    immediately. Quoting a reference disturbance in N·s and feeding it to the
    existing scenario therefore understates a kerb, silently, in the one
    channel that topples the board."""
    r = kerb_strike_vs_com_impulse(0.020, 6.0)
    assert r["com_impulse_pitch_rate_deg_s"] == 0.0
    assert min(r["pitch_rate_deg_s"]) > 100.0
    assert "different disturbance" in r["note"]


def test_even_a_modest_lip_at_speed_exceeds_the_inherited_20_ns():
    """The headline for #142: the inherited nominal is not conservative. A
    20 mm lip — a brick edge, a root heave, the kind of thing a pavement has
    every few metres — clears 20 N·s at every speed the board is meant to ride
    at. This is measured against the LOWER bound, so it does not depend on
    believing the harsher model."""
    for v in (2.0, 4.0, 6.0, 8.0):
        lo = kerb_strike_impulse(0.020, v)["impulse_ns"][0]
        assert lo > 20.0, f"{v} m/s gives only {lo:.1f} N·s"
