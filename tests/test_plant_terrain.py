"""Tilted ground plane — the filmable hill, and the cross-check against the
rotated-gravity one.

Answers issue #17 from Senior Controls. `hill.py` models a grade by rotating
gravity and leaving the ground flat; this adds a variant that tilts the ground
and leaves gravity vertical.

THE POINT OF HAVING BOTH IS THAT THEY ARE INDEPENDENT
-----------------------------------------------------
Rotated gravity and a tilted plane are the same problem for a body on an
infinite uniform slope, reached by completely different routes: one changes
`opt.gravity`, the other changes the contact geometry. So they are a genuine
cross-check on each other, and
`test_tilted_ground_agrees_with_rotated_gravity` is the test this file exists
for. If those two ever disagree, one of them is wrong and the disagreement is
the finding.

They are not redundant, either. Rotated gravity can only ever be an infinite
uniform plane -- no crest, no grade transition -- and it CANNOT BE FILMED: it
renders as a board on flat ground accelerating for no visible reason, which
reads as a bug rather than a hill. Tilted ground is the one that can carry a
crest later, and the only one that can go in a build log.

WHAT IS DELIBERATELY NOT HERE
-----------------------------
No control-loop assertions, no envelope numbers, no estimator error. Those are
Senior Controls' acceptance criteria and live in `test_hill.py`. This file
tests the WORLD: that the plane is where it says it is, the board starts on it,
the contact geometry is right at both ends, and the two hill models agree.
Keeping the seam clean is what stops a terrain test becoming a tripwire on
somebody else's tuning.
"""

import math

import mujoco
import numpy as np
import pytest

from sim.scenarios.plant import (
    MODEL_PATH,
    build_model,
    slope_normal,
    slope_rad,
    strike_angles_deg,
    tilt_ground,
)

GRADES = [-20.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 20.0]

#: Flat-ground strike angle, derived from the collision hull in the model.
FLAT_STRIKE_DEG = 18.57


def _model(grade_pct=0.0):
    return build_model(0.0, 0.0, 40.0, grade_pct=grade_pct)


def _downhill_unit(grade_pct):
    """In-plane direction of steepest descent, world frame."""
    phi = slope_rad(grade_pct)
    return np.array([-math.cos(phi), 0.0, -math.sin(phi)])


# ---------------------------------------------------------------------------
# The plane is where it claims to be
# ---------------------------------------------------------------------------

@pytest.mark.parametrize("grade", GRADES)
def test_slope_normal_matches_the_compiled_model(grade):
    """The analytic normal and MuJoCo's compiled one, to machine precision.

    `slope_normal` is used to derive strike angles and to project displacement,
    so if it drifts from what MuJoCo actually compiled, every number downstream
    is quietly wrong while the model still looks tilted.
    """
    m = _model(grade)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    gid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_GEOM, "ground")
    compiled = d.geom_xmat[gid].reshape(3, 3)[:, 2]
    assert np.allclose(compiled, slope_normal(grade), atol=1e-12)


def test_zero_grade_is_a_byte_for_byte_no_op():
    """Every existing caller passes no grade, so grade 0 must change nothing.

    Checked on the XML rather than on behaviour: a model that merely *behaves*
    identically could still have picked up a 0-degree euler that a later diff
    would have to explain.
    """
    xml = MODEL_PATH.read_text()
    assert tilt_ground(xml, 0.0) == xml


def test_tilt_refuses_to_silently_no_op():
    """A strip that does not strip is the failure this repo has already had.

    If the ground geom's markup changes, tilting must raise rather than hand
    back a flat model that the caller believes is a hill.
    """
    with pytest.raises(RuntimeError, match="ground plane geom"):
        tilt_ground("<mujoco><worldbody/></mujoco>", 10.0)


@pytest.mark.parametrize("grade", [g for g in GRADES if g != 0.0])
def test_the_axle_starts_exactly_one_rolling_radius_off_the_slope(grade):
    """Tilting the plane without raising the axle buries the wheel.

    The perpendicular distance from the un-raised axle to a plane through the
    origin is `h*cos(phi)` -- short of the rolling radius, so MuJoCo resolves it
    as a penetration and ejects the board on the first step. This is the edit
    that is easy to forget because the XML still looks right.
    """
    m = _model(grade)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "frame")
    flat = mujoco.MjData(_model(0.0))
    mujoco.mj_forward(_model(0.0), flat)
    r_flat = float(flat.xpos[bid][2])
    assert float(np.array(d.xpos[bid]) @ slope_normal(grade)) == pytest.approx(
        r_flat, abs=1e-6
    )


@pytest.mark.parametrize("grade", [0.0, 10.0, 20.0])
def test_the_board_settles_on_the_slope_rather_than_being_spat_out(grade):
    """Two seconds of free settling: the axle stays a rolling radius off the
    surface, to within tyre compression.

    A model that starts penetrated does not fail loudly -- it launches, then
    lands, and every early sample of whatever scenario is running is garbage.
    """
    m = _model(grade)
    d = mujoco.MjData(m)
    mujoco.mj_forward(m, d)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "frame")
    n = slope_normal(grade)
    dists = []
    for _ in range(1000):
        mujoco.mj_step(m, d)
        dists.append(float(np.array(d.xpos[bid]) @ n))
    excursion_mm = 1000.0 * (max(dists) - min(dists))
    assert excursion_mm < 5.0, f"{excursion_mm:.2f} mm of bounce — board was ejected"


# ---------------------------------------------------------------------------
# Sign convention — measured, not asserted
# ---------------------------------------------------------------------------

def test_positive_grade_rolls_forward_downhill():
    """`grade_pct > 0` means downhill is FORWARD (-x), matching hill.py.

    Measured by letting the board roll, not derived. Getting this backwards
    gives a scenario that quietly measures a climb while its filename says
    descent, and this repo has already shipped one 'derived' frame convention
    that was wrong.
    """
    bid = None
    drift = {}
    for grade in (+10.0, -10.0):
        m = _model(grade)
        d = mujoco.MjData(m)
        bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "frame")
        mujoco.mj_forward(m, d)
        for _ in range(3000):
            mujoco.mj_step(m, d)
        drift[grade] = float(d.xpos[bid][0])

    assert drift[+10.0] < -0.5, (
        f"positive grade drifted x={drift[+10.0]:+.3f}; downhill must be FORWARD (-x)"
    )
    assert drift[-10.0] > +0.5, (
        f"negative grade drifted x={drift[-10.0]:+.3f}; downhill must be BACKWARD (+x)"
    )


# ---------------------------------------------------------------------------
# Contact geometry at BOTH ends
# ---------------------------------------------------------------------------

def test_flat_ground_reproduces_the_known_strike_angle_at_both_bumpers():
    """The anchor. On the flat the geometry is symmetric and both bumpers reach
    the ground at the same angle in opposite directions."""
    s = strike_angles_deg(_model(0.0), 0.0)
    assert s["nose"] == pytest.approx(-FLAT_STRIKE_DEG, abs=0.02)
    assert s["tail"] == pytest.approx(+FLAT_STRIKE_DEG, abs=0.02)


@pytest.mark.parametrize("grade", GRADES)
def test_relative_strike_angle_is_invariant_across_grades(grade):
    """Tilting the world must not move the contact case.

    This is the property that made rotated gravity the right first call, and it
    has to survive here or the two hill models are measuring different vehicles.
    The angle BETWEEN the deck and the ground at which a bumper lands is a rigid
    -body fact; only its expression in world pitch changes with the slope.
    """
    s = strike_angles_deg(_model(grade), grade)
    assert s["relative_deg"] == pytest.approx(FLAT_STRIKE_DEG, abs=0.05)


@pytest.mark.parametrize("grade", [5.0, 10.0, 15.0, 20.0])
def test_a_descent_brings_the_tail_strike_closer_and_a_climb_the_nose(grade):
    """The asymmetry Senior Controls measured, falling out of geometry.

    The hill gate found that descents fail tail-down and climbs fail nose-down.
    That is not a control result — it is where the board already sits before it
    has pitched at all. Resting flat on a slope IS a world pitch of -phi, so a
    descent has already spent `phi` of its tail margin and gained `phi` of nose
    margin.

    Asserting it here means the model reproduces a measured hardware-shaped
    finding rather than merely being self-consistent.
    """
    phi = math.degrees(slope_rad(grade))
    down = strike_angles_deg(_model(+grade), +grade)
    up = strike_angles_deg(_model(-grade), -grade)

    assert down["tail"] == pytest.approx(FLAT_STRIKE_DEG - phi, abs=0.05)
    assert down["nose"] == pytest.approx(-(FLAT_STRIKE_DEG + phi), abs=0.05)
    assert up["nose"] == pytest.approx(-(FLAT_STRIKE_DEG - phi), abs=0.05)
    assert up["tail"] == pytest.approx(FLAT_STRIKE_DEG + phi, abs=0.05)

    assert abs(down["tail"]) < abs(down["nose"]), "descent must fail tail-first"
    assert abs(up["nose"]) < abs(up["tail"]), "climb must fail nose-first"


# ---------------------------------------------------------------------------
# The cross-check this file exists for
# ---------------------------------------------------------------------------

def _roll_tilted(grade, steps):
    """Free-roll on a tilted plane; return along-slope displacement."""
    m = _model(grade)
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "frame")
    mujoco.mj_forward(m, d)
    p0 = np.array(d.xpos[bid], dtype=float)
    u = _downhill_unit(grade)
    out = []
    for _ in range(steps):
        mujoco.mj_step(m, d)
        out.append(float((np.array(d.xpos[bid]) - p0) @ u))
    return np.asarray(out)


def _roll_rotated_gravity(grade, steps):
    """Free-roll on flat ground with gravity rotated; return the same quantity.

    This is `hill.py`'s model, reproduced here directly rather than imported, so
    the cross-check does not depend on that module's scenario plumbing.
    """
    m = _model(0.0)
    phi = slope_rad(grade)
    m.opt.gravity = [-9.81 * math.sin(phi), 0.0, -9.81 * math.cos(phi)]
    d = mujoco.MjData(m)
    bid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "frame")
    mujoco.mj_forward(m, d)
    x0 = float(d.xpos[bid][0])
    out = []
    for _ in range(steps):
        mujoco.mj_step(m, d)
        out.append(-(float(d.xpos[bid][0]) - x0))   # downhill is -x
    return np.asarray(out)


@pytest.mark.parametrize("grade", [0.0, 10.0])
def test_tilted_ground_agrees_with_rotated_gravity(grade):
    """**The reason both models exist.**

    Two independent routes to a slope — one changes `opt.gravity`, the other
    changes the contact geometry — must produce the same free-roll down it. On
    the flat they are the same model and must agree exactly; at a mid grade they
    are genuinely different formulations and are allowed to differ only by
    contact-solver detail.

    Senior Controls asked for this explicitly in issue #17: *"if they disagree,
    that disagreement is itself a finding and I would rather see it early."*
    """
    steps = 1500                      # 3 s at the model's 2 ms step
    a = _roll_tilted(grade, steps)
    b = _roll_rotated_gravity(grade, steps)

    if grade == 0.0:
        assert np.allclose(a, b, atol=1e-9), "on the flat these must be one model"
        return

    travelled = abs(b[-1])
    assert travelled > 0.5, "the reference model barely moved; test proves nothing"
    err = abs(a[-1] - b[-1]) / travelled
    assert err < 0.02, (
        f"tilted ground and rotated gravity disagree by {err:.2%} over "
        f"{travelled:.2f} m at {grade:g}% — one of the two hill models is wrong"
    )


def test_the_two_models_disagree_when_they_should():
    """A negative control, so the agreement test above cannot pass vacuously.

    Comparing a 10% tilted plane against 0% rotated gravity has to fail the same
    tolerance the real comparison passes. Without this, a bug that made both
    helpers return zeros would look like perfect agreement.
    """
    steps = 1500
    a = _roll_tilted(10.0, steps)
    b = _roll_rotated_gravity(0.0, steps)
    assert abs(a[-1] - b[-1]) > 0.5, "the comparison is not sensitive to grade"
