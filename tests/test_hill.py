"""Hill scenario: the sign convention, the guards, and the gate.

Three kinds of test here, deliberately separated:

  CONTRACT   the grade sign and the cutback model, checked by derivation or
             arithmetic rather than by running the plant
  GUARD      the refusals -- a hill result must not be producible against a
             motor that never derates, an ideal profile, or a missing wheel
             rate. These exist because this codebase has twice shipped a
             result that was wrong because something optional was left at its
             default
  GATE       what the board can actually do, and the evidence that the
             ATTITUDE ESTIMATE is what takes the hill away rather than the
             motor
"""

import math

import numpy as np
import pytest

from sim.scenarios.hill import (
    HillParams,
    grade_angle_deg,
    gravity_for_grade,
    margin_table,
    run,
)
from sim.scenarios.imperfections import (
    IDEAL,
    STAGE0_CUTBACK,
    STAGE0_PLACEHOLDER,
    ImperfectionProfile,
    ImperfectionState,
)
from sim.scenarios.impulse_response import FORWARD

# Keep the suite affordable: 10 s is past the 2 s settle and 2 s ramp with room
# to cruise, and every failure this file asserts happens well inside it.
SHORT = 10.0


# --------------------------------------------------------------------------
# CONTRACT
# --------------------------------------------------------------------------

def test_positive_grade_is_downhill_forward():
    """The sign, measured against the model's own forward axis.

    Getting this backwards would silently swap every descent in the suite for
    a climb, and the two fail in opposite directions -- nose vs tail -- so the
    mistake would look like a physics finding rather than a sign error.
    """
    for pct, expect_downhill in ((10.0, True), (-10.0, False)):
        g = np.array(gravity_for_grade(pct))
        along_forward = float(np.dot(g, FORWARD))
        assert (along_forward > 0.0) is expect_downhill, (
            f"grade {pct:+.0f}% put {along_forward:+.3f} m/s^2 along forward; "
            "positive grade must pull the board FORWARD (downhill)"
        )


def test_rotating_gravity_preserves_its_magnitude():
    """A rotation, not a scaling. If |g| drifted with grade, every hill result
    would be confounded by a lighter or heavier board."""
    for pct in (-20.0, -5.0, 0.0, 5.0, 20.0):
        assert np.linalg.norm(gravity_for_grade(pct)) == pytest.approx(9.81, abs=1e-9)


def test_grade_percent_is_not_the_slope_angle():
    """A 15% grade is 8.53 deg. Treating percent as degrees would overstate the
    disturbance by ~75%, and the estimator error is compared against this."""
    assert grade_angle_deg(15.0) == pytest.approx(8.5308, abs=1e-3)
    assert grade_angle_deg(100.0) == pytest.approx(45.0, abs=1e-9)


def test_cutback_is_symmetric_in_direction():
    """Cutback is about how fast the wheel turns, not which way. Braking into a
    descent is exactly the case where a one-sided model would be wrong."""
    p = STAGE0_CUTBACK
    for w in (10.0, 30.0, 45.0, 90.0):
        assert p.available_current_a(w) == pytest.approx(p.available_current_a(-w))


def test_cutback_falls_monotonically_to_the_floor():
    p = STAGE0_CUTBACK
    ws = np.linspace(0.0, p.derate_full_rad_s * 1.5, 60)
    avail = [p.available_current_a(w) for w in ws]
    assert all(b <= a + 1e-9 for a, b in zip(avail, avail[1:])), "not monotonic"
    assert avail[0] == pytest.approx(p.max_current_a)
    assert avail[-1] == pytest.approx(p.max_current_a * p.derate_floor_frac)


# --------------------------------------------------------------------------
# GUARD
# --------------------------------------------------------------------------

def test_apply_current_refuses_a_missing_wheel_rate_when_cutback_is_modelled():
    """The third instance of this bug class, refused at the door.

    The first two shipped: a shuttle that never passed its IMU, and an analysis
    loop whose estimator was never in the loop. Both degraded quietly and both
    produced numbers that looked like results.
    """
    st = ImperfectionState(profile=STAGE0_CUTBACK, dt_s=0.002)
    with pytest.raises(ValueError, match="cutback"):
        st.apply_current(40.0)
    # ...and is fine once told.
    assert st.apply_current(40.0, 0.0) == pytest.approx(0.0, abs=40.0)


def test_a_profile_without_cutback_still_takes_no_wheel_rate():
    """The guard must not become a tax on every other scenario."""
    st = ImperfectionState(profile=STAGE0_PLACEHOLDER, dt_s=0.002)
    st.apply_current(1.0)  # must not raise


def test_run_refuses_a_profile_that_models_no_cutback():
    with pytest.raises(ValueError, match="does not model cutback"):
        run(HillParams(duration_s=SHORT), profile=STAGE0_PLACEHOLDER)


def test_run_accepts_no_cutback_only_when_asked_in_as_many_words():
    r = run(HillParams(grade_pct=5.0, duration_s=SHORT),
            profile=STAGE0_PLACEHOLDER, allow_no_cutback=True)
    assert r.metrics.imperfection_profile_id == STAGE0_PLACEHOLDER.profile_id


def test_run_refuses_an_ideal_profile():
    with pytest.raises(ValueError, match="ideal"):
        run(HillParams(duration_s=SHORT), profile=IDEAL, allow_no_cutback=True)


def test_run_refuses_the_driverless_plant():
    """CoM below the axle inverts the outer loop's coupling. The shuttle refuses
    it for the same reason; a hill result on that plant would be meaningless."""
    with pytest.raises(ValueError, match="RIDDEN"):
        run(HillParams(ballast_mass_kg=0.0, duration_s=SHORT))


# --------------------------------------------------------------------------
# GATE
# --------------------------------------------------------------------------

def test_a_crashed_run_can_never_report_that_it_held_speed():
    """**The bug this file's first draft shipped.**

    A crashed board coasts to rest, so `v_final ~= 0`, so every speed check
    passes trivially -- the first sweep reported `held_speed = True` on runs
    that had struck the ground. Validity has to sit outside the statistics.
    """
    r = run(HillParams(grade_pct=20.0, v_ref_m_s=2.0, duration_s=SHORT))
    m = r.metrics
    assert m.nose_strike, "expected a 20% grade to defeat the estimator"
    assert not m.survived
    assert not m.held_speed, "a run that struck the ground reported holding speed"


def test_the_run_stops_at_the_strike():
    """No statistics from a board tumbling on the ground. Before this, a
    diverged run reported a 179.97 deg peak pitch and a 271 deg estimator RMS."""
    r = run(HillParams(grade_pct=20.0, v_ref_m_s=2.0, duration_s=SHORT))
    assert r.metrics.nose_strike
    assert r.t[-1] == pytest.approx(r.metrics.t_strike_s, abs=0.01)
    assert r.metrics.peak_abs_pitch_deg < 25.0, (
        "peak pitch kept growing after the strike -- the run did not stop"
    )


def test_the_struck_end_is_named_and_the_two_directions_differ():
    """Descending and climbing fail in OPPOSITE directions, and an
    `abs(pitch) >= 18.57` test would have called both a nose strike.

    Descending, the controller over-holds and the TAIL goes down. Climbing, it
    under-holds and the NOSE goes down. That asymmetry is the mechanism.
    """
    down = run(HillParams(grade_pct=20.0, v_ref_m_s=2.0, duration_s=SHORT)).metrics
    up = run(HillParams(grade_pct=-15.0, v_ref_m_s=2.0, duration_s=SHORT)).metrics
    assert down.nose_strike and up.nose_strike
    assert down.struck_end == "tail", f"descent struck {down.struck_end!r}"
    assert up.struck_end == "nose", f"climb struck {up.struck_end!r}"


@pytest.mark.parametrize("v_ref", [1.0, 2.0, 3.0])
def test_the_board_holds_a_medium_descent(v_ref):
    """**The gate.** A 10% descent, driven, at three commanded speeds.

    10% is the medium grade the scenario is specified around: comfortably
    inside what the board can do today, and steep enough that the estimator
    error is a substantial fraction of the slope.
    """
    m = run(HillParams(grade_pct=10.0, v_ref_m_s=v_ref, duration_s=SHORT)).metrics
    assert m.survived, f"struck the {m.struck_end} at {m.t_strike_s}s"
    assert m.held_speed, f"v_ref {v_ref} -> v_final {m.v_final_m_s:.2f}"
    assert abs(m.v_final_m_s - v_ref) < 0.25, (
        f"commanded {v_ref} m/s, finished at {m.v_final_m_s:.2f} m/s"
    )


def test_the_board_holds_a_medium_climb():
    """The other direction, which the handoff never measured. A 5% climb is
    inside the envelope; -10% is not, and that asymmetry is asserted below."""
    m = run(HillParams(grade_pct=-5.0, v_ref_m_s=2.0, duration_s=SHORT)).metrics
    assert m.survived, f"struck the {m.struck_end} at {m.t_strike_s}s"
    assert m.held_speed


def test_climbing_is_the_harder_direction():
    """Symmetric grades, opposite signs: the climb fails where the descent does
    not. Sweeping only descents -- as the original measurement did -- would
    have missed the tighter limit entirely."""
    descent = run(HillParams(grade_pct=10.0, v_ref_m_s=2.0, duration_s=SHORT)).metrics
    climb = run(HillParams(grade_pct=-10.0, v_ref_m_s=2.0, duration_s=SHORT)).metrics
    assert descent.survived, "expected a 10% descent to be inside the envelope"
    assert not climb.survived, "expected a 10% climb to be outside it"


def test_the_estimator_is_what_loses_the_hill_not_the_motor():
    """**The headline result, and the reason this scenario exists.**

    On truth pitch the board holds a 15% descent. Driving the same loop from
    the attitude ESTIMATE, it puts the tail on the ground. The motor is not the
    limiting factor and never was -- the handoff said so before the frame-map
    fix, and it is still true after it.
    """
    truth = run(HillParams(grade_pct=15.0, v_ref_m_s=2.0, use_estimator=False,
                           duration_s=SHORT)).metrics
    est = run(HillParams(grade_pct=15.0, v_ref_m_s=2.0, use_estimator=True,
                         duration_s=SHORT)).metrics

    assert truth.survived, "truth pitch should hold a 15% descent"
    assert truth.held_speed
    assert not est.survived, (
        "the estimator survived a 15% descent -- if this is now genuinely fixed, "
        "re-derive the envelope rather than loosening this test"
    )
    assert est.struck_end == "tail"


def test_the_estimator_absorbs_the_slope_into_its_attitude_error():
    """The predicted first-order mechanism, measured.

    `CommandFeedforward` attributes gravity-holding current to acceleration, so
    the apparent-tilt error approaches the slope angle itself and the estimator
    believes a board on a hill is level. Holding "level" on a hill means
    accelerating down it.
    """
    for grade in (5.0, 10.0):
        m = run(HillParams(grade_pct=grade, v_ref_m_s=1.0, duration_s=SHORT)).metrics
        assert m.survived
        assert 0.6 < m.est_error_over_slope < 1.3, (
            f"{grade}% grade: estimator error was {m.est_error_over_slope:.2f}x the "
            f"slope angle ({m.pitch_est_rms_deg:.2f} deg vs {m.slope_angle_deg:.2f} deg). "
            "The mechanism predicts ~1x; a value far from it means the mechanism moved."
        )


def test_cutback_is_reported_even_when_it_never_binds():
    """Honesty about the derate row: at the grades where the board currently
    fails, it fails before it is fast enough to be cut back.

    This is asserted rather than left implicit so nobody reads the presence of
    a cutback model as evidence that cutback was the mechanism.
    """
    m = run(HillParams(grade_pct=10.0, v_ref_m_s=2.0, duration_s=SHORT)).metrics
    assert m.imperfection_profile_id == "stage0-cutback-v1"
    assert m.min_available_a > 0.0
    assert m.cutback_binding_cycles == 0, (
        "cutback now binds on a 10% descent -- the envelope moved and the "
        "hill story needs re-deriving with the drive as a live constraint"
    )


def test_margin_table_renders():
    """The sweep's human output is part of the deliverable -- a gate that can
    only say pass/fail cannot show where the edge is."""
    rs = [run(HillParams(grade_pct=g, v_ref_m_s=1.0, duration_s=SHORT))
          for g in (5.0, 20.0)]
    text = margin_table(rs)
    assert "grade" in text and "upright" in text and "TAIL" in text


# --------------------------------------------------------------------------
# DETERMINISM
# --------------------------------------------------------------------------

def test_repeat_runs_are_bit_identical():
    """Same property `impulse_response` and `closed_loop` already pin, checked
    here too rather than assumed: a hill run crosses more state per step --
    rotated gravity, cutback, the estimator -- than either of those, so
    determinism elsewhere is not evidence of determinism here."""
    p = HillParams(grade_pct=10.0, v_ref_m_s=2.0, duration_s=SHORT)
    a = run(p)
    b = run(p)
    assert np.array_equal(a.pitch_deg, b.pitch_deg)
    assert np.array_equal(a.v_m_s, b.v_m_s)
    assert np.array_equal(a.travel_m, b.travel_m)
    assert a.to_json_dict() == b.to_json_dict()
