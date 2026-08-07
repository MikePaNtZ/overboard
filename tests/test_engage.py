"""The engage state machine, and the rest-pitch derivation it depends on.

Isolated from any scenario so the transition logic is testable without
paying for a full MuJoCo step loop -- see `test_terrain.py` for the scenario
-level wiring (spawn contact, engage timing) this machine is built into.
"""

import math

import pytest

from sim.scenarios.engage import (
    AT_REST_SPEED_M_S,
    DISENGAGE_SPEED_M_S,
    LEVEL_THRESHOLD_RAD,
    EngageState,
    resting_pitch_deg,
    transition,
)
from sim.scenarios.plant import build_model


def test_disengaged_stays_disengaged_without_a_rider():
    assert transition(EngageState.DISENGAGED, False, 0.0, 0.0) is EngageState.DISENGAGED


def test_rider_present_but_not_level_goes_to_armed_not_engaged():
    steep = LEVEL_THRESHOLD_RAD * 3.0
    s = transition(EngageState.DISENGAGED, True, steep, 0.0)
    assert s is EngageState.ARMED


def test_rider_present_but_moving_goes_to_armed_not_engaged():
    fast = AT_REST_SPEED_M_S * 10.0
    s = transition(EngageState.DISENGAGED, True, 0.0, fast)
    assert s is EngageState.ARMED


def test_all_three_conditions_at_once_engages_directly():
    """A scripted mount that satisfies rider/level/rest in the same step must
    not be forced to spend an extra step "in transit" through ARMED -- there
    is no physical reason for that delay."""
    s = transition(EngageState.DISENGAGED, True, 0.0, 0.0)
    assert s is EngageState.ENGAGED


def test_armed_engages_once_conditions_are_met():
    s = transition(EngageState.ARMED, True, 0.0, 0.0)
    assert s is EngageState.ENGAGED


def test_engaged_survives_rider_present_regardless_of_speed():
    fast = 5.0
    assert transition(EngageState.ENGAGED, True, 0.0, fast) is EngageState.ENGAGED


def test_engaged_survives_losing_the_rider_above_dismount_speed():
    """Above ~1 mph, a single remaining footpad sensor keeps the board
    engaged -- the real board's heel-lift rule."""
    fast = DISENGAGE_SPEED_M_S + 0.1
    s = transition(EngageState.ENGAGED, False, 0.0, fast)
    assert s is EngageState.ENGAGED


def test_engaged_disengages_on_heel_lift_below_dismount_speed():
    slow = DISENGAGE_SPEED_M_S - 0.1
    s = transition(EngageState.ENGAGED, False, 0.0, slow)
    assert s is EngageState.DISENGAGED


def test_disengage_speed_matches_the_published_one_mph_figure():
    assert DISENGAGE_SPEED_M_S == pytest.approx(1.0 * 1609.344 / 3600.0, abs=1e-6)


# ---------------------------------------------------------------------------
# Rest pitch -- derived from the same collision hull strike_angles_deg sweeps
# ---------------------------------------------------------------------------

def test_resting_pitch_matches_the_known_flat_strike_angle():
    """Anchored against `test_plant_terrain.py::FLAT_STRIKE_DEG` -- the same
    18.57 deg figure, reached by importing the same underlying sweep."""
    probe = build_model(0.0, 0.0, None)
    assert resting_pitch_deg(probe, 0.0) == pytest.approx(-18.57, abs=0.02)


def test_resting_pitch_is_nose_down():
    probe = build_model(0.0, 0.0, None)
    assert resting_pitch_deg(probe, 0.0) < 0.0


def test_axle_height_is_unchanged_by_the_rest_pitch():
    """The property the whole derivation leans on: pitching the frame about
    its own wheel-spin axis does not change the wheel's ground clearance, so
    the same axle height that holds the board level also holds it resting
    nose-down -- no separate correction needed at spawn time."""
    import mujoco

    probe = build_model(0.0, 0.0, None)
    data = mujoco.MjData(probe)
    mujoco.mj_forward(probe, data)
    frame = mujoco.mj_name2id(probe, mujoco.mjtObj.mjOBJ_BODY, "frame")
    level_z = float(data.xpos[frame][2])

    nose_deg = resting_pitch_deg(probe, 0.0)
    data.qpos[3:7] = _euler_y_quat(math.radians(nose_deg))
    mujoco.mj_forward(probe, data)
    tilted_z = float(data.xpos[frame][2])
    assert tilted_z == pytest.approx(level_z, abs=1e-9)


def test_spawning_at_the_rest_pitch_makes_ground_contact():
    """The property that makes this a valid spawn: at the derived angle both
    the wheel and the bumper actually touch, unlike the analytic level spawn
    this replaces (which had `ncon == 0`)."""
    import mujoco

    probe = build_model(0.0, 0.0, None)
    data = mujoco.MjData(probe)
    mujoco.mj_forward(probe, data)
    nose_deg = resting_pitch_deg(probe, 0.0)
    data.qpos[3:7] = _euler_y_quat(math.radians(nose_deg))
    mujoco.mj_forward(probe, data)
    assert data.ncon > 0


def _euler_y_quat(theta_rad: float):
    """Quaternion (w, x, y, z) for a pure rotation about +Y by theta_rad --
    MuJoCo's own convention for the frame body's `euler="0 deg 0"` markup,
    reproduced here rather than imported so this test does not depend on
    where a scenario happens to build its XML string."""
    half = theta_rad / 2.0
    return (math.cos(half), 0.0, math.sin(half), 0.0)
