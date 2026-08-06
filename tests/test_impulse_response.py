"""Acceptance gate for the impulse disturbance-response scenario (GitHub #2).

This is the sim-in-the-loop CI gate. It runs on every push, takes a couple of
seconds, and needs no GL context -- rendering is a separate, mainline-only step
so that a broken graphics stack can never fail the physics gate.

The suite is built around one idea: the topple must be CAUSED BY THE IMPULSE.
A test that only checks "big kick -> falls over" would still pass against a
model that falls over on its own, which is exactly the bug that motivated the
rewrite (the old model faked instability with a rider-mass-on-a-mast). So the
positive case is always paired with a negative one.
"""

import mujoco
import numpy as np
import pytest

from sim.scenarios.impulse_response import (
    NOMINAL_IMPULSE_NS,
    SUBTHRESHOLD_IMPULSE_NS,
    ImpulseParams,
    frame_pitch_deg,
    load_model,
    nose_strike_angle_deg,
    run,
)

#: Documented strike angle. Geometry-derived, so a mesh or tire-radius change
#: should trip this and force the docs to be updated with it.
EXPECTED_STRIKE_ANGLE_DEG = 18.57


@pytest.fixture(scope="module")
def model():
    return load_model()


# --------------------------------------------------------------------------
# The model itself
# --------------------------------------------------------------------------

def test_no_rider_proxy_bodies(model):
    """The lollipop is gone and must stay gone.

    `rider_mass` on a `mast` was a stand-in that made a driverless board
    unstable by construction. Ridden dynamics belong to D4, not to this test.
    """
    names = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i) for i in range(model.nbody)
    }
    assert "rider_mass" not in names
    assert "mast" not in names
    assert names >= {"frame", "wheel"}, "expected the real frame-on-wheel topology"


def test_mass_is_a_plausible_driverless_board(model):
    total = float(model.body_subtreemass[1])
    assert 10.0 <= total <= 16.0, f"{total:.2f} kg is not a believable onewheel"


def test_centre_of_mass_sits_at_or_below_the_axle(model):
    """Why the board is stable at rest: battery and hub motor are low.

    If this ever goes positive the board becomes an inverted pendulum again and
    the whole premise of the scenario (a *disturbance* causes the topple) is
    void -- so it is asserted rather than left as a comment.
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    com_z = float(data.subtree_com[frame][2])
    axle_z = float(data.xpos[frame][2])
    assert com_z < axle_z, f"CoM {com_z:.4f} m is above the axle {axle_z:.4f} m"


def test_motor_and_imu_seam_exists(model):
    """The closed-loop hook. One torque actuator on the wheel hinge, plus the
    IMU/drive signals the controller will read, so wiring the Rust stack in is
    a substitution and not a model change."""
    assert model.nu == 1
    assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_ACTUATOR, "wheel_motor") == 0
    for s in ("frame_quat", "frame_gyro", "frame_accel", "wheel_vel", "motor_torque"):
        assert mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_SENSOR, s) >= 0, s


# --------------------------------------------------------------------------
# The topple criterion
# --------------------------------------------------------------------------

def test_strike_angle_is_geometry_derived_and_documented(model):
    angle = nose_strike_angle_deg(model)
    assert angle == pytest.approx(EXPECTED_STRIKE_ANGLE_DEG, abs=0.1)


def test_forty_five_degrees_is_unreachable(model):
    """Guards the reason the original 45 deg acceptance criterion was dropped.

    The board cannot tilt to 45 deg while upright -- the bumper is on the
    ground less than halfway there. A 45 deg gate would have been unfalsifiable.
    """
    assert nose_strike_angle_deg(model) < 45.0


# --------------------------------------------------------------------------
# Disturbance response -- the paired positive/negative case
# --------------------------------------------------------------------------

def test_board_rests_upright_with_no_disturbance(model):
    """Undisturbed, a driverless onewheel just sits there. This is the control
    case: without it, a topple proves nothing about the impulse."""
    result = run(ImpulseParams(magnitude_ns=0.0, sim_seconds=5.0), model=model)
    assert not result.metrics.nose_strike
    assert np.abs(result.pitch_deg).max() < 0.1
    assert abs(result.metrics.travel_m) < 1e-3


def test_nominal_impulse_causes_a_nose_strike(model):
    m = run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS), model=model).metrics
    assert m.nose_strike
    assert m.t_strike_s > ImpulseParams().t0_s, "strike must follow the kick, not precede it"
    assert m.peak_abs_pitch_deg >= nose_strike_angle_deg(model) - 0.1
    # Severity, not just occurrence: a graze and a real dig-in are different.
    assert m.speed_at_strike_ms > 0.5
    # Magnitude, because the rate is signed in the ICD convention and a nose
    # striking the ground is pitching DOWN, i.e. negative. Severity is how fast
    # it was moving, not which way.
    assert m.pitch_rate_at_strike_dps < 0, "the nose must be falling when it strikes"
    assert abs(m.pitch_rate_at_strike_dps) > 10.0


def test_subthreshold_impulse_does_not_topple(model):
    """The other half of causality: a small push must be survivable, with
    margin. Together with the nominal case this brackets the real threshold.

    NOT RE-MEASURED (widened) this session (issue: realistic-motor-torque) --
    left FAILING. This is a margin criterion ("survivable, WITH MARGIN"), not
    a descriptive pin, and the finding underneath it is the reason not to
    touch the bound: `SUBTHRESHOLD_IMPULSE_NS` (6.0 N*s) now peaks at 17.35
    deg -- 93.4% of the 18.57 deg strike angle, not "roughly half" as
    designed -- because the passive open-loop knee (docstring in `sim/
    scenarios/impulse_response.py`: "~12.5 N*s") has moved down to
    somewhere around 6-8 N*s. A "subthreshold" constant sitting almost
    exactly ON the knee it was chosen to sit well clear of is a safety-margin
    collapse, not a number that drifted -- widening the 0.75x bound to match
    would conceal exactly that. Left failing and reported; `sim/scenarios/
    impulse_response.py`'s `SUBTHRESHOLD_IMPULSE_NS` and its knee docstring
    are shared production code, not a test-local pin, and are deliberately
    NOT touched here either -- this needs a driver/oracle decision, not an
    edit.
    """
    m = run(ImpulseParams(magnitude_ns=SUBTHRESHOLD_IMPULSE_NS), model=model).metrics
    assert not m.nose_strike
    assert m.peak_abs_pitch_deg < 0.75 * nose_strike_angle_deg(model)
    assert m.travel_m > 0.1, "it should still roll -- otherwise the kick did nothing"


def test_response_is_monotonic_in_impulse(model):
    """Bigger kick, bigger pitch. Cheap, but it catches sign errors and
    saturation bugs that a single-point test sails past."""
    peaks = [
        run(ImpulseParams(magnitude_ns=j, sim_seconds=4.0), model=model).metrics.peak_abs_pitch_deg
        for j in (2.0, 4.0, 6.0, 8.0, 10.0)
    ]
    assert all(b > a for a, b in zip(peaks, peaks[1:])), peaks


def test_forward_impulse_drives_the_board_forward(model):
    """Sign check across the whole convention stack -- forward is -X, pitch is
    NOSE-UP-POSITIVE (ICD 10.1). Sign errors here are the classic way a balance
    controller ends up driving itself into the ground."""
    r = run(ImpulseParams(magnitude_ns=SUBTHRESHOLD_IMPULSE_NS), model=model)
    assert r.metrics.travel_m > 0
    assert r.pitch_deg.min() < 0, "a forward push must pitch the nose DOWN, i.e. negative"


# --------------------------------------------------------------------------
# Determinism
# --------------------------------------------------------------------------

def test_repeat_runs_are_bit_identical(model):
    """Same params, same trajectory -- exactly. Asserted on one machine, where
    it is a real property; the CI gate compares metrics against thresholds with
    margin rather than baselines, because MuJoCo is not bit-portable across
    platforms."""
    a = run(ImpulseParams(), model=model)
    b = run(ImpulseParams(), model=model)
    assert np.array_equal(a.pitch_deg, b.pitch_deg)
    assert np.array_equal(a.travel_m, b.travel_m)
    assert a.to_json_dict()["metrics"] == b.to_json_dict()["metrics"]


def test_fresh_model_load_gives_the_same_answer():
    """Guards against state leaking through the module-scoped model fixture."""
    a = run(ImpulseParams(), model=load_model())
    b = run(ImpulseParams(), model=load_model())
    assert np.array_equal(a.pitch_deg, b.pitch_deg)


# --------------------------------------------------------------------------
# The closed-loop hook (the controller itself is a later milestone)
# --------------------------------------------------------------------------

def test_motor_sign_matches_icd(model):
    """BoardIo ICD §7.3: `amps > 0` => forward wheel acceleration => nose up.

    The ICD calls a sign error across this seam the most dangerous bug in the
    system, and says not to implement it from memory -- so it is asserted here
    against the plant rather than trusted to a comment. Pitch is
    NOSE-UP-POSITIVE (ICD 10.1), so "nose up" means pitch goes POSITIVE.

    This caught a real inversion: with the wheel hinge on +Y, positive current
    drove the board backwards and pitched the nose down, which would have
    flipped the sign of the balance law the moment a controller was attached.
    """
    fwd = run(ImpulseParams(magnitude_ns=0.0, sim_seconds=1.5), model=model,
              controller=lambda t, p, pr, w, **_: +6.0)
    assert fwd.metrics.travel_m > 0.1, "positive current must drive the board FORWARD"
    assert fwd.pitch_deg[-1] > 1.0, "positive current must pitch the nose UP"
    assert fwd.wheel_rate_rads[-1] > 0, "forward roll must be positive joint velocity"

    # ...and the mirror, so the test cannot pass on a model that ignores sign.
    rev = run(ImpulseParams(magnitude_ns=0.0, sim_seconds=1.5), model=model,
              controller=lambda t, p, pr, w, **_: -6.0)
    assert rev.metrics.travel_m < -0.1
    assert rev.pitch_deg[-1] < -1.0


def test_balance_law_sign_is_stabilising(model):
    """The consequence of 7.3 that actually matters: feedback must push the
    board back toward upright rather than drive it into the ground.

    With pitch nose-up-positive (ICD 10.1) the stabilising law is the ICD's own
    `current ~= -K*pitch`, K > 0, with no convention flip anywhere. That is the
    whole point of having moved the sim onto the ICD's convention: the law is
    now written the same way in the document, the model comment and the code.

    Measured, not assumed -- inverting the sign here drives the board over.

    Gains are in AMPS PER RADIAN, because the hook takes radians.
    """
    def pd(t, pitch_rad, pitch_rate_rad_s, wheel_rate, **_):
        amps = -(80.0 * pitch_rad + 11.0 * pitch_rate_rad_s)
        return max(-40.0, min(40.0, amps))

    open_loop = run(ImpulseParams(), model=model)
    damped = run(ImpulseParams(), model=model, controller=pd)
    assert damped.metrics.peak_abs_pitch_deg < open_loop.metrics.peak_abs_pitch_deg, (
        f"stabilising feedback made it worse: {damped.metrics.peak_abs_pitch_deg:.2f} deg "
        f"vs {open_loop.metrics.peak_abs_pitch_deg:.2f} deg open loop -- sign is inverted"
    )
    assert not damped.metrics.nose_strike, "PD feedback should prevent the strike entirely"


def test_inverting_the_balance_law_drives_the_board_over(model):
    """The mutation test for the sign gate.

    A polarity assertion that has never been seen to fail is not a gate. This
    runs the same law with K negated and requires the board to strike -- so if
    someone flips a sign somewhere and the stabilising test starts passing for
    the wrong reason, this one stops passing.
    """
    def inverted_pd(t, pitch_rad, pitch_rate_rad_s, wheel_rate, **_):
        amps = +(80.0 * pitch_rad + 11.0 * pitch_rate_rad_s)
        return max(-40.0, min(40.0, amps))

    r = run(ImpulseParams(), model=model, controller=inverted_pd)
    assert r.metrics.nose_strike, (
        "positive feedback must drive the board into the ground; if this passes "
        "the pitch or motor sign convention has moved and the gate is inert"
    )
