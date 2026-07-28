"""The imperfection profile — ICD §12, SR-SIM-3.

Two jobs. The unit tests check that each imperfection does what it claims,
because a profile that is silently inert is worse than none — it makes a gate
*look* rigorous. The integration tests check the controller against it, and
record the margin.
"""

import numpy as np
import pytest

from sim.scenarios.imperfections import (
    IDEAL,
    STAGE0_PLACEHOLDER,
    ImperfectionProfile,
    ImperfectionState,
)
from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS, ImpulseParams, run
from sim.scenarios.plant import build_model
from sim.scenarios.rust_controller import RustController

DT = 0.002


def state(**kw):
    return ImperfectionState(profile=ImperfectionProfile("t", **kw), dt_s=DT)


# --------------------------------------------------------------------------
# SR-SIM-3: no ideal-only mode in CI
# --------------------------------------------------------------------------

def test_the_ideal_profile_is_recognisable_as_ideal():
    assert IDEAL.is_ideal()


def test_the_working_profile_is_not_ideal():
    """The requirement, as a test. If someone hollows out the placeholder to
    make a gate pass, this is what should stop them."""
    assert not STAGE0_PLACEHOLDER.is_ideal()


def test_the_scenarios_default_to_a_non_ideal_profile():
    """A gate is only meaningful if the imperfections are on by DEFAULT.
    Requiring callers to opt in would mean every test that forgot ran ideal."""
    import inspect

    from sim.scenarios import impulse_response, shuttle_run

    for mod in (impulse_response, shuttle_run):
        default = inspect.signature(mod.run).parameters["profile"].default
        assert not default.is_ideal(), f"{mod.__name__}.run defaults to an ideal profile"


# --------------------------------------------------------------------------
# Actuation delay
# --------------------------------------------------------------------------

def test_zero_delay_passes_the_command_straight_through():
    st = state(max_current_a=100.0)
    assert st.apply_current(1.0) == pytest.approx(1.0)


def test_a_whole_cycle_delay_holds_the_command_for_exactly_one_step():
    st = state(actuation_delay_s=DT, max_current_a=100.0)
    assert st.apply_current(1.0) == pytest.approx(0.0)
    assert st.apply_current(1.0) == pytest.approx(1.0)


def test_a_half_cycle_delay_is_interpolated_not_rounded_away():
    """The bug this replaces: `round(0.001/0.002)` is 0 in Python, so the ICD's
    own 1 ms estimate silently became NO delay at the 2 ms timestep — deleting
    the imperfection while appearing to model it."""
    st = state(actuation_delay_s=DT / 2, max_current_a=100.0)
    assert st.apply_current(1.0) == pytest.approx(0.5)
    assert st.apply_current(1.0) == pytest.approx(1.0)


def test_delay_is_expressed_in_seconds_not_cycles():
    """The same physical delay must behave the same at any timestep, or the
    profile is describing the simulation rather than the hardware."""
    fine = ImperfectionState(
        profile=ImperfectionProfile("f", actuation_delay_s=0.004, max_current_a=100.0),
        dt_s=0.001,
    )
    coarse = ImperfectionState(
        profile=ImperfectionProfile("c", actuation_delay_s=0.004, max_current_a=100.0),
        dt_s=0.004,
    )
    fine_out = [fine.apply_current(1.0) for _ in range(8)]
    coarse_out = [coarse.apply_current(1.0) for _ in range(2)]
    assert fine_out[3] == pytest.approx(0.0)  # still delayed at 3 ms
    assert fine_out[4] == pytest.approx(1.0)  # arrived at 4 ms
    assert coarse_out[0] == pytest.approx(0.0)
    assert coarse_out[1] == pytest.approx(1.0)


# --------------------------------------------------------------------------
# Current loop, cap, sensing
# --------------------------------------------------------------------------

def test_the_current_loop_lags_rather_than_stepping():
    """ICD §12 names echoing the command back as measured current as
    non-conforming: it hides the phase the controller must tolerate."""
    st = state(current_loop_tau_s=0.004, max_current_a=100.0)
    first = st.apply_current(1.0)
    assert 0.0 < first < 1.0
    settled = [st.apply_current(1.0) for _ in range(200)][-1]
    assert settled == pytest.approx(1.0, abs=1e-3)


def test_the_current_cap_binds_symmetrically():
    st = state(max_current_a=10.0)
    assert st.apply_current(999.0) == pytest.approx(10.0)
    st2 = state(max_current_a=10.0)
    assert st2.apply_current(-999.0) == pytest.approx(-10.0)


def test_gyro_noise_is_present_and_seeded():
    a = state(gyro_noise_rad_s=0.01, gyro_bias_rad_s=0.0)
    b = state(gyro_noise_rad_s=0.01, gyro_bias_rad_s=0.0)
    sa = [a.gyro(0.0) for _ in range(500)]
    sb = [b.gyro(0.0) for _ in range(500)]
    assert sa == sb, "same seed must give the same noise, or every gate is flaky"
    assert np.std(sa) == pytest.approx(0.01, rel=0.2)


def test_gyro_bias_is_a_fixed_offset_not_noise():
    st = state(gyro_bias_rad_s=0.05)
    assert st.gyro(0.0) == pytest.approx(0.05)
    assert st.gyro(1.0) == pytest.approx(1.05)


def test_wheel_rate_is_quantised():
    st = state(wheel_rate_quantum_rad_s=0.1)
    assert st.wheel_rate(0.34, 0.0) == pytest.approx(0.3)
    assert st.wheel_rate(0.36, 10.0) == pytest.approx(0.4)


def test_wheel_rate_is_held_between_updates_not_interpolated():
    """A stale sample is what the loop actually receives. Smoothing it would
    hide the phase the staleness costs."""
    st = state(wheel_rate_quantum_rad_s=0.0, wheel_rate_update_hz=100.0)
    assert st.wheel_rate(1.0, 0.000) == pytest.approx(1.0)
    assert st.wheel_rate(5.0, 0.005) == pytest.approx(1.0), "should still hold the old sample"
    assert st.wheel_rate(5.0, 0.011) == pytest.approx(5.0), "and refresh after 10 ms"


# --------------------------------------------------------------------------
# Against the controller
# --------------------------------------------------------------------------

@pytest.fixture(scope="module")
def ridden():
    return build_model(70.0, 0.75, 40.0)


def _impulse(model, profile, delay_s=None):
    p = profile if delay_s is None else ImperfectionProfile(
        f"probe-{delay_s}", actuation_delay_s=delay_s,
        current_loop_tau_s=profile.current_loop_tau_s,
        gyro_noise_rad_s=profile.gyro_noise_rad_s,
        gyro_bias_rad_s=profile.gyro_bias_rad_s,
        wheel_rate_quantum_rad_s=profile.wheel_rate_quantum_rad_s,
        wheel_rate_update_hz=profile.wheel_rate_update_hz,
    )
    with RustController(kp_a_per_rad=200.0, kd_a_per_rad_s=30.0, max_current_a=40.0,
                        kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02,
                        com_above_axle=True) as c:
        return run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=8.0),
                   model=model, controller=c, profile=p)


def test_the_controller_survives_the_working_profile(ridden):
    r = _impulse(ridden, STAGE0_PLACEHOLDER)
    assert not r.metrics.nose_strike
    assert r.metrics.peak_abs_pitch_deg < 8.0
    assert r.metrics.imperfection_profile_id == STAGE0_PLACEHOLDER.profile_id


def test_the_profile_costs_something_measurable(ridden):
    """Not much, but not nothing. If the profile made literally no difference
    it would be decoration, and the gate would be ideal in all but name."""
    ideal = _impulse(ridden, IDEAL)
    real = _impulse(ridden, STAGE0_PLACEHOLDER)
    assert real.metrics.peak_abs_pitch_deg > ideal.metrics.peak_abs_pitch_deg


def test_there_is_large_margin_on_actuation_delay(ridden):
    """The Stage-0 go/no-go number, with its actual margin.

    ICD §5.4 estimates 0.4-1.0 ms. The loop is unbothered at 20 ms -- twenty
    times the estimate -- because at ~12 rad/s bandwidth a millisecond costs
    almost no phase. Measured, so the claim is not an assertion about a Bode
    plot nobody drew.

    Re-baselined for the honest number: `RustController` now runs the
    estimator by default (issue #24 AC1 -- every headline number was
    previously measured against ground-truth pitch, which no real IMU has).
    Fusing the estimate costs real margin here: 6.80 deg on truth pitch,
    9.71 deg on the estimate. Both are well clear of the 18.57 deg strike
    angle, so "unbothered" still holds -- but the number moved, which is
    exactly the point of measuring it honestly instead of on truth.
    """
    r = _impulse(ridden, STAGE0_PLACEHOLDER, delay_s=0.020)
    assert not r.metrics.nose_strike
    assert r.metrics.peak_abs_pitch_deg < 10.5


def test_enough_delay_does_break_it(ridden):
    """The other half, so the margin above is a measurement rather than an
    untested hope. Breaks between 40 and 60 ms."""
    r = _impulse(ridden, STAGE0_PLACEHOLDER, delay_s=0.080)
    assert r.metrics.nose_strike, "80 ms of delay should be fatal; if not, re-derive the margin"


def test_runs_with_the_profile_are_deterministic(ridden):
    a = _impulse(ridden, STAGE0_PLACEHOLDER)
    b = _impulse(ridden, STAGE0_PLACEHOLDER)
    assert a.to_json_dict()["metrics"] == b.to_json_dict()["metrics"]
