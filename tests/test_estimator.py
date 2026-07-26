"""Attitude estimator, measured against MuJoCo truth.

**The acceptance criterion is NOT met yet, deliberately recorded as such.** The
mini design doc set ≤0.5° RMS / ≤1.0° peak; the v1 complementary filter achieves
0.75° / 2.5° on the ridden plant under disturbance, and the shortfall is concentrated
exactly where the doc predicted it would be — under acceleration, where the
accelerometer's gravity reference is corrupted by the board's own motion.

These tests pin the *measured* behaviour rather than the target, so the gap
stays visible and any change to it is deliberate. `test_the_ac_is_not_met_yet`
is the one to delete when accel compensation lands.
"""

import numpy as np
import pytest

from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS, ImpulseParams, run
from sim.scenarios.plant import build_model
from sim.scenarios.rust_controller import RustController

TAU = 1.0


@pytest.fixture(scope="module")
def ridden():
    return build_model(70.0, 0.75, 40.0)


OFF, ACTIVE, SHADOW = 0, 1, 2


def _run(model, use_estimator, tau=TAU, secs=8.0, impulse=NOMINAL_IMPULSE_NS):
    with RustController(
        kp_a_per_rad=200.0, kd_a_per_rad_s=30.0, max_current_a=40.0,
        kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True,
        use_estimator=use_estimator, estimator_tau_s=tau,
    ) as c:
        return run(ImpulseParams(magnitude_ns=impulse, sim_seconds=secs),
                   model=model, controller=c)


def test_running_on_truth_reports_no_estimator_error(ridden):
    """The control: with the estimator off, `pitch_used` IS truth."""
    r = _run(ridden, OFF)
    assert r.metrics.pitch_est_rms_deg < 0.05


def test_shadow_mode_does_not_perturb_the_outcome(ridden):
    """Shadow must be observation-only, or every accuracy number measured
    through it is contaminated by the thing it was supposed to isolate."""
    off = _run(ridden, OFF)
    shadow = _run(ridden, SHADOW)
    assert np.allclose(off.pitch_deg, shadow.pitch_deg, atol=1e-9)
    assert shadow.metrics.pitch_est_rms_deg > 0.1, "shadow should still be estimating"


def test_the_estimator_is_actually_in_the_control_path(ridden):
    """The mutation test for a bug that already happened.

    An earlier revision computed the estimate, reported it in `pitch_used_rad`,
    and then regulated on truth anyway. Every metric looked healthy — the
    reported estimator error was a plausible 1.13° — while the estimator was
    entirely inert. Comparing the estimate against truth cannot catch that;
    only comparing the resulting MOTION can.
    """
    truth = _run(ridden, OFF)
    est = _run(ridden, ACTIVE)
    assert not np.allclose(truth.pitch_deg, est.pitch_deg, atol=1e-6), (
        "identical trajectories mean the estimator is not driving the controller"
    )


def test_the_filter_substantially_beats_the_raw_accelerometer(ridden):
    """Otherwise it is an atan2 with extra steps.

    The raw accelerometer sees ~4.4° RMS on this trajectory, dominated by
    excursions past 50° during the impulse; the filter should be several times
    better because it only believes the accelerometer slowly.
    """
    # Measured in SHADOW, so this is estimator accuracy alone rather than
    # accuracy tangled with whatever the closed loop then does about it.
    r = _run(ridden, SHADOW)
    assert r.metrics.pitch_est_rms_deg < 1.5, "filter is not filtering"
    assert r.metrics.pitch_est_max_deg < 4.0


def test_the_estimator_meets_the_ac_on_an_undisturbed_board(ridden):
    """Half the AC is already met, and saying so is the point.

    Quiet, the filter runs at ~0.10 deg RMS -- comfortably inside the 0.5 deg
    budget, and consistent with the predicted bias*tau term. So the shortfall
    under disturbance is NOT a broken filter or a bad crossover; it is the
    accelerometer corruption, isolated.
    """
    r = _run(ridden, SHADOW, impulse=0.0)
    assert r.metrics.pitch_est_rms_deg <= 0.5
    assert r.metrics.pitch_est_max_deg <= 1.0


def test_closing_the_loop_on_the_v1_estimator_destabilises_the_board(ridden):
    """**The headline result, recorded rather than hidden.**

    In shadow the filter is accurate to ~1.1 deg RMS. Driving the loop with it
    strikes. The mechanism is feedback: estimate error produces a wrong command,
    the wrong command produces acceleration, acceleration corrupts the
    accelerometer's gravity reference, and the error grows. A filter can be
    accurate open-loop and still be unusable closed-loop, which is exactly why
    shadow mode exists.

    Delete this test when accel compensation lands -- it should stop being true.
    """
    r = _run(ridden, ACTIVE)
    assert r.metrics.nose_strike, (
        "if this now survives, the estimator has improved and the xfail below "
        "plus this test should both be revisited"
    )


@pytest.mark.xfail(
    strict=True,
    reason="v1 complementary filter misses the AC (0.5 deg RMS / 1.0 deg peak) "
    "because the accelerometer's gravity reference is corrupted by the board's "
    "own acceleration -- predicted as open question 1 in the mini design doc. "
    "The fix is to subtract the wheel-odometry-derived linear term, deferred "
    "from v1 on purpose. Delete this xfail when that lands.",
)
def test_the_ac_is_not_met_yet(ridden):
    r = _run(ridden, SHADOW)
    assert r.metrics.pitch_est_rms_deg <= 0.5
    assert r.metrics.pitch_est_max_deg <= 1.0


def test_estimator_error_is_worse_while_accelerating(ridden):
    """The diagnosis, asserted.

    If the error ever stops correlating with acceleration, the cause has
    changed and the planned fix may no longer be the right one.
    """
    # A genuinely undisturbed run, not a short window -- a 3 s run still
    # contains the impulse at t = 0.5 s and is not quiet at all.
    quiet = _run(ridden, SHADOW, impulse=0.0)
    disturbed = _run(ridden, SHADOW, impulse=NOMINAL_IMPULSE_NS)
    assert quiet.metrics.pitch_est_rms_deg < 0.5 * disturbed.metrics.pitch_est_rms_deg, (
        f"quiet {quiet.metrics.pitch_est_rms_deg:.2f} deg vs "
        f"disturbed {disturbed.metrics.pitch_est_rms_deg:.2f} deg -- if these are "
        "close, acceleration is no longer the dominant error source and the "
        "planned fix may be wrong"
    )


def test_a_longer_crossover_trades_lag_for_noise_rejection(ridden):
    """Both ends of the tau trade are real, so neither extreme is a free win."""
    short = _run(ridden, SHADOW, tau=0.3)
    long = _run(ridden, SHADOW, tau=4.0)
    assert short.metrics.pitch_est_max_deg > long.metrics.pitch_est_max_deg


def test_estimator_runs_are_deterministic(ridden):
    a = _run(ridden, SHADOW)
    b = _run(ridden, SHADOW)
    assert a.to_json_dict()["metrics"] == b.to_json_dict()["metrics"]
