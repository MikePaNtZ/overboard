"""The control rate, derived — issue #113.

`docs/design-pi-image-stage0b.md` says of its own 2 ms period: *"500 Hz is an
ICD number, not physics."* These tests are the physics. They pin three things,
and each one is a claim someone could otherwise quietly break:

1.  **The linearisation is the model's, not a story about it.** Every constant
    is read out of the compiled MuJoCo model, and the resulting linear system
    is checked against that model's own free divergence and torque-step
    response. This is what pins the actuator reaction SIGN — get it backwards
    and the linear model under-predicts by 1.62x, which is exactly what
    happened while this was being derived.

2.  **The delay budget, two independent ways.** A continuous phase-margin
    calculation and an exact discrete ZOH-plus-delay eigenvalue calculation
    must agree to the `Ts/2` term. Neither is trusted alone.

3.  **The rate's whole contribution is `Ts/2`.** Which is the point: AC-6
    measures latency, and the rate spends only half a period of the latency
    budget. At 500 Hz that is 1 ms out of ~65.

MEASURED, NOT ASSUMED — every threshold below (issue #24 AC5's convention):

| assertion | threshold | observed on this checkout | status |
|---|---|---|---|
| ridden unstable pole | 5.0-5.5 rad/s | 5.2386 rad/s | MEASURED |
| point-mass approximation | 3.90-4.00 rad/s | 3.9353 rad/s | MEASURED (the figure #113 was filed with) |
| ridden delay margin | > 60 ms | 71.40 ms | MEASURED |
| driverless delay margin | > 60 ms | 65.61 ms | MEASURED |
| continuous vs sampled agreement | < 2% | < 0.05% over 31.25-1000 Hz | MEASURED |
| critical kp, ridden | 130-150 A/rad | 140.5 A/rad (200 in use) | MEASURED |
| simulated delay boundary, 500 Hz | > 16 ms | survives 32 ms, strikes at 48 ms | MEASURED |
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np
import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.impulse_response import (  # noqa: E402
    NOMINAL_IMPULSE_NS,
    ImpulseParams,
    frame_pitch_rad,
    run,
)
from sim.scenarios.loop_rate import (  # noqa: E402
    SWEEP_RATES_HZ,
    Gains,
    delay_margin,
    expm,
    integrated_expm,
    linearise,
    sampled_delay_budget,
    simulate_point,
)
from sim.scenarios.plant import build_model  # noqa: E402
from sim.scenarios.rust_controller import RustController  # noqa: E402

BALLAST_KG, BALLAST_M = 70.0, 0.75
RIDDEN_GAINS = dict(kp_a_per_rad=200.0, kd_a_per_rad_s=30.0, max_current_a=40.0,
                    kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True)
CURRENT_LOOP_TAU_S = 0.001

#: AC-6's ceiling, from `docs/design-pi-image-stage0b.md`. Reused verbatim, not
#: re-picked -- the point of these tests is to say what fraction of the real
#: budget it is.
AC6_MAX_S = 0.002


@pytest.fixture(scope="module")
def ridden():
    return build_model(BALLAST_KG, BALLAST_M, 40.0, "figure")


@pytest.fixture(scope="module")
def driverless():
    return build_model(0.0, 0.0, 40.0, "figure")


@pytest.fixture(scope="module")
def rp(ridden):
    return linearise(ridden)


@pytest.fixture(scope="module")
def dp(driverless):
    return linearise(driverless)


# ---------------------------------------------------------------------------
# The numerics this module had to bring its own of
# ---------------------------------------------------------------------------
def test_expm_reproduces_a_known_answer():
    """A rotation generator: exp([[0,-1],[1,0]]*t) is the rotation by t."""
    t = 1.3
    got = expm(np.array([[0.0, -1.0], [1.0, 0.0]]) * t)
    want = np.array([[math.cos(t), -math.sin(t)], [math.sin(t), math.cos(t)]])
    assert np.allclose(got, want, atol=1e-12)


def test_expm_survives_a_stiff_matrix():
    """Scaling-and-squaring is there for this case; without it the Taylor
    series diverges long before it converges."""
    a = np.diag([-1000.0, -1.0, 0.0])
    assert np.allclose(expm(a * 0.01), np.diag([math.exp(-10.0), math.exp(-0.01), 1.0]),
                       atol=1e-12)


def test_integrated_expm_matches_the_scalar_closed_form():
    """int_0^h e^{as} ds = (e^{ah} - 1)/a."""
    a, h = -3.0, 0.25
    assert integrated_expm(np.array([[a]]), h)[0, 0] == pytest.approx(
        (math.exp(a * h) - 1.0) / a, abs=1e-12
    )


# ---------------------------------------------------------------------------
# The linearisation, checked against the model it claims to describe
# ---------------------------------------------------------------------------
def _mj_free(model, theta0, seconds):
    import mujoco

    data = mujoco.MjData(model)
    data.qpos[3:7] = [math.cos(theta0 / 2), 0.0, math.sin(theta0 / 2), 0.0]
    mujoco.mj_forward(model, data)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    ts, th = [], []
    for _ in range(int(seconds / model.opt.timestep)):
        mujoco.mj_step(model, data)
        ts.append(float(data.time))
        th.append(frame_pitch_rad(model, data))
    return np.array(ts), np.array(th)


def _mj_torque_step(model, tau, seconds):
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    ts, th = [], []
    for _ in range(int(seconds / model.opt.timestep)):
        data.ctrl[0] = tau
        mujoco.mj_step(model, data)
        ts.append(float(data.time))
        th.append(frame_pitch_rad(model, data))
    return np.array(ts), np.array(th)


def _lin(plant, theta0, tau, seconds, dt=2e-4):
    s = np.array([0.0, theta0, 0.0, 0.0])
    ts, th = [], []
    for k in range(int(seconds / dt)):
        s = s + dt * (plant.A @ s + (plant.B * tau).ravel())
        ts.append((k + 1) * dt)
        th.append(s[1])
    return np.array(ts), np.array(th)


@pytest.mark.parametrize("t_check", [0.1, 0.2, 0.4, 0.6])
def test_the_linear_model_reproduces_the_models_own_divergence(ridden, rp, t_check):
    """Released from 0.5 deg, the linear model must follow MuJoCo while the
    small-angle assumption holds. 0.6 s is 5.9 deg -- a third of the way to the
    nose strike, and the furthest a linearisation has any right to be believed.
    """
    theta0 = math.radians(0.5)
    tm, thm = _mj_free(ridden, theta0, 0.7)
    tl, thl = _lin(rp, theta0, 0.0, 0.7)
    i, j = int(np.argmin(abs(tm - t_check))), int(np.argmin(abs(tl - t_check)))
    assert thl[j] == pytest.approx(thm[i], rel=0.02), (
        f"at t={t_check}s linear {math.degrees(thl[j]):.4f} deg vs MuJoCo "
        f"{math.degrees(thm[i]):.4f} deg"
    )


@pytest.mark.parametrize("t_check", [0.05, 0.1, 0.2, 0.3])
def test_the_linear_model_reproduces_a_torque_step(ridden, rp, t_check):
    """**This is the test that pins the actuator reaction's sign.**

    Driving the wheel forward accelerates the axle out from under the centre of
    mass (nose up) AND torques the body through the hinge. Those two ADD. An
    earlier derivation had them opposing, giving a torque leverage of 3.21
    instead of 5.21, and this comparison came out low by a factor of 1.62 --
    which is 5.21/3.21 exactly. Nothing else in the module would have caught it.
    """
    tm, thm = _mj_torque_step(ridden, 5.0, 0.35)
    tl, thl = _lin(rp, 0.0, 5.0, 0.35)
    i, j = int(np.argmin(abs(tm - t_check))), int(np.argmin(abs(tl - t_check)))
    assert thl[j] == pytest.approx(thm[i], rel=0.03), (
        f"at t={t_check}s linear {math.degrees(thl[j]):.4f} deg vs MuJoCo "
        f"{math.degrees(thm[i]):.4f} deg"
    )


def test_the_torque_leverage_adds_rather_than_opposes(rp):
    """The same finding as a scalar, so a reader does not have to run a
    trajectory to see it. Base motion 4.21, hinge reaction 1.0, total 5.21."""
    base_motion = (rp.m_body_kg * rp.com_above_axle_m
                   / (rp.translating_mass * rp.wheel_radius_m))
    assert base_motion == pytest.approx(4.213, rel=1e-3)
    assert rp.torque_leverage == pytest.approx(base_motion + 1.0, rel=1e-9)


# ---------------------------------------------------------------------------
# The unstable pole
# ---------------------------------------------------------------------------
def test_the_ridden_board_is_an_inverted_pendulum_and_this_is_how_fast(rp):
    assert rp.mgl > 0.0
    assert rp.unstable_pole == pytest.approx(5.24, abs=0.25)
    assert rp.doubling_time_s * 1e3 == pytest.approx(132.0, abs=8.0)


def test_the_wheel_makes_the_plant_diverge_faster_than_a_pendulum_would(rp):
    """The correction that matters. A free axle removes `(m*l)^2 / M_t` from
    the effective pitch inertia, so this vehicle is 33% faster than the
    point-mass approximation and 64% faster than a pinned pivot."""
    assert rp.point_mass_pole() == pytest.approx(3.935, abs=0.05), (
        "the point-mass figure issue #113 was filed with; kept reproducible so "
        "the comparison stays checkable"
    )
    assert rp.fixed_pivot_pole() < rp.point_mass_pole() < rp.unstable_pole


def test_the_driverless_board_is_not_an_inverted_pendulum_at_all(dp):
    """Its centre of mass sits BELOW the axle, so gravity restores. Any
    'unstable pole' for it would be a modelling error, not a plant."""
    assert dp.mgl < 0.0
    assert dp.unstable_pole == pytest.approx(0.0, abs=1e-6)


def test_pitch_feedback_alone_is_only_just_enough_on_the_ridden_board(rp):
    """140 A/rad is where pitch feedback stops the divergence on its own; the
    cascade runs 200. Below 140 the outer velocity loop is not a refinement,
    it is the only thing holding the board up."""
    assert rp.critical_kp_a_per_rad() == pytest.approx(140.5, abs=3.0)
    assert RIDDEN_GAINS["kp_a_per_rad"] > rp.critical_kp_a_per_rad()


# ---------------------------------------------------------------------------
# The delay budget — the number AC-6 should actually be judged against
# ---------------------------------------------------------------------------
def test_the_loop_tolerates_tens_of_milliseconds_not_units_of_them(rp, dp):
    ridden = delay_margin(rp, Gains(200.0, 30.0, 0.05, 0.02, True))
    drv = delay_margin(dp, Gains(80.0, 11.0))
    assert ridden.delay_margin_s * 1e3 == pytest.approx(71.4, abs=3.0)
    assert drv.delay_margin_s * 1e3 == pytest.approx(65.6, abs=3.0)
    assert min(ridden.delay_margin_s, drv.delay_margin_s) > 20 * AC6_MAX_S, (
        "AC-6's 2 ms ceiling should be a small fraction of the budget; if this "
        "fails the threshold has stopped being conservative"
    )


def test_a_stated_phase_margin_still_leaves_room_for_ac6(dp):
    """AC-2 of issue #113 asks for the budget AT a stated phase margin, not
    just at the edge. On the loop that has margin to state, 30 deg leaves
    37 ms -- still 18x AC-6's ceiling."""
    m = delay_margin(dp, Gains(80.0, 11.0))
    assert m.delay_at_phase_margin_s(30.0) * 1e3 == pytest.approx(37.4, abs=2.0)
    assert m.delay_at_phase_margin_s(30.0) > 18 * AC6_MAX_S


def test_the_ridden_cascades_own_phase_margin_is_the_thing_that_is_thin(rp):
    """Reported rather than fixed. The cascade crosses over at 3.4 rad/s with
    13.8 deg of phase margin, so it cannot be quoted at a 30 deg criterion at
    any delay -- `delay_at_phase_margin_s(30)` is negative on purpose. That is
    a tuning observation for a separate increment, not a rate finding, and
    retuning it here would be scope creep dressed as a fix."""
    m = delay_margin(rp, Gains(200.0, 30.0, 0.05, 0.02, True))
    assert m.phase_margin_deg == pytest.approx(13.8, abs=2.0)
    assert m.delay_at_phase_margin_s(30.0) < 0.0


@pytest.mark.parametrize("rate_hz", SWEEP_RATES_HZ)
def test_the_sample_rate_costs_exactly_half_a_period_of_the_budget(rp, rate_hz):
    """**The headline.** Two independent derivations -- a continuous phase
    margin and an exact discrete ZOH-with-delay eigenvalue sweep -- must agree
    once `Ts/2` is accounted for. That they do over five octaves is what says
    the rate enters the answer through exactly one term, and it is a small one.
    """
    gains = Gains(200.0, 30.0, 0.05, 0.02, True)
    sampled = sampled_delay_budget(rp, gains, rate_hz, CURRENT_LOOP_TAU_S)
    total = sampled + 0.5 / rate_hz
    continuous = delay_margin(rp, gains).delay_margin_s - CURRENT_LOOP_TAU_S
    assert total == pytest.approx(continuous, rel=0.02), (
        f"at {rate_hz} Hz the sampled budget plus Ts/2 is {total*1e3:.2f} ms "
        f"but the continuous margin is {continuous*1e3:.2f} ms"
    )


def test_going_from_500_hz_to_1000_hz_buys_half_a_millisecond(rp):
    """Stated as the trade it is. Doubling the rate returns 0.5 ms of a 70 ms
    budget -- 0.7%. Halving it costs 1 ms. Neither is a decision."""
    gains = Gains(200.0, 30.0, 0.05, 0.02, True)
    at_500 = sampled_delay_budget(rp, gains, 500.0, CURRENT_LOOP_TAU_S)
    at_1000 = sampled_delay_budget(rp, gains, 1000.0, CURRENT_LOOP_TAU_S)
    assert (at_1000 - at_500) * 1e3 == pytest.approx(0.5, abs=0.1)


# ---------------------------------------------------------------------------
# The scenario knobs the sweep needs, and their no-op guarantee
# ---------------------------------------------------------------------------
def test_an_explicit_control_period_of_one_timestep_changes_nothing(ridden):
    """`control_period_s=None` and `control_period_s=timestep` must be the same
    run to the bit. If they ever are not, every pinned metric in this repo has
    silently moved and the rate sweep is measuring the change rather than the
    rate."""
    base = ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=1.5)
    explicit = ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=1.5,
                             control_period_s=float(ridden.opt.timestep))
    with RustController(**RIDDEN_GAINS) as a:
        first = run(base, model=ridden, controller=a)
    with RustController(**RIDDEN_GAINS) as b:
        second = run(explicit, model=ridden, controller=b)
    assert np.array_equal(first.pitch_deg, second.pitch_deg)
    assert first.metrics.peak_abs_pitch_deg == second.metrics.peak_abs_pitch_deg


def test_zero_jitter_never_touches_the_random_stream(ridden):
    """Two different seeds must produce identical runs when jitter is off --
    which is only true if the generator is never drawn from. A jitter model
    that consumed randomness unconditionally would make every existing gate's
    baseline depend on a field nobody set."""
    kw = dict(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=1.5,
              control_period_s=0.002)
    with RustController(**RIDDEN_GAINS) as a:
        first = run(ImpulseParams(**kw, control_seed=1), model=ridden, controller=a)
    with RustController(**RIDDEN_GAINS) as b:
        second = run(ImpulseParams(**kw, control_seed=2), model=ridden, controller=b)
    assert np.array_equal(first.pitch_deg, second.pitch_deg)


def test_a_jittered_run_is_still_bit_identical_on_a_repeat(ridden):
    kw = dict(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=1.5,
              control_period_s=0.002, control_jitter_s=0.004)
    with RustController(**RIDDEN_GAINS) as a:
        first = run(ImpulseParams(**kw), model=ridden, controller=a)
    with RustController(**RIDDEN_GAINS) as b:
        second = run(ImpulseParams(**kw), model=ridden, controller=b)
    assert np.array_equal(first.pitch_deg, second.pitch_deg)


def test_the_physics_timestep_override_is_restored(ridden):
    """A shared model object must not be left mutated -- every other test in
    this repo would then be running different physics depending on order."""
    before = float(ridden.opt.timestep)
    run(ImpulseParams(sim_seconds=0.2, physics_timestep_s=0.0005), model=ridden)
    assert float(ridden.opt.timestep) == before


# ---------------------------------------------------------------------------
# The simulated boundary, small enough to sit in the required gate
# ---------------------------------------------------------------------------
def test_the_simulated_delay_boundary_is_far_beyond_ac6s_ceiling(ridden):
    """The full grid lives in `scripts/analyse_loop_rate.py`; this is the two
    points that matter. 32 ms of transport delay is survivable and 48 ms is
    not, so AC-6's 2 ms ceiling has more than an order of magnitude in hand."""
    survives = simulate_point(ridden, RIDDEN_GAINS, rate_hz=500.0,
                              delay_s=0.032, sim_seconds=4.0, phases=1)
    fails = simulate_point(ridden, RIDDEN_GAINS, rate_hz=500.0,
                           delay_s=0.048, sim_seconds=4.0, phases=1)
    assert not survives.struck
    assert survives.peak_abs_pitch_deg < 13.0
    assert fails.struck


def test_the_delay_boundary_is_reached_by_running_out_of_current_not_phase(ridden):
    """Why the simulated boundary (~40 ms) is tighter than the linear one
    (~71 ms): the cascade saturates the 40 A envelope before it runs out of
    phase margin. The linear model has no current limit and cannot say this.
    """
    easy = simulate_point(ridden, RIDDEN_GAINS, rate_hz=500.0, delay_s=0.024,
                          sim_seconds=4.0, phases=1)
    hard = simulate_point(ridden, RIDDEN_GAINS, rate_hz=500.0, delay_s=0.040,
                          sim_seconds=4.0, phases=1)
    assert easy.saturated_cycles == 0
    assert hard.saturated_cycles > 0
    assert not hard.struck


def test_quartering_the_rate_is_worth_less_than_a_degree_of_peak_pitch(ridden):
    """125 Hz against 500 Hz at the same 1 ms of transport delay. If the rate
    were the binding constraint this is where it would show, and it does not.
    """
    slow = simulate_point(ridden, RIDDEN_GAINS, rate_hz=125.0, delay_s=0.001,
                          sim_seconds=4.0, phases=1)
    fast = simulate_point(ridden, RIDDEN_GAINS, rate_hz=500.0, delay_s=0.001,
                          sim_seconds=4.0, phases=1)
    assert not slow.struck and not fast.struck
    assert slow.peak_abs_pitch_deg - fast.peak_abs_pitch_deg < 1.0


def test_raising_the_rate_does_not_amplify_sensor_noise(ridden):
    """The classic objection to a high rate is that derivative action amplifies
    noise. It does not apply here: the D term consumes a MEASURED gyro rate,
    not a difference of two pitch samples, so its noise is the gyro's and is
    independent of the rate. Asserted at 10x the ICD §12 placeholder, where
    the effect would be visible if it existed."""
    slow = simulate_point(ridden, RIDDEN_GAINS, rate_hz=125.0, delay_s=0.001,
                          noise_scale=10.0, sim_seconds=4.0, phases=1)
    fast = simulate_point(ridden, RIDDEN_GAINS, rate_hz=500.0, delay_s=0.001,
                          noise_scale=10.0, sim_seconds=4.0, phases=1)
    assert fast.peak_abs_pitch_deg <= slow.peak_abs_pitch_deg
    assert fast.pitch_est_rms_deg <= slow.pitch_est_rms_deg * 1.05
