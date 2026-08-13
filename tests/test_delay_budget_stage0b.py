"""Issue #113 -- replace AC-6's plant-ignorant threshold with a delay budget.

The old story was "the SPI tail might eat a whole 2 ms control period, so a
1.5-2 ms spike is disqualifying." That treats the loop rate as the thing that
matters. It is not: 500 Hz is independently justified by IMU anti-aliasing
(`crates/board-types/src/lib.rs`, `crates/control-core/src/lib.rs`), and the
real question is how much TOTAL actuation delay the ridden/cascade closed
loop -- the vehicle being built -- can absorb before it noses in.

This file pins the two numbers `scripts/analyse_control.py`'s `delay_budget()`
found by bisection, so they cannot silently drift: the closed loop (real Rust
`PitchRegulator` + `VelocityLoop`, current-clamped, nominal impulse) survives
51 ms of pure actuation delay and strikes at 52 ms -- with the estimator in
the loop, which is the honest default. With truth pitch instead of the
estimate, the same plant survives to 59 ms. **The estimator alone costs about
9 ms of the closed loop's delay margin** -- the MANDATORY, measured line item
issue #113 asked for, not an assumed one.

**Correction (#133): re-measured at the PRODUCTION estimator time constant.**
Every number in this file previously used `estimator_tau_s`'s FFI default of
1.0 s, not the `tau = 2 s` `sim/scenarios/hill.py:140` documents as the
recommended production config -- nothing had ever asked the estimator for the
config that actually ships. The direction of the correction is the opposite
of what was assumed: a longer tau trusts the (delay-free) gyro more and the
(phase-corrupted) accelerometer less -- see `WheelAccelEstimator`'s comment in
`crates/control-core/src/lib.rs` for why that branch is expensive -- so it
COSTS LESS margin, not more. At tau=1.0 s the estimator cost ~21 ms; at the
production tau=2.0 s it costs ~9 ms, consistently, across every disturbance
amplitude swept in `scripts/analyse_delay_budget.py`. Still a real cost, and
still the dominant named line item next to the ~1.5-2 ms SPI tail -- just a
smaller one than previously documented.

See docs/design-delay-budget-stage0b.md for the full derivation and the
delay-budget table this backs.

GATE NUMBERS -- MEASURED (this issue, via `scripts/analyse_control.py`,
estimator_tau_s=2.0)
------------------------------------------------------------------------
| Assertion                                                | Threshold                          | Observed |
|-----------------------------------------------------------|-------------------------------------|----------|
| `test_the_ridden_closed_loop_survives_at_51ms`             | `not nose_strike`                   | peak 13.05 deg (< 18.57 strike angle) |
| `test_the_ridden_closed_loop_strikes_at_52ms`               | `nose_strike`                        | peak 34.06 deg |
| `test_truth_pitch_survives_to_59ms`                         | `not nose_strike`                   | peak 12.70 deg |
| `test_truth_pitch_strikes_at_60ms`                          | `nose_strike`                        | peak 74.21 deg |
| `test_the_unstable_pole_is_the_expected_order_of_magnitude`  | `2.5 < p < 4.0 rad/s`                | 3.191 rad/s |
"""

import mujoco

from sim.scenarios.imperfections import STAGE0_PLACEHOLDER, ImperfectionProfile
from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS, ImpulseParams, run
from sim.scenarios.plant import build_model, plant_summary
from sim.scenarios.rust_controller import RustController

# 200 A/rad, 30 A/(rad/s) at kt = 0.7 N*m/A, re-denominated in torque (#137).
CASCADE = dict(kp_nm_per_rad=140.0, kd_nm_per_rad_s=21.0, max_current_a=40.0,
               com_above_axle=True, kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02)

#: The recommended production config (`sim/scenarios/hill.py:140`), not the
#: FFI's neutral default of 1.0 -- see the module docstring's #133 correction.
ESTIMATOR_TAU_S = 2.0


def _delayed_run(delay_ms, use_estimator):
    model = build_model(70.0, 0.75, 40.0)
    profile = ImperfectionProfile(
        f"delay-{delay_ms}ms-test", actuation_delay_s=delay_ms / 1000.0,
        current_loop_tau_s=STAGE0_PLACEHOLDER.current_loop_tau_s,
        gyro_noise_rad_s=STAGE0_PLACEHOLDER.gyro_noise_rad_s,
        gyro_bias_rad_s=STAGE0_PLACEHOLDER.gyro_bias_rad_s,
        accel_noise_m_s2=STAGE0_PLACEHOLDER.accel_noise_m_s2,
        wheel_rate_quantum_rad_s=STAGE0_PLACEHOLDER.wheel_rate_quantum_rad_s,
        wheel_rate_update_hz=STAGE0_PLACEHOLDER.wheel_rate_update_hz,
    )
    with RustController(use_estimator=use_estimator, estimator_tau_s=ESTIMATOR_TAU_S,
                        **CASCADE) as c:
        return run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=8.0),
                   model=model, controller=c, profile=profile)


def test_the_ridden_closed_loop_survives_at_51ms():
    """Last survivor, WITH the estimator -- the honest default (issue #24 AC1)."""
    r = _delayed_run(51, use_estimator=True)
    assert not r.metrics.nose_strike
    assert r.metrics.peak_abs_pitch_deg < 18.57


def test_the_ridden_closed_loop_strikes_at_52ms():
    """One millisecond more and it goes over -- the measured boundary AC-6
    should gate on, not an arbitrary fraction of the 2 ms control period."""
    r = _delayed_run(52, use_estimator=True)
    assert r.metrics.nose_strike


def test_repeat_runs_at_the_boundary_are_bit_identical():
    """The boundary itself must not be a coin flip -- same convention as the
    determinism audit in issue #24 AC4."""
    first = _delayed_run(52, use_estimator=True)
    second = _delayed_run(52, use_estimator=True)
    assert first.to_json_dict()["metrics"] == second.to_json_dict()["metrics"]


def test_truth_pitch_survives_to_59ms():
    """Same plant, same gains, ground-truth pitch instead of the estimate --
    isolates what the estimator itself costs the delay budget."""
    r = _delayed_run(59, use_estimator=False)
    assert not r.metrics.nose_strike


def test_truth_pitch_strikes_at_60ms():
    r = _delayed_run(60, use_estimator=False)
    assert r.metrics.nose_strike


def test_the_estimator_costs_a_large_fraction_of_the_delay_budget():
    """The mandatory, measured line item: the estimator is not a rounding
    error next to the SPI tail. Bounded rather than pinned to the exact
    bisected ms so a one-cycle (2 ms) shift in either boundary from an
    unrelated change does not flake this gate.

    Threshold lowered from 1.4x (#113) to 1.15x as part of #133's tau
    correction -- measured ~1.21x at the production estimator config, stable
    to three digits across delay=40..51 ms, well clear of 1.15."""
    with_est = _delayed_run(51, use_estimator=True)
    without_est = _delayed_run(51, use_estimator=False)
    assert not without_est.metrics.nose_strike, (
        "51 ms must still survive on truth pitch, or the comparison below is vacuous"
    )
    assert with_est.metrics.peak_abs_pitch_deg > 1.15 * without_est.metrics.peak_abs_pitch_deg, (
        "expected the estimator to cost real margin at the same delay, not a rounding error"
    )


def test_the_unstable_pole_is_the_expected_order_of_magnitude():
    """Reads the compiled RIDDEN model directly (mass, geometry, inertia) --
    same discipline as `test_r_eff_matches_model.py`: a derived number is
    re-computed from the model, not hand-copied, so it cannot silently drift.

    Bound is deliberately loose (2.5-4.0 rad/s): this is an order-of-magnitude
    tipping-mode estimate (ignores wheel-rolling/translation coupling), not a
    tuned target -- see docs/design-delay-budget-stage0b.md.
    """
    model = build_model(70.0, 0.75, 40.0)
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    axle = data.xpos[mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")].copy()

    summary = plant_summary(model)
    i_total = 0.0
    for i in range(model.nbody):
        mass = float(model.body_mass[i])
        if mass <= 0:
            continue
        iyy_own = float(model.body_inertia[i][1])
        dx = float(data.xipos[i][0] - axle[0])
        dz = float(data.xipos[i][2] - axle[2])
        i_total += iyy_own + mass * (dx**2 + dz**2)

    pole_rad_s = (summary["mgl_n_m_per_rad"] / i_total) ** 0.5
    assert 2.5 < pole_rad_s < 4.0, f"unstable pole {pole_rad_s:.3f} rad/s out of expected range"
