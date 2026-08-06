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
38 ms of pure actuation delay and strikes at 39 ms -- with the estimator in
the loop, which is the honest default. With truth pitch instead of the
estimate, the same plant survives to 59 ms. **The estimator alone costs about
21 ms of the closed loop's delay margin** -- the MANDATORY, measured line item
issue #113 asked for, not an assumed one.

**RE-MEASURED against the frozen plant (issue: realistic-motor-torque; drag:
the derived three-term model). The boundary MOVED FAR OUT and, more
importantly, the estimator's effect on it REVERSED SIGN.** With the
estimator, the closed loop now survives to 151 ms and strikes at 152 ms
(was 38/39 ms). On truth pitch it now survives to 147 ms and strikes at
148 ms (was 59/60 ms). **The estimator now costs less delay margin than
truth pitch, not more** -- it survives 4 ms LONGER, the opposite of the
finding this file was written to pin. At the original 38 ms comparison
point the OLD direction still holds, just far more weakly (peak pitch with
the estimator is 1.11x truth's, not 1.4x+); at delays approaching either new
boundary the direction flips (e.g. at 145 ms, with-estimator peak is 0.93x
truth's). This is recorded as a finding, not smoothed over by only updating
the boundary numbers -- see the annotated tests below and
`test_the_estimator_costs_a_large_fraction_of_the_delay_budget`'s docstring
in particular.

See docs/design-delay-budget-stage0b.md for the full derivation and the
delay-budget table this backs (not re-derived this session).

GATE NUMBERS -- MEASURED (this issue, via `scripts/analyse_control.py`)
------------------------------------------------------------------------
| Assertion                                                | Threshold                          | Observed |
|-----------------------------------------------------------|-------------------------------------|----------|
| `test_the_ridden_closed_loop_survives_at_38ms`             | `not nose_strike`                   | peak 18.23 deg (< 18.57 strike angle) -- NOT re-measured this session, still passes far inside the new 151 ms boundary |
| `test_the_ridden_closed_loop_strikes_at_39ms`               | `nose_strike`                        | RE-MEASURED: was 39 ms / peak 19.88 deg, now measures the boundary at 152 ms / peak 179.99 deg (see file docstring for why) |
| `test_truth_pitch_survives_to_59ms`                         | `not nose_strike`                   | peak 12.70 deg -- NOT re-measured this session, still passes far inside the new 147 ms boundary |
| `test_truth_pitch_strikes_at_60ms`                          | `nose_strike`                        | RE-MEASURED: was 60 ms / peak 74.21 deg, now measures the boundary at 148 ms / peak 179.94 deg |
| `test_the_unstable_pole_is_the_expected_order_of_magnitude`  | `2.5 < p < 4.0 rad/s`                | 3.191 rad/s -- NOT re-measured this session (geometry-only, not current/drag dependent) |
"""

import mujoco

from sim.scenarios.imperfections import STAGE0_PLACEHOLDER, ImperfectionProfile
from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS, ImpulseParams, run
from sim.scenarios.plant import build_model, plant_summary
from sim.scenarios.rust_controller import RustController

# 200 A/rad, 30 A/(rad/s) at kt = 0.7 N*m/A, re-denominated in torque (#137).
CASCADE = dict(kp_nm_per_rad=140.0, kd_nm_per_rad_s=40.0, max_current_a=40.0,
               com_above_axle=True, kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02)


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
    with RustController(use_estimator=use_estimator, **CASCADE) as c:
        return run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=8.0),
                   model=model, controller=c, profile=profile)


def test_the_ridden_closed_loop_survives_at_38ms():
    """Survivor, WITH the estimator -- the honest default (issue #24 AC1).

    No longer the LAST survivor: the boundary re-measured this session (see
    file docstring and `test_the_ridden_closed_loop_strikes_at_39ms`) moved
    out to 151/152 ms. Kept at 38 ms anyway as a stable, cheap regression
    check well inside the new margin; the name is legacy and not re-derived
    against the new boundary, since the assertion itself does not depend on
    being close to it.
    """
    r = _delayed_run(38, use_estimator=True)
    assert not r.metrics.nose_strike
    assert r.metrics.peak_abs_pitch_deg < 18.57


def test_the_ridden_closed_loop_strikes_at_39ms():
    """One millisecond more and it goes over -- the measured boundary AC-6
    should gate on, not an arbitrary fraction of the 2 ms control period.

    RE-MEASURED against the frozen plant (issue: realistic-motor-torque; drag:
    the derived three-term model): was 39 ms / peak 19.88 deg at 40 A /
    28 N*m. The boundary moved to 152 ms / peak 179.99 deg -- survives to
    151 ms (peak 14.44 deg). Read the file docstring before trusting this
    test's own name: the function name is legacy (kept so the re-pin is a
    diff, not a rename) and the delay it now runs is 152 ms, not 39.
    """
    r = _delayed_run(152, use_estimator=True)
    assert r.metrics.nose_strike


def test_repeat_runs_at_the_boundary_are_bit_identical():
    """The boundary itself must not be a coin flip -- same convention as the
    determinism audit in issue #24 AC4.

    Delay updated to 152 ms to track the re-measured boundary (was 39 ms; see
    `test_the_ridden_closed_loop_strikes_at_39ms`) -- this is a determinism
    check, not a threshold, so it is kept pointed at whatever the actual
    boundary is rather than left at a now-arbitrary value.
    """
    first = _delayed_run(152, use_estimator=True)
    second = _delayed_run(152, use_estimator=True)
    assert first.to_json_dict()["metrics"] == second.to_json_dict()["metrics"]


def test_truth_pitch_survives_to_59ms():
    """Same plant, same gains, ground-truth pitch instead of the estimate --
    isolates what the estimator itself costs the delay budget.

    No longer close to the boundary: re-measured this session (see file
    docstring and `test_truth_pitch_strikes_at_60ms`) at 147/148 ms. Kept at
    59 ms as a stable regression check well inside the new margin; the name
    is legacy for the same reason `test_the_ridden_closed_loop_survives_at_
    38ms`'s is.
    """
    r = _delayed_run(59, use_estimator=False)
    assert not r.metrics.nose_strike


def test_truth_pitch_strikes_at_60ms():
    """RE-MEASURED against the frozen plant (issue: realistic-motor-torque;
    drag: the derived three-term model): was 60 ms / peak 74.21 deg at 40 A /
    28 N*m. The boundary moved to 148 ms / peak 179.94 deg -- survives to
    147 ms (peak 12.46 deg). Function name is legacy; see file docstring.
    """
    r = _delayed_run(148, use_estimator=False)
    assert r.metrics.nose_strike


def test_the_estimator_costs_a_large_fraction_of_the_delay_budget():
    """The mandatory, measured line item: the estimator is not a rounding
    error next to the SPI tail, it is the dominant term. Bounded rather than
    pinned to the exact bisected ms so a one-cycle (2 ms) shift in either
    boundary from an unrelated change does not flake this gate.

    **RE-MEASURED against the frozen plant, and read this before trusting
    the docstring above.** At 38 ms the OLD direction still holds (estimator
    costs pitch, not free) but far more weakly: peak ratio is now 1.109, not
    the >1.4 originally measured -- re-pinned to `> 1.05`, just under the
    measured value. **The bigger finding is that this direction does not
    hold near either boundary any more.** The re-measured boundaries
    (151/152 ms with the estimator, 147/148 ms on truth pitch -- see
    `test_the_ridden_closed_loop_strikes_at_39ms` and `test_truth_pitch_
    strikes_at_60ms`) show the estimator surviving 4 ms LONGER than truth
    pitch, the reverse of "the estimator costs delay margin". Sampled at
    100-147 ms the ratio is consistently below 1 (0.90-0.97): the estimator's
    nose-up trim (ADR-0011's "correcting the estimator makes the board
    worse") is now generically HELPING the delay budget at operating points
    close to where either boundary actually is. This test keeps measuring
    the fixed 38 ms comparison point rather than the boundary because that
    comparison point is still simply defined and reproducible; it is NOT
    representative of the delay budget's dominant term at this envelope any
    more, and this docstring is the record of that rather than a quiet
    re-pin.
    """
    with_est = _delayed_run(38, use_estimator=True)
    without_est = _delayed_run(38, use_estimator=False)
    assert not without_est.metrics.nose_strike, (
        "38 ms must still survive on truth pitch, or the comparison below is vacuous"
    )
    assert with_est.metrics.peak_abs_pitch_deg > 1.05 * without_est.metrics.peak_abs_pitch_deg, (
        "expected the estimator to cost some margin at this fixed delay, not a rounding error"
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
