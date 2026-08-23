"""Issue #132, revised order step 3 -- `scripts/gain_margin_sweep.py`.

Checks the feasibility-map machinery itself: the reduced-model calibration is
faithful to the full model, the map is deterministic and internally
consistent (gain margin rises with `kp`, the RHP threshold does not depend on
`kd`), and the shipped gains land close to #132's own reported 2.9 dB --
"close", not exact, because the plant has moved since that comment was
written (see the module docstring's note on `p`), and this test pins that
tolerance rather than the stale exact figure.
"""

import math

import numpy as np

from scripts.analyse_delay_budget import (
    BALLAST_KG, BALLAST_M, CLAMP_A, analytic_pole, linearize,
    pitch_output_map, settle_upright,
)
from scripts.gain_margin_sweep import (
    RESONANCE_CEILING_RAD_S, SHIPPED_KD_NM_PER_RAD_S, SHIPPED_KP_NM_PER_RAD,
    evaluate, fit_reduced_gain_constant, loop_transfer, recommended_band,
    select_feasible, sweep,
)
from sim.scenarios.plant import build_model


def _linearize_ridden():
    model = build_model(BALLAST_KG, BALLAST_M, CLAMP_A)
    dt = float(model.opt.timestep)
    data = settle_upright(model)
    A, B = linearize(model, data)
    Cp = pitch_output_map(model, data)
    p = analytic_pole(model)["p_rolling_support_rad_s"]
    omega = np.logspace(-1, math.log10(math.pi / dt), 4000)
    return A, B, Cp, p, omega


def test_shipped_gains_downward_margin_is_close_to_the_issues_own_figure():
    """#132's comment measured 2.9 dB at (kp=200 A/rad, kd=30 A/(rad/s),
    kt=0.7) -- 140/21 in today's torque-native units. The model has since
    picked up mechanical changes (this repo's `p` here is 5.56 rad/s, not the
    comment's 5.24), so this is a tolerance check, not a pin: it catches a
    method error (wrong sign, wrong units, wrong crossover) without flaking
    on every future mass/inertia tweak Sr. Mechanical & Systems makes."""
    A, B, Cp, p, omega = _linearize_ridden()
    b = fit_reduced_gain_constant(A, B, Cp, omega, p,
                                   SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)
    row = evaluate(A, B, Cp, omega, p, b, SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)

    assert row["stable"]
    assert 2.0 < row["downward_gain_margin_db"] < 3.5, (
        f"expected close to #132's measured 2.9 dB, got {row['downward_gain_margin_db']:.2f} dB"
    )
    # Same cross-check as analyse_delay_budget.py's own history: crossover
    # near 4.5 rad/s, phase margin in the low-to-mid 30s.
    assert 4.0 < row["omega_c_rad_s"] < 5.0
    assert 25.0 < row["phase_margin_deg"] < 45.0


def test_reduced_model_agrees_with_the_full_model_up_to_crossover():
    """The `b`-fit is only valid if the 2-state reduction actually tracks the
    full 14-state model over the range this script reads it at. Checked at
    five points from the omega floor to the shipped-gains crossover -- if a
    future change (e.g. a much stiffer tyre model) breaks this agreement, the
    gain-margin numbers this script reports would quietly stop meaning
    anything, and this is the test that would catch it."""
    A, B, Cp, p, omega = _linearize_ridden()
    b = fit_reduced_gain_constant(A, B, Cp, omega, p,
                                   SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)

    for w in (0.1, 0.5, 1.0, 2.0, 4.0):
        full = abs(loop_transfer(A, B, Cp, np.array([w]),
                                  SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)[0])
        reduced = b * math.sqrt(SHIPPED_KP_NM_PER_RAD ** 2
                                 + (SHIPPED_KD_NM_PER_RAD_S * w) ** 2) / (w * w + p * p)
        rel_err = abs(full - reduced) / full
        assert rel_err < 0.10, f"at w={w}: full={full:.4f} reduced={reduced:.4f} ({rel_err:.1%} off)"


def test_downward_gain_margin_rises_monotonically_with_kp():
    """`kp_min` is a fixed threshold (RHP stability condition); margin above
    it must strictly increase with `kp` at fixed `kd` -- the whole point of
    calling it a margin."""
    A, B, Cp, p, omega = _linearize_ridden()
    b = fit_reduced_gain_constant(A, B, Cp, omega, p,
                                   SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)

    margins_db = [evaluate(A, B, Cp, omega, p, b, kp, SHIPPED_KD_NM_PER_RAD_S)["downward_gain_margin_db"]
                  for kp in (110.0, 140.0, 200.0, 300.0)]
    assert margins_db == sorted(margins_db)
    assert margins_db[0] < margins_db[-1]


def test_kp_min_does_not_depend_on_kd():
    """Routh-Hurwitz on the reduced model: the marginal-`kp` threshold comes
    only from the constant term of the closed-loop characteristic polynomial,
    which `kd` never enters. If a future refactor accidentally folds `kd`
    into `kp_min`, this is the test that notices."""
    A, B, Cp, p, omega = _linearize_ridden()
    b = fit_reduced_gain_constant(A, B, Cp, omega, p,
                                   SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)

    kp_min_a = evaluate(A, B, Cp, omega, p, b, 200.0, 10.0)["kp_min_nm_per_rad"]
    kp_min_b = evaluate(A, B, Cp, omega, p, b, 200.0, 60.0)["kp_min_nm_per_rad"]
    assert kp_min_a == kp_min_b


def test_below_threshold_gains_are_reported_unstable_not_extrapolated():
    """A `kp` at or below `kp_min` must not produce a (nonsensical, since
    `log10` of a non-positive number is undefined) gain-margin figure."""
    A, B, Cp, p, omega = _linearize_ridden()
    b = fit_reduced_gain_constant(A, B, Cp, omega, p,
                                   SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)
    kp_min = p * p / b

    row = evaluate(A, B, Cp, omega, p, b, 0.5 * kp_min, SHIPPED_KD_NM_PER_RAD_S)
    assert not row["stable"]
    assert row["downward_gain_margin_db"] is None
    assert row["omega_c_rad_s"] is None


def test_sweep_is_deterministic():
    """Same convention as issue #24 AC4's determinism audit: two runs of the
    same grid over the same linearization must be bit-identical."""
    A, B, Cp, p, omega = _linearize_ridden()
    b = fit_reduced_gain_constant(A, B, Cp, omega, p,
                                   SHIPPED_KP_NM_PER_RAD, SHIPPED_KD_NM_PER_RAD_S)
    small_kp = np.linspace(80.0, 200.0, 4)
    small_kd = np.linspace(10.0, 40.0, 4)

    first = sweep(A, B, Cp, omega, p, b, small_kp, small_kd)
    second = sweep(A, B, Cp, omega, p, b, small_kp, small_kd)
    assert first == second


def test_recommended_band_reports_when_empty_rather_than_silently_widening():
    """`recommended_band` is the testable half of the "does 1.5p still fit
    under the ~8 rad/s unvalidated-crossover ceiling" question -- checked on
    synthetic `p` values so this does not depend on the live model's current
    pole and stays meaningful if the plant changes again."""
    target, empty = recommended_band(p=4.0)  # 1.5*4.0 = 6.0, under the 8.0 ceiling
    assert not empty
    assert target == 6.0

    target, empty = recommended_band(p=6.0)  # 1.5*6.0 = 9.0, over the 8.0 ceiling
    assert empty
    assert target == 9.0

    # The boundary itself is inclusive-safe: exactly at the ceiling is not "empty".
    target, empty = recommended_band(p=RESONANCE_CEILING_RAD_S / 1.5)
    assert not empty


def test_select_feasible_excludes_points_outside_the_recommended_band():
    """A synthetic grid exercising `select_feasible`'s three gates
    independently: unstable, below the crossover floor, and above the
    resonance ceiling must all be excluded; only the point clearing all
    three, with positive phase margin, should survive."""
    rows = [
        {"stable": False, "omega_c_rad_s": 9.0, "phase_margin_deg": 30.0},
        {"stable": True, "omega_c_rad_s": 3.0, "phase_margin_deg": 30.0},
        {"stable": True, "omega_c_rad_s": 9.0, "phase_margin_deg": 30.0},
        {"stable": True, "omega_c_rad_s": 7.0, "phase_margin_deg": -5.0},
        {"stable": True, "omega_c_rad_s": 7.0, "phase_margin_deg": 12.0},
    ]
    feasible = select_feasible(rows, target_omega_c=6.0, ceiling=8.0)
    assert feasible == [rows[4]]
