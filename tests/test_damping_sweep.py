"""ADR-0011 criterion (g), MEASURED for the first time -- issue #229.

`docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`
holds the launch on the undocumented `wheel_hinge damping="0.08"` in
`sim/models/*.xml` -- "the only load-bearing constant in the MJCF with no
provenance comment" -- and conditions its non-blocking status entirely on one
criterion:

    (g) Every pass must hold across a damping sweep of 0.5x-2x on the
    wheel-hinge damping="0.08" (see below). Cheap in sim, and it converts a
    soft cliff into a bounded one.

Before this file there was no way to change `damping="0.08"` without editing
`sim/models/` (Sr. Mechanical & Systems' fidelity contract, not a per-run
knob), so (g) had never been run at either the shipped 40 A envelope or the
candidate 60 A one. `sim-host --damping-scale` (`crates/sim-host`,
`crates/sim-backend`, `crates/plant-mujoco`) is the instrument; this file is
the measurement, at the 60 A envelope (issue: realistic-motor-torque).

HEADLINE RESULT: **criterion (g) FAILS.** ADR-0011's own acceptance matrix
(criterion (a)) has three entries; two currently pass at 60 A on the deployed
(estimator) signal path -- full stick from rest, and full stick reversal at
speed. Swept across 0.5x-2x wheel-hinge damping, ONE of those two stops
holding, well inside the mandated range:

* **Full stick from rest holds across the ENTIRE 0.5x-2x range** -- peak lean
  never exceeds 15.59 deg of the 17.19 deg ceiling. Current headroom does
  not, though: it runs out (saturates) at damping scale >= ~1.42x and is
  negative (-4.44 A, i.e. saturated for 350 of ~9250 cycles) at 2.0x. The
  board rides this out, but "with room" is no longer true.
* **Full stick reversal at speed -- the worst case ADR-0011 itself names --
  INVERTS at damping scale ~1.42x-1.45x and stays inverted through 2.0x.**
  Bisected: holds at 1.40x (peak demand 59.29 A, headroom 0.71 A -- already
  razor-thin), first saturates at 1.42x, and inverts at 1.45x (`FALLEN` at
  20.37 s, past 90 deg at 20.63 s). This is a real cliff, not a taper, exactly
  the shape ADR-0011 uses to describe the ORIGINAL 40 A defect.
* **Kerb strike remains inverted at every scale tested (0.5x, 1.0x, 2.0x)**,
  consistent with ADR-0011's own reasoning that this entry is a geometry/
  actuator-rate fact unrelated to `MAX_CURRENT_A` or wheel-hinge damping, not
  a new finding.

WHY THIS CONTRADICTS THE STATED CALIBRATION EXPECTATION
---------------------------------------------------------
Before this measurement, the expectation on record was: peak demand for full
stick from rest at 60 A is ~42 A/unit; doubled damping drag costs ~+7 A at
the speed cap; so ~49 A against 60 A "should pass with room". Measured at
2.0x: peak demand is **64.44 A -- past the 60 A envelope itself**, not the
predicted 49 A. The prediction assumed a roughly linear cost; measured, the
cost is markedly nonlinear above ~1.5x, and on the reversal entry it is not
merely nonlinear, it is a cliff. The sweep wins over the expectation, per
this file's own charter.

WHAT THIS MEANS FOR ADR-0011
-----------------------------
The ADR states plainly: `damping="0.08"` "does not block launch PROVIDED
criterion (g) holds." It does not. This is a new fact, not a re-statement of
a known one, and per this repo's own rule (`docs/decisions/ADR-0011...md`'s
own account of "closing a report on a plausible mechanism rather than on a
measurement") it belongs escalated rather than absorbed quietly here.

METHOD
------
`--damping-scale` (`sim_host::HostConfig::damping_scale`, threaded to
`sim_backend::SimBackend::set_damping_scale`) multiplies the OPEN model's
resolved `wheel_hinge` `dof_damping` at `open()`, before the first `mj_step`
-- mirrors `--incline-deg`'s timing and its "read the resolved value, don't
assert a literal" reasoning exactly. `1.0` (or omitting the flag) is a
bit-identical no-op; proven below AND in
`crates/sim-backend/src/lib.rs::tests::damping_scale_of_one_is_bit_identical_
to_leaving_it_unset`.

Every run is `--scripted-scenario ... --free-run --pitch-source estimator`
(the deployed signal path -- criterion (f) is measured separately and is not
this file's subject), so the whole sweep is deterministic and fast.

THE BINARY MUST BE BUILT -- same rule every other measurement file in this
directory carries, for the same reason: a green gate that quietly did
nothing is worse than no gate.
"""

from __future__ import annotations

import csv
import math
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.plant import kerb_strike_impulse  # noqa: E402

#: Ports well away from both the documented 9601/9602 and every other
#: verification file's own throwaway ports, so a parallel run cannot collide.
STATE_OUT = "127.0.0.1:19801"
INPUT_IN = "127.0.0.1:19802"

#: ICD 10.1 / `host.rs`'s own constants, at the 60 A envelope (issue:
#: realistic-motor-torque). Duplicated the same way `test_cmd_envelope_
#: reserve.py` duplicates them -- this file has no Rust binding.
MAX_CURRENT_A = 60.0
KT_NM_PER_A = 0.7
KP_NM_PER_RAD = 140.0
PITCH_CEILING_DEG = math.degrees(MAX_CURRENT_A * KT_NM_PER_A / KP_NM_PER_RAD)

#: Pitch beyond which the board is unambiguously going over -- not the wire's
#: own `FALLEN` threshold (20 deg), which this file measures the lateness of.
INVERTED_DEG = 90.0

#: `sim/scenarios/plant.py`'s own model parameters (issue #142 AC2), for the
#: kerb-strike entry -- Sr. Mechanical & Systems' derivation, not invented
#: here.
KERB_MOMENT_ARM_M = 0.77885 - 0.1454
DISTURBANCE_WINDOW_S = 0.002


def _run(tmp_path: Path, name: str, scenario: str, **kw) -> list[dict]:
    """One deterministic free run; returns the per-cycle trace.

    Through `cargo run` rather than the raw binary path, for the same reason
    every other file here is: Cargo puts the linked libmujoco's `OUT_DIR` on
    the dynamic loader path.
    """
    out = tmp_path / f"{name}.csv"
    argv = [
        "cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
        "--scripted-scenario", scenario,
        "--free-run",
        "--stats-path", "none",
        "--pitch-source", "estimator",
        "--state-out-addr", STATE_OUT,
        "--input-in-addr", INPUT_IN,
        "--trace-csv", str(out),
    ]
    for flag, value in kw.items():
        if value is None:
            continue
        argv += [f"--{flag.replace('_', '-')}", str(value)]
    proc = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert proc.returncode == 0, f"sim-host failed:\n{proc.stderr[-4000:]}"
    with out.open() as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def _inversion_time(rows):
    return next((r["sim_time_s"] for r in rows if abs(r["truth_pitch_deg"]) > INVERTED_DEG), None)


def _fallen_time(rows):
    return next((r["sim_time_s"] for r in rows if r["fallen"] > 0.5), None)


def _healthy(rows):
    """Rows up to the moment the board is committed, or all of them -- peak
    statistics past that point describe a tumbling board, not a controller."""
    t = _inversion_time(rows)
    return [r for r in rows if t is None or r["sim_time_s"] < t]


def _peak_demand_a(rows):
    return max(abs(r["proposed_amps"]) for r in _healthy(rows))


def _peak_pitch_deg(rows):
    return max(abs(r["truth_pitch_deg"]) for r in _healthy(rows))


def _saturated_cycles(rows):
    return sum(1 for r in _healthy(rows) if r["saturated"] > 0.5)


def test_the_sim_host_binary_is_actually_built():
    assert (REPO / "target" / "release" / "sim-host").exists(), (
        "sim-host is not built, so this whole gate would be vacuous. Run:\n"
        "    cargo build --release -p sim-host --bin sim-host"
    )


# ---------------------------------------------------------------------------
# The positive control: is `--damping-scale` really scaling the damping?
# ---------------------------------------------------------------------------


def test_the_damping_scale_knob_is_a_multiplier_and_not_something_else(tmp_path):
    """Everything below is worthless if this knob does something other than
    what it claims.

    1. **1.0 is a no-op.** Bit-identical to a run that never passed the flag
       -- `dof_damping` is left alone rather than rewritten with its own
       value (mirrors `--incline-deg 0`'s own control in
       `test_incline_tolerance.py`).
    2. **More damping costs more current, monotonically.** Extra hinge drag
       is an extra load the motor must overcome to hold a given
       acceleration; peak demand for the SAME schedule must not go down as
       the scale goes up.
    """
    unset = _run(tmp_path, "ctrl_unset", "full-stick", pitch_source="estimator")
    one = _run(tmp_path, "ctrl_one", "full-stick", pitch_source="estimator", damping_scale=1.0)
    assert [list(r.values()) for r in unset] == [list(r.values()) for r in one], (
        "--damping-scale 1.0 is not bit-identical to omitting the flag, so the "
        "knob perturbs a shipped run and every other test in this repo is now "
        "measuring something slightly different"
    )

    peaks = []
    for scale in (0.5, 1.0, 1.5, 2.0):
        rows = _run(tmp_path, f"mono{scale}", "full-stick", pitch_source="estimator", damping_scale=scale)
        peaks.append(_peak_demand_a(rows))
    assert peaks == sorted(peaks), (
        f"peak demand is not monotonically increasing with damping scale "
        f"({peaks}) -- whatever --damping-scale is doing, it is not applying "
        "a uniform extra drag, and no number in this file means anything"
    )


# ---------------------------------------------------------------------------
# Criterion (g): the sweep itself, across ADR-0011's own acceptance matrix
# ---------------------------------------------------------------------------


def test_criterion_g_full_stick_from_rest_holds_across_the_full_sweep(tmp_path):
    """(a)-entry-1 across 0.5x-2x. Holds throughout -- but headroom is
    consumed, not preserved, at the top of the range."""
    print(f"\n(a)1 full stick from rest, {MAX_CURRENT_A:.0f} A envelope, damping swept 0.5x-2x:")
    print(f"{'scale':>6} {'peak_A':>8} {'hdrm_A':>8} {'peak_deg':>9} {'hdrm_deg':>9} {'sat_cyc':>8}  verdict")
    worst_headroom = None
    for scale in (0.5, 0.75, 1.0, 1.25, 1.5, 1.75, 2.0):
        rows = _run(tmp_path, f"a1_{scale}", "full-stick", pitch_source="estimator", damping_scale=scale)
        pa, pp = _peak_demand_a(rows), _peak_pitch_deg(rows)
        sat = _saturated_cycles(rows)
        t = _inversion_time(rows)
        hdrm_a = MAX_CURRENT_A - pa
        worst_headroom = hdrm_a if worst_headroom is None else min(worst_headroom, hdrm_a)
        print(
            f"{scale:6.2f} {pa:8.3f} {hdrm_a:8.3f} {pp:9.3f} {PITCH_CEILING_DEG - pp:9.3f} "
            f"{sat:8d}  {'held' if t is None else f'INVERTED at {t:.2f}s'}"
        )
        assert t is None, (
            f"full stick from rest INVERTED at damping scale {scale}x -- criterion "
            "(g) fails here for entry (a)-1, not only for (a)-2"
        )
    assert worst_headroom < 0, (
        "expected current headroom to run out somewhere in this range (measured "
        f"minimum {worst_headroom:.2f} A) -- if it no longer does, the calibration "
        "expectation this file's docstring records as WRONG may have become "
        "right again; re-read and update the docstring rather than silently "
        "tightening this assertion"
    )


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (g): full stick reversal at speed (a-entry-2, the "
        "ADR's own named worst case) INVERTS at wheel-hinge damping scale "
        "~1.45x, well inside the mandated 0.5x-2x sweep. Bisected: holds at "
        "1.40x (peak demand 59.29 A of 60 A, headroom 0.71 A), first "
        "saturates at 1.42x, inverts at 1.45x (FALLEN at 20.37s, past 90 deg "
        "at 20.63s), and stays inverted through 2.0x. This is a NEW finding, "
        "not a restatement of a known one: at the shipped damping (1.0x) this "
        "entry passes with 12.93 A / 5.05 deg of headroom. `strict=True` so "
        "a future change that closes this gap (a headroom-based fix, a "
        "damping re-derivation) turns this XPASS rather than staying quietly "
        "green."
    ),
)
def test_criterion_g_full_stick_reversal_holds_across_the_full_sweep(tmp_path):
    """(a)-entry-2 across 0.5x-2x -- the entry criterion (g) actually breaks."""
    print(f"\n(a)2 full stick reversal at speed, {MAX_CURRENT_A:.0f} A envelope, damping swept 0.5x-2x:")
    print(f"{'scale':>6} {'peak_A':>8} {'hdrm_A':>8} {'peak_deg':>9} {'hdrm_deg':>9} {'sat_cyc':>8}  verdict")
    for scale in (0.5, 0.75, 1.0, 1.25, 1.4, 1.45, 1.5, 1.75, 2.0):
        rows = _run(tmp_path, f"a2_{scale}", "stick-reversal", pitch_source="estimator", damping_scale=scale)
        pa, pp = _peak_demand_a(rows), _peak_pitch_deg(rows)
        sat = _saturated_cycles(rows)
        t = _inversion_time(rows)
        tf = _fallen_time(rows)
        print(
            f"{scale:6.2f} {pa:8.3f} {MAX_CURRENT_A - pa:8.3f} {pp:9.3f} {PITCH_CEILING_DEG - pp:9.3f} "
            f"{sat:8d}  {'held' if t is None else f'INVERTED at {t:.2f}s (FALLEN {tf:.2f}s)'}"
        )
        assert t is None, (
            f"full stick reversal at speed INVERTED at damping scale {scale}x "
            f"(FALLEN at {tf}s) -- this is the measurement this test exists to "
            "surface, not a defect in the test"
        )


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (a) entry 3 (kerb strike) already fails "
        "independent of damping and MAX_CURRENT_A -- see "
        "test_cmd_envelope_reserve.py's own xfail for the full reasoning "
        "(201 deg/s imparted pitch rate against a ~76 deg/s KD channel, a "
        "geometry/actuator-rate fact). Swept here across three points to "
        "confirm the sweep does not change that conclusion; strict so a "
        "surprise pass anywhere in the range is caught rather than silently "
        "absorbed."
    ),
)
def test_criterion_g_kerb_strike_remains_a_known_failure_across_the_sweep(tmp_path):
    """(a)-entry-3, at the sweep's endpoints and centre -- not the full grid,
    since this entry is not expected to depend on wheel-hinge damping at
    all and the point is confirming that, not characterising a curve."""
    t0, height_m = 6.0, 0.020
    print(f"\n(a)3 kerb strike, {height_m * 1000:.0f} mm lip, damping scale in (0.5, 1.0, 2.0):")
    for scale in (0.5, 1.0, 2.0):
        baseline = _run(tmp_path, f"a3base_{scale}", "full-stick", pitch_source="estimator", damping_scale=scale)
        speed = next(
            abs(r["forward_speed_m_s"]) for r in baseline if abs(r["sim_time_s"] - t0) < 1e-3
        )
        strike = kerb_strike_impulse(height_m, speed)
        impulse_ns = strike["impulse_ns"][0]
        force_n = impulse_ns / DISTURBANCE_WINDOW_S
        torque_nm = -(KERB_MOMENT_ARM_M * force_n)
        rows = _run(
            tmp_path, f"a3_{scale}", "full-stick", pitch_source="estimator", damping_scale=scale,
            disturbance=f"{t0},{DISTURBANCE_WINDOW_S},{force_n},0,0,0,{torque_nm},0",
        )
        t = _inversion_time(rows)
        print(
            f"  scale {scale:.2f}: struck at {speed:.2f} m/s, {impulse_ns:.1f} N*s -> "
            f"{'held' if t is None else f'INVERTED at {t:.2f}s'}"
        )
        assert t is None, f"kerb strike at damping scale {scale}x inverted at {t:.3f}s"


def test_verdict_criterion_g_fails_and_the_calibration_expectation_was_wrong(tmp_path):
    """Not a new measurement -- states the conclusion this file's other tests
    already establish, as a single always-green assertion so a CI summary
    that only reads test names still carries the finding.

    ADR-0011: "(g) Every pass must hold across a damping sweep of 0.5x-2x on
    the wheel-hinge damping=0.08 ... It does not block launch PROVIDED
    criterion (g) holds." Measured: it does not hold -- (a)-entry-2 inverts
    at damping scale ~1.45x, inside the mandated range, at the 60 A envelope.
    """
    print(
        "\nVERDICT: ADR-0011 criterion (g) FAILS. Full stick reversal at "
        "speed -- the ADR's own named worst case -- inverts at wheel-hinge "
        "damping scale ~1.45x, inside the mandated 0.5x-2x sweep, at the "
        "60 A envelope. See test_criterion_g_full_stick_reversal_holds_"
        "across_the_full_sweep for the bisected numbers. The stated "
        "calibration expectation (~49 A peak demand at 2x damping, passing "
        "with room) was WRONG: measured peak demand at 2x damping (full "
        "stick from rest) is 64.44 A, past the 60 A envelope itself."
    )
