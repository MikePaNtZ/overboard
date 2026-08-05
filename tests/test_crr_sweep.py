"""ADR-0011 criterion (g), RE-DERIVED for the physically-grounded drag model
-- issue: drag-model.

`docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`'s
criterion (g) reads:

    (g) Every pass must hold across a damping sweep of 0.5x-2x on the
    wheel-hinge `damping="0.08"` (see below). Cheap in sim, and it converts
    a soft cliff into a bounded one.

That instrument no longer applies. `wheel_hinge damping="0.08"` -- "the only
load-bearing constant in the MJCF with no provenance comment" per the ADR's
own words -- has been replaced by two physically derived terms
(`frictionloss` for rolling resistance, `damping` for bearing/motor losses;
see `sim/models/overboard_rider.xml`'s `wheel_hinge` comment). The remaining
physical uncertainty is no longer an arbitrary 0.5x-2x band on an undocumented
number -- it is Crr, the rolling-resistance coefficient, which has PUBLISHED
BOUNDS: Engineering Toolbox's generic pneumatic-tyre table runs 0.007-0.02,
and this vehicle's own 18 psi wide-tyre working band (see the model comment)
runs 0.015-0.025. This file sweeps the UNION of both -- **0.007 to 0.025** --
because 0.007 is the conservative low end a harder/higher-pressure tyre could
plausibly land at, and 0.025 is the high end of this vehicle's own working
band; sweeping only the narrower working band would silently drop the
conservative bound ADR-0011's spirit ("a damping sweep... converts a soft
cliff into a bounded one") calls for. Expressed as a multiplier on this
model's own central Crr = 0.02, that is **0.35x-1.25x** on `frictionloss`
(`--crr-scale`, `sim_host::HostConfig::crr_scale`, threaded to
`sim_backend::SimBackend::set_crr_scale`) -- deliberately NOT the old 0.5x-2x
range, which was never anchored to a physical bound in the first place.

HEADLINE RESULT: **criterion (g) still FAILS, and not only from the sweep.**
ADR-0011's own acceptance matrix (criterion (a)) has three entries:

* **Full stick from rest holds across the ENTIRE 0.35x-1.25x range** -- peak
  current demand rises monotonically from 26.65 A (0.35x) to 37.05 A (1.25x)
  against the 40 A envelope, so headroom shrinks from 13.35 A to 2.95 A but
  never runs out.
* **Full stick reversal at speed -- ADR-0011's own named worst case --
  INVERTS AT THE CENTRAL, UNSWEPT VALUE (1.0x, i.e. the shipped Crr = 0.02).**
  This is NOT a sweep finding: it is a new fact about the drag-model change
  itself, independent of (g)'s instrument. It PASSED before this change
  (`test_cmd_envelope_reserve.py::test_criterion_a2_a_full_stick_reversal_at_
  speed_does_not_invert` was an ordinary, non-xfail test). Across the swept
  range the outcome is NOT monotonic in Crr: it holds at 0.35x-0.65x and at
  1.10x, and inverts at 0.80x, 0.95x, 1.00x and 1.25x. That non-monotonicity
  is itself part of the finding -- this is a chaotic boundary near a
  bifurcation, not a clean cliff, which is reported as measured rather than
  smoothed into a single threshold.
* **Kerb strike remains inverted at every scale tested (0.35x-1.25x)**,
  consistent with ADR-0011's own reasoning that this entry is a geometry/
  actuator-rate fact unrelated to wheel-hinge drag, not a new finding.

WHAT THIS MEANS FOR ADR-0011
-----------------------------
The ADR conditions `damping="0.08"`'s non-blocking status "PROVIDED criterion
(g) holds." Under the OLD instrument (g) already failed at 60 A (see the
banked `feat/controls/damping-sweep-instrument` branch). Under the physically
re-derived drag model, at the SHIPPED 40 A envelope, (a)-entry-2 fails even
without invoking the sweep at all -- the drag-model change alone moved a
previously-passing criterion to failing. This is new information for
whoever owns the launch-hold review, not a restatement of the old (g)
finding, and per this ADR's own stated rule (closing a report on a plausible
mechanism rather than on a measurement) it is reported here rather than
absorbed quietly.

METHOD
------
`--crr-scale` multiplies the OPEN model's resolved `wheel_hinge`
`dof_frictionloss` at `open()`, before the first `mj_step` -- mirrors
`--incline-deg`/`--damping-scale`'s timing and "read the resolved value,
don't assert a literal" reasoning exactly. `1.0` (or omitting the flag) is a
bit-identical no-op; proven in `crates/sim-backend/src/lib.rs::tests::
crr_scale_of_one_is_bit_identical_to_leaving_it_unset`.

Every run is `--scripted-scenario ... --free-run --pitch-source estimator`
(the deployed signal path -- criterion (f) is measured separately), so the
whole sweep is deterministic and fast.

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

#: Ports well away from the documented 9601/9602 and every other
#: verification file's own throwaway ports.
STATE_OUT = "127.0.0.1:19701"
INPUT_IN = "127.0.0.1:19702"

#: ICD 10.1 / `host.rs`'s own constants, at the SHIPPED 40 A envelope --
#: unlike the banked `damping-sweep-instrument` branch, this file does not
#: assume the 60 A candidate.
MAX_CURRENT_A = 40.0
KT_NM_PER_A = 0.7
KP_NM_PER_RAD = 140.0
PITCH_CEILING_DEG = math.degrees(MAX_CURRENT_A * KT_NM_PER_A / KP_NM_PER_RAD)

#: Pitch beyond which the board is unambiguously going over -- not the wire's
#: own `FALLEN` threshold (20 deg), which this file measures the lateness of.
INVERTED_DEG = 90.0

#: This model's own central Crr (see `overboard_rider.xml`'s `wheel_hinge`
#: comment). The sweep below is expressed as a multiplier on `frictionloss`,
#: which is linear in Crr, so a `--crr-scale` of `s` corresponds to
#: `Crr = 0.02 * s`.
CENTRAL_CRR = 0.02

#: The physically defensible Crr range this file sweeps, expressed as
#: (Crr, scale) pairs -- the UNION of Engineering Toolbox's generic
#: pneumatic-tyre table (0.007-0.02) and this vehicle's own 18 psi wide-tyre
#: working band (0.015-0.025); see this file's own module docstring for why
#: the union rather than either alone.
#: The shipped, unswept centre (Crr = 0.02, scale 1.0) is deliberately
#: INCLUDED here, not just implied -- (a)-entry-2 below fails AT this exact
#: point, which is the finding this file exists to surface, and dropping it
#: from the grid would silently hide the fact that (g)'s failure does not
#: depend on sweeping at all.
CRR_SWEEP = tuple(
    (crr, crr / CENTRAL_CRR)
    for crr in (0.007, 0.010, 0.013, 0.016, 0.019, 0.020, 0.022, 0.025)
)

#: `sim/scenarios/plant.py`'s own model parameters (issue #142 AC2), for the
#: kerb-strike entry -- Sr. Mechanical & Systems owns that derivation, not
#: invented here.
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
# The positive control: is `--crr-scale` really scaling Crr/frictionloss?
# ---------------------------------------------------------------------------


def test_the_crr_scale_knob_is_a_multiplier_and_not_something_else(tmp_path):
    """Everything below is worthless if this knob does something other than
    what it claims.

    1. **1.0 is a no-op.** Bit-identical to a run that never passed the flag
       -- `dof_frictionloss` is left alone rather than rewritten with its own
       value (mirrors `--incline-deg 0`'s own control in
       `test_incline_tolerance.py`, and `--damping-scale 1.0`'s in the banked
       `damping-sweep-instrument` branch).
    2. **More rolling resistance costs more current, monotonically**, on the
       entry that stays inside its linear regime (full stick from rest).
       Extra hinge drag is an extra load the motor must overcome to hold a
       given acceleration; peak demand for the SAME schedule must not go down
       as Crr goes up.
    """
    unset = _run(tmp_path, "ctrl_unset", "full-stick", pitch_source="estimator")
    one = _run(tmp_path, "ctrl_one", "full-stick", pitch_source="estimator", crr_scale=1.0)
    assert [list(r.values()) for r in unset] == [list(r.values()) for r in one], (
        "--crr-scale 1.0 is not bit-identical to omitting the flag, so the "
        "knob perturbs a shipped run and every other test in this repo is now "
        "measuring something slightly different"
    )

    peaks = []
    for _, scale in CRR_SWEEP:
        rows = _run(tmp_path, f"mono{scale:.3f}", "full-stick", pitch_source="estimator", crr_scale=scale)
        peaks.append(_peak_demand_a(rows))
    assert peaks == sorted(peaks), (
        f"peak demand is not monotonically increasing with Crr scale "
        f"({peaks}) -- whatever --crr-scale is doing, it is not applying a "
        "uniform extra rolling resistance, and no number in this file means "
        "anything"
    )


# ---------------------------------------------------------------------------
# Criterion (g), re-derived: the sweep itself, across ADR-0011's own
# acceptance matrix, at the SHIPPED 40 A envelope.
# ---------------------------------------------------------------------------


def test_criterion_g_full_stick_from_rest_holds_across_the_full_sweep(tmp_path):
    """(a)-entry-1 across Crr 0.007-0.025 (0.35x-1.25x). Holds throughout --
    headroom shrinks with Crr but never runs out."""
    print(f"\n(a)1 full stick from rest, {MAX_CURRENT_A:.0f} A envelope, Crr swept 0.007-0.025:")
    print(f"{'Crr':>6} {'scale':>6} {'peak_A':>8} {'hdrm_A':>8} {'peak_deg':>9} {'hdrm_deg':>9} {'sat_cyc':>8}  verdict")
    worst_headroom = None
    for crr, scale in CRR_SWEEP:
        rows = _run(tmp_path, f"a1_{scale:.3f}", "full-stick", pitch_source="estimator", crr_scale=scale)
        pa, pp = _peak_demand_a(rows), _peak_pitch_deg(rows)
        sat = _saturated_cycles(rows)
        t = _inversion_time(rows)
        hdrm_a = MAX_CURRENT_A - pa
        worst_headroom = hdrm_a if worst_headroom is None else min(worst_headroom, hdrm_a)
        print(
            f"{crr:6.3f} {scale:6.2f} {pa:8.3f} {hdrm_a:8.3f} {pp:9.3f} {PITCH_CEILING_DEG - pp:9.3f} "
            f"{sat:8d}  {'held' if t is None else f'INVERTED at {t:.2f}s'}"
        )
        assert t is None, (
            f"full stick from rest INVERTED at Crr {crr} (scale {scale:.2f}x) -- "
            "criterion (g) fails here for entry (a)-1, not only for (a)-2"
        )
    assert worst_headroom > 0, (
        f"expected current headroom to stay positive across this range (measured "
        f"minimum {worst_headroom:.2f} A) -- if it no longer does, re-derive "
        "rather than tightening this assertion"
    )


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (g), re-derived (issue: drag-model): full stick "
        "reversal at speed (a-entry-2, the ADR's own named worst case) "
        "INVERTS AT THE CENTRAL, UNSWEPT Crr = 0.02 (scale 1.0x) -- at 25.55s. "
        "This is a NEW finding independent of the sweep itself: the "
        "physically-derived drag model change alone moved a PREVIOUSLY "
        "PASSING criterion (`test_cmd_envelope_reserve.py::test_criterion_a2_"
        "a_full_stick_reversal_at_speed_does_not_invert` was an ordinary, "
        "non-xfail test before this change) to failing. Swept across Crr "
        "0.007-0.025 the outcome is NOT monotonic: holds at 0.007-0.013 and "
        "at scale 1.10x (Crr 0.022), inverts at 0.016, 0.019, 0.020 (central) "
        "and 0.025 -- a chaotic boundary near a bifurcation, not a clean "
        "cliff. `strict=True` so a future change that closes this gap turns "
        "this XPASS rather than staying quietly green."
    ),
)
def test_criterion_g_full_stick_reversal_holds_across_the_full_sweep(tmp_path):
    """(a)-entry-2 across Crr 0.007-0.025 -- the entry criterion (g) fails
    even at the unswept centre."""
    print(f"\n(a)2 full stick reversal at speed, {MAX_CURRENT_A:.0f} A envelope, Crr swept 0.007-0.025:")
    print(f"{'Crr':>6} {'scale':>6} {'peak_A':>8} {'hdrm_A':>8} {'peak_deg':>9} {'sat_cyc':>8}  verdict")
    for crr, scale in CRR_SWEEP:
        rows = _run(tmp_path, f"a2_{scale:.3f}", "stick-reversal", pitch_source="estimator", crr_scale=scale)
        pa, pp = _peak_demand_a(rows), _peak_pitch_deg(rows)
        sat = _saturated_cycles(rows)
        t = _inversion_time(rows)
        tf = _fallen_time(rows)
        print(
            f"{crr:6.3f} {scale:6.2f} {pa:8.3f} {MAX_CURRENT_A - pa:8.3f} {pp:9.3f} "
            f"{sat:8d}  {'held' if t is None else f'INVERTED at {t:.2f}s (FALLEN {tf})'}"
        )
        assert t is None, (
            f"full stick reversal at speed INVERTED at Crr {crr} (scale "
            f"{scale:.2f}x, FALLEN at {tf}s) -- this is the measurement this "
            "test exists to surface, not a defect in the test"
        )


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (a) entry 3 (kerb strike) already fails "
        "independent of wheel-hinge drag and MAX_CURRENT_A -- see "
        "test_cmd_envelope_reserve.py's own xfail for the full reasoning "
        "(a geometry/actuator-rate fact). Swept here across the full Crr "
        "range to confirm the sweep does not change that conclusion; strict "
        "so a surprise pass anywhere in the range is caught rather than "
        "silently absorbed."
    ),
)
def test_criterion_g_kerb_strike_remains_a_known_failure_across_the_sweep(tmp_path):
    """(a)-entry-3, across the full Crr sweep -- not expected to depend on
    wheel-hinge drag at all, and the point is confirming that."""
    t0, height_m = 6.0, 0.020
    print(f"\n(a)3 kerb strike, {height_m * 1000:.0f} mm lip, Crr swept 0.007-0.025:")
    for crr, scale in CRR_SWEEP:
        baseline = _run(tmp_path, f"a3base_{scale:.3f}", "full-stick", pitch_source="estimator", crr_scale=scale)
        speed = next(
            abs(r["forward_speed_m_s"]) for r in baseline if abs(r["sim_time_s"] - t0) < 1e-3
        )
        strike = kerb_strike_impulse(height_m, speed)
        impulse_ns = strike["impulse_ns"][0]
        force_n = impulse_ns / DISTURBANCE_WINDOW_S
        torque_nm = -(KERB_MOMENT_ARM_M * force_n)
        rows = _run(
            tmp_path, f"a3_{scale:.3f}", "full-stick", pitch_source="estimator", crr_scale=scale,
            disturbance=f"{t0},{DISTURBANCE_WINDOW_S},{force_n},0,0,0,{torque_nm},0",
        )
        t = _inversion_time(rows)
        print(
            f"  Crr {crr:.3f} (scale {scale:.2f}x): struck at {speed:.2f} m/s, "
            f"{impulse_ns:.1f} N*s -> {'held' if t is None else f'INVERTED at {t:.2f}s'}"
        )
        assert t is None, f"kerb strike at Crr {crr} (scale {scale:.2f}x) inverted at {t:.3f}s"


def test_verdict_criterion_g_fails_and_it_is_worse_than_the_old_finding(tmp_path):
    """Not a new measurement -- states the conclusion this file's other
    tests already establish, as a single always-green assertion so a CI
    summary that only reads test names still carries the finding.

    ADR-0011: "(g) Every pass must hold across a damping sweep ... It does
    not block launch PROVIDED criterion (g) holds." Measured: it does not
    hold, and unlike the old lumped-damping sweep (which needed 2x scaling
    before (a)-2 failed at 60 A, per the banked `damping-sweep-instrument`
    branch), the physically re-derived drag model fails (a)-2 AT ITS OWN
    CENTRAL, UNSWEPT VALUE, at the shipped 40 A envelope. The drag-model
    change itself -- not merely the sweep it enables -- moved a previously
    passing criterion to failing.
    """
    print(
        "\nVERDICT: ADR-0011 criterion (g) FAILS. Full stick reversal at "
        "speed -- the ADR's own named worst case -- inverts at the CENTRAL "
        "(unswept) Crr = 0.02, at the shipped 40 A envelope; this is a new "
        "fact about the drag-model change itself, not only about the sweep. "
        "Across the swept Crr range (0.007-0.025) the outcome is "
        "non-monotonic -- a chaotic boundary, not a clean cliff. See "
        "test_criterion_g_full_stick_reversal_holds_across_the_full_sweep "
        "for the full table."
    )
