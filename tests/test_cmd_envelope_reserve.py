"""ADR-0011's acceptance matrix, measured -- including the entries it fails.

`docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`
holds the launch until the board stops inverting at full stick, and specifies
the fix as a **command-envelope reserve**: scale fore/aft stick by a derived
constant upstream of the estimator, the regulator and the safety envelope,
changing nothing else. This file is the evidence for that fix and, more
importantly, the evidence for where it stops working.

SUPERSEDED IN PART by issue: realistic-motor-torque (`MAX_CURRENT_A` 40 -> 60
A / 28 -> 42 N*m). Every ORDINARY test below still measures what its name
says; several of them now measure a materially better result than when this
docstring was written, and are annotated inline rather than rewritten here.
The short version, read in full at the annotated tests:

* `CMD_ENVELOPE_RESERVE` (`host.rs`) is now 1.00 -- SATURATED, i.e. no
  shaping -- because the measured peak-demand slope (~42 A/unit, essentially
  unchanged from the 40 A measurement) no longer over-commands a 60 A
  envelope at all. The mechanism this whole file exists to test is a no-op on
  the ESTIMATOR path at this envelope.
* On the ESTIMATOR path, this file's own former positive control (unshaped
  full stick from rest) NO LONGER INVERTS -- see
  `test_the_unshaped_command_map_no_longer_inverts_the_board_at_60a`.
* On the TRUTH-fed path, criterion (f)'s literal reading STILL FAILS -- the
  board still inverts, just later (11.4 s vs 4.1 s at 40 A). This is the one
  reading of (f) that survives unchanged.
* Criterion (f)'s ROBUSTNESS reading (a worst-case static pitch bias) now
  PASSES where it used to fail hard and stably -- see
  `test_criterion_f_the_matrix_holds_under_a_worst_case_static_pitch_bias`.

WHAT THIS FILE ASSERTS, AND WHAT IT DELIBERATELY MARKS xfail
------------------------------------------------------------
At the 40 A envelope this docstring was originally written against, three of
ADR-0011's criteria passed on the deployed signal path -- (b), (a)-entry-1,
(a)-entry-2 -- and two did not, at any value of the constant, written as
`xfail(strict=True)`: (a)-entry-3 (the kerb strike) and (f) (both readings).

At 60 A, **only (a)-entry-3 and the LITERAL reading of (f) still carry an
`xfail(strict=True)`** -- see `test_criterion_a3_full_stick_during_a_kerb_
strike_does_not_invert` and `test_criterion_f_full_stick_holds_with_the_
estimator_bias_removed`. The kerb strike is unaffected because it was never
about the motor -- ADR-0011's own note names the ballast stroke and CoM
height as the undersized elements, not current authority, and the 20 mm
lip's imparted pitch rate has nothing to do with `MAX_CURRENT_A`.

`strict=True` matters: the day somebody fixes the underlying problem these
turn RED as XPASS, which is the only mechanism that reliably tells a future
session a known failure has stopped being one -- and that is exactly what
happened to this file's robustness-reading test for (f) at 60 A, and to its
own former positive control. A green suite that quietly stopped covering the
thing it was written for is the failure mode this repo keeps hitting; this
docstring update, and the inline annotations at each changed test, are the
record that it did not happen quietly here.

WHY CRITERION (f) IS ALSO FLAGGED AS MIS-SPECIFIED
---------------------------------------------------
The ADR implements "with the estimator bias removed" as "feed the regulator
MuJoCo truth". Measured here (`test_the_estimator_residual_is_specific_force_
not_a_static_bias`), the `est - truth` residual is NOT a static bias: it
regresses on the specific force the accel aiding failed to remove at ~5.8 deg
per m/s^2, which is one radian per g -- the signature of a complementary
filter reading the APPARENT VERTICAL rather than pitch. So `--pitch-source
truth` does not remove an error; it deletes an acceleration reference and
leaves a pitch-only regulator. Both readings of the criterion are therefore
run: the ADR's literal one, and the robustness one it was reaching for (a
worst-case STATIC bias injected on top of the real signal path). Both fail,
so the conclusion does not depend on which reading is right.

CRITERION (a)-2 IS ON A CHAOTIC BOUNDARY -- issue: statistical-acceptance
--------------------------------------------------------------------------
`test_criterion_a2_a_full_stick_reversal_at_speed_does_not_invert`'s
noiseless run does not reach a full inversion, but its peak lean already
sits a few percent over `PITCH_CEILING_DEG` -- read on its own that looks
like "holds, barely". It is not a real margin: run the IDENTICAL scenario
through the datasheet-derived IMU noise model at each of 25 fixed seeds
(`sim.scenarios.seeded_acceptance`) and the board inverts fully, 25 times out
of 25 -- see `test_criterion_a2_the_reversal_holds_across_a_seeded_imu_noise_
distribution`. The noiseless test is kept (it is still the cheapest way to
see a regression on this exact point move), but it no longer stands alone as
this criterion's evidence.

DETERMINISM
------------
Every run is `--scripted-scenario ... --free-run`: the schedule is indexed on
simulated time (no wall clock, no socket, no staleness gate -- issue #190) and
the plant is a fixed-timestep RK4 integrator with no stochastic terms, so
these numbers are reproducible bit for bit and the whole matrix costs seconds
rather than minutes of real time. The seeded distribution test above is the
one exception, by design: it deliberately injects `--imu-noise-seed` so its
"stochastic" input is itself a fixed, literal seed list -- see `sim.
scenarios.seeded_acceptance`'s own docstring.

THE BINARY MUST BE BUILT. If it is missing this FAILS, not skips -- same rule
`test_closed_loop.py` and `test_rust_hosted_impulse_response.py` carry, for
the same reason: a green gate that quietly did nothing is worse than no gate.
"""

from __future__ import annotations

import csv
import math
import re
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.plant import kerb_strike_impulse  # noqa: E402
from sim.scenarios.seeded_acceptance import run_seeded  # noqa: E402

HOST_RS = REPO / "crates" / "sim-host" / "src" / "host.rs"

#: Ports well away from the documented 9601/9602, so a verification run can
#: never collide with a live capture session -- the same reason `sim-host`
#: grew `--state-out-addr`/`--input-in-addr` in the first place.
STATE_OUT = "127.0.0.1:19601"
INPUT_IN = "127.0.0.1:19602"

#: Pitch beyond which the board is unambiguously going over. Not the wire's
#: own `FALLEN` threshold (20 deg) -- that one is a proxy this file measures
#: the lateness OF, so it cannot also be the definition of failure.
INVERTED_DEG = 90.0

#: ICD 10.1 / the host's own constants. Duplicated here rather than imported
#: because this file has no Rust binding; `test_the_shipped_constants_are_the_
#: ones_this_file_measured` pins every one of them against `host.rs` so the
#: duplication cannot drift silently. Raised 40 -> 60 A with `host.rs`'s own
#: `MAX_CURRENT_A` (issue: realistic-motor-torque).
MAX_CURRENT_A = 60.0
#: DERIVED (issue: real-motor-constants) -- see `host.rs`'s own `KT_NM_PER_A`
#: doc comment for the Hypercore hub motor derivation (Kt = 1.5 * 15 *
#: 0.02793 = 0.6284 N*m/A). Was 0.7 (unfitted placeholder) when this file's
#: docstring narrative above was written.
KT_NM_PER_A = 0.6284
KP_NM_PER_RAD = 140.0

#: Total pitch authority, degrees: `MAX_CURRENT_A * KT / KP` = 37.704 / 140 =
#: 0.2693 rad = 15.43 deg at the current, derived KT. Every "pitch headroom"
#: number in this file is measured against this.
PITCH_CEILING_DEG = math.degrees(MAX_CURRENT_A * KT_NM_PER_A / KP_NM_PER_RAD)

#: The frictionless-normal strike enters the frame at the axle, this far below
#: the vehicle CoM -- `sim/scenarios/plant.py`'s own model parameters
#: (`com_height_m` 0.77885, `wheel_radius_m` 0.1454), not numbers invented
#: here. Sr. Mechanical & Systems owns that derivation (issue #142 AC2).
KERB_MOMENT_ARM_M = 0.77885 - 0.1454

#: One control cycle, which is also one physics step (`overboard_rider.xml`
#: declares `timestep="0.002"` and `sim-backend` asserts the ratio is exactly
#: 1), so an impulse delivered over this window is exact.
DISTURBANCE_WINDOW_S = 0.002

#: Standard gravity, m/s^2. CODATA, not the model's own `9.81` -- this is the
#: constant in the `1 radian per g` identity, which is a statement about
#: physics rather than about `overboard_rider.xml`.
G_M_S2 = 9.80665

#: The window, in seconds of simulated time, over which the estimator trim is
#: measured -- see `_settled_trim`. Late in the run on purpose, and stated as
#: a constant so it is a fixed instrument rather than a fitting parameter.
TRIM_WINDOW_S = (12.0, 18.0)

#: Half-width of the central difference used to recover forward acceleration
#: from the speed trace, in control cycles. 50 cycles is +-0.1 s.
DIFF_HALF_CYCLES = 50

#: ADR-0011 (f1). The estimator trim as it stands today, degrees of `est -
#: truth` over `TRIM_WINDOW_S` of the shipped full-stick hold. A MEASURED
#: value being frozen, not a target: see
#: `test_f1_the_estimator_trim_is_pinned_so_a_retune_cannot_move_it_silently`
#: for why this is a drift detector and may never be cited as a tolerance.
PINNED_TRIM_DEG = -2.501

#: How far `PINNED_TRIM_DEG` may move before the build goes red. Set from the
#: trim movement that puts `CMD_ENVELOPE_RESERVE`'s own derivation out of
#: tolerance (measured: 0.10 deg of trim moves the peak-demand slope 5.2%,
#: against a 5% band) -- NOT from what happens to pass. See the same test.
PINNED_TRIM_BAND_DEG = 0.10


# ---------------------------------------------------------------------------
# Harness
# ---------------------------------------------------------------------------


def _rust_constant(name: str) -> float:
    """Read a `const NAME: f32 = value;` straight out of `host.rs`.

    So the numbers this file asserts against are the SHIPPED ones rather than
    a copy of them. A constant edited in Rust and not here would otherwise
    leave this suite green while measuring a controller that no longer exists.
    """
    m = re.search(rf"^const {name}: f32 = ([0-9.]+);", HOST_RS.read_text(), re.M)
    assert m is not None, f"{name} is not a plain f32 const in {HOST_RS}"
    return float(m.group(1))


def test_the_sim_host_binary_is_actually_built():
    assert (REPO / "target" / "release" / "sim-host").exists(), (
        "sim-host is not built, so this whole gate would be vacuous. Run:\n"
        "    cargo build --release -p sim-host --bin sim-host"
    )


def _run(tmp_path: Path, name: str, scenario: str, **kw) -> list[dict]:
    """One deterministic free run; returns the per-cycle trace.

    Invoked through `cargo run` rather than the raw binary path for the same
    reason `test_rust_hosted_impulse_response.py` does: Cargo puts the linked
    libmujoco's `OUT_DIR` on the dynamic loader path, so this resolves on
    macOS without hand-set environment variables.
    """
    out = tmp_path / f"{name}.csv"
    argv = [
        "cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
        "--scripted-scenario", scenario,
        "--free-run",
        "--stats-path", "none",
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


def _first(rows, pred):
    return next((r["sim_time_s"] for r in rows if pred(r)), None)


def _inversion_time(rows):
    return _first(rows, lambda r: abs(r["truth_pitch_deg"]) > INVERTED_DEG)


def _healthy(rows):
    """Rows up to the moment the board is committed, or all of them.

    Peak statistics taken after an inversion describe a tumbling board, not a
    controller, and quoting them would be meaningless in both directions.
    """
    t = _inversion_time(rows)
    return [r for r in rows if t is None or r["sim_time_s"] < t]


def _peak_demand_a(rows):
    return max(abs(r["proposed_amps"]) for r in _healthy(rows))


def _peak_pitch_deg(rows):
    return max(abs(r["truth_pitch_deg"]) for r in _healthy(rows))


def _settled_trim(rows, window=TRIM_WINDOW_S):
    """The estimator trim, measured where the identity is actually testable.

    Returns `(residual_deg, apparent_vertical_deg)`, both means over `window`:

    * `residual_deg` -- mean `est - truth` pitch. This IS the trim: the lean
      the estimator believes in that the plant does not have.
    * `apparent_vertical_deg` -- mean `atan(a_unaided / g)`, the tilt an
      accelerometer reports on a body under that specific force, where
      `a_unaided = a_x - ACCEL_FF_GAIN * I` is the part the command
      feedforward did not already remove.

    **Why a settled window and not a regression over the whole run.** The
    obvious instrument -- least squares of residual against specific force
    across the acceleration ramp -- is badly conditioned here, and measuring
    it says so: the fitted slope moves from 4.9 to 7.1 deg per m/s^2 across
    reasonable choices of fit window on the SAME run, because the
    complementary filter's 2 s time constant means the residual lags the
    specific force throughout the ramp and the fit picks up whichever part of
    that transient the window happens to contain. A band tight enough to be a
    pin would chatter on the window choice; a band loose enough not to would
    detect nothing. The identity is a STEADY-STATE statement, so it is
    measured in steady state, where the ratio holds to a few percent.
    """
    t_lo, t_hi = window
    half = DIFF_HALF_CYCLES
    # Read from `host.rs`, not copied: the command feedforward already removes
    # this much specific force per amp before the filter sees it, so a stale
    # copy here would mis-state what the filter was left reading and the trim
    # pin below would fire for the wrong reason -- or, worse, fail to.
    accel_ff_gain = _rust_constant("ACCEL_FF_GAIN_M_S2_PER_A")
    residual, apparent = [], []
    for i in range(half, len(rows) - half):
        if not t_lo <= rows[i]["sim_time_s"] <= t_hi:
            continue
        a_x = (rows[i + half]["forward_speed_m_s"] - rows[i - half]["forward_speed_m_s"]) / (
            rows[i + half]["sim_time_s"] - rows[i - half]["sim_time_s"]
        )
        unaided = a_x - accel_ff_gain * rows[i]["applied_amps"]
        residual.append(rows[i]["est_pitch_deg"] - rows[i]["truth_pitch_deg"])
        apparent.append(math.degrees(math.atan(unaided / G_M_S2)))
    assert residual, f"no rows in the trim window {window}"
    return sum(residual) / len(residual), sum(apparent) / len(apparent)


def _margin(rows) -> str:
    """The measured margin, in BOTH currencies ADR-0011 criterion (b) demands.

    Never the word "stable" -- that claim is withdrawn and gated (ADR-0003).
    """
    a, p = _peak_demand_a(rows), _peak_pitch_deg(rows)
    return (
        f"peak demand {a:.2f} A of {MAX_CURRENT_A:.0f} A "
        f"(current headroom {MAX_CURRENT_A - a:.2f} A, {100 * (1 - a / MAX_CURRENT_A):.1f}%); "
        f"peak lean {p:.2f} deg of {PITCH_CEILING_DEG:.2f} deg "
        f"(pitch headroom {PITCH_CEILING_DEG - p:.2f} deg, "
        f"{100 * (1 - p / PITCH_CEILING_DEG):.1f}%)"
    )


# ---------------------------------------------------------------------------
# Criterion (b): the constant is DERIVED
# ---------------------------------------------------------------------------


def test_the_shipped_constants_are_the_ones_this_file_measured():
    """Guard against the duplication above drifting from `host.rs`."""
    assert _rust_constant("MAX_CURRENT_A") == MAX_CURRENT_A
    assert _rust_constant("KT_NM_PER_A") == KT_NM_PER_A
    assert _rust_constant("KP_NM_PER_RAD") == KP_NM_PER_RAD


def test_peak_current_demand_is_linear_in_stick_at_the_documented_slope(tmp_path):
    """`PEAK_DEMAND_A_PER_UNIT_STICK` is a MEASUREMENT, re-taken here.

    It is the whole provenance of `CMD_ENVELOPE_RESERVE`: the constant is
    `STATED_ENVELOPE_RESERVE_FRACTION * MAX_CURRENT_A / slope`, so a slope
    that has moved means a constant that has to move with it. ADR-0011 forbids
    the reverse -- tuning the constant to make a scenario pass.

    Demand is taken PRE-envelope (`proposed_amps`), because the clamp cannot
    reveal a demand it has already removed, and on the ESTIMATOR path, because
    a command map is a property of the command path and the truth-fed runs
    have no steady state to take a peak FROM (see criterion (f) below).

    The 5% band is stated before the comparison: the per-run ratio across this
    sweep spans 41.6-43.5 A/unit, so a tighter band would be asserting a
    precision the measurement does not have.
    """
    fractions = (0.30, 0.50, 0.70, 0.90)
    peaks = [
        _peak_demand_a(_run(tmp_path, f"slope{u}", "full-stick", cmd_reserve=u, pitch_source="estimator"))
        for u in fractions
    ]
    slope = sum(u * a for u, a in zip(fractions, peaks)) / sum(u * u for u in fractions)
    documented = _rust_constant("PEAK_DEMAND_A_PER_UNIT_STICK")
    print(f"\nmeasured peak-demand slope: {slope:.2f} A/unit (host.rs: {documented})")
    assert abs(slope - documented) / documented < 0.05, (
        f"peak demand is now {slope:.2f} A per unit stick, not the documented "
        f"{documented}. CMD_ENVELOPE_RESERVE is derived from this number -- "
        f"re-derive it ({_rust_constant('STATED_ENVELOPE_RESERVE_FRACTION')} * "
        f"{MAX_CURRENT_A} / {slope:.2f} = "
        f"{_rust_constant('STATED_ENVELOPE_RESERVE_FRACTION') * MAX_CURRENT_A / slope:.3f}) "
        "rather than adjusting this tolerance."
    )
    # INVERTED as of the 60 A / 42 N*m envelope (issue: realistic-motor-torque):
    # this slope (~42 A/unit) is essentially unchanged from the 42.03 A/unit
    # measured at the old 40 A envelope -- it is a property of the control law's
    # transient response, not of the envelope. What changed is what it means:
    # against a 40 A envelope it was an over-command; against 60 A it is not.
    # See `host.rs`'s `CMD_ENVELOPE_RESERVE` doc comment -- ADR-0011's founding
    # premise for this whole mechanism no longer holds, and this assertion
    # states that rather than hiding it.
    assert slope < MAX_CURRENT_A, (
        "the premise this mechanism was built on (full stick over-commands the "
        f"envelope) no longer holds; measured {slope:.2f} A/unit against "
        f"{MAX_CURRENT_A} A -- CMD_ENVELOPE_RESERVE has saturated to a no-op, "
        "see host.rs"
    )


def _peak_demand_slope(tmp_path, tag, **kw) -> float:
    """Least squares through the origin of peak demand against stick."""
    fractions = (0.30, 0.50, 0.70, 0.90)
    peaks = [
        _peak_demand_a(_run(tmp_path, f"{tag}{u}", "full-stick", cmd_reserve=u,
                            pitch_source="estimator", **kw))
        for u in fractions
    ]
    return sum(u * a for u, a in zip(fractions, peaks)) / sum(u * u for u in fractions)


def test_f2_the_peak_demand_slope_is_trim_derived_not_geometry_derived(tmp_path):
    """ADR-0011 (f2): pin the reserve's DERIVATION, not just its value.

    `CMD_ENVELOPE_RESERVE` = 0.80 is
    `STATED_ENVELOPE_RESERVE_FRACTION * MAX_CURRENT_A / PEAK_DEMAND_A_PER_UNIT_STICK`,
    and `host.rs` enforces that arithmetic at compile time. What no compiler
    can enforce is the status of the divisor: **41.97 A per unit stick was
    measured at the current operating trim.** Describing the constant as
    derived from the actuator envelope, and stopping there, reads as though it
    were a geometric property that travels with the vehicle. It is not, and
    calling it one would be a rationalisation.

    This test makes the dependence a measurement rather than a caveat: shift
    the trim by one `PINNED_TRIM_BAND_DEG` and re-take the slope.

    The result is why (f1) and (f2) are one pin and not two. A 0.10 deg trim
    shift moves the slope by about 5.2%, and
    `test_peak_current_demand_is_linear_in_stick_at_the_documented_slope`
    grants the slope a 5% band -- so **one pinned band of trim drift is
    already enough to put the reserve's own provenance check out of
    tolerance.** That is the calibration behind `PINNED_TRIM_BAND_DEG`: it is
    the trim movement at which `CMD_ENVELOPE_RESERVE` stops being derived from
    anything true, which is a stronger reason for its width than any statement
    about what the board survives.

    The bar asserted is deliberately loose. A geometry-derived constant would
    move by exactly 0%; 3% is low enough to be robust to platform
    floating-point differences and far too high to be reached by rounding, so
    clearing it refutes "geometry-derived" without pinning a derivative.

    The trim is shifted with `--pitch-bias-deg` rather than by retuning the
    filter, because the question is what a MOVED TRIM does to the slope, and
    an injected offset moves the trim while changing nothing else -- a
    retuned `ESTIMATOR_TAU_S` would move several things at once and the result
    could not be attributed.
    """
    base = _peak_demand_slope(tmp_path, "f2base")
    moved = {
        sign * PINNED_TRIM_BAND_DEG: _peak_demand_slope(
            tmp_path, f"f2{sign}", pitch_bias_deg=sign * PINNED_TRIM_BAND_DEG
        )
        for sign in (-1, 1)
    }
    print(f"\n(f2) peak-demand slope at the shipped trim: {base:.3f} A/unit")
    for shift, slope in sorted(moved.items()):
        print(
            f"     trim shifted {shift:+.2f} deg: {slope:.3f} A/unit "
            f"({100 * (slope - base) / base:+.2f}%)"
        )

    for shift, slope in moved.items():
        moved_pct = 100 * abs(slope - base) / base
        assert moved_pct > 3.0, (
            f"shifting the trim {shift:+.2f} deg moved the peak-demand slope "
            f"only {moved_pct:.2f}%. If the slope has genuinely stopped "
            "depending on the trim then CMD_ENVELOPE_RESERVE is better founded "
            "than ADR-0011 claims and the ADR should be amended to say so -- "
            "but do not reach that conclusion by loosening this test."
        )


# ---------------------------------------------------------------------------
# Criterion (a): the named matrix, on the deployed signal path
# ---------------------------------------------------------------------------


def test_the_unshaped_command_map_no_longer_inverts_the_board_at_60a(tmp_path):
    """**INVERTED as of the 60 A / 42 N*m envelope (issue:
    realistic-motor-torque) -- this was the positive control for the whole
    matrix below, asserting the board MUST still invert unshaped.**

    At 40 A this reproduced ADR-0011's own numbers exactly: first saturation
    at 4.92 s, `FALLEN` (20 deg) at 5.868 s, past 90 deg at 6.142 s (reproduced
    against the pre-change checkout as part of this re-derivation). At 60 A,
    on the SAME scripted schedule, on the SAME estimator signal path, with the
    reserve fully disabled (`--cmd-reserve 1.0`, i.e. unshaped): the board
    never saturates, never reaches `FALLEN`, and peaks at 10.64 deg over the
    full run. `CMD_ENVELOPE_RESERVE` (see `host.rs`) has therefore saturated
    to 1.00 -- no shaping -- and this is not a bug in the harness, it is the
    reason that constant saturated: there is no over-command left to shape.

    This test used to be the harness's own positive control -- if it stopped
    seeing the ADR-0011 defect, every "did not invert" assertion below it was
    suspect. It is kept, inverted, for the same reason: a matrix of "does not
    invert" assertions is only meaningful with an explicit statement of what,
    if anything, still does. On the ESTIMATOR path, at this envelope, nothing
    in this file's schedules does any more -- see
    `test_criterion_f_full_stick_holds_with_the_estimator_bias_removed` for
    the TRUTH-fed reading, which still fails.
    """
    rows = _run(tmp_path, "unshaped", "full-stick", cmd_reserve=1.0, pitch_source="estimator")
    t = _inversion_time(rows)
    assert t is None, (
        f"full stick with the reserve disabled INVERTED at {t} s. Either the "
        "60 A / 42 N*m envelope regressed back toward the 40 A finding, or "
        "this needs re-reading against the plant/gains that were current when "
        "this was written."
    )
    print("\nunshaped full stick (60 A) survives the full hold -- ADR-0011's founding defect is gone on this path")


def test_criterion_a1_full_stick_from_rest_does_not_invert(tmp_path):
    rows = _run(tmp_path, "a1", "full-stick", pitch_source="estimator")
    assert _inversion_time(rows) is None, "full stick from rest inverted the board"
    print(f"\n(a)1 full stick from rest -- {_margin(rows)}")

    peak = _peak_demand_a(rows)
    bound = (_rust_constant("STATED_ENVELOPE_RESERVE_FRACTION") + 0.01) * MAX_CURRENT_A
    assert peak <= bound, (
        f"peak demand {peak:.2f} A exceeds the stated reserve's {bound:.2f} A -- "
        "the reserve is not delivering the headroom it is derived to deliver"
    )
    assert _peak_pitch_deg(rows) < PITCH_CEILING_DEG


def test_criterion_a2_a_full_stick_reversal_at_speed_does_not_invert(tmp_path):
    """The entry ADR-0011 records as NEVER TESTED, and names as the worst case.

    It is the worst case for a reason the speed cap makes structural: the cap
    only withdraws authority when stick and motion share a sign, so a reversal
    passes through at FULL authority at any speed -- including above the
    8.34 m/s onset, where the cap is the only thing that was unloading the
    board. Commanded deceleration and gravity then load the same side.

    The schedule runs it in both directions in one deterministic run; the
    ADR names the reverse-to-forward one and the forward-to-reverse one is
    its entry condition.

    **NOISELESS ONLY -- read together with
    `test_criterion_a2_the_reversal_holds_across_a_seeded_imu_noise_
    distribution` below.** This run is deterministic and cheap, and it is
    still the cleanest way to see a regression on this exact scenario move.
    It is NOT, on its own, a certified margin: this scenario sits on a
    chaotic boundary (issue: statistical-acceptance), and the seeded test
    below is what actually measures whether the margin implied by "did not
    invert" here is real.
    """
    rows = _run(tmp_path, "a2", "stick-reversal", pitch_source="estimator")
    assert _inversion_time(rows) is None, "a full stick reversal at speed inverted the board"
    print(f"\n(a)2 full stick reversal at speed, noiseless -- {_margin(rows)}")
    assert _peak_pitch_deg(rows) < PITCH_CEILING_DEG


def test_criterion_a2_the_reversal_holds_across_a_seeded_imu_noise_distribution(tmp_path):
    """The seeded-distribution reading of criterion (a)-2 -- issue:
    statistical-acceptance.

    `test_criterion_a2_a_full_stick_reversal_at_speed_does_not_invert` above
    is a single NOISELESS point, and this scenario sits on a chaotic
    boundary: that noiseless run does not reach a full inversion, but its
    peak lean already sits a few percent over `PITCH_CEILING_DEG` -- a
    result that reads as "holds, barely". It is not a real margin. Re-run
    the IDENTICAL scenario through the datasheet-derived IMU noise model
    (`IMU_NOISE_DATASHEET_V1`, `crates/sim-backend/src/imperfections.rs`) at
    each of `sim.scenarios.seeded_acceptance.SEEDS` and the board inverts
    fully, EVERY TIME -- see that module's own docstring for the measured
    25/25 result and for why N = 25.

    A binary pass/fail on the noiseless point above was measuring which side
    of the knife's edge that one trajectory happened to land on, not the
    controller's margin. This is the measurement that was missing: the pass
    condition is 0 inversions out of N, full stop -- ADR-0011 criterion (a)
    is a hard safety criterion, and a chaotic boundary does not earn a rate
    tolerance, only an honest report of where the rate actually sits.
    """
    dist = run_seeded(
        tmp_path, "a2seed", "stick-reversal", STATE_OUT, INPUT_IN, pitch_source="estimator"
    )
    print(f"\n(a)2 full stick reversal at speed, seeded IMU noise -- {dist.summary()}")
    assert dist.inversions == 0, (
        f"{dist.inversions}/{dist.n} seeded IMU-noise runs inverted the board on "
        f"the IDENTICAL (a)-2 scenario -- {dist.summary()}. A hard safety "
        "criterion is 0 inversions out of N, full stop; this is not a rate to "
        "tolerate, it is the finding: the noiseless run's apparent margin above "
        "is not real."
    )


def test_the_worst_point_in_the_matrix_is_quoted_in_both_currencies(tmp_path):
    """Criterion (b): margin as a measured number, at the worst matrix point.

    Deliberately re-derived across the matrix rather than asserted from one
    scenario, so "the worst point" cannot quietly become "the point somebody
    happened to measure". No threshold beyond "there is some headroom left" --
    the number is the deliverable; a pass/fail bound on it would be a target,
    and ADR-0011 forbids tuning to targets.
    """
    matrix = {
        "full stick from rest": _run(tmp_path, "w1", "full-stick", pitch_source="estimator"),
        "full stick reversal at speed": _run(tmp_path, "w2", "stick-reversal", pitch_source="estimator"),
    }
    worst = min(matrix, key=lambda k: MAX_CURRENT_A - _peak_demand_a(matrix[k]))
    print(f"\nWORST POINT IN THE MATRIX: {worst} -- {_margin(matrix[worst])}")
    assert _peak_demand_a(matrix[worst]) < MAX_CURRENT_A
    assert _peak_pitch_deg(matrix[worst]) < PITCH_CEILING_DEG


# ---------------------------------------------------------------------------
# Criterion (c): the loss-of-authority warning
# ---------------------------------------------------------------------------


def test_criterion_c_the_warning_leads_the_outcome_where_fallen_trails_it(tmp_path):
    """The warning must fire BEFORE the board is committed. `FALLEN` does not.

    **Reference scenario CHANGED at the 60 A / 42 N*m envelope (issue:
    realistic-motor-torque).** ADR-0011's own numbers, and this test's
    original reference event (first envelope saturation at 4.92 s on
    `--scripted-scenario full-stick --cmd-reserve 1.0 --pitch-source
    estimator`), were measured at 40 A. At 60 A that exact run never
    saturates, never warns and never falls at all -- see
    `test_the_unshaped_command_map_no_longer_inverts_the_board_at_60a`. There
    is no "committed point" left on the estimator path for the warning to
    lead.

    The TRUTH-fed full-stick hold still commits (criterion (f) still fails,
    see `test_criterion_f_full_stick_holds_with_the_estimator_bias_removed`),
    so that is the reference scenario here instead -- the warning mechanism
    (criterion (c)) is still live and still worth measuring, just no longer
    on the path this test originally used.

    Lead is asserted against a floor rather than pinned to a value, because
    the number is a measurement and pinning it would make an improvement look
    like a regression; the measured value is printed either way.
    """
    rows = _run(tmp_path, "warn", "full-stick", cmd_reserve=1.0, pitch_source="truth")
    t_sat = _first(rows, lambda r: r["saturated"] > 0.5)
    t_warn = _first(rows, lambda r: r["authority_warning"] > 0.5)
    t_fallen = _first(rows, lambda r: r["fallen"] > 0.5)
    assert t_sat is not None and t_warn is not None and t_fallen is not None

    print(
        f"\n(c) warning {t_warn:.3f} s | saturation {t_sat:.3f} s | FALLEN {t_fallen:.3f} s"
        f"\n    warning lead over saturation: {t_sat - t_warn:+.3f} s"
        f"\n    FALLEN  lead over saturation: {t_sat - t_fallen:+.3f} s"
        f"\n    warning lead over FALLEN:     {t_fallen - t_warn:+.3f} s"
    )
    assert t_sat - t_warn > 1.5, "the warning no longer leads the committed point"
    assert t_sat - t_fallen < 0.0, (
        "FALLEN now leads saturation, which would mean the premise of this "
        "criterion has changed"
    )
    assert t_fallen - t_warn > 2.5


def test_the_warning_stays_quiet_on_a_survivable_run(tmp_path):
    """A warning that fires on the shipped configuration's ordinary full-stick
    acceleration is noise, and noise is how a warning gets ignored."""
    rows = _run(tmp_path, "quiet", "full-stick", pitch_source="estimator")
    assert _first(rows, lambda r: r["authority_warning"] > 0.5) is None


# ---------------------------------------------------------------------------
# The instrument criterion (f) depends on
# ---------------------------------------------------------------------------


def test_the_truth_pitch_rate_is_the_derivative_of_the_truth_pitch(tmp_path):
    """Criterion (f) feeds truth pitch AND truth pitch rate to the regulator's
    D term. A wrong rate there would invert the damping and be indistinguishable
    from a physics finding, so it is checked against a signal that shares none
    of its machinery: the numerical derivative of the reported pitch."""
    rows = _run(tmp_path, "rate", "full-stick", pitch_source="estimator")
    dt = rows[1]["sim_time_s"] - rows[0]["sim_time_s"]
    worst = 0.0
    for a, b in zip(rows[:-1], rows[1:]):
        numeric = (b["truth_pitch_deg"] - a["truth_pitch_deg"]) / dt
        worst = max(worst, abs(numeric - b["truth_pitch_rate_deg_s"]))
    print(f"\ntruth pitch rate vs differenced pitch: worst {worst:.4f} deg/s")
    assert worst < 1.0


def test_the_estimator_residual_is_specific_force_not_a_static_bias(tmp_path):
    """WHY CRITERION (f) WAS MIS-SPECIFIED, as a measurement.

    ADR-0011's first ratification described the `est - truth` gap as a "~1 deg
    nose-down estimator bias" and implemented "remove the bias" as "feed the
    regulator truth". It is not a bias, and one identity settles it:

        1 radian per g = 57.2958 / 9.80665 = 5.8425 deg per m/s^2

    and ADR-0011's own geometric lean sensitivity -- how far this board must
    lean to sustain acceleration `a` -- is 5.84 deg per m/s^2. **Those are the
    same number because they are the same physics.** The lean an inverted
    pendulum needs to sustain `a` is `atan(a/g)`, and `atan(a/g)` is exactly
    the tilt of the apparent vertical a complementary filter reads off an
    accelerometer. The estimator is not carrying an error that flatters the
    controller; it is implementing the textbook balance-vehicle lean.

    Asserted here as the identity itself rather than as a fitted slope: the
    settled residual over `TRIM_WINDOW_S` against `atan(a_unaided/g)` over the
    same window, whose ratio is 1 if and only if the estimate is the apparent
    vertical. `_settled_trim` records why the fitted-slope form was the wrong
    instrument.

    The consequence is structural, not cosmetic: the estimate carries an
    acceleration reference, so replacing it with truth does not correct an
    error, it deletes a feedback path and leaves a pitch-only regulator. That
    is why the criterion (f) result below is a continuum in time-to-inversion
    rather than a threshold in stick.
    """
    rows = _run(tmp_path, "resid", "full-stick", pitch_source="estimator")
    residual, apparent = _settled_trim(rows)
    ratio = residual / apparent
    static_part = residual - apparent
    print(
        f"\nsettled trim over t in {TRIM_WINDOW_S} s:"
        f"\n    est-truth residual        {residual:+.4f} deg"
        f"\n    apparent vertical         {apparent:+.4f} deg  (= atan(a_unaided/g))"
        f"\n    ratio                     {ratio:.4f}   (1 rad/g holds iff this is 1)"
        f"\n    unexplained static part   {static_part:+.4f} deg "
        f"({100 * abs(static_part / apparent):.1f}% of the apparent-vertical term)"
    )

    # 5% is set by the instrument and by what the error MEANS, not by the
    # measured 2.8%: it is about 1.7x this measurement's own sensitivity to
    # the choice of settled window (the ratio moves over 1.009-1.039 across
    # reasonable windows on one run), and 5% of the apparent-vertical term at
    # this operating point is 0.12 deg of supplied lean -- half the smallest
    # static perturbation this repo has characterised, so a scale error large
    # enough to matter cannot hide inside it.
    assert abs(ratio - 1.0) < 0.05, (
        f"the est-truth residual is no longer the apparent vertical (ratio "
        f"{ratio:.3f}). ADR-0011's second ratification rests on that identity; "
        "if this has genuinely changed, the ADR needs revisiting again rather "
        "than this band being widened."
    )
    # A RELATIVE bound, because the claim under test is relative: the residual
    # is specific force RATHER THAN a static bias. 10% of the apparent-vertical
    # term is the largest static component that still leaves that sentence
    # true. Not derived from the measured value, which is 2.8%.
    assert abs(static_part) < 0.10 * abs(apparent), (
        f"a static component of {static_part:+.3f} deg against an "
        f"apparent-vertical term of {apparent:+.3f} deg would mean there IS a "
        "meaningful static bias after all, making the ADR's original framing "
        "partly right"
    )


def test_f1_the_estimator_trim_is_pinned_so_a_retune_cannot_move_it_silently(tmp_path):
    """ADR-0011 (f1), and the reason the second ratification exists.

    The board survives full stick because the estimator supplies the lean the
    acceleration physically requires. That is correct behaviour, but nothing
    designed it and until this test nothing held it in place: any change to
    `ESTIMATOR_TAU_S`, `ACCEL_FF_GAIN_M_S2_PER_A`, the IMU wiring or the
    regulator gains could move the trim, and the first symptom would be the
    board flipping again. (f1) exists to make that a RED BUILD instead.

    # What the band is, and what it is emphatically not

    `PINNED_TRIM_BAND_DEG` is a **drift detector**, not a tolerance. It does
    not assert that a trim anywhere inside it is safe, and no acceptance
    number anywhere in this repo may be justified by it.

    Its width comes from what a moved trim BREAKS, which is measured in
    `test_f2_the_peak_demand_slope_is_trim_derived_not_geometry_derived`:
    shifting the trim by 0.10 deg moves the peak-demand slope by about 5.2%,
    and that slope is the divisor `CMD_ENVELOPE_RESERVE` is derived from, held
    to a 5% band by
    `test_peak_current_demand_is_linear_in_stick_at_the_documented_slope`. So
    0.10 deg is the trim movement at which the shipped reserve stops being
    derived from anything true. The pin fires exactly where the derivation
    goes stale, which is the only place it has a principled reason to fire.

    It is worth stating what this band is NOT derived from. The measured
    static-error characterisation -- 0.25 deg survived, 0.5 deg inverted the
    board -- is **not a threshold**, and ADR-0011 is explicit that promoting
    0.25 deg to a bar would be deriving acceptance from whatever happened to
    pass, the precise move criterion (f) exists to forbid. Those numbers are
    used here only as a sanity check in the safe direction: the band that the
    derivation argument produced happens to sit well inside them, so the pin
    also cannot fail to fire before a trim reaches a value whose consequences
    are unknown. Had the derivation argument produced a wider band, the answer
    would have been to look harder, not to quietly borrow 0.25 deg.

    There is no lower bound worth arguing about. Every run here is
    bit-deterministic (`--free-run`, fixed-timestep RK4, schedule indexed on
    simulated time), so a band this size cannot chatter; it can only be moved
    by somebody actually changing the controller.

    # If this test fails

    Do not widen the band, and do not adjust the pinned value to match. The
    trim moved, which means `CMD_ENVELOPE_RESERVE`'s provenance moved with it
    (see `test_f2_...`) and ADR-0011's named blocking prerequisite -- the
    headroom-based fix -- has come due. Re-derive; do not re-baseline.
    """
    rows = _run(tmp_path, "f1trim", "full-stick", pitch_source="estimator")
    residual, apparent = _settled_trim(rows)
    print(
        f"\n(f1) pinned trim: {residual:+.4f} deg measured, "
        f"{PINNED_TRIM_DEG:+.3f} {chr(177)} {PINNED_TRIM_BAND_DEG:.2f} deg pinned"
    )
    assert abs(residual - PINNED_TRIM_DEG) < PINNED_TRIM_BAND_DEG, (
        f"the estimator trim has moved to {residual:+.4f} deg from the pinned "
        f"{PINNED_TRIM_DEG:+.3f} deg. ADR-0011's exit rests on the trim being "
        "frozen: the 0.80 command-envelope reserve was derived at THIS "
        "operating trim and is honest only for it, and the ADR names any "
        "retune that moves this band as a blocking prerequisite for the "
        "headroom-based fix. Re-derive the reserve from a re-measured slope; "
        "do not re-baseline this number to make the build green."
    )


# ---------------------------------------------------------------------------
# The criteria that do NOT pass. Written as they should read; xfail(strict).
# ---------------------------------------------------------------------------


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (f), literal reading: fed MuJoCo truth the board "
        "inverts at EVERY value of CMD_ENVELOPE_RESERVE down to 0.05 -- 4.42 s "
        "at 1.00, 5.95 s at the shipped 0.80, 28.67 s at 0.05, and only exactly "
        "zero stick survives. The constant buys TIME, not survival, so no "
        "input-shaping value can satisfy this. See "
        "test_the_estimator_residual_is_specific_force_not_a_static_bias for "
        "why the truth-fed loop is a different controller rather than a "
        "de-biased one. NOT a harness defect: the same harness reproduces the "
        "ADR's own saturation (4.92 s), FALLEN (5.87 s) and truth-fed delta "
        "(-1.74 s) figures."
    ),
)
def test_criterion_f_full_stick_holds_with_the_estimator_bias_removed(tmp_path):
    rows = _run(tmp_path, "f1", "full-stick", pitch_source="truth")
    t = _inversion_time(rows)
    assert t is None, f"fed MuJoCo truth, full stick from rest inverted at {t:.3f} s"


@pytest.mark.parametrize("bias_deg", [0.5])
def test_criterion_f_the_matrix_holds_under_a_worst_case_static_pitch_bias(tmp_path, bias_deg):
    """**XPASS as of the 60 A / 42 N*m envelope (issue: realistic-motor-torque)
    -- the `xfail(strict=True)` this carried is REMOVED, per this file's own
    stated policy: "the day somebody fixes the underlying problem these turn
    red as XPASS, which is the only mechanism that reliably tells a future
    session a known failure has stopped being one."**

    At 40 A this inverted "hard and stably" at 6.71 s and was unchanged by the
    braking-reserve change (issue #218/#219). At 60 A, the SAME +0.5 deg
    nose-up static pitch bias on the SAME worst-case scenario (a full stick
    reversal at speed) now survives. This is one point, not the whole
    robustness reading -- see `test_the_nose_down_static_bias_band_is_not_
    monotonic`, run in the same sweep, for the nose-down side (also now
    surviving everywhere it was swept, which is its own finding).

    **Do not read this as "criterion (f) now passes."** The LITERAL reading
    (`test_criterion_f_full_stick_holds_with_the_estimator_bias_removed`,
    fed MuJoCo truth) still inverts -- later (11.4 s vs 4.1 s at 40 A), but it
    still inverts, at every value the ADR swept. This test is the ROBUSTNESS
    reading the ADR was reaching for as a substitute, and it has flipped; the
    literal one has not.
    """
    rows = _run(tmp_path, f"f2{bias_deg}", "stick-reversal", pitch_source="estimator", pitch_bias_deg=bias_deg)
    t = _inversion_time(rows)
    assert t is None, f"a {bias_deg:+} deg static pitch bias inverted the board at {t:.3f} s"


def test_the_nose_down_static_bias_band_now_holds_everywhere_swept(tmp_path):
    """**INVERTED as of the 60 A / 42 N*m envelope (issue:
    realistic-motor-torque) -- read the history before trusting the name.**

    This test used to correct how the +-0.25 / +-0.5 characterisation read:
    at 40 A the pair was recorded as though the board simply survives a
    quarter of a degree of nose-down bias and inverts at a half, but the
    nose-down side actually had a knife-edge in it --

        bias      reserve 0.80    reserve 0.90
        -0.70 deg     held            held
        -0.60 deg     INVERTED        held
        -0.55 deg     INVERTED        held
        -0.50 deg     INVERTED        held
        -0.45 deg     INVERTED        INVERTED
        -0.40 deg     INVERTED        held

    -- a larger bias surviving where a smaller one inverted, not a tolerance.
    The test's own original assertion anticipated exactly the result below and
    named the risk explicitly: "a clean sweep is more likely to mean the
    harness stopped working." It has NOT: this file's positive controls
    reproduce the 40 A ADR-0011 numbers exactly on the same harness (see
    `test_the_unshaped_command_map_no_longer_inverts_the_board_at_60a`'s
    history), and the TRUTH-fed full-stick hold
    (`test_criterion_f_full_stick_holds_with_the_estimator_bias_removed`)
    still inverts on this exact build. The clean sweep below is real.

    At 60 A, every bias in this band now HOLDS -- the knife-edge did not just
    move, it disappeared from this band entirely. **This does not mean
    "the board is now robust to nose-down bias" as a general claim** -- it is
    one band, on one scenario (stick-reversal, estimator path), and the
    literal criterion (f) reading still fails on the truth-fed path. It does
    mean this specific, previously-documented non-monotonic knife-edge is not
    reproducible at this envelope, which is itself worth recording rather than
    silently losing when the band was widened or re-checked.
    """
    band = (-0.60, -0.50, -0.45, -0.40)
    outcomes = {}
    for bias in band:
        rows = _run(
            tmp_path, f"nm{bias}", "stick-reversal",
            pitch_source="estimator", pitch_bias_deg=bias,
        )
        outcomes[bias] = _inversion_time(rows)
    print("\nnose-down static bias, shipped configuration:")
    for bias, t in outcomes.items():
        print(f"  {bias:+.2f} deg: {'held' if t is None else f'INVERTED at {t:.2f} s'}")

    inverted = [b for b, t in outcomes.items() if t is not None]
    assert not inverted, (
        f"a nose-down bias in {inverted} now inverts the board again -- the "
        "60 A / 42 N*m envelope's clean sweep did not hold; re-check against "
        "the table in this test's docstring for whether the knife-edge is back"
    )


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (a) entry 3: full stick during a kerb strike. The "
        "board rides out roughly a 1 mm lip at a calm point of the hold and "
        "nothing at its worst point; a 20 mm lip -- brick edge, root heave, "
        "the kind of thing a pavement has every few metres (issue #142) -- is "
        "201 deg/s of imparted pitch rate against a KD channel that can afford "
        "about 76 deg/s at zero lean. This is NOT fixable by input shaping and "
        "is not what CMD_ENVELOPE_RESERVE was specified to fix; it is the "
        "measurement behind ADR-0011's own note that the BALLAST STROKE and "
        "CoM height, not the motor, are the undersized elements."
    ),
)
def test_criterion_a3_full_stick_during_a_kerb_strike_does_not_invert(tmp_path):
    """The strike magnitude comes from `sim/scenarios/plant.py`, not from here.

    `kerb_strike_impulse` is Sr. Mechanical & Systems' derivation (issue #142
    AC2) and stays the single source: this test reads the horizontal impulse
    and the pitch rate it imparts, and injects force and torque that deliver
    them. The moment arm is the axle-to-CoM distance the same function uses,
    so the injected angular impulse is `J * l` by construction rather than by
    coincidence.
    """
    # Struck mid-hold, at the speed the un-struck run is doing there.
    t0, height_m = 6.0, 0.020
    baseline = _run(tmp_path, "a3base", "full-stick", pitch_source="estimator")
    speed = next(
        abs(r["forward_speed_m_s"]) for r in baseline if abs(r["sim_time_s"] - t0) < 1e-3
    )
    strike = kerb_strike_impulse(height_m, speed)
    impulse_ns = strike["impulse_ns"][0]  # frictionless-normal LOWER bound
    force_n = impulse_ns / DISTURBANCE_WINDOW_S
    torque_nm = -(KERB_MOMENT_ARM_M * force_n)  # below the CoM => nose-down
    print(
        f"\n(a)3 kerb {height_m * 1000:.0f} mm at {speed:.2f} m/s: {impulse_ns:.1f} N*s, "
        f"{strike['pitch_rate_deg_s'][0]:.0f} deg/s imparted"
    )
    rows = _run(
        tmp_path, "a3", "full-stick", pitch_source="estimator",
        # forward is -X, so a retarding strike is +X
        disturbance=f"{t0},{DISTURBANCE_WINDOW_S},{force_n},0,0,0,{torque_nm},0",
    )
    t = _inversion_time(rows)
    assert t is None, f"a {height_m * 1000:.0f} mm lip at {speed:.1f} m/s inverted the board at {t:.3f} s"
