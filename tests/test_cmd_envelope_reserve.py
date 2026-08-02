"""ADR-0011's acceptance matrix, measured -- including the two entries it fails.

`docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`
holds the launch until the board stops inverting at full stick, and specifies
the fix as a **command-envelope reserve**: scale fore/aft stick by a derived
constant upstream of the estimator, the regulator and the safety envelope,
changing nothing else. This file is the evidence for that fix and, more
importantly, the evidence for where it stops working.

WHAT THIS FILE ASSERTS, AND WHAT IT DELIBERATELY MARKS xfail
------------------------------------------------------------
Three of ADR-0011's criteria pass on the DEPLOYED signal path and are asserted
here as ordinary tests: (b) the constant is derived and the margin is a
measured number, (a)-entry-1 full stick from rest, (a)-entry-2 the full stick
reversal at speed -- which the ADR records as never having been tested and
names as the worst case.

Two do not pass, at any value of the constant, and are written here as
`xfail(strict=True)`:

* **(a)-entry-3, the kerb strike.** During a full-stick hold the board cannot
  ride out a strike much above a 1 mm lip.
* **(f), "every pass must hold with the estimator bias removed".**

They are written as the criteria SHOULD read, and marked expected-to-fail,
rather than being softened into something that passes or left out. `strict=True`
matters: the day somebody fixes the underlying problem these turn RED as
XPASS, which is the only mechanism that reliably tells a future session a known
failure has stopped being one. A green suite that quietly stopped covering the
thing it was written for is the failure mode this repo keeps hitting.

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

DETERMINISM
------------
Every run is `--scripted-scenario ... --free-run`: the schedule is indexed on
simulated time (no wall clock, no socket, no staleness gate -- issue #190) and
the plant is a fixed-timestep RK4 integrator with no stochastic terms, so
these numbers are reproducible bit for bit and the whole matrix costs seconds
rather than minutes of real time.

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
#: duplication cannot drift silently.
MAX_CURRENT_A = 40.0
KT_NM_PER_A = 0.7
KP_NM_PER_RAD = 140.0

#: Total pitch authority, degrees: `MAX_CURRENT_A * KT / KP` = 0.2 rad. Every
#: "pitch headroom" number in this file is measured against this.
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
    assert slope > MAX_CURRENT_A, (
        "the premise of the whole change is that full stick over-commands the "
        f"envelope; measured {slope:.2f} A/unit against {MAX_CURRENT_A} A"
    )


# ---------------------------------------------------------------------------
# Criterion (a): the named matrix, on the deployed signal path
# ---------------------------------------------------------------------------


def test_the_unshaped_command_map_still_inverts_the_board(tmp_path):
    """The positive control, and it has to run FIRST in spirit.

    Every "the board did not invert" assertion below is worthless unless this
    harness can still see the defect ADR-0011 is about. With the reserve
    disabled (`--cmd-reserve 1.0`) the board must still go over, at the time
    the ADR measured.
    """
    rows = _run(tmp_path, "unshaped", "full-stick", cmd_reserve=1.0, pitch_source="estimator")
    t = _inversion_time(rows)
    assert t is not None, (
        "full stick with the reserve disabled did NOT invert the board. Either "
        "the plant, the gains or the harness has changed, and every acceptance "
        "number in this file is measuring something else."
    )
    print(f"\nunshaped full stick inverts at {t:.3f} s")


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
    """
    rows = _run(tmp_path, "a2", "stick-reversal", pitch_source="estimator")
    assert _inversion_time(rows) is None, "a full stick reversal at speed inverted the board"
    print(f"\n(a)2 full stick reversal at speed -- {_margin(rows)}")
    assert _peak_pitch_deg(rows) < PITCH_CEILING_DEG


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

    The reference event is the FIRST ENVELOPE SATURATION, which is what
    ADR-0011's own lead times are quoted against -- its table puts it at
    4.92 s, and its `FALLEN` figure of -0.95 s is exactly `FALLEN` minus that.
    Lead is asserted against a floor rather than pinned to a value, because
    the number is a measurement and pinning it would make an improvement look
    like a regression; the measured value is printed either way.
    """
    rows = _run(tmp_path, "warn", "full-stick", cmd_reserve=1.0, pitch_source="estimator")
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
    """WHY CRITERION (f) IS MIS-SPECIFIED, as a measurement.

    ADR-0011 describes the `est - truth` gap as a "~1 deg nose-down estimator
    bias" and implements "remove the bias" as "feed the regulator truth". It
    is not a bias. Regressed against the specific force the command feedforward
    failed to remove (`a_x - ACCEL_FF_GAIN * I`), it comes out at ~5.8 deg per
    m/s^2 -- and 1 radian per g is 57.2958/9.81 = 5.84 deg per m/s^2. That is
    a complementary filter reporting the APPARENT VERTICAL, which is what an
    accelerometer measures on a vehicle that accelerates.

    The consequence is structural, not cosmetic: the estimate carries an
    acceleration reference, so replacing it with truth does not correct an
    error, it removes a feedback path and leaves a pitch-only regulator. That
    is why the criterion (f) result below is a continuum in time-to-inversion
    rather than a threshold in stick.
    """
    rows = _run(tmp_path, "resid", "full-stick", cmd_reserve=0.6, pitch_source="estimator")
    accel_ff_gain = 0.0584  # ACCEL_FF_GAIN_M_S2_PER_A, host.rs
    half = 50  # +-0.1 s central difference, to see through per-cycle noise
    xs, ys = [], []
    for i in range(half, len(rows) - half):
        if not 1.0 <= rows[i]["sim_time_s"] <= 15.0:
            continue
        a_x = (rows[i + half]["forward_speed_m_s"] - rows[i - half]["forward_speed_m_s"]) / (
            rows[i + half]["sim_time_s"] - rows[i - half]["sim_time_s"]
        )
        xs.append(a_x - accel_ff_gain * rows[i]["applied_amps"])
        ys.append(rows[i]["est_pitch_deg"] - rows[i]["truth_pitch_deg"])
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    slope = (n * sum(x * y for x, y in zip(xs, ys)) - sx * sy) / (n * sum(x * x for x in xs) - sx * sx)
    intercept = (sy - slope * sx) / n
    one_rad_per_g = math.degrees(1.0) / 9.81
    print(
        f"\nest-truth residual vs unaided specific force: {slope:.2f} deg per m/s^2 "
        f"(1 rad/g = {one_rad_per_g:.2f}), intercept {intercept:+.3f} deg"
    )
    assert abs(abs(slope) - one_rad_per_g) / one_rad_per_g < 0.25, (
        "the est-truth residual no longer looks like an apparent-vertical "
        "reading. If that is real, ADR-0011 criterion (f)'s framing needs "
        "revisiting again, in the other direction."
    )
    assert abs(intercept) < 0.5, (
        "a large intercept would mean there IS a static bias component after "
        "all, which would make the ADR's framing partly right"
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


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (f), robustness reading -- the question the ADR "
        "was reaching for. A worst-case STATIC pitch-estimate error injected "
        "on top of the real signal path inverts the board at +-0.5 deg "
        "(survives +-0.25 deg). The tolerance band on the pitch estimate is "
        "therefore about a quarter of a degree, which is the number that "
        "matters most for hardware: an IMU mounted half a degree out flips "
        "this board. Both readings of criterion (f) fail, so the conclusion "
        "does not depend on which one is right."
    ),
)
@pytest.mark.parametrize("bias_deg", [-0.5, 0.5])
def test_criterion_f_the_matrix_holds_under_a_worst_case_static_pitch_bias(tmp_path, bias_deg):
    rows = _run(tmp_path, f"f2{bias_deg}", "stick-reversal", pitch_source="estimator", pitch_bias_deg=bias_deg)
    t = _inversion_time(rows)
    assert t is None, f"a {bias_deg:+} deg static pitch bias inverted the board at {t:.3f} s"


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
