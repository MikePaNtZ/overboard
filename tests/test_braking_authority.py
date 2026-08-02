"""Braking authority: what it buys, what it costs, and where the ceiling is.

The CEO drove the build and reported two things:

    "I would say you should be able to stop faster by leaning back. hopefully
    you can tune that up on the input side. but we dont need to overdo it. but
    in general i should be able to cut a tight turn by breaking and turning at
    same time while keeping speed."

This file is the measurement behind the answer to both.

THE CHANGE THIS PINS
--------------------
`CMD_ENVELOPE_RESERVE` = 0.80 was derived from a peak-demand slope measured on
the **forward full-stick hold** (ADR-0011). It was then applied to every
fore/aft command including braking, which was never derived -- the shaping was
one multiply and the sign never entered it. `CMD_ENVELOPE_RESERVE_BRAKING`
gives braking its own number, spent only when the stick opposes the motion.

WHY THE OPPOSITION TEST AND THE SPEED GATE BOTH MATTER
-------------------------------------------------------
"Aft stick gets more authority" would reintroduce ADR-0011's defect in mirror
image: full aft from rest is a backward standing start, and it over-commands
the envelope exactly as the forward one does. So the test is `stick * speed <
0` -- braking is a relationship, not a direction.

The speed gate then excludes the standing start proper. A full-stick launch
rocks the board backward to -0.085 m/s for six tenths of a second before it
moves off, and without the gate that transient draws the braking reserve --
silently changing ADR-0011's `full-stick` run, which is the run the estimator
trim and `PEAK_DEMAND_A_PER_UNIT_STICK` are both pinned against.
`test_the_pinned_adr_0011_runs_are_untouched_by_braking_authority` is the
guard, and it is the most important test in this file.

THE FINDING THAT MATTERS MORE THAN THE TUNING
----------------------------------------------
**The stop is lean-rate limited, not envelope limited.** Half a second after
the stick is slammed to full aft from 9.68 m/s, the board has produced about
3 A of braking out of 40 available; it takes ~2 s to reach 57% of the
envelope, and the peak demand of the whole schedule occurs 8 s later during
the reverse standing start, not during the stop at all.

That is why scaling the command buys so little: scaling raises the FINAL
lean, and the stopping distance is spent in the first second, at nearly zero
braking effort, at the highest speed. The dead time is structural -- the only
thing that leans this board is ballast mass being dragged fore/aft, and the
balance regulator actively resists it, because it is regulating pitch to zero
and has no lean setpoint.

The fix for that is already named in ADR-0011 and deliberately deferred:
criterion (f3), `theta_ref = atan(a_des / g)` as an explicit lean feedforward.
Nothing on the input side substitutes for it, and this file exists partly to
document that the input side has been tried and measured.

NOTHING HERE IS AN ACCEPTANCE THRESHOLD.
The stop times and distances are characterisations of current feel. ADR-0011
forbids deriving a bar from whatever happened to pass, and a stopping-distance
requirement would have to come from the game design, not from the vehicle.
"""

from __future__ import annotations

import csv
import math
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

HOST_RS = REPO / "crates" / "sim-host" / "src" / "host.rs"

STATE_OUT = "127.0.0.1:19605"
INPUT_IN = "127.0.0.1:19606"

INVERTED_DEG = 90.0
MAX_CURRENT_A = 40.0
KT_NM_PER_A = 0.7
KP_NM_PER_RAD = 140.0
PITCH_CEILING_DEG = math.degrees(MAX_CURRENT_A * KT_NM_PER_A / KP_NM_PER_RAD)

#: `ACCEPTANCE_SETTLE_S + BRAKE_BUILD_S` from `scenario.rs` -- when the brake
#: goes in on `brake-stop`/`brake-turn`.
BRAKE_T0_S = 12.5


def _rust_constant(name: str) -> float:
    import re

    m = re.search(rf"^const {name}: f32 = ([0-9.]+);", HOST_RS.read_text(), re.M)
    assert m is not None, f"{name} is not a plain f32 const in {HOST_RS}"
    return float(m.group(1))


def _run(tmp_path: Path, name: str, scenario: str, **kw) -> list[dict]:
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


def _healthy(rows):
    t = _inversion_time(rows)
    return [r for r in rows if t is None or r["sim_time_s"] < t]


def _at(rows, t):
    return min(rows, key=lambda r: abs(r["sim_time_s"] - t))


def _stop(rows):
    """`(entry_speed, time_to_stop, distance_to_stop)` from brake application."""
    dt = rows[1]["sim_time_s"] - rows[0]["sim_time_s"]
    v0 = _at(rows, BRAKE_T0_S)["forward_speed_m_s"]
    dist = 0.0
    for r in rows:
        if r["sim_time_s"] < BRAKE_T0_S:
            continue
        if r["forward_speed_m_s"] <= 0.0:
            return v0, r["sim_time_s"] - BRAKE_T0_S, dist
        dist += r["forward_speed_m_s"] * dt
    return v0, None, dist


def test_the_sim_host_binary_is_actually_built():
    assert (REPO / "target" / "release" / "sim-host").exists(), (
        "sim-host is not built, so this whole gate would be vacuous. Run:\n"
        "    cargo build --release -p sim-host --bin sim-host"
    )


# ---------------------------------------------------------------------------
# The guard that lets this change exist at all
# ---------------------------------------------------------------------------


def test_the_pinned_adr_0011_runs_are_untouched_by_braking_authority(tmp_path):
    """**The most important test in this file.**

    ADR-0011's second ratification pins the estimator trim and the
    `PEAK_DEMAND_A_PER_UNIT_STICK` slope, both measured on the `full-stick`
    hold, and names any retune that moves the trim band as a blocking
    prerequisite for the deferred headroom fix. A braking-authority change
    must therefore be provably orthogonal to that run -- not "probably fine",
    and not "the pin still passes", but bit-identical.

    Asserted at the extremes of the braking reserve's whole legal range. If
    the run were sensitive to it at all, 0.0 against 1.0 would show it.

    This is what the speed gate buys. Without it the launch transient -- the
    board rocks backward for six tenths of a second before moving off -- draws
    the braking reserve, and the trim moves for a manoeuvre containing no
    braking.
    """
    for scenario in ("full-stick",):
        lo = _run(tmp_path, f"{scenario}lo", scenario, cmd_reserve_braking=0.0)
        hi = _run(tmp_path, f"{scenario}hi", scenario, cmd_reserve_braking=1.0)
        assert [list(r.values()) for r in lo] == [list(r.values()) for r in hi], (
            f"the {scenario} acceptance run is no longer independent of the "
            "braking reserve. ADR-0011's trim pin and the reserve's own "
            "provenance slope are both measured on it, so this change is no "
            "longer orthogonal to them -- fix the gate, do not re-baseline."
        )


def test_cmd_reserve_alone_still_scales_the_whole_command(tmp_path):
    """`--cmd-reserve X` on its own must still mean "scale the WHOLE fore/aft
    command", including `--cmd-reserve 0` meaning "no stick at all".

    This is a regression test for a trap this change walked straight into.
    Every sweep written before braking had its own reserve assumes that
    meaning -- `PEAK_DEMAND_A_PER_UNIT_STICK`'s provenance sweep, and
    `tests/test_incline_tolerance.py`'s released-board runs, which use
    `--cmd-reserve 0` as the way to say "hands off". Once braking had a
    separate reserve that ignored the override, a "hands off" run started
    issuing a full braking command the instant the board rolled backwards on
    a slope, and the measured free-roll acceleration silently changed sign.

    So the braking override falls back to `--cmd-reserve` before it falls back
    to the shipped constant.
    """
    rows = _run(tmp_path, "zero", "full-stick", cmd_reserve=0.0, incline_deg=1.0)
    worst = max(rows, key=lambda r: abs(r["shaped_fore_aft"]))
    print(
        f"\n--cmd-reserve 0 on a 1 deg slope: largest shaped command "
        f"{worst['shaped_fore_aft']:+.6f}"
    )
    assert all(r["shaped_fore_aft"] == 0.0 for r in rows), (
        "--cmd-reserve 0 no longer means zero stick. A run that asked for no "
        "command is issuing one, and every measurement that uses this idiom to "
        "mean 'hands off' is now measuring a commanded board."
    )


def test_the_braking_reserve_is_not_spent_on_a_standing_start(tmp_path):
    """A stick opposing motion below the speed gate is a standing start.

    The failure this forbids is the ADR-0011 defect in mirror image: full aft
    from rest is a BACKWARD launch, and it over-commands the envelope exactly
    as the forward one does. The gate is what keeps "braking" meaning "shed
    speed you already have".

    Measured on the launch transient itself rather than argued: during the
    first second of a full-stick launch the board really is moving backward,
    and the shaped command must still be the accelerating reserve.
    """
    gate = _rust_constant("BRAKING_RESERVE_MIN_SPEED_M_S")
    reserve = _rust_constant("CMD_ENVELOPE_RESERVE")
    rows = _run(tmp_path, "standing", "full-stick")
    opposing = [
        r for r in rows
        if r["stick_fore_aft"] * r["forward_speed_m_s"] < 0.0
        and abs(r["forward_speed_m_s"]) < gate
    ]
    assert opposing, (
        "the launch transient no longer produces a backward-moving board under "
        "forward stick, so this test is not measuring what it says it is"
    )
    worst = max(opposing, key=lambda r: abs(r["shaped_fore_aft"]))
    print(
        f"\n{len(opposing)} cycles with the stick opposing motion below the "
        f"{gate} m/s gate; largest shaped command {worst['shaped_fore_aft']:+.4f}"
    )
    assert abs(worst["shaped_fore_aft"]) <= abs(worst["stick_fore_aft"]) * reserve + 1e-6, (
        "a command below the speed gate was shaped with the braking reserve"
    )


# ---------------------------------------------------------------------------
# What the tuning buys, and what it costs
# ---------------------------------------------------------------------------


def test_braking_authority_buys_a_shorter_stop_and_costs_reversal_headroom(tmp_path):
    """Both halves of the trade, in one place, because neither means anything
    without the other.

    The costed entry is ADR-0011 criterion (a)'s second matrix point -- the
    full reverse-to-forward stick reversal at speed, which the ADR names as
    the worst case. It is a braking event by the opposition test, so it spends
    this same reserve. **The two cannot be separated:** braking hard from
    speed IS the load case, so every metre of stopping distance is bought out
    of that entry's margin.

    No pass/fail bound on the stopping numbers -- they are the deliverable,
    and a bound on them would be a target. The assertions are only that the
    trade still points the way the shipped constant was chosen on: braking
    authority must still shorten the stop, and the worst matrix point must
    still hold with margin in BOTH currencies ADR-0011 quotes.
    """
    shipped = _rust_constant("CMD_ENVELOPE_RESERVE_BRAKING")
    accel = _rust_constant("CMD_ENVELOPE_RESERVE")

    print(f"\nBUYS -- stop from cruise (shipped braking reserve {shipped})")
    print(f"{'braking':>9} {'t_stop':>8} {'distance':>9}")
    stops = {}
    for br in (accel, shipped):
        rows = _run(tmp_path, f"stop{br}", "brake-stop", cmd_reserve_braking=br)
        v0, t_stop, dist = _stop(rows)
        stops[br] = (t_stop, dist)
        assert t_stop is not None, f"the board never stopped at braking reserve {br}"
        print(f"{br:9.2f} {t_stop:8.3f} {dist:9.2f}   (entry speed {v0:.3f} m/s)")
    gain_t = 100 * (stops[accel][0] - stops[shipped][0]) / stops[accel][0]
    gain_d = 100 * (stops[accel][1] - stops[shipped][1]) / stops[accel][1]
    print(f"  shipped vs the old symmetric reserve: {gain_t:.1f}% quicker, {gain_d:.1f}% shorter")
    assert stops[shipped][0] < stops[accel][0], (
        "the braking reserve no longer shortens the stop, which is the only "
        "thing it exists to do"
    )

    print("\nCOSTS -- ADR-0011's worst matrix point (stick reversal at speed)")
    print(f"{'braking':>9} {'peak A':>8} {'A left':>8} {'peak lean':>10} {'deg left':>9}")
    for br in (accel, shipped):
        rows = _run(tmp_path, f"rev{br}", "stick-reversal", cmd_reserve_braking=br)
        h = _healthy(rows)
        pk = max(abs(r["proposed_amps"]) for r in h)
        pl = max(abs(r["truth_pitch_deg"]) for r in h)
        print(f"{br:9.2f} {pk:8.2f} {MAX_CURRENT_A - pk:8.2f} {pl:10.2f} "
              f"{PITCH_CEILING_DEG - pl:9.2f}")
        assert _inversion_time(rows) is None, (
            f"the stick reversal inverted the board at braking reserve {br}"
        )
        assert pk < MAX_CURRENT_A, f"peak demand {pk:.2f} A reached the envelope"
        assert pl < PITCH_CEILING_DEG, (
            f"peak lean {pl:.2f} deg reached the {PITCH_CEILING_DEG:.2f} deg pitch "
            "ceiling -- the board has no pitch authority left at the worst point "
            "in ADR-0011's matrix. Lower CMD_ENVELOPE_RESERVE_BRAKING."
        )


def test_the_stop_is_lean_rate_limited_not_envelope_limited(tmp_path):
    """The finding that says the input side is nearly tapped out.

    If the stop were envelope limited, the board would be sitting at the
    current ceiling for most of it and more command would buy nothing. It is
    the opposite: for the first half-second of a maximum-effort stop the board
    produces almost no braking at all, because the only thing that leans it is
    ballast mass being dragged against a regulator that is holding pitch to
    zero.

    Recorded as an assertion so that the day somebody implements ADR-0011's
    deferred (f3) lean feedforward, this test turns red and says so -- at
    which point the whole trade above should be re-derived, because braking
    would stop being rate limited and the reserve would stop being the lever.
    """
    rows = _run(tmp_path, "ratelimit", "brake-stop")
    early = _at(rows, BRAKE_T0_S + 0.5)
    late = _at(rows, BRAKE_T0_S + 2.0)
    print(
        f"\n0.5 s into a maximum-effort stop: {abs(early['proposed_amps']):.2f} A of "
        f"{MAX_CURRENT_A:.0f} ({100 * abs(early['proposed_amps']) / MAX_CURRENT_A:.0f}% "
        f"of envelope), lean {early['truth_pitch_deg']:.2f} deg, still doing "
        f"{early['forward_speed_m_s']:.2f} m/s"
        f"\n2.0 s in: {abs(late['proposed_amps']):.2f} A "
        f"({100 * abs(late['proposed_amps']) / MAX_CURRENT_A:.0f}%), "
        f"lean {late['truth_pitch_deg']:.2f} deg"
    )
    assert abs(early["proposed_amps"]) < 0.25 * MAX_CURRENT_A, (
        "the board now reaches a quarter of its envelope within half a second "
        "of a full braking command. If that is real, the stop is no longer "
        "rate limited and CMD_ENVELOPE_RESERVE_BRAKING's whole justification "
        "needs re-deriving -- see ADR-0011 (f3)."
    )


def test_braking_into_a_turn_is_stable_and_does_not_reach_the_envelope(tmp_path):
    """The CEO's second manoeuvre: brake and turn at the same time.

    Worth measuring rather than assuming, because it is a load case ADR-0011's
    straight-line matrix does not contain: yaw is injected kinematically, so
    the board is being turned while its pitch loop is at its hardest-working
    point.

    **What this test cannot tell you, and the answer to the other half of the
    report:** turn radius here is `1 / (steer * curvature * roll_authority)` --
    it has no speed term at all, so braking cannot tighten the turn by any
    amount. That is a property of the yaw model, not something a measurement
    reveals, and changing it is a change to a declared non-physical channel.
    Recorded here so the next reader does not go looking for it in the data.
    """
    rows = _run(tmp_path, "braketurn", "brake-turn")
    h = _healthy(rows)
    pk = max(abs(r["proposed_amps"]) for r in h)
    pl = max(abs(r["truth_pitch_deg"]) for r in h)
    print(
        f"\nbrake into a full-steer turn: peak demand {pk:.2f} A of {MAX_CURRENT_A:.0f}, "
        f"peak lean {pl:.2f} deg of {PITCH_CEILING_DEG:.2f}"
    )
    assert _inversion_time(rows) is None, "braking into a turn inverted the board"
    assert pk < MAX_CURRENT_A
    assert pl < PITCH_CEILING_DEG
