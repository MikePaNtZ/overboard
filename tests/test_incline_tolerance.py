"""The authored-terrain incline this board tolerates, MEASURED.

ADR-0011's second ratification makes the launch hold conditional on three
things, and the second is a world-authoring constraint:

    The authored world is constrained to what the controller survives, and the
    constraint is encoded as a checkable asset rule -- not a hope. [...]
    authored inclines must stay well inside the measured static tolerance (a
    0.5 deg slope is an effective static pitch disturbance).

An asset rule needs a number, and issue #207 says plainly that the number in
that sentence is a prediction rather than a measurement. This file is the
measurement. Its headline result is that **the prediction is wrong, and wrong
in a way that matters more than its magnitude.**

WHAT THE PREDICTION ASSUMED, AND WHY IT DOES NOT HOLD
-----------------------------------------------------
The prediction reasons: a slope is an effective static pitch disturbance; the
board inverts at 0.5 deg of static pitch error and survives 0.25 deg;
therefore it tolerates well under 0.5 deg of incline.

The middle step is the one that fails. A static pitch *estimate error* is a
signal the controller **cannot see** -- it regulates a lie, and the lean it
holds is wrong by that much forever. A slope is a signal the controller **can
see**: the IMU still measures true gravity, so the board's attitude estimate
is correct on a hill, and the slope shows up as an ordinary along-track force
the balance loop already knows how to lean against. The two are not the same
disturbance and they do not have the same tolerance.

Measured: the board does not invert at any incline in the swept range, which
reaches 24x the predicted 0.5 deg. Nothing in the ADR's acceptance matrix
inverts on a hill.

WHAT ACTUALLY BINDS, WHICH IS NOT INVERSION
--------------------------------------------
The board is a pitch regulator. It has no position loop and no speed loop --
`MAX_GROUND_SPEED_M_S` shapes the *stick*, and a stick cap cannot brake
against gravity. So on a slope:

* **Station-keeping is zero at every nonzero incline.** Released with no
  stick, the board rolls downhill and keeps accelerating, to 43 m/s on a
  15 deg grade inside one 18.5 s schedule, with nothing in the host
  intervening at any point. Measured acceleration is linear in `sin(phi)` at
  0.70 g from 0.25 to 15 deg, the reduction being the wheel's rotational
  inertia and the balancing lean. There is no incline small enough for a
  stationary board to stay put.
* **The board can arrest itself, but only up to a measured grade.** Under the
  largest sustained braking this host ever commands on its own -- 0.6 of full
  lean -- a downhill roll is arrested up to 6.5 deg and outrun at 7.0 deg.

So the constraint an asset rule needs is **not only an angle**. A slope that
is individually harmless produces an arbitrary speed if it is long enough,
and speed is the axis on which nothing else in this repo is characterised.
The rule needs an angle AND a run-out length, and this file measures the
inputs to both.

**A number this file deliberately does not give.** The steepest grade the
board can CLIMB at full stick is somewhere between 4 and 7 deg, and it is not
reported: the 18.5 s schedule is not long enough to tell a board that is
climbing slowly from one that is about to slide back, and a number produced
by squinting at that is worse than no number. It needs a longer scripted
schedule, which is its own increment. It is also the least safety-relevant of
the four -- a grade the board cannot climb is a stuck player, not a fall.

WHAT THIS FILE IS NOT
---------------------
None of these numbers is an acceptance threshold, and none may be promoted to
one. ADR-0011 forbids deriving a bar from whatever happened to pass -- that
is what criterion (f) exists to prevent -- and a tolerance requirement comes
from the environment (what the world author needs to build) rather than from
the vehicle (what it happened to survive). These are measured
characterisations, offered as the input the asset rule is written FROM. The
rule itself is the world-authoring role's to write, and it should sit inside
these numbers with whatever margin that role judges, not on them.

METHOD
------
A slope is applied by rotating `mjModel::opt.gravity` about `Y` at plant open
(`--incline-deg`, `sim_backend::SimBackend::set_incline_deg`). A flat ground
plane under gravity tilted by `phi` and a ground plane inclined by `phi` under
vertical gravity are the same rigid-body problem in two frames, so this is a
real slope rather than an approximation of one -- and unlike an external force
on one body it applies the along-slope component to every body's own mass.
Positive is uphill in the board's forward direction.
`test_the_incline_knob_is_a_slope_and_not_something_else` is the positive
control for all of it.

Every run is `--scripted-scenario ... --free-run`, so the whole sweep is
deterministic and costs seconds. THE BINARY MUST BE BUILT -- same rule as
`tests/test_cmd_envelope_reserve.py`, for the same reason.
"""

from __future__ import annotations

import csv
import math
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

#: Ports well away from both the documented 9601/9602 and the ones
#: `test_cmd_envelope_reserve.py` uses, so a parallel run cannot collide.
STATE_OUT = "127.0.0.1:19603"
INPUT_IN = "127.0.0.1:19604"

#: Pitch beyond which the board is unambiguously going over. Measured against
#: the SLOPE normal, which is what MuJoCo reports under this transform; at the
#: inclines swept here the difference from true vertical is irrelevant at 90
#: deg.
INVERTED_DEG = 90.0

#: `overboard_rider.xml`'s own `<option gravity>` magnitude. Used only to
#: predict the free-roll acceleration this file checks the slope against, so
#: it must be the MODEL's value and not standard gravity.
G_MODEL_M_S2 = 9.81

#: Where `SPEED_CAP_MARGIN_M_S` starts withdrawing accelerating authority
#: (`host.rs`: `MAX_GROUND_SPEED_M_S` 9.34 - `SPEED_CAP_MARGIN_M_S` 1.0).
#: Used here only as the reference speed for the free-roll run-out figures --
#: the first speed at which the board is outside its shaped envelope. The cap
#: itself scales the STICK and so does nothing on a released run.
SPEED_CAP_ONSET_M_S = 8.34

#: The scripted schedule runs 15.5 s plus a 3 s tail. Speed trend is taken
#: over the last 2 s of that.
RUN_END_S = 18.5
TREND_WINDOW_S = 2.0


def _run(tmp_path: Path, name: str, scenario: str, **kw) -> list[dict]:
    """One deterministic free run; returns the per-cycle trace.

    Through `cargo run` rather than the raw binary for the reason
    `test_cmd_envelope_reserve.py` documents: Cargo puts the linked
    libmujoco's `OUT_DIR` on the dynamic loader path.
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


def _at(rows, t):
    return min(rows, key=lambda r: abs(r["sim_time_s"] - t))


def _released(tmp_path, name, incline_deg):
    """Board released on a slope with the stick at zero."""
    return _run(tmp_path, name, "full-stick", incline_deg=incline_deg, cmd_reserve=0.0)


def test_the_sim_host_binary_is_actually_built():
    assert (REPO / "target" / "release" / "sim-host").exists(), (
        "sim-host is not built, so this whole gate would be vacuous. Run:\n"
        "    cargo build --release -p sim-host --bin sim-host"
    )


# ---------------------------------------------------------------------------
# The positive control: is `--incline-deg` really a slope?
# ---------------------------------------------------------------------------


def test_the_incline_knob_is_a_slope_and_not_something_else(tmp_path):
    """Everything below is worthless if this knob does something other than
    what it claims, and "I rotated the gravity vector" is exactly the kind of
    change that can look plausible while being wrong by a sign or a frame.

    Three independent checks, none of which can pass by accident:

    1. **Zero is a no-op.** Not merely "flat", but bit-identical to a run that
       never passed the flag -- the gravity vector is left alone rather than
       rewritten with its own value.
    2. **A released board accelerates downhill at `g sin(phi)`, times a
       constant.** The constant is below 1 because the wheel's rotational
       inertia and the balancing lean both resist, and it must be the SAME
       constant at every angle -- proportionality to `sin(phi)` over a 40x
       range is a property a mis-scaled or mis-framed transform does not have.
    3. **The sign is right.** Positive is uphill in the forward direction, so
       a released board rolls BACKWARD.
    """
    flat = _run(tmp_path, "ctrl_flat", "full-stick", incline_deg=0.0)
    unset = _run(tmp_path, "ctrl_unset", "full-stick")
    assert [list(r.values()) for r in flat] == [list(r.values()) for r in unset], (
        "--incline-deg 0 is not bit-identical to omitting the flag, so the "
        "knob perturbs a flat run and every other test in this repo is now "
        "measuring something slightly different"
    )

    print("\nfree-roll check -- a released board on a slope:")
    ratios = []
    for phi in (0.25, 1.0, 5.0, 10.0):
        rows = _released(tmp_path, f"ctrl{phi}", phi)
        v0, v1 = _at(rows, 0.5), _at(rows, 4.0)
        dv = v1["forward_speed_m_s"] - v0["forward_speed_m_s"]
        measured = dv / (v1["sim_time_s"] - v0["sim_time_s"])
        free_roll = G_MODEL_M_S2 * math.sin(math.radians(phi))
        ratios.append(abs(measured) / free_roll)
        print(
            f"  {phi:5.2f} deg: {measured:+.4f} m/s^2 measured, "
            f"g sin(phi) = {free_roll:.4f}, ratio {abs(measured) / free_roll:.4f}"
        )
        assert measured < 0, (
            f"at +{phi} deg (uphill forward) the board should roll BACKWARD; "
            f"measured acceleration {measured:+.4f} m/s^2. The sign of the "
            "gravity rotation is wrong."
        )

    spread = max(ratios) - min(ratios)
    print(f"  ratio spread across a 40x range of angle: {spread:.4f}")
    assert spread < 0.02, (
        f"acceleration is not proportional to sin(phi) (ratios {ratios}). "
        "Whatever --incline-deg is doing, it is not applying a uniform "
        "gravitational field, and no number in this file means anything."
    )
    assert 0.5 < min(ratios) < 1.0, (
        f"a released board accelerates at {min(ratios):.3f} of g sin(phi). "
        "Above 1 is unphysical for a body with rotational inertia; far below "
        "it would mean something is braking that should not be."
    )


# ---------------------------------------------------------------------------
# Inversion: the thing the prediction was about, and the thing that does not
# turn out to bind
# ---------------------------------------------------------------------------


def test_adr_0011s_acceptance_matrix_does_not_invert_on_any_swept_incline(tmp_path):
    """The prediction was "under 0.5 deg". Measured, nothing inverts at 24x it.

    Both deployed-path entries of ADR-0011's criterion (a) matrix are re-run
    across the sweep. The kerb-strike entry is not: it fails on flat ground
    already (`tests/test_cmd_envelope_reserve.py` records it as a strict
    xfail), so running it on a hill would measure nothing new.

    Asserted as "no inversion anywhere in the swept range" rather than as a
    bisected boundary. A boundary here would be a number begging to be read as
    a tolerance, and this file's whole point is that inversion is not what
    binds -- see `test_a_released_board_does_not_stay_put_on_any_incline`.
    """
    for scenario, sweep in (
        ("full-stick", (-12.0, -8.0, -4.0, -1.0, 0.0, 1.0, 4.0, 8.0, 12.0)),
        ("stick-reversal", (-8.0, -4.0, -2.0, -0.5, 0.0, 0.5, 2.0, 4.0, 8.0)),
    ):
        print(f"\n{scenario} on a slope (+ uphill):")
        for phi in sweep:
            rows = _run(tmp_path, f"m{scenario}{phi}", scenario, incline_deg=phi)
            t = _inversion_time(rows)
            peak_pitch = max(abs(r["truth_pitch_deg"]) for r in rows)
            peak_amps = max(abs(r["proposed_amps"]) for r in rows)
            print(
                f"  {phi:+6.1f} deg: {'held' if t is None else f'INVERTED at {t:.2f} s'}, "
                f"peak lean {peak_pitch:5.2f} deg, peak demand {peak_amps:5.2f} A"
            )
            assert t is None, (
                f"{scenario} inverted at {phi:+} deg of incline. That is a "
                "real change in the world-authoring input and the number in "
                "this file's docstring is now wrong -- re-measure it, and tell "
                "whoever owns the asset rule."
            )


# ---------------------------------------------------------------------------
# What actually binds
# ---------------------------------------------------------------------------


def test_a_released_board_does_not_stay_put_on_any_incline(tmp_path):
    """Station-keeping tolerance is ZERO, and this is the finding that most
    changes what an asset rule has to say.

    The board has no position loop and no speed loop. `MAX_GROUND_SPEED_M_S`
    shapes the STICK, and a stick cap cannot brake against gravity, so a board
    left alone on any slope rolls away and keeps accelerating. The asset rule
    therefore cannot be written as "slopes up to X deg are fine": a 0.25 deg
    slope is fine for a metre and not fine for a kilometre.

    Run DOWNHILL-FORWARD (negative incline) on purpose. Rolling backwards the
    board leaves the drivable corridor after `CORRIDOR_X_MAX_M` = 50 m and the
    host applies `CORRIDOR_BRAKE_LEAN`, which would arrest the roll and make
    this test measure the corridor brake instead of the absence of a speed
    loop. Forward there are 700 m, which no run here comes close to using, and
    the test asserts directly that the host never intervened.

    The run-out distances printed alongside are kinematics from the measured
    acceleration, and are labelled derived rather than measured because the
    18.5 s schedule is far too short to drive them.
    """
    print("\nreleased on a downgrade, does it stay put?")
    print(f"{'incline':>8} {'a (m/s^2)':>10} {'v at end':>9} {'still gaining':>14} "
          f"{'run-out to cap':>15}")
    for phi in (-0.25, -0.5, -1.0, -3.0, -7.0, -15.0):
        rows = _released(tmp_path, f"stay{phi}", phi)
        assert all(r["applied_fore_aft"] == 0.0 for r in rows), (
            f"at {phi} deg the host applied a nonzero fore/aft command on a run "
            "whose stick is at zero -- the corridor brake or the speed cap has "
            "intervened, and this run is measuring that instead of free roll"
        )
        v0, v1 = _at(rows, 0.5), _at(rows, 4.0)
        a = abs(v1["forward_speed_m_s"] - v0["forward_speed_m_s"]) / (
            v1["sim_time_s"] - v0["sim_time_s"]
        )
        end = _at(rows, RUN_END_S)
        prev = _at(rows, RUN_END_S - TREND_WINDOW_S)
        gaining = abs(end["forward_speed_m_s"]) > abs(prev["forward_speed_m_s"])
        print(
            f"{phi:8.2f} {a:10.4f} {abs(end['forward_speed_m_s']):9.3f} "
            f"{str(gaining):>14} {SPEED_CAP_ONSET_M_S**2 / (2 * a):12.0f} m*"
        )
        assert gaining, (
            f"at {phi} deg the board stopped gaining speed with the stick at "
            "zero. Something now supplies station-keeping authority that did "
            "not before, and this file's central claim needs re-deriving."
        )
    print("  * distance from rest to speed-cap onset, derived from the measured")
    print("    acceleration -- NOT measured: the 18.5 s schedule is far too short.")


def test_the_downgrade_at_which_full_braking_stops_arresting_the_roll(tmp_path):
    """The steepest sustained downgrade the board can arrest itself on.

    Since nothing arrests a free roll (above), the useful number is the one
    that applies when the board *does* brake. This measures it using the
    host's own emergency authority: leave the drivable corridor and the host
    applies `CORRIDOR_BRAKE_LEAN` = 0.6 of full lean against the direction of
    travel, sustained, for the rest of the run. It is the largest sustained
    braking command this host ever issues on its own.

    **This is the corridor brake and not the speed cap**, which is worth
    stating because the two are easy to confuse here and the first version of
    this measurement confused them. The speed cap scales the stick; on a run
    whose stick is zero it has nothing to scale and contributes nothing. The
    arrest visible below is `applied_fore_aft` going to exactly 0.6, and the
    test asserts that is what happened rather than assuming it.

    Classification, stated before the numbers so it cannot be fitted to them:
    once the brake is applied, ARRESTED if speed is falling over the final
    `TREND_WINDOW_S`, OUTRUN if it is still rising.

    Deliberately NOT bisected finer than the 0.5 deg sweep. A more precise
    answer would be a number precise enough to be mistaken for a limit, which
    is exactly what this file must not produce.
    """
    print("\nrolling downhill under the host's own full braking:")
    print(f"{'incline':>8} {'v at end':>9} {'trend':>8}  verdict")
    arrested, outrun = [], []
    for phi in (4.0, 5.0, 5.5, 6.0, 6.5, 7.0, 8.0, 9.0):
        rows = _released(tmp_path, f"brake{phi}", phi)
        braking = [r for r in rows if r["applied_fore_aft"] != 0.0]
        assert braking, f"at {phi} deg the corridor brake never engaged"
        assert all(abs(abs(r["applied_fore_aft"]) - 0.6) < 1e-6 for r in braking), (
            "the sustained command here is no longer CORRIDOR_BRAKE_LEAN, so "
            "this test is measuring a different braking authority than it says"
        )
        end = _at(rows, RUN_END_S)
        prev = _at(rows, RUN_END_S - TREND_WINDOW_S)
        v_end = abs(end["forward_speed_m_s"])
        trend = (v_end - abs(prev["forward_speed_m_s"])) / TREND_WINDOW_S
        if trend > 0.0:
            verdict, _ = "OUTRUN", outrun.append(phi)
        else:
            verdict, _ = "arrested", arrested.append(phi)
        print(f"{phi:8.2f} {v_end:9.3f} {trend:+8.3f}  {verdict}")

    assert arrested and outrun, (
        "the sweep no longer straddles the boundary it exists to locate; "
        f"arrested={arrested}, outrun={outrun}. Widen the sweep and re-measure."
    )
    assert max(arrested) < min(outrun), (
        f"arrested and outrun inclines interleave (arrested={arrested}, "
        f"outrun={outrun}), so there is no single boundary and the asset rule "
        "cannot be written from one angle. That is a real finding, not a test "
        "defect -- report it rather than adjusting the sweep."
    )
    print(
        f"\nMEASURED, and not a threshold: at 0.6 of full lean the board "
        f"arrests a downhill roll up to {max(arrested):.1f} deg and is outrun "
        f"at {min(outrun):.1f} deg. Sweep resolution 0.5 deg."
    )
