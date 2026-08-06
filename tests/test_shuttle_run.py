"""Shuttle Run — commanded velocity, reversal, and return to home.

The impulse gate asks whether the board survives something done *to* it. This
asks whether it does what it is *told*, which fails in different ways: a lag,
a creep, a reversal that never happens, or a return-to-home that scores well
because the board barely moved.

The thresholds here are **not tight**. The outer loop is deliberately slow, so
the board trails its command by design; these bound the behaviour rather than
certify it, and they are expected to move once an estimator and an imperfection
profile are in the loop.

GATE NUMBERS -- MEASURED VS ASSUMED (issue #24 AC5 audit, this session)
-------------------------------------------------------------------------
The module docstring above already says these bounds are loose by design --
that is itself the "explicitly marked as an assumption" this audit asks for.
Re-run against this checkout to put an actual number next to each bound:

| Assertion                          | Threshold  | Observed this session | Status |
|-------------------------------------|------------|------------------------|--------|
| `test_it_returns_to_home`           | `< 0.30 m` | 0.233 m                | ASSUMED-but-loose -- the docstring above states outright these are not certified margins; 0.30 m was chosen as "not obviously broken," not derived from a stated worst case |
| `test_it_holds_station_during_the_pauses` | `< 0.40 m` | 0.198 m          | ASSUMED-but-loose, same basis |
| `test_it_never_strikes_and_never_saturates` (peak pitch) | `< 12.0 deg` | 7.825 deg | ASSUMED-but-loose, same basis |
| `test_the_pitch_reference_stays_off_its_clamp` | `< 4.5 deg` | 3.467 deg | ASSUMED-but-loose, same basis |
| `test_it_actually_goes_out_and_comes_back_past_home` | `> 1.5 m` out, `< -0.5 m` back | route legs are +2.0/-3.0/+1.0 m, so this is a geometric consequence of the route rather than a measured controller property | MEASURED (it is arithmetic on `DEFAULT_ROUTE`, not a sim result) |

None of the four physics-derived bounds were tightened here -- the module
docstring's "not tight, expected to move" framing already is the honest
disclosure this audit exists to require, and re-deriving tighter margins is
tuning work outside AC5's scope (an audit, not a re-tune). Recorded here so a
future tightening pass has today's actual numbers to start from instead of
re-measuring blind.

`test_it_holds_station_during_the_pauses` and `test_the_pitch_reference_
stays_off_its_clamp` are LEFT FAILING, NOT RE-PINNED, against the frozen
plant (drag: the derived three-term model; `MAX_CURRENT_A` 40 -> 60 A)
------------------------------------------------------------------------
`max_hold_drift_m` moved from 0.198 m to 1.724 m -- not a proportional drift
but a qualitative change. Plotting position during the "hold-out" pause
shows the board overshoots the +2.0 m target to a peak of ~3.46 m, then
swings all the way back down through the target to ~0.99 m before the next
leg starts pulling it further -- a single slow, large, lightly-damped
oscillation across the ~4 s hold window, not station-keeping creep. All
three hold windows in the route show the same pattern (drift 0.96-1.73 m).
`peak_abs_pitch_ref_deg` (4.98 vs the 4.5 deg bound, 3.467 deg previously
observed) is the smaller of the two failures in absolute terms but is the
same finding: the pitch reference has to swing further to drive that larger
position overshoot. This reads as the outer velocity/position loop -- tuned
against the 40 A plant -- now underdamped at 60 A, not as either bound
having been loosely chosen (which the table above already discloses for
other reasons). Left failing and reported rather than re-pinned, since
raising `max_hold_drift_m` by 8.7x would hide a real outer-loop damping
regression rather than record a measurement.
"""

import pytest

from sim.scenarios.shuttle_run import Leg, ShuttleParams, VelocityProfile, run


@pytest.fixture(scope="module")
def result():
    return run()


# --------------------------------------------------------------------------
# The profile itself, before any physics
# --------------------------------------------------------------------------

def test_the_route_integrates_to_zero_net_displacement():
    """Return-to-home is a property of the COMMAND, not of a position
    controller. If the route stopped summing to zero, the board finishing at
    the start would mean the controller was ignoring it."""
    assert sum(l.distance_m for l in ShuttleParams().route) == pytest.approx(0.0)


def test_the_profile_commands_both_directions():
    """A route that only ever drove forward could be passed by a controller
    with an inverted sign."""
    p = ShuttleParams()
    prof = VelocityProfile(p)
    samples = [prof.v_ref(t * 0.05) for t in range(int(prof.duration_s / 0.05))]
    assert max(samples) > 0.4, "expected a forward leg"
    assert min(samples) < -0.4, "expected a reverse leg"


def test_the_profile_starts_and_ends_at_rest():
    prof = VelocityProfile(ShuttleParams())
    assert prof.v_ref(0.0) == pytest.approx(0.0)
    assert prof.v_ref(prof.duration_s - 0.01) == pytest.approx(0.0)


def test_a_leg_too_short_to_reach_cruise_does_not_overshoot_its_distance():
    """Guards the triangular-profile branch: a short hop must still command
    the distance asked for, not whatever a full trapezoid would have covered."""
    p = ShuttleParams(route=(Leg("hop", distance_m=0.15),))
    prof = VelocityProfile(p)
    dt = 0.001
    travelled = sum(prof.v_ref(i * dt) * dt for i in range(int(prof.duration_s / dt)))
    assert travelled == pytest.approx(0.15, abs=0.03)


# --------------------------------------------------------------------------
# The run
# --------------------------------------------------------------------------

def test_it_actually_goes_out_and_comes_back_past_home(result):
    """The manoeuvre happened at all.

    Paired with the return-to-home assertion below: without this, a board that
    never moved would score a perfect return error.
    """
    m = result.metrics
    assert m.max_forward_m > 1.5, f"only reached {m.max_forward_m:.2f} m outbound"
    assert m.min_forward_m < -0.5, f"only reached {m.min_forward_m:.2f} m on the return"


def test_it_returns_to_home(result):
    """The headline: closed-loop position accuracy over a 6 m round trip."""
    assert result.metrics.return_error_m < 0.30, (
        f"finished {result.metrics.return_error_m * 100:.0f} cm from home"
    )


def test_it_holds_station_during_the_pauses(result):
    """Measured over the settled half of each pause, so this is creep rather
    than the tail of the previous leg's deceleration.

    NOT RE-PINNED against the frozen plant -- left failing as a genuine
    finding. See the file docstring's dedicated section: this is now a large,
    slow position oscillation across the hold window, not creep.
    """
    assert result.metrics.max_hold_drift_m < 0.40


def test_it_never_strikes_and_never_saturates(result):
    m = result.metrics
    assert not m.nose_strike
    assert m.saturated_cycles == 0
    assert m.peak_abs_pitch_deg < 12.0


def test_the_pitch_reference_stays_off_its_clamp(result):
    """A reference pinned at the clamp is the signature of an outer loop that
    has run out of authority — how E05's divergence on the driverless board
    announced itself. On the ridden plant it should have room to spare.

    NOT RE-PINNED against the frozen plant -- left failing as a genuine
    finding, the same one as `test_it_holds_station_during_the_pauses`. See
    the file docstring's dedicated section.
    """
    assert result.metrics.peak_abs_pitch_ref_deg < 4.5


def test_the_board_trails_its_command_rather_than_leading_it(result):
    """Sanity on the sign of the lag.

    A slow outer loop should always be *behind* the commanded trajectory. If
    the board were consistently ahead, the command is not what is driving it.
    """
    import numpy as np

    moving = np.abs(result.v_ref_m_s) > 0.3
    err = (result.pos_m - result.pos_cmd_m)[moving]
    forward = result.v_ref_m_s[moving] > 0
    assert np.mean(err[forward]) < 0.0, "should trail while driving forward"
    assert np.mean(err[~forward]) > 0.0, "should trail while driving backward"


def test_runs_are_deterministic():
    """Same property `impulse_response` and `closed_loop` pin on the full
    trajectory, not just the summary metrics -- two runs could agree on every
    metric while differing sample-by-sample, and a metrics-only check would
    never see it."""
    import numpy as np

    a, b = run(), run()
    assert np.array_equal(a.pos_m, b.pos_m)
    assert np.array_equal(a.pitch_deg, b.pitch_deg)
    assert np.array_equal(a.motor_current_a, b.motor_current_a)
    assert a.to_json_dict() == b.to_json_dict()
