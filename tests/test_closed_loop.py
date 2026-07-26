"""First-light gate: the REAL Rust control law, closing the loop in MuJoCo.

Two plants, deliberately. The first half runs the DRIVERLESS board, where the
inner loop alone is the whole story. The second half runs the RIDDEN plant --
a 70 kg mass above the axle -- which is the vehicle being built, and where the
cascade is gated.

What this proves is narrow and worth stating exactly: that
`control_core::PitchRegulator` and `VelocityLoop` behind `safety::Envelope`,
reached over the C ABI, change the physics in the right direction. It does
**not** prove the balance controller works. Pitch is MuJoCo truth rather than
an estimator, the rider is a rigid lump with no compliance, and there is no
tyre model.

THE LIBRARY MUST BE BUILT. If it is missing these tests FAIL; they do not skip.
A closed-loop test that quietly passes because the controller was absent is the
"green build that proves nothing" failure this project has already hit twice in
CI plumbing, and it would be far more expensive here.

    cargo build --release -p control-ffi
"""

import pytest

from sim.scenarios.impulse_response import (
    NOMINAL_IMPULSE_NS,
    SUBTHRESHOLD_IMPULSE_NS,
    ImpulseParams,
    load_model,
    run,
)
from sim.scenarios.rust_controller import RustController, library_path


@pytest.fixture(scope="module")
def model():
    return load_model()


@pytest.fixture
def controller():
    with RustController() as c:
        yield c


def test_the_control_library_is_actually_built():
    """Fails loudly rather than letting the rest of the file skip."""
    assert library_path() is not None, (
        "control-ffi is not built, so the closed-loop gate would be vacuous. "
        "Run: cargo build --release -p control-ffi"
    )


def test_abi_version_is_the_one_this_glue_was_written_against(controller):
    assert controller.abi_version == 1


def test_closed_loop_prevents_the_nose_strike(model, controller):
    """The headline. Open-loop this exact impulse noses into the ground."""
    r = run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS), model=model, controller=controller)
    assert not r.metrics.nose_strike
    assert r.metrics.peak_abs_pitch_deg < 3.0, (
        f"peak |pitch| {r.metrics.peak_abs_pitch_deg:.2f} deg -- the strike angle is "
        "18.6 deg, so anything near it means the controller is barely helping"
    )


def test_closed_loop_beats_open_loop_on_the_same_disturbance(model, controller):
    """Paired against the baseline, so 'it did not strike' cannot pass by the
    disturbance having been too small."""
    p = ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS)
    open_loop = run(p, model=model)
    closed = run(p, model=model, controller=controller)

    assert open_loop.metrics.nose_strike, "baseline must still fail, or this proves nothing"
    assert closed.metrics.peak_abs_pitch_deg < 0.25 * open_loop.metrics.peak_abs_pitch_deg


def test_the_controller_does_nothing_when_nothing_happens(model, controller):
    """Zero disturbance must produce zero motion.

    The test people skip, and the one that catches a sign error, an offset or a
    limit cycle -- all of which look fine in a disturbance-rejection plot.
    """
    r = run(ImpulseParams(magnitude_ns=0.0, sim_seconds=6.0), model=model, controller=controller)
    assert r.metrics.peak_abs_pitch_deg < 0.2
    assert abs(r.metrics.travel_m) < 0.05
    assert abs(r.motor_current_a).max() < 0.5, "a still board should need no current"


def test_closed_loop_does_not_make_a_survivable_disturbance_worse(model, controller):
    """The sub-threshold case survives open-loop already. A controller that
    improves the big case while degrading the small one has traded one failure
    for another, and a single-point test would not see it."""
    p = ImpulseParams(magnitude_ns=SUBTHRESHOLD_IMPULSE_NS)
    open_loop = run(p, model=model)
    closed = run(p, model=model, controller=controller)
    assert not closed.metrics.nose_strike
    assert closed.metrics.peak_abs_pitch_deg <= open_loop.metrics.peak_abs_pitch_deg


def test_the_gains_leave_headroom(model, controller):
    """Peak current well inside the clamp.

    A controller that only works while saturated is not tuned, it is bounded --
    and the margin would vanish the moment the plant gets heavier or the limit
    gets more honest.
    """
    r = run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS), model=model, controller=controller)
    peak = float(abs(r.motor_current_a).max())
    assert peak < 20.0, f"peak {peak:.1f} A is more than half the 40 A clamp"
    assert controller.saturated_cycles == 0


def test_closed_loop_runs_are_deterministic(model):
    """Same property the open-loop gate asserts, but across the FFI -- so a
    controller carrying hidden state between runs would be caught."""
    p = ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=3.0)
    with RustController() as a:
        first = run(p, model=model, controller=a)
    with RustController() as b:
        second = run(p, model=model, controller=b)
    assert first.to_json_dict()["metrics"] == second.to_json_dict()["metrics"]


# --------------------------------------------------------------------------
# The cascade. Gated on the BALLASTED plant, because that is the vehicle:
# with a rider the centre of mass sits above the axle and the pitch-to-velocity
# coupling has the opposite sign to the driverless board.
# --------------------------------------------------------------------------

BALLAST_KG, BALLAST_M = 70.0, 0.75
INNER = dict(kp_a_per_rad=200.0, kd_a_per_rad_s=30.0, max_current_a=40.0)
OUTER = dict(kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True)


@pytest.fixture(scope="module")
def ridden_model():
    import sys
    from pathlib import Path

    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "scripts"))
    from experiment import build_model

    return build_model(BALLAST_KG, BALLAST_M, 40.0, "figure")


def _run(model, secs=12.0, **kw):
    with RustController(**kw) as c:
        r = run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=secs),
                model=model, controller=c)
        return r, c


def test_the_ridden_board_survives_the_impulse_at_all(ridden_model):
    """Baseline for everything below: the inner loop alone must not strike."""
    r, _ = _run(ridden_model, **INNER)
    assert not r.metrics.nose_strike
    assert r.metrics.peak_abs_pitch_deg < 8.0


def test_without_the_outer_loop_the_ridden_board_rides_away(ridden_model):
    """The motivating failure, asserted rather than remembered.

    A pure inner loop holds attitude beautifully and does not regulate
    position at all. If this ever stops being true the outer loop has stopped
    being necessary, and the tests below are measuring nothing.
    """
    r, _ = _run(ridden_model, **INNER)
    assert abs(r.metrics.travel_m) > 2.0, "expected it to ride away"
    assert abs(float(r.wheel_rate_rads[-1])) > 1.0, "expected it to still be moving"


def test_the_cascade_brings_the_ridden_board_back_to_rest(ridden_model):
    """The point of the outer loop: stop, and stop roughly where you started.

    Integrating velocity error IS position error, which is what turns "comes
    to a halt" into "holds station".
    """
    r, c = _run(ridden_model, **INNER, **OUTER)
    assert not r.metrics.nose_strike
    assert abs(r.metrics.travel_m) < 0.25, f"ended {r.metrics.travel_m:.2f} m away"
    assert abs(float(r.wheel_rate_rads[-1])) < 0.5, "should have stopped"
    assert c.saturated_cycles == 0


def test_the_cascade_costs_some_pitch_and_that_is_the_trade(ridden_model):
    """Position regulation is bought with attitude excursion -- the outer loop
    leans the board on purpose. Worth pinning so the cost stays visible and
    bounded rather than drifting."""
    inner, _ = _run(ridden_model, **INNER)
    both, _ = _run(ridden_model, **INNER, **OUTER)
    assert both.metrics.peak_abs_pitch_deg > inner.metrics.peak_abs_pitch_deg
    assert both.metrics.peak_abs_pitch_deg < 2.0 * inner.metrics.peak_abs_pitch_deg


def test_inverting_the_plant_coupling_flips_the_board(ridden_model):
    """The mutation test for the outer loop's sign.

    `com_above_axle=False` is the driverless coupling applied to a ridden
    plant -- positive feedback in velocity. It must fail loudly, because on
    the driverless board the same mistake passes every test.
    """
    wrong = dict(OUTER, com_above_axle=False)
    r, _ = _run(ridden_model, **INNER, **wrong)
    assert r.metrics.nose_strike, "inverted coupling must not look survivable"
    assert r.metrics.peak_abs_pitch_deg > 90.0, "expected it to go right over"


def test_a_speed_setpoint_is_tracked(ridden_model):
    """Not just station keeping: ask for 1 m/s and get it."""
    r, _ = _run(ridden_model, secs=14.0, **INNER, **dict(OUTER, v_ref_m_s=1.0))
    import numpy as np

    v = np.asarray(r.wheel_rate_rads) * 0.14605
    settled = v[r.t >= r.t[-1] - 2.0]
    assert not r.metrics.nose_strike
    assert abs(float(settled.mean()) - 1.0) < 0.25, f"settled at {settled.mean():.2f} m/s"


def test_the_outer_loop_does_not_suit_the_driverless_plant(model):
    """A characterisation test, not a target.

    The driverless board's centre of mass sits 19 mm BELOW the axle, so
    holding 5 deg of lean is worth 0.205 N*m against the ridden board's
    44.7 N*m -- roughly 200x less authority to change speed with. The outer
    loop therefore pegs its pitch reference at the clamp and never catches
    the error: travel diverges (-29 m at 40 s, -64 m at 90 s).

    Recorded so nobody "fixes" the driverless case by retuning. It is not a
    tuning problem, and it is not the vehicle.
    """
    import math

    r, c = _run(model, kp_a_per_rad=80.0, kd_a_per_rad_s=11.0, max_current_a=40.0,
                kp_v_rad_per_m_s=0.20, ki_v_rad_per_m=0.10, com_above_axle=False)
    assert math.degrees(c.peak_abs_pitch_ref_rad) > 4.9, (
        "expected the reference pegged at its clamp; if this now passes "
        "comfortably, the plant or the clamp has changed"
    )
    assert abs(r.metrics.travel_m) > 1.0, "expected it not to hold station"
