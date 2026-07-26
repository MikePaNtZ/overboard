"""First-light gate: the REAL Rust control law, closing the loop in MuJoCo.

What this proves is narrow and worth stating exactly. It proves that
`control_core::PitchRegulator` behind `safety::Envelope`, reached over the C
ABI, changes the physics in the right direction. It does **not** prove the
balance controller works: the board is a stable pendulum, pitch is truth from
MuJoCo rather than an estimator, and there is no outer loop, so the board holds
attitude while riding away. Position regulation is the next increment.

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


@pytest.mark.xfail(
    reason="known and expected: a pure inner loop holds pitch but does not "
    "regulate position, so the board rides away. The outer velocity loop is "
    "the next increment; this is here so it converts to a pass rather than "
    "being remembered.",
    strict=True,
)
def test_board_returns_to_rest_near_where_it_started(model, controller):
    r = run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS), model=model, controller=controller)
    assert abs(r.metrics.travel_m) < 0.25
    assert abs(float(r.wheel_rate_rads[-1])) < 0.5
