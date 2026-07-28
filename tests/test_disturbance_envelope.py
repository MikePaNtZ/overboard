"""Disturbance-rejection ENVELOPE gate (issue #24, AC2).

`tests/test_closed_loop.py` proves the controller survives one fixed
impulse, `NOMINAL_IMPULSE_NS`. That is a single point. This file sweeps a
grid of magnitudes on the same driverless closed-loop configuration and
pins the recovery boundary the sweep actually measures -- see
`sim/scenarios/disturbance_envelope.py` for why the sweep is a coarse grid
rather than a bisection to the exact knife's edge.

THE LIBRARY MUST BE BUILT, same as `test_closed_loop.py`:

    cargo build --release -p control-ffi
"""

import pytest

from sim.scenarios.disturbance_envelope import EnvelopePoint, EnvelopeResult, sweep_closed_loop
from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS, load_model
from sim.scenarios.rust_controller import RustController, library_path


@pytest.fixture(scope="module")
def model():
    return load_model()


def test_the_control_library_is_actually_built():
    """Fails loudly rather than letting the rest of the file skip."""
    assert library_path() is not None, (
        "control-ffi is not built, so the envelope gate would be vacuous. "
        "Run: cargo build --release -p control-ffi"
    )


# --------------------------------------------------------------------------
# EnvelopeResult -- the arithmetic, tested against constructed points so it
# does not need a sim run to check its edge cases.
# --------------------------------------------------------------------------

def _points(*pairs):
    """`(magnitude_ns, nose_strike)` pairs -> a tuple of EnvelopePoint."""
    return tuple(
        EnvelopePoint(magnitude_ns=m, nose_strike=s, peak_abs_pitch_deg=0.0) for m, s in pairs
    )


def test_recovery_boundary_is_one_grid_step_below_first_failure():
    result = EnvelopeResult(_points((20, False), (40, False), (60, True), (80, True)))
    assert result.first_failure_ns == 60
    assert result.recovery_boundary_ns == 40


def test_recovery_boundary_is_the_top_of_the_sweep_if_nothing_failed():
    result = EnvelopeResult(_points((20, False), (40, False), (60, False)))
    assert result.first_failure_ns is None
    assert result.recovery_boundary_ns == 60


def test_recovery_boundary_raises_if_the_sweep_starts_too_high():
    """The smallest sampled magnitude already struck -- there is no
    survivable point in this sweep to report a boundary from, and guessing
    one (e.g. zero) would be worse than an explicit failure."""
    result = EnvelopeResult(_points((60, True), (80, True)))
    with pytest.raises(ValueError):
        result.recovery_boundary_ns


def test_recovery_boundary_ignores_a_later_reversion_to_survival():
    """A magnitude past the first failure that happens to survive again
    (the knife's-edge non-monotonicity documented in the module) must not
    be picked as "the" boundary -- it would overstate the margin."""
    result = EnvelopeResult(_points((20, False), (40, False), (60, True), (80, False)))
    assert result.first_failure_ns == 60
    assert result.recovery_boundary_ns == 40


# --------------------------------------------------------------------------
# The actual sweep, against the real closed-loop driverless board.
# --------------------------------------------------------------------------

#: 20 N*s steps from the existing nominal point up to comfortably past where
#: the controller was measured to first fail. Coarse deliberately -- see
#: `sim/scenarios/disturbance_envelope.py` for why finer steps chase a
#: knife's edge rather than mapping a margin.
SWEEP_MAGNITUDES_NS = tuple(range(int(NOMINAL_IMPULSE_NS), 321, 20))


def test_the_envelope_is_mapped_not_a_single_point(model):
    result = sweep_closed_loop(model, RustController, SWEEP_MAGNITUDES_NS)
    assert len(result.points) > 2, "a sweep of one or two points is not an envelope"
    assert [p.magnitude_ns for p in result.points] == sorted(p.magnitude_ns for p in result.points)


def test_the_envelope_sweep_is_deterministic(model):
    """Same property every other scenario gate asserts: same inputs, same
    trace, so a controller carrying hidden state across runs would be
    caught."""
    first = sweep_closed_loop(model, RustController, SWEEP_MAGNITUDES_NS[:3])
    second = sweep_closed_loop(model, RustController, SWEEP_MAGNITUDES_NS[:3])
    assert first.to_json_dict() == second.to_json_dict()


def test_recovery_boundary_has_large_margin_over_the_nominal_impulse(model):
    """Measured, not assumed -- re-baseline deliberately if the physics or
    gains change, the same convention `NOMINAL_IMPULSE_NS` and
    `EXPECTED_STRIKE_ANGLE_DEG` already use.

    260 N*s is 13x `NOMINAL_IMPULSE_NS` (20 N*s), which is itself already
    ~60% clear of the open-loop topple threshold -- so the closed-loop
    controller carries a large, now-measured margin rather than an assumed
    one.
    """
    result = sweep_closed_loop(model, RustController, SWEEP_MAGNITUDES_NS)
    assert result.first_failure_ns == 280.0
    assert result.recovery_boundary_ns == 260.0
    assert result.recovery_boundary_ns > 10 * NOMINAL_IMPULSE_NS
