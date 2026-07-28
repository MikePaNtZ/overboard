"""The Python replica of the Rust complementary filter, gated.

`scripts/analyse_estimator_phase.py` carries `PyComplementary` and
`PyWheelAccel` -- a second implementation of `control-core`'s filter, in Python.
It exists because the FFI exposes one switch for the estimator and separating
the P and D paths for the frequency-domain analysis needed a replica that could
be taken apart.

**Every number in notebook 4 flows through that replica.** It was validated
once, by hand, to 2.19e-05 deg -- and then nothing ran the validation again.
`validate_replica()` has existed the whole time and no test called it, so a
change to the Rust filter would have let the replica drift silently and turned
notebook 4 into fiction without a single test going red.

This is that test. It is cheap (one shadow-mode shuttle run) and it is the only
thing standing between `control-core` changing and a published notebook quietly
becoming wrong.
"""

import pytest

from scripts.analyse_estimator_phase import validate_replica

#: The replica is a line-for-line transcription, not an approximation, so the
#: bar is machine precision rather than "close enough". `validate_replica`
#: itself uses 1e-3 as its own `faithful` flag; this is tighter on purpose --
#: the hand validation came out at 2.19e-05 deg, and anything materially above
#: that means the two implementations have genuinely diverged rather than
#: merely rounded differently.
MAX_DIVERGENCE_DEG = 1e-3
EXPECTED_ORDER_DEG = 1e-4


def test_the_python_replica_still_matches_the_shipped_rust_filter():
    """If this fails, do NOT relax it. Notebook 4 is downstream.

    The failure means one of two things, and both are worse than a red test:
    `control-core`'s filter changed and the replica did not follow, or the
    replica was edited and the shipped filter did not. Either way every figure
    in `04-estimator-in-the-loop.ipynb` and every conclusion drawn from the
    Bode plots is measuring something that is not the flight code.

    Fix the replica, re-run `scripts/analyse_estimator_phase.py`, and
    re-execute the notebook -- in that order, per notebooks/README.md.
    """
    v = validate_replica()

    assert v["samples"] > 1000, (
        f"only {v['samples']} samples compared -- the validation run is too "
        "short to be evidence of anything"
    )
    assert v["faithful"], (
        f"replica diverged from the shipped Rust filter: max "
        f"{v['max_abs_deg']:.3e} deg, RMS {v['rms_deg']:.3e} deg"
    )
    assert v["max_abs_deg"] < MAX_DIVERGENCE_DEG, (
        f"max divergence {v['max_abs_deg']:.3e} deg exceeds {MAX_DIVERGENCE_DEG:.0e}. "
        "Notebook 4's numbers all flow through this replica -- fix the replica, "
        "do not loosen the bound"
    )


def test_the_replica_matches_at_the_precision_it_was_originally_validated_to():
    """Separate from the pass/fail bound above, and deliberately tighter.

    The hand validation recorded 2.19e-05 deg. A drift to, say, 5e-4 would
    still clear `faithful` while indicating the two implementations had started
    to disagree in a way that was not there before. This catches that early,
    while it is still a curiosity rather than a published error.
    """
    v = validate_replica()
    assert v["max_abs_deg"] < EXPECTED_ORDER_DEG, (
        f"max divergence {v['max_abs_deg']:.3e} deg. Still within the hard bound, "
        f"but well above the {EXPECTED_ORDER_DEG:.0e} the replica was validated to -- "
        "the implementations are drifting apart. Investigate before it matters."
    )
