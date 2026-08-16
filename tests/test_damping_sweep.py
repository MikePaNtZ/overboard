"""ADR-0011 criterion (g) -- the wheel-hinge damping sweep, MEASURED.

Issue #229's finding: criterion (g) ("every pass must hold across a damping
sweep of 0.5x-2x on the wheel-hinge damping=0.08") had never actually been
taken, at either the 40 A or 60 A operating point -- there was no way to vary
`wheel_hinge`'s declared damping without editing `sim/models/` (Sr.
Mechanical & Systems' fidelity contract) and rebuilding for every sweep
point. `sim-host`'s `--damping-scale` flag exists to close exactly that gap,
the same "runtime knob, not a model edit" pattern `--incline-deg` already
established for the incline-tolerance sweep (`test_incline_tolerance.py`).

WHAT THIS FILE FOUND
---------------------
Criterion (g), taken literally, FAILS at the current 40 A operating point.
Both deployed-path entries of ADR-0011's criterion (a) matrix (full stick
from rest, the stick-reversal at speed) invert well inside the 0.5x-2x
sweep -- there is a real cliff between 1.4x (holds, peak 9.0 deg) and 1.5x
(inverts at 8.04 s). The full range up to 2x is not a close call: at 2x
damping the board is upside down by 5.64 s. See
`test_the_acceptance_matrix_does_not_hold_across_the_full_0_5x_2x_sweep`,
marked `xfail(strict=True)` for the same reason `test_cmd_envelope_reserve.py`
marks its own known-failing criteria that way: written as the criterion
SHOULD read, not softened into something that passes, so the day someone
fixes the underlying problem this turns red as XPASS rather than staying a
silent, uninformative green.

This does NOT mean the board is unsafe at today's *declared* damping (1.0x,
`wheel_hinge`'s actual `damping="0.08"`) -- that point holds cleanly in both
scenarios, and is asserted as a plain (non-xfail) test below. It means the
specific claim "the pass is robust to a 2x error in that undocumented
constant" is false, which is exactly the thing criterion (g) exists to
check for, and exactly the kind of finding ADR-0011 says should not be
tuned away or silently absorbed into a passing threshold.

Whether this blocks the launch-hold exit, or is re-homed to the hardware
gate the way criterion (a)-3 (the kerb strike) already was, is issue #229's
open Ask 3 -- a decision, not a measurement, and not this file's call.

DETERMINISM
-----------
Same rule as `test_incline_tolerance.py` and `test_cmd_envelope_reserve.py`:
every run is `--scripted-scenario ... --free-run`, schedule indexed on
simulated time, fixed-timestep RK4, no stochastic terms -- reproducible bit
for bit.

THE BINARY MUST BE BUILT. Same rule as the two files above: FAILS, not
skips, if it is missing.
"""

from __future__ import annotations

import csv
import subprocess
import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

#: Ports well away from every other test file's own pair (19601-19606) --
#: see test_incline_tolerance.py's own comment on why this matters.
STATE_OUT = "127.0.0.1:19607"
INPUT_IN = "127.0.0.1:19608"

#: Pitch beyond which the board is unambiguously going over -- same
#: threshold `test_incline_tolerance.py` uses.
INVERTED_DEG = 90.0

#: `wheel_hinge`'s declared damping in both MJCF models
#: (`sim/models/overboard_onewheel.xml`, `overboard_rider.xml`). The sweep
#: below is centred on this; if it ever moves, re-derive the sweep rather
#: than assume it is still 0.08.
DECLARED_DAMPING = 0.08

#: ADR-0011 criterion (g)'s own stated range.
SWEEP_LO, SWEEP_HI = 0.5, 2.0

#: The measured cliff (see module docstring): holds through 1.4x, inverts at
#: 1.5x and above, on both swept scenarios.
CLIFF_SCALES = (0.5, 1.0, 1.2, 1.4, 1.5, 1.6, 1.8, 2.0)


def _run(tmp_path: Path, name: str, scenario: str, **kw) -> list[dict]:
    """One deterministic free run; returns the per-cycle trace.

    Through `cargo run` rather than the raw binary -- see
    `test_cmd_envelope_reserve.py`'s own comment: Cargo puts the linked
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


def test_the_sim_host_binary_is_actually_built():
    assert (REPO / "target" / "release" / "sim-host").exists(), (
        "sim-host is not built, so this whole gate would be vacuous. Run:\n"
        "    cargo build --release -p sim-host --bin sim-host"
    )


# ---------------------------------------------------------------------------
# The positive control: is `--damping-scale` a no-op at 1.0, the way
# `--incline-deg 0` is a no-op at zero?
# ---------------------------------------------------------------------------


def test_a_damping_scale_of_one_is_bit_identical_to_omitting_the_flag(tmp_path):
    """`SimBackend::set_damping_scale`'s `None` case is exercised by omitting
    the flag; an explicit `1.0` takes the `Some` path and re-derives the same
    value from the model. Both must produce the identical trajectory, or the
    knob is perturbing every other test in this repo that never passes it.

    `crates/sim-backend/src/lib.rs::set_damping_scale_scales_the_declared_wheel_hinge_damping`
    already checks this at the Rust unit level (the damping VALUE matches);
    this is the black-box counterpart (the resulting TRAJECTORY matches).
    """
    unset = _run(tmp_path, "ds_unset", "full-stick")
    explicit_one = _run(tmp_path, "ds_one", "full-stick", damping_scale=1.0)
    assert [list(r.values()) for r in unset] == [list(r.values()) for r in explicit_one], (
        "--damping-scale 1.0 is not bit-identical to omitting the flag, so the "
        "knob perturbs a default run and every other test in this repo is now "
        "measuring something slightly different"
    )


# ---------------------------------------------------------------------------
# What holds today, at the model's own declared damping
# ---------------------------------------------------------------------------


def test_the_acceptance_matrix_holds_at_the_declared_damping(tmp_path):
    """Not a criterion (g) claim -- criterion (g) is about ROBUSTNESS to an
    error in the constant, not about today's board being flyable at its own
    declared value. This is the baseline the sweep below is measured against.
    """
    for scenario in ("full-stick", "stick-reversal"):
        rows = _run(tmp_path, f"declared_{scenario}", scenario, damping_scale=1.0)
        t = _inversion_time(rows)
        assert t is None, (
            f"{scenario} inverted at {t:.3f} s at the MODEL'S OWN declared damping "
            f"({DECLARED_DAMPING}) -- that is not a criterion (g) finding, it is a "
            "plant regression and belongs to whoever's change caused it"
        )


# ---------------------------------------------------------------------------
# Criterion (g) itself, taken literally
# ---------------------------------------------------------------------------


@pytest.mark.xfail(
    strict=True,
    reason=(
        "ADR-0011 criterion (g), literal reading: 'every pass must hold across "
        "a damping sweep of 0.5x-2x'. Measured at the current 40 A operating "
        "point: it does not. Both full-stick-from-rest and the stick-reversal "
        "hold cleanly through 1.4x (peak 9.0 deg) and invert at 1.5x (8.04 s) "
        "and every point above it, reaching full inversion by 5.64 s at 2.0x. "
        "This is a real cliff, not numerical noise -- see the printed sweep "
        "table. Whether this blocks the launch-hold exit or is re-homed to the "
        "hardware gate (as criterion (a)-3, the kerb strike, already was) is "
        "issue #229's open decision, not this test's to make."
    ),
)
def test_the_acceptance_matrix_does_not_hold_across_the_full_0_5x_2x_sweep(tmp_path):
    for scenario in ("full-stick", "stick-reversal"):
        print(f"\n{scenario} across the damping sweep ({SWEEP_LO}x-{SWEEP_HI}x):")
        for scale in CLIFF_SCALES:
            rows = _run(tmp_path, f"g_{scenario}_{scale}", scenario, damping_scale=scale)
            t = _inversion_time(rows)
            peak_pitch = max(abs(r["truth_pitch_deg"]) for r in rows)
            print(
                f"  {scale:4.2f}x (damping={scale * DECLARED_DAMPING:.3f}): "
                f"{'held' if t is None else f'INVERTED at {t:.2f} s'}, "
                f"peak lean {peak_pitch:6.2f} deg"
            )
            assert t is None, (
                f"{scenario} inverted at {t:.3f} s at damping scale {scale}x -- "
                "criterion (g) does not hold across the full swept range"
            )
