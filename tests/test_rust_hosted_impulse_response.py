"""Issue #107 (I1c) AC6: the impulse-response closed loop, hosted through the
Rust `hal` seam, must agree with the Python-hosted result within a tolerance
stated here BEFORE the numbers were looked at, not fitted to them afterward.

WHY THIS IS A SEPARATE TEST FROM `test_closed_loop.py`
--------------------------------------------------------
`test_closed_loop.py` exercises the Python-hosted loop (Python steps MuJoCo,
calls into `control-ffi` for the control law) and is the existing, unrelated
production gate for the published `recovery.peak-pitch` claim. This test
exercises the OTHER direction: `board-app-driverless`'s `impulse-response-rust`
binary hosts the whole ICD 5.2 loop in Rust -- `wait_observe()` steps the real
plant through `sim-backend` (issue #107), and the SAME `control-core` /
`safety` objects `control-ffi` wires for the Python side are wired here too,
reached natively instead of through the C ABI. If this test fails, the
carried-forward instruction from issue #107 (and from I1b/#106 before it) is:
suspect the ordering seam in `crates/plant-mujoco` / `sim-backend` before the
physics -- a closed-loop, statistical comparison like this one cannot on its
own distinguish a one-tick seam bug from legitimate divergence. I1b's
bit-exact open-loop equivalence (`test_rust_python_plant_replay_equivalence.py`)
is what rules that out, and it must be green for this test's result to mean
anything.

WHY THE CONTROLLER CONFIG STILL DIFFERS FROM `test_closed_loop.py`'s DEFAULT
--------------------------------------------------------------------------
Both hosts here now run `estimator_accel_aiding=1` (wheel odometry) --
`RustController()`'s own default, the same mode `test_closed_loop.py` gates
on (issue #121; `sim-backend` now populates `erpm` from the plant, so this is
no longer forced to mode 2 by a hal field nobody filled in). The two runs are
still not directly comparable to `test_closed_loop_prevents_the_nose_strike`'s
0.879 deg baseline: this scenario is the driverless plant / impulse response,
that one is the ridden/cascade plant -- different mass, different disturbance,
different everything except the mode number.

WHY `profile=IDEAL`
--------------------
`IDEAL` (`sim/scenarios/imperfections.py`) is the sanctioned "pin the
algebra/seam, not a margin" profile -- see `test_bench_spinup.py`'s own
header for the precedent. `sim-backend`'s own actuation-delay model is still
the stub's crude one-whole-cycle buffer (this crate's header says so); a
noisy/delayed Python-side profile would inject a SECOND, Mechanical-owned
signal-path difference into a test whose entire point is isolating the plant
seam. Gating a published margin claim on `IDEAL` is exactly what
`imperfections.py`'s own docstring forbids; this is not that -- it is a seam
check, like `test_rust_python_plant_replay_equivalence.py` before it.

THE TOLERANCE, STATED BEFORE THE COMPARISON RAN
-------------------------------------------------
Both hosts run the literal same compiled `control-core`/`safety` objects
(one reached via `control-ffi`'s C ABI, one called natively) against the
literal same `libmujoco.so` I1b already proved bit-identical open-loop. Mode 1
now round-trips wheel rate through ERPM on the Rust side (`sim-backend`
divides by `RAD_S_PER_ERPM`, this binary multiplies back) where the
Python-hosted comparator casts `data.qvel[6]` to `f32` directly -- an extra
`f32` rounding through a ratio that otherwise cancels algebraically, smaller
than the summation-order noise below it. So the only mechanism left that
could make the two diverge over a 4000-step, chaotic, contact-rich trajectory
is still floating-point noise -- utterly negligible relative to the control
loop's own excursions. So the bound below is generous on purpose, not fitted
to what was observed:

  * `peak_abs_pitch_deg`: absolute difference < 0.1 deg
  * `t_peak_s` / `settle_time_s`: absolute difference < 0.01 s (5 control
    periods at 500 Hz)
  * `final_pitch_deg`: absolute difference < 0.1 deg

THE BINARIES MUST BE BUILT. If either is missing this FAILS, not skips -- see
`test_closed_loop.py`'s header for why a green "gate" that quietly did
nothing is worse than no gate.
"""

from __future__ import annotations

import subprocess
from pathlib import Path

import pytest

from sim.scenarios.imperfections import IDEAL
from sim.scenarios.impulse_response import (
    NOMINAL_IMPULSE_NS,
    ImpulseParams,
    load_model,
    run,
)
from sim.scenarios.rust_controller import RustController, library_path

REPO = Path(__file__).resolve().parents[1]

PEAK_PITCH_TOL_DEG = 0.1
TIME_TOL_S = 0.01
FINAL_PITCH_TOL_DEG = 0.1


def _rust_binary_path() -> Path:
    return REPO / "target" / "release" / "impulse-response-rust"


def test_the_control_library_is_actually_built():
    """Fails loudly rather than letting the rest of the file skip."""
    assert library_path() is not None, (
        "control-ffi is not built, so this gate would be vacuous. Run:\n"
        "    cargo build --release -p control-ffi"
    )


def test_the_rust_hosted_binary_is_actually_built():
    assert _rust_binary_path().exists(), (
        "board-app-driverless's Rust-hosted impulse harness is not built, so "
        "this gate would be vacuous. Run:\n"
        "    cargo build --release -p board-app-driverless --bin impulse-response-rust"
    )


def _python_hosted_metrics() -> dict:
    model = load_model()
    with RustController(
        com_above_axle=False,
        estimator_accel_aiding=1,
    ) as controller:
        result = run(
            ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS),
            model=model,
            controller=controller,
            profile=IDEAL,
        )
    return {
        "peak_abs_pitch_deg": result.metrics.peak_abs_pitch_deg,
        "t_peak_s": result.metrics.t_peak_s,
        "settle_time_s": result.metrics.settle_time_s,
        "final_pitch_deg": result.metrics.final_pitch_deg,
        "nose_strike": result.metrics.nose_strike,
    }


def _run_rust_binary(out_path: Path) -> None:
    # Invoked via `cargo run`, not the raw binary path -- same reason
    # `test_rust_python_plant_replay_equivalence.py` does this for
    # `plant-replay`: Cargo adds the linked library's `OUT_DIR` to
    # `DYLD_LIBRARY_PATH`/`LD_LIBRARY_PATH` for every `cargo test`/`cargo run`
    # in the same invocation (see `crates/plant-mujoco/build.rs`), which is
    # what makes the linked libmujoco resolve at runtime on macOS without
    # hand-set env vars. `_rust_binary_path()`'s existence check above
    # already asserts the binary is built, so this only ever re-links.
    subprocess.run(
        [
            "cargo", "run", "--release", "-q",
            "-p", "board-app-driverless", "--bin", "impulse-response-rust", "--",
            str(out_path),
        ],
        cwd=REPO,
        check=True,
    )


def _rust_hosted_metrics(tmp_path: Path) -> dict:
    out_path = tmp_path / "impulse_rust_out.txt"
    _run_rust_binary(out_path)
    peak, t_peak, settle, final = (float(x) for x in out_path.read_text().split())
    return {
        "peak_abs_pitch_deg": peak,
        "t_peak_s": t_peak,
        # -1.0 is impulse-response-rust's "never settled" sentinel, mirroring
        # the Python side's None.
        "settle_time_s": None if settle < 0.0 else settle,
        "final_pitch_deg": final,
    }


def test_rust_hosted_impulse_response_agrees_with_python_hosted(tmp_path):
    py = _python_hosted_metrics()
    rs = _rust_hosted_metrics(tmp_path)

    # Sanity: this must not be a vacuous zero-vs-zero match -- the impulse has
    # to have actually moved the board on both sides.
    assert py["peak_abs_pitch_deg"] > 0.5, "python-hosted run never pitched -- proves nothing"
    assert rs["peak_abs_pitch_deg"] > 0.5, "rust-hosted run never pitched -- proves nothing"

    assert not py["nose_strike"], "python-hosted run noses in -- not a valid baseline"

    assert abs(rs["peak_abs_pitch_deg"] - py["peak_abs_pitch_deg"]) < PEAK_PITCH_TOL_DEG, (
        f"peak |pitch| diverged: rust={rs['peak_abs_pitch_deg']:.4f} deg, "
        f"python={py['peak_abs_pitch_deg']:.4f} deg (tolerance {PEAK_PITCH_TOL_DEG} deg)"
    )
    assert abs(rs["t_peak_s"] - py["t_peak_s"]) < TIME_TOL_S, (
        f"time of peak diverged: rust={rs['t_peak_s']:.4f} s, python={py['t_peak_s']:.4f} s "
        f"(tolerance {TIME_TOL_S} s)"
    )
    assert abs(rs["final_pitch_deg"] - py["final_pitch_deg"]) < FINAL_PITCH_TOL_DEG, (
        f"final pitch diverged: rust={rs['final_pitch_deg']:.4f} deg, "
        f"python={py['final_pitch_deg']:.4f} deg (tolerance {FINAL_PITCH_TOL_DEG} deg)"
    )

    # settle_time_s is None on either side only if the board never returns to
    # the settle band -- both sides must agree on THAT before comparing the
    # numeric value.
    assert (rs["settle_time_s"] is None) == (py["settle_time_s"] is None), (
        f"settle-vs-never-settled disagreement: rust={rs['settle_time_s']}, "
        f"python={py['settle_time_s']}"
    )
    if rs["settle_time_s"] is not None:
        assert abs(rs["settle_time_s"] - py["settle_time_s"]) < TIME_TOL_S, (
            f"settle time diverged: rust={rs['settle_time_s']:.4f} s, "
            f"python={py['settle_time_s']:.4f} s (tolerance {TIME_TOL_S} s)"
        )


@pytest.mark.parametrize("run_index", [1, 2])
def test_rust_hosted_repeat_runs_are_bit_identical(tmp_path, run_index):
    """AC5 (issue #107, carried forward from #74): repeat-run bit-identity
    must survive the real-plant wiring, not just the stub's bookkeeping."""
    out_a = tmp_path / f"a_{run_index}.txt"
    out_b = tmp_path / f"b_{run_index}.txt"
    _run_rust_binary(out_a)
    _run_rust_binary(out_b)
    assert out_a.read_text() == out_b.read_text(), (
        "two runs of impulse-response-rust produced different output -- repeat-run "
        "bit-identity (issue #74) is broken by the real-plant wiring"
    )
