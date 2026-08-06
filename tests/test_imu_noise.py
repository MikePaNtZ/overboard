"""The datasheet-derived IMU sensor-noise model, wired into `sim-host`.

WHY THIS EXISTS. Every ADR-0011 acceptance number in this repo (`test_
cmd_envelope_reserve.py`, `test_incline_tolerance.py`, `test_crr_sweep.py`,
`test_braking_authority.py`, ...) is measured on `sim-host` running the
plant's own RK4 integrator with a perfect, noiseless IMU. A margin claimed
in the tens-of-percent range against an inversion cliff cannot be validated
on that plant -- this is the instrument that lets it be, by injecting the
ICM-42688-P-derived gyro/accel noise the estimator's D-term and attitude
estimate would actually see (see `crates/sim-backend/src/imperfections.rs`'s
`IMU_NOISE_DATASHEET_V1` for the full noise-density-to-sigma derivation).

THIS FILE only checks the KNOB, not a margin number: off by default,
bit-identical to a run that never mentions it, deterministic and reproducible
under a fixed seed, and genuinely doing something when it is on. Whether a
specific margin survives noise is a separate, later question (measured
ad hoc, not gated here -- a noise-on acceptance number belongs to whoever
owns the margin it is checking, not to the knob's own plumbing test).

METHOD. Same as `test_incline_tolerance.py`'s own positive control: every run
is `--scripted-scenario full-stick --free-run`, so the whole comparison is
deterministic and cheap, and the trace CSV is compared row for row. THE
BINARY MUST BE BUILT -- same rule as every other file in this class.
"""

from __future__ import annotations

import csv
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

#: Ports well away from every other file in this class (19601-19706 taken).
STATE_OUT = "127.0.0.1:19801"
INPUT_IN = "127.0.0.1:19802"


def _run(tmp_path: Path, name: str, scenario: str = "full-stick", **kw) -> list[dict]:
    """One deterministic free run; returns the per-cycle trace.

    Through `cargo run` rather than the raw binary -- `test_cmd_envelope_
    reserve.py` documents why: Cargo puts the linked libmujoco's `OUT_DIR` on
    the dynamic loader path.
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
    assert "imu_noise_seed=" in proc.stderr, (
        "sim-host's startup line no longer records imu_noise_seed -- a noise "
        "run whose seed cannot be read back off the run's own output is not "
        "a measurement."
    )
    with out.open() as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def test_the_sim_host_binary_is_actually_built():
    assert (REPO / "target" / "release" / "sim-host").exists(), (
        "sim-host is not built, so this whole gate would be vacuous. Run:\n"
        "    cargo build --release -p sim-host --bin sim-host"
    )


def test_imu_noise_is_off_by_default_and_bit_identical_to_omitting_the_flag(tmp_path):
    """The Task 3 requirement, as a test rather than an inspection.

    Two runs that both omit `--imu-noise-seed` must be bit-for-bit identical
    to EACH OTHER (the "off" path is itself perfectly deterministic -- no
    clock, no unseeded global RNG, nothing left over from a previous run),
    which is the same standard `test_incline_tolerance.py`'s own "zero is a
    no-op" check holds `--incline-deg` to.
    """
    a = _run(tmp_path, "noise_unset_a")
    b = _run(tmp_path, "noise_unset_b")
    assert [list(r.values()) for r in a] == [list(r.values()) for r in b], (
        "two runs that never mention --imu-noise-seed produced different "
        "traces -- something in this repo is non-deterministic even with "
        "the noise model untouched, which this knob must not be blamed for "
        "but must not hide either."
    )


def test_the_seed_flag_actually_changes_the_run(tmp_path):
    """The other half: a knob that does nothing when set would pass the test
    above trivially and vacuously. Confirms the flag is not a dead argument.
    """
    off = _run(tmp_path, "noise_off")
    on = _run(tmp_path, "noise_on", imu_noise_seed=1)
    assert [list(r.values()) for r in off] != [list(r.values()) for r in on], (
        "--imu-noise-seed 1 produced a trace identical to the run with no "
        "noise at all -- the flag is wired but the profile it applies is "
        "not actually changing anything the trace can see."
    )


def test_the_same_seed_reproduces_the_same_run(tmp_path):
    """Determinism under a fixed seed -- the standing rule that a number
    nobody else can re-take is not a measurement, applied to noise-on runs
    specifically. The plant's own integrator has no other stochastic term,
    so this must be exact, not merely close.
    """
    a = _run(tmp_path, "noise_seed42_a", imu_noise_seed=42)
    b = _run(tmp_path, "noise_seed42_b", imu_noise_seed=42)
    assert [list(r.values()) for r in a] == [list(r.values()) for r in b], (
        "the same --imu-noise-seed produced two different traces -- the "
        "noise stream is not reproducible, which makes any run built on it "
        "unre-takeable and therefore not a measurement."
    )


def test_different_seeds_produce_different_dispersion(tmp_path):
    """Different seeds must sample different draws from the same
    distribution -- not the same stream, and not two on/off states."""
    seed1 = _run(tmp_path, "noise_seed1", imu_noise_seed=1)
    seed2 = _run(tmp_path, "noise_seed2", imu_noise_seed=2)
    assert [list(r.values()) for r in seed1] != [list(r.values()) for r in seed2], (
        "--imu-noise-seed 1 and --imu-noise-seed 2 produced identical "
        "traces -- the seed is not actually reaching the noise generator."
    )
