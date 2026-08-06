"""Seeded-distribution harness for ADR-0011 acceptance tests that sit on a
chaotic boundary -- issue: statistical-acceptance.

WHY THIS EXISTS
----------------
Some `sim-host` scenarios sit close enough to a bifurcation in the plant's
own dynamics that a single NOISELESS run's invert/hold verdict measures the
chaos, not the controller. Measured directly, on `--scripted-scenario
stick-reversal --pitch-source estimator` (ADR-0011 criterion (a) entry 2,
KD=40): the noiseless run does not reach a full inversion, and its peak lean
sits about 3% over `PITCH_CEILING_DEG` -- a result that reads as "holds,
barely". Re-run the IDENTICAL scenario through the datasheet-derived IMU
noise model (`IMU_NOISE_DATASHEET_V1`, `crates/sim-backend/src/
imperfections.rs`) at 25 different seeds and the board inverts fully, 25
times out of 25. The noiseless margin was not real; it was one point on a
knife's edge that any realistic sensor noise pushes over.

A binary pass/fail on that single noiseless point is therefore not a
measurement of the controller -- it is a measurement of which side of the
knife's edge the noiseless trajectory happens to land on, which can (and, on
this repo's own evidence, does) flip with a change too small to be a
regression. This module runs a scenario across a FIXED set of seeds instead,
and returns the resulting distribution: how many of them inverted, and what
the peak-lean and peak-current distributions looked like across all of them.
That is a number a chaotic boundary can actually be measured against.

FIXED SEED LIST, NOT GENERATED AT RUN TIME
-------------------------------------------
`SEEDS` is a literal tuple, not `random.sample(...)`, a hash of the test
name, or anything else evaluated per run. A seed list assembled at run time
would make "the same N seeds" an illusion: a re-run claiming to reproduce a
rate could silently be sampling a different N points off the noise
distribution, which is exactly the kind of unre-takeable number this repo's
own determinism rules (see e.g. `tests/test_imu_noise.py`) exist to forbid.

WHY N = 25
----------
25 is the N this pattern was first measured at (25/25 inversions on the
(a)-2 scenario above), so keeping it is what lets a re-run of this module
reproduce that number bit for bit rather than a new, differently-sized one.

What 25 seeds can and cannot resolve, stated plainly: for a TRUE underlying
inversion probability p, the chance of observing zero inversions in 25
independent seeds is `(1-p)^25`. Solving for the p at which a clean 0/25 is
still a 1-in-20 (95th percentile) occurrence gives `p = 1 - 0.05^(1/25) ~=
10.9%`. **A 0/25 result cannot distinguish a controller with zero real
failure probability from one that fails on roughly 1 seed in 10** -- that
rate can hide below this N's detection floor and still show a clean 0/25 on
any given run. What 25 seeds CAN resolve is the case this module was built
to catch: a boundary so degenerate that it fails on nearly every seed (the
measured case above is 25/25, not a marginal rate), which is exactly the
gap a single noiseless run cannot tell apart from "holds with margin".
Raising N narrows that floor; it was not raised here because the case this
module exists for did not need it to make its point, and a larger N belongs
to whoever next needs the tighter bound, derived for that purpose rather
than inflated on suspicion.
"""

from __future__ import annotations

import csv
import statistics
import subprocess
from dataclasses import dataclass
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]

#: Pitch beyond which the board is unambiguously going over -- the same
#: instrument every acceptance file in this class uses (`INVERTED_DEG` in
#: `tests/test_cmd_envelope_reserve.py`, `tests/test_crr_sweep.py`, ...).
INVERTED_DEG = 90.0

#: The fixed, explicit seed list every seeded acceptance test in this repo
#: shares -- see the module docstring for why it is a literal tuple and why
#: N = 25.
SEEDS: tuple[int, ...] = tuple(range(1, 26))


@dataclass(frozen=True)
class SeededRunResult:
    """One seed's outcome on one scenario."""

    seed: int
    inverted: bool
    inversion_time_s: float | None
    peak_pitch_deg: float
    peak_current_a: float


def _quantiles(values: list[float]) -> dict[str, float]:
    ordered = sorted(values)
    p95_idx = min(len(ordered) - 1, round(0.95 * (len(ordered) - 1)))
    return {
        "min": ordered[0],
        "median": statistics.median(ordered),
        "p95": ordered[p95_idx],
        "max": ordered[-1],
    }


@dataclass(frozen=True)
class SeededDistribution:
    """The distribution of outcomes across `SEEDS` for one scenario.

    Never reduces to a single verdict on its own -- `inversions`/`n` is the
    rate, and the peak-lean/peak-current stats are quoted as a distribution
    (min/median/p95/max), not averaged into one number, because a chaotic
    boundary is exactly the case where the tail is the point.
    """

    results: tuple[SeededRunResult, ...]

    @property
    def n(self) -> int:
        return len(self.results)

    @property
    def inversions(self) -> int:
        return sum(1 for r in self.results if r.inverted)

    @property
    def inversion_rate(self) -> float:
        return self.inversions / self.n

    @property
    def peak_pitch_deg_stats(self) -> dict[str, float]:
        return _quantiles([r.peak_pitch_deg for r in self.results])

    @property
    def peak_current_a_stats(self) -> dict[str, float]:
        return _quantiles([r.peak_current_a for r in self.results])

    def summary(self) -> str:
        pp, pc = self.peak_pitch_deg_stats, self.peak_current_a_stats
        return (
            f"{self.inversions}/{self.n} seeded runs inverted "
            f"({100 * self.inversion_rate:.0f}%); peak lean deg "
            f"[min {pp['min']:.2f}, median {pp['median']:.2f}, "
            f"p95 {pp['p95']:.2f}, max {pp['max']:.2f}]; peak current A "
            f"[min {pc['min']:.2f}, median {pc['median']:.2f}, "
            f"p95 {pc['p95']:.2f}, max {pc['max']:.2f}]"
        )


def run_seeded(
    tmp_path: Path,
    name: str,
    scenario: str,
    state_out_addr: str,
    input_in_addr: str,
    seeds: tuple[int, ...] = SEEDS,
    **kw,
) -> SeededDistribution:
    """Runs `scenario` once per seed in `seeds` through `sim-host`, with
    `IMU_NOISE_DATASHEET_V1` wired in via `--imu-noise-seed`, and returns the
    resulting distribution.

    Same invocation shape as every acceptance file's own `_run` helper (e.g.
    `tests/test_cmd_envelope_reserve.py::_run`) -- `cargo run` rather than
    the raw binary path, for the same reason: Cargo puts the linked
    libmujoco's `OUT_DIR` on the dynamic loader path. `**kw` is passed
    through as `--flag value` exactly the same way, so a caller migrating an
    existing single-run scenario to this harness passes the identical
    keyword arguments it already had (`pitch_source`, `pitch_bias_deg`,
    `crr_scale`, ...).

    `state_out_addr`/`input_in_addr` are the caller's own ports (every
    acceptance file already owns a block of these to avoid colliding with a
    live capture session or another file's tests) -- runs here are
    sequential, so reusing one pair across all `len(seeds)` runs is safe.
    """
    results = []
    for seed in seeds:
        out = tmp_path / f"{name}_seed{seed}.csv"
        argv = [
            "cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
            "--scripted-scenario", scenario,
            "--free-run",
            "--stats-path", "none",
            "--state-out-addr", state_out_addr,
            "--input-in-addr", input_in_addr,
            "--trace-csv", str(out),
            "--imu-noise-seed", str(seed),
        ]
        for flag, value in kw.items():
            if value is None:
                continue
            argv += [f"--{flag.replace('_', '-')}", str(value)]
        proc = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
        assert proc.returncode == 0, f"sim-host failed (seed {seed}):\n{proc.stderr[-4000:]}"
        assert f"imu_noise_seed=Some({seed})" in proc.stderr, (
            f"sim-host's startup line does not record imu_noise_seed=Some({seed}) "
            "-- a seeded run whose seed cannot be read back off the run's own "
            "output is not a measurement (see tests/test_imu_noise.py)"
        )
        with out.open() as f:
            rows = [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]
        t_inv = next(
            (r["sim_time_s"] for r in rows if abs(r["truth_pitch_deg"]) > INVERTED_DEG), None
        )
        healthy = [r for r in rows if t_inv is None or r["sim_time_s"] < t_inv]
        results.append(
            SeededRunResult(
                seed=seed,
                inverted=t_inv is not None,
                inversion_time_s=t_inv,
                peak_pitch_deg=max(abs(r["truth_pitch_deg"]) for r in healthy),
                peak_current_a=max(abs(r["proposed_amps"]) for r in healthy),
            )
        )
    return SeededDistribution(tuple(results))
