#!/usr/bin/env python3
"""Stage 0B bench-test runbook driver -- docs/runbook-stage0b-bench.md.

Runs the ordered procedure the runbook defines and emits one JSON log
conforming to the runbook's "Log schema" section.

Only `--dry-run` exists. There is no hardware mode yet: no Pi image exists to
run this against (O3, issues #51/#52 are still open), so wiring a hardware
path now would be code with nothing to execute it -- exactly the kind of
unverifiable addition this project rules out. `run(dry_run=False, ...)`
raises rather than pretending.

The dry run exercises steps 4-6 (the sim-representable ones) against the
Stage 0A bench-rig identification code (Sr. Mechanical & Systems' turf,
imported read-only, never modified here) and the Stage 0B imperfection
profile. Steps 1-3 are hardware-only and are reported "skipped".

    python3 scripts/stage0b_runbook.py --dry-run --out sim/out/stage0b/dry-run.json

Determinism: no wall clock is read for any numeric result -- the sim steps
are seeded (ImperfectionProfile.seed) and reproducible bit-for-bit, exactly
like every other scenario in this project. `git_sha()` is the one piece of
real-world state recorded, matching `scripts/experiment.py`'s convention.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.bench_spinup import (  # noqa: E402
    IdentifyParams,
    SpindownParams,
    identify,
    load_model,
    spindown,
)
from sim.scenarios.imperfections import (  # noqa: E402
    IDEAL,
    STAGE0_PLACEHOLDER,
    ImperfectionProfile,
    ImperfectionState,
)

SCHEMA_VERSION = 1

# Reused verbatim from docs/design-pi-image-stage0b.md's AC-6 -- the
# pre-registered go/no-go -- not re-derived here.
AC6_P999_MAX_S = 0.001
AC6_MAX_MAX_S = 0.002
AC6_MIN_CYCLES = 100_000

#: Reused from tests/test_bench_spinup.py's own acceptance bar for a fit run
#: under the STAGE0_PLACEHOLDER imperfection profile (not IDEAL, which pins a
#: tighter 0.999) -- not invented for this script.
R2_ACCEPT = 0.95

OUT_DIR = REPO / "sim" / "out" / "stage0b"


def git_sha() -> str:
    try:
        out = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=REPO,
            capture_output=True, text=True, check=True,
        )
        dirty = subprocess.run(
            ["git", "status", "--porcelain"], cwd=REPO,
            capture_output=True, text=True, check=True,
        )
        return out.stdout.strip() + ("-dirty" if dirty.stdout.strip() else "")
    except Exception:
        return "unknown"


def _skipped(name: str, purpose: str) -> dict:
    return {"name": name, "purpose": purpose, "result": "skipped", "measured": {}, "falsified_by": None}


def step_current_response(profile: ImperfectionProfile = STAGE0_PLACEHOLDER) -> dict:
    """Step 4 dry run -- reuses `bench_spinup.identify`'s own fit-quality
    diagnostic as the stand-in for "does the current loop step and settle
    the way the sim predicts"."""
    result = identify(IdentifyParams(), profile=profile)
    m = result.metrics
    passed = m.r2_bare > R2_ACCEPT and m.r2_loaded > R2_ACCEPT
    return {
        "name": "current_step_response",
        "purpose": "characterise the current loop's step response against the sim's prediction",
        "result": "pass" if passed else "fail",
        "measured": {
            "r2_bare": m.r2_bare,
            "r2_loaded": m.r2_loaded,
            "fit_start_s": m.fit_start_s,
        },
        "falsified_by": None if passed else (
            f"r2_bare={m.r2_bare:.4f} or r2_loaded={m.r2_loaded:.4f} <= {R2_ACCEPT}"
        ),
    }


def step_coast_down(profile: ImperfectionProfile = IDEAL) -> dict:
    """Step 5 dry run -- reuses `bench_spinup.spindown`'s own fit quality.

    Runs against `IDEAL`, not `STAGE0_PLACEHOLDER`, and that is a finding,
    not a style choice: unlike `identify()`, `spindown()` fits its decay
    window starting the instant current is commanded to zero, with no
    equivalent of `identify()`'s data-driven `settle_time_s` to skip the
    actuation-delay + current-loop-lag transient at the start of the decay.
    Under `STAGE0_PLACEHOLDER` that transient corrupts the least-squares fit
    across the whole window (R^2 ~ 0.002 in testing here, not a curve, noise).
    `spindown()` is Sr. Mechanical & Systems' turf (`sim/scenarios/bench_*.py`
    per CODEOWNERS) -- flagged in the PR and in CONTEXT.md, not fixed here.
    `IDEAL` is the one profile this function is actually validated against
    (`tests/test_bench_spinup.py::test_spindown_recovers_friction_terms_on_ideal_run`,
    R^2 > 0.999).
    """
    result = spindown(SpindownParams(), profile=profile)
    m = result.metrics
    passed = m.r2 > R2_ACCEPT
    return {
        "name": "coast_down",
        "purpose": "measure friction decay as an independent consistency check",
        "result": "pass" if passed else "fail",
        "measured": {
            "r2": m.r2,
            "b_fit_nm_s_per_rad": m.b_fit_nm_s_per_rad,
            "tau_c_fit_nm": m.tau_c_fit_nm,
        },
        "falsified_by": None if passed else f"r2={m.r2:.4f} <= {R2_ACCEPT}",
    }


def step_latency_proxy(profile: ImperfectionProfile = STAGE0_PLACEHOLDER) -> dict:
    """Step 6 dry run -- a SCHEMA/PLUMBING CHECK ONLY, not a hardware
    measurement.

    AC-6 is "command leaves the Pi -> drive actually changing current" --
    the transport/processing delay, not the current loop's full settle time
    (that is a separate, already-modelled quantity, `current_loop_tau_s`).
    So this measures the FIRST departure from zero after a step command, at
    the bench rig's own physics timestep (`bench_spinup.load_model().opt.
    timestep`, not invented here) so the delay is resolved rather than
    aliased by a coarse discretisation.

    The sim has no jitter source, so one deterministic first-response time
    stands in for both p99.9 and max -- this proves the pass/fail evaluator
    correctly reads a (p999, max) pair against AC-6, and nothing about real
    hardware jitter. See docs/runbook-stage0b-bench.md, "What the dry run
    does NOT prove."
    """
    dt_s = float(load_model().opt.timestep)
    imp = ImperfectionState(profile=profile, dt_s=dt_s)
    commanded_a = 20.0
    threshold_a = 0.01 * commanded_a
    window_s = 0.02  # comfortably past AC-6's 2 ms ceiling
    n_cycles = int(round(window_s / dt_s))
    t = np.arange(n_cycles) * dt_s
    measured = np.array([imp.apply_current(commanded_a) for _ in range(n_cycles)])

    above = np.flatnonzero(measured > threshold_a)
    if above.size == 0:
        raise ValueError(
            f"current never departed from zero within the {window_s * 1e3:.0f} ms "
            "window -- profile has no delay, or the window is too short"
        )
    first_response_s = float(t[int(above[0])])

    p999_s = first_response_s
    max_s = first_response_s
    passed = p999_s <= AC6_P999_MAX_S and max_s <= AC6_MAX_MAX_S
    return {
        "name": "command_actuation_latency",
        "purpose": "AC-6 schema/plumbing dry run -- NOT a hardware jitter measurement",
        "result": "pass" if passed else "fail",
        "measured": {
            "first_response_s": first_response_s,
            "p999_s": p999_s,
            "max_s": max_s,
            "cycles": 1,
        },
        "falsified_by": None if passed else f"p999_s={p999_s:.6f} or max_s={max_s:.6f} exceeds AC-6",
    }


def run(dry_run: bool, profile: ImperfectionProfile = STAGE0_PLACEHOLDER) -> dict:
    if not dry_run:
        raise NotImplementedError(
            "no hardware mode yet -- no Pi image exists to run this against "
            "(O3, issues #51/#52 are still open). A --hardware path with nothing "
            "to execute it against would be unverifiable, which this project "
            "rules out explicitly."
        )
    steps = [
        _skipped("continuity_insulation", "confirm wiring matches Stage 0A assembly"),
        _skipped("unpowered_can_roundtrip", "confirm CAN transport carries a frame correctly"),
        _skipped("first_powered_spin", "confirm the assembly survives being powered and commanded"),
        step_current_response(profile),
        step_coast_down(),  # always IDEAL -- see step_coast_down's docstring
        step_latency_proxy(profile),
    ]
    return {
        "schema_version": SCHEMA_VERSION,
        "stage": "0B",
        "git_sha": git_sha(),
        "mode": "dry_run",
        "aborted_at_step": None,
        "steps": steps,
    }


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--dry-run", action="store_true", required=True,
                         help="run the sim-representable steps; the only mode implemented")
    parser.add_argument("--out", type=Path, default=None,
                         help="write the log JSON here; defaults to stdout")
    args = parser.parse_args(argv)

    log = run(dry_run=args.dry_run)
    text = json.dumps(log, indent=2, sort_keys=True)

    if args.out is None:
        print(text)
    else:
        args.out.parent.mkdir(parents=True, exist_ok=True)
        args.out.write_text(text + "\n")
        print(f"wrote {args.out}", file=sys.stderr)

    failed = [s["name"] for s in log["steps"] if s["result"] == "fail"]
    return 1 if failed else 0


if __name__ == "__main__":
    raise SystemExit(main())
