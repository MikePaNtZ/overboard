#!/usr/bin/env python3
"""Report the closed-loop disturbance-rejection envelope (issue #24, AC2).

Sweeps a grid of impulse magnitudes against the driverless closed-loop
board, prints the per-magnitude table, and writes the same data as JSON so
it is archived rather than left in a terminal scrollback -- the same
git-SHA-traceability convention `scripts/experiment.py` uses.

    cargo build --release -p control-ffi
    .venv/bin/python scripts/disturbance_envelope.py

Requires the control-ffi library to be built (see `sim/scenarios/rust_controller.py`).
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.disturbance_envelope import sweep_closed_loop  # noqa: E402
from sim.scenarios.impulse_response import NOMINAL_IMPULSE_NS, load_model  # noqa: E402
from sim.scenarios.rust_controller import RustController  # noqa: E402

OUT_DIR = REPO / "sim" / "out"


def git_sha() -> str:
    try:
        out = subprocess.run(["git", "rev-parse", "--short", "HEAD"], cwd=REPO,
                             capture_output=True, text=True, check=True)
        dirty = subprocess.run(["git", "status", "--porcelain"], cwd=REPO,
                               capture_output=True, text=True, check=True)
        return out.stdout.strip() + ("-dirty" if dirty.stdout.strip() else "")
    except Exception:
        return "unknown"


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--low", type=float, default=NOMINAL_IMPULSE_NS, help="N*s, sweep start")
    ap.add_argument("--high", type=float, default=320.0, help="N*s, sweep end (inclusive)")
    ap.add_argument("--step", type=float, default=20.0, help="N*s, grid spacing")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    magnitudes = []
    m = args.low
    while m <= args.high + 1e-9:
        magnitudes.append(m)
        m += args.step

    model = load_model()
    result = sweep_closed_loop(model, RustController, magnitudes)

    print(f"{'magnitude (N*s)':>16}  {'nose strike':>11}  {'peak |pitch| (deg)':>18}")
    for p in result.points:
        print(f"{p.magnitude_ns:16.1f}  {str(p.nose_strike):>11}  {p.peak_abs_pitch_deg:18.2f}")

    print(f"\nfirst failure       {result.first_failure_ns} N*s")
    print(f"recovery boundary   {result.recovery_boundary_ns} N*s")

    args.out_dir.mkdir(parents=True, exist_ok=True)
    out_path = args.out_dir / "disturbance_envelope_metrics.json"
    out_path.write_text(
        json.dumps({"git_sha": git_sha(), **result.to_json_dict()}, indent=2) + "\n"
    )
    print(f"\nwrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
