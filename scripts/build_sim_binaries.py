#!/usr/bin/env python3
"""Build the Rust release binaries the sim test suite and sim-artifact
publishing both need.

`sim` and `publish-sim-artifact` (`.github/workflows/ci.yml`) each build this
same set before running pytest -- the workspace suite shells out to these
binaries and FAILS rather than skips if one is missing (I1b/I1c: a
closed-loop gate that passes because the controller was silently absent
would be worse than no test at all).

Every time a binary was added here, `sim` picked it up and
`publish-sim-artifact` did not -- three times in one day (#124), because the
build step lived by hand in two workflow jobs. This list is now the one
place to edit; both jobs call this script instead of repeating the steps.
"""

from __future__ import annotations

import subprocess
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]

# (package, optional --bin name). Add a new binary HERE, not in ci.yml.
BINARIES: list[tuple[str, str | None]] = [
    ("control-ffi", None),
    ("plant-mujoco", "plant-replay"),
    ("board-app-driverless", "impulse-response-rust"),
]


def main() -> int:
    for package, bin_name in BINARIES:
        cmd = ["cargo", "build", "--release", "-p", package]
        if bin_name:
            cmd += ["--bin", bin_name]
        print(f"+ {' '.join(cmd)}", flush=True)
        result = subprocess.run(cmd, cwd=REPO_ROOT)
        if result.returncode != 0:
            return result.returncode
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
