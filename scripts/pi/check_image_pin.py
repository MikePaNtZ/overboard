#!/usr/bin/env python3
"""The kernel pin appears twice. Fail the build if the copies disagree.

`scripts/pi/pins.env` is the single source of truth and is what the
verification jobs check against the live archive. But rpi-image-gen reads
YAML and cannot source a shell file, so `scripts/pi/image/layer/linux-v8-rt.yaml`
carries the same version a second time.

Duplication is normally a mistake, and this repository has the scar to prove
it -- issue #124, "sim and publish-sim-artifact duplicate every build step,
and drift apart every time a binary is added, three for three". The
difference between a duplicate that is fine and one that rots is whether
anything checks. This checks.

The failure it prevents is specific and nasty: pins.env says 6.12.75, the
image layer says something else, `verify_kernel.sh` happily verifies the
version in pins.env, and the card boots a kernel nobody validated. Every
latency number taken on it would be conditioned on an unrecorded variable --
which is precisely what design section 2 pins the kernel to prevent.
"""

from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[2]
PINS = REPO / "scripts" / "pi" / "pins.env"
LAYER = REPO / "scripts" / "pi" / "image" / "layer" / "linux-v8-rt.yaml"


def read_pins(path: Path) -> dict[str, str]:
    out: dict[str, str] = {}
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        key, _, value = line.partition("=")
        out[key.strip()] = value.strip().strip('"').strip("'")
    return out


def layer_packages(path: Path) -> list[str]:
    """Every `- <pkg>=<ver>` entry under the layer's packages list."""
    return re.findall(r"^\s*-\s*(linux-image-\S+=\S+)\s*$", path.read_text(), re.M)


def layer_holds(path: Path) -> list[str]:
    """Packages the layer passes to `apt-mark hold`."""
    return re.findall(r"apt-mark\s+hold\s+(\S+)", path.read_text())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--pins", type=Path, default=PINS)
    ap.add_argument("--layer", type=Path, default=LAYER)
    args = ap.parse_args()

    for p in (args.pins, args.layer):
        if not p.exists():
            print(f"error: missing {p}", file=sys.stderr)
            return 1

    pins = read_pins(args.pins)
    pkg, ver = pins.get("KERNEL_PACKAGE", ""), pins.get("KERNEL_VERSION", "")
    if not pkg or not ver:
        print("error: pins.env has no KERNEL_PACKAGE/KERNEL_VERSION", file=sys.stderr)
        return 1

    expected = f"{pkg}={ver}"
    problems: list[str] = []

    packages = layer_packages(args.layer)
    if expected not in packages:
        problems.append(
            f"the image layer does not install the pinned kernel.\n"
            f"    pins.env expects: {expected}\n"
            f"    layer installs:   {packages or ['<nothing matching linux-image-*=*>']}"
        )

    # A pin that is installed but not held is a pin with a shelf life: one
    # `apt upgrade` on the bench and the measurement variable has moved.
    holds = layer_holds(args.layer)
    if pkg not in holds:
        problems.append(
            f"the image layer installs {pkg} but never `apt-mark hold`s it "
            f"(AC-2). Held packages found: {holds or '<none>'}"
        )

    # The design's hard floor is >= 6.12, not this specific patch release.
    m = re.search(r"(\d+)\.(\d+)", ver)
    if m and (int(m.group(1)), int(m.group(2))) < (6, 12):
        problems.append(
            f"kernel {ver} is below the 6.12 floor. 6.12 is required because it "
            f"contains the mcp251xfd receive-latency fix (upstream eb9a839); "
            f"below it, the CAN tail this project measures is a different "
            f"kernel's tail."
        )

    if "-rt" not in pkg:
        problems.append(
            f"{pkg} is not a real-time kernel. The whole of Stage 0B is a "
            f"latency measurement; a non-RT kernel would build green and "
            f"measure the wrong thing."
        )

    if problems:
        print("IMAGE PIN CHECK FAILED\n")
        for p in problems:
            print(f"  x {p}\n")
        print("pins.env is the source of truth. Change it there, then match the layer.")
        return 1

    print(f"image pin check: {expected} installed and held, consistent with pins.env")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
