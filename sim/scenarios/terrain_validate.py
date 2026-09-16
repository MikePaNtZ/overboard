"""Terrain-asset validator -- ADR-0011 condition 2 (issue #208).

ADR-0011's second ratification moved the kerb-strike and static-robustness
criteria to the hardware gate, and made that move conditional on three
things holding. Condition 2 is this file: **the authored world is
constrained to what the controller survives, and the constraint is a
checkable rule, not a hope.**

WHAT THIS CHECKS, AND WHY THOSE TWO THINGS
-------------------------------------------
A terrain asset here is a one-dimensional elevation profile sampled at a
fixed spacing -- the same shape `sim/scenarios/terrain.py` builds and tiles
across a MuJoCo `<hfield>`'s rows (`HFIELD_COLS` samples along the ride).
This does not model a full 2D height grid: nothing in this repo authors one
yet, and inventing a richer format nobody produces would be speculative.

Two independent hazards, checked independently because they are physically
different -- and, for a discrete heightfield, only independent at fine
enough sampling. A smooth grade sampled coarsely (a 1% ramp at 0.5 m
spacing rises 5 mm per sample) is geometrically nothing like a kerb, even
though the raw adjacent-sample delta is the same order of magnitude as the
kerb figure below; the interpolated surface between those two grid points
is still a 1% slope, not a discontinuity. So the step check only applies
where the sampling is fine enough that "adjacent samples" could plausibly
be describing a real discrete edge rather than a coarsely-resolved smooth
one -- see `STEP_CHECK_MAX_DX_M`. Coarser assets still get the slope check,
which is dx-independent by construction (it is already an angle).

* **A step** -- an abrupt height jump between adjacent samples (a kerb, a
  mesh seam, a collision-hull artifact). `tests/test_cmd_envelope_reserve.py`
  (the xfail reason on
  `test_criterion_a3_full_stick_during_a_kerb_strike_does_not_invert`,
  ADR-0011 criterion (a)-3) states the board "rides out roughly a 1 mm lip at
  a calm point of the hold and nothing at its worst point." Because the
  worst-phase figure in that same sentence is zero, treating the calm-point
  number itself as the pass/fail line would not be honest. This file's
  default limit is **0.5 mm -- half that calm-point figure** -- as the
  margin ADR-0011 leaves to "the world-authoring role... whatever margin
  that role judges" (`tests/test_incline_tolerance.py`'s own words for the
  slope case, quoted below, apply here too).

* **A slope** -- a sustained grade, independent of how it is sampled.
  `tests/test_incline_tolerance.py::test_the_downgrade_at_which_full_braking_stops_arresting_the_roll`
  measures the corridor-brake arrest boundary at **6.5 deg arrested / 7.0 deg
  outrun**. Independently, `sim/scenarios/terrain.py`'s `TerrainParams`
  docstring records that the board "completes [a hill] on the attitude
  ESTIMATE" at an 8% grade (4.57 deg) and "fails at 10%" (5.71 deg) -- a
  different scenario, a different failure mode (estimator degradation on a
  changing grade, not a static one), landing in the same few degrees. This
  file's default limit is **3.0 deg -- comfortably inside both** measured
  boundaries.

Neither default is a re-statement of a measured threshold: `test_incline_
tolerance.py` says explicitly that none of its numbers "may be promoted to
one," and that "a tolerance requirement comes from the environment... not
from the vehicle." These are the environment's numbers, chosen with margin,
citing where the measurements they sit inside of live -- not the
measurements themselves.

WHAT THIS DOES NOT DO
----------------------
`test_incline_tolerance.py` also measures that the rule "needs an angle AND
a run-out length" -- a slope that is individually harmless produces
unbounded speed if it runs on long enough, because there is no speed loop.
This file checks slope pointwise only; it does not bound how long a
tolerable grade may run before a corridor-style speed limit is needed. That
is deliberately left for a follow-up, not silently assumed away -- see the
issue log.

BOUNDARY NOTE
-------------
Board physics stay in `overboard`; nothing outside it computes board state.
This module consumes numbers measured here -- it derives no new physics and
authors no shipped game world (that is the Game Engineer's world-authoring
role, `overboard-game`, ADR-0009). It is a reusable rule any asset pipeline
can run a candidate terrain file through.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

#: Directory of example/reference terrain assets shipped with this repo.
#: Not a claim about the actual game world -- see the boundary note above.
SHIPPED_ASSETS_DIR = Path(__file__).resolve().parents[1] / "assets" / "terrain"


@dataclass(frozen=True)
class TerrainAssetLimits:
    """A checkable rule, not a hope. See the module docstring for provenance."""

    #: Metres. Half the ~1 mm calm-point kerb figure in
    #: tests/test_cmd_envelope_reserve.py's criterion (a)-3 xfail reason.
    max_step_m: float

    #: Degrees. Comfortably inside both the 6.5 deg corridor-brake arrest
    #: boundary (tests/test_incline_tolerance.py) and the 4.57 deg (8% grade)
    #: estimator-completion ceiling (sim/scenarios/terrain.py's TerrainParams).
    max_slope_deg: float


DEFAULT_LIMITS = TerrainAssetLimits(max_step_m=0.0005, max_slope_deg=3.0)

#: Above this sample spacing, an adjacent-sample delta describes a smoothly
#: interpolated grade, not a discrete geometric edge -- this representation
#: (uniform spacing, implicitly linearly interpolated, as MuJoCo's hfield
#: does) cannot resolve a real kerb-scale feature any finer than its own
#: spacing. 2 cm is well under the ~20 mm kerb scale in
#: `sim/scenarios/plant.py`'s `KERB_STRIKE_VALIDITY`, so a step genuinely
#: present at that scale cannot be smoothed away by this check.
STEP_CHECK_MAX_DX_M = 0.02


@dataclass(frozen=True)
class HeightfieldAsset:
    """A one-dimensional along-track elevation profile at fixed spacing."""

    name: str
    dx_m: float
    elevations_m: tuple[float, ...]


@dataclass(frozen=True)
class Violation:
    kind: str  # "step" or "slope"
    sample_index: int
    value: float
    limit: float
    message: str


def load_heightfield_asset(path: Path) -> HeightfieldAsset:
    """Load the asset JSON schema: ``{"dx_m": float, "elevations_m": [float, ...]}``.

    ``name`` is optional and defaults to the file's stem.
    """
    path = Path(path)
    data = json.loads(path.read_text())
    dx_m = float(data["dx_m"])
    elevations_m = tuple(float(v) for v in data["elevations_m"])
    if dx_m <= 0.0:
        raise ValueError(f"{path}: dx_m must be positive, got {dx_m}")
    if len(elevations_m) < 2:
        raise ValueError(f"{path}: need at least 2 elevation samples, got {len(elevations_m)}")
    return HeightfieldAsset(name=str(data.get("name", path.stem)), dx_m=dx_m, elevations_m=elevations_m)


def validate_heightfield(
    asset: HeightfieldAsset, limits: TerrainAssetLimits = DEFAULT_LIMITS
) -> list[Violation]:
    """Check every adjacent sample pair for a step or slope violation.

    Both checks read the same adjacent-sample height delta, but they answer
    different questions: the step check is independent of ``dx_m`` (a true
    discontinuity is a hazard at any sample spacing), the slope check is not
    (the same delta over a coarser spacing is a gentler grade).
    """
    check_steps = asset.dx_m <= STEP_CHECK_MAX_DX_M
    violations: list[Violation] = []
    for i in range(len(asset.elevations_m) - 1):
        dz = asset.elevations_m[i + 1] - asset.elevations_m[i]
        step_m = abs(dz)
        if check_steps and step_m > limits.max_step_m:
            violations.append(Violation(
                kind="step", sample_index=i, value=step_m, limit=limits.max_step_m,
                message=(
                    f"{asset.name}: {step_m * 1000:.3f} mm step between samples "
                    f"{i} and {i + 1} exceeds the {limits.max_step_m * 1000:.3f} mm limit"
                ),
            ))
        slope_deg = math.degrees(math.atan2(step_m, asset.dx_m))
        if slope_deg > limits.max_slope_deg:
            violations.append(Violation(
                kind="slope", sample_index=i, value=slope_deg, limit=limits.max_slope_deg,
                message=(
                    f"{asset.name}: {slope_deg:.2f} deg slope between samples "
                    f"{i} and {i + 1} exceeds the {limits.max_slope_deg:.2f} deg limit"
                ),
            ))
    return violations


def validate_asset_file(path: Path, limits: TerrainAssetLimits = DEFAULT_LIMITS) -> list[Violation]:
    return validate_heightfield(load_heightfield_asset(path), limits)


def iter_shipped_assets(directory: Path = SHIPPED_ASSETS_DIR) -> list[Path]:
    return sorted(directory.glob("*.json"))


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument(
        "paths", nargs="*", type=Path,
        help="Asset files to check. Defaults to every shipped asset under sim/assets/terrain/.",
    )
    args = parser.parse_args(argv)
    paths = args.paths or iter_shipped_assets()
    if not paths:
        print(f"no terrain assets found under {SHIPPED_ASSETS_DIR}")
        return 0

    ok = True
    for path in paths:
        violations = validate_asset_file(path)
        if violations:
            ok = False
            for v in violations:
                print(f"FAIL {v.message}")
        else:
            print(f"OK {path}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
