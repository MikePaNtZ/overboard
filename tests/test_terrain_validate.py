"""ADR-0011 condition 2 (issue #208): the world-authoring asset rule.

Proves `sim.scenarios.terrain_validate` actually rejects a violating asset --
reading the validator is not evidence it works (issue #208's AC1) -- and that
every asset this repo ships passes it, which is the CI gate itself.
"""

from __future__ import annotations

import json
import math

import pytest

from sim.scenarios.terrain_validate import (
    DEFAULT_LIMITS,
    HeightfieldAsset,
    iter_shipped_assets,
    load_heightfield_asset,
    validate_asset_file,
    validate_heightfield,
)


def _write_asset(tmp_path, name, dx_m, elevations_m):
    path = tmp_path / f"{name}.json"
    path.write_text(json.dumps({"name": name, "dx_m": dx_m, "elevations_m": elevations_m}))
    return path


def test_a_flat_asset_has_no_violations():
    asset = HeightfieldAsset(name="flat", dx_m=0.5, elevations_m=(0.0,) * 20)
    assert validate_heightfield(asset) == []


def test_a_gentle_ramp_inside_both_limits_passes():
    # 1 deg grade, well inside the 3 deg default, at a spacing coarse enough
    # that the per-sample delta is also well inside the step limit.
    dx = 0.5
    dz_per_sample = dx * math.tan(math.radians(1.0))
    elevations = [i * dz_per_sample for i in range(50)]
    asset = HeightfieldAsset(name="gentle", dx_m=dx, elevations_m=tuple(elevations))
    assert validate_heightfield(asset) == []


def test_a_deliberately_authored_step_is_caught(tmp_path):
    """AC1: author a violating asset on purpose and watch it fail."""
    # 6 mm jump between two samples -- well above the 0.5 mm default step
    # limit -- at a spacing (1 cm) inside STEP_CHECK_MAX_DX_M, so it is
    # checkable as a discrete edge at all. It also trips the slope check
    # (atan(0.006 / 0.01) ~ 31 deg); that is correct, a true discontinuity
    # reads as an impossible grade too. The dedicated case below isolates a
    # slope violation from a step violation instead.
    elevations = [0.0, 0.0, 0.006, 0.006, 0.006]
    path = _write_asset(tmp_path, "step_violation", dx_m=0.01, elevations_m=elevations)

    violations = validate_asset_file(path)

    steps = [v for v in violations if v.kind == "step"]
    assert len(steps) == 1, violations
    assert steps[0].sample_index == 1
    assert steps[0].value == pytest.approx(0.006)
    assert steps[0].limit == DEFAULT_LIMITS.max_step_m


def test_a_coarse_ramp_is_not_falsely_flagged_as_a_step(tmp_path):
    """Regression: a smooth grade sampled coarser than STEP_CHECK_MAX_DX_M
    produces adjacent-sample deltas the same order of magnitude as the kerb
    figure, but it is not a step -- the surface between those two grid
    points is still the same smooth grade. Caught by
    test_a_gentle_ramp_inside_both_limits_passes during development; pinned
    explicitly here so it cannot regress silently."""
    dx = 0.5  # coarser than STEP_CHECK_MAX_DX_M
    dz_per_sample = dx * math.tan(math.radians(1.0))  # 1 deg grade, ~8.7 mm/sample
    assert dz_per_sample > DEFAULT_LIMITS.max_step_m, "the setup must actually exercise the coarse-sampling case"
    elevations = [i * dz_per_sample for i in range(20)]
    path = _write_asset(tmp_path, "coarse_ramp", dx_m=dx, elevations_m=elevations)

    violations = validate_asset_file(path)

    assert violations == []


def test_a_deliberately_authored_slope_is_caught_independent_of_step(tmp_path):
    """A sustained 10% grade sampled finely enough that no single step alone
    would trip the step check -- isolates the slope check from the step one."""
    dx = 0.001  # 1 mm spacing
    dz_per_sample = dx * 0.10  # 10% grade -> 0.1 mm per sample, under the 0.5 mm step limit
    assert dz_per_sample < DEFAULT_LIMITS.max_step_m
    elevations = [i * dz_per_sample for i in range(200)]
    path = _write_asset(tmp_path, "slope_violation", dx_m=dx, elevations_m=elevations)

    violations = validate_asset_file(path)

    assert violations, "a sustained 10% grade must be flagged"
    assert all(v.kind == "slope" for v in violations), violations
    assert violations[0].value == pytest.approx(math.degrees(math.atan(0.10)), rel=1e-6)


def test_load_heightfield_asset_rejects_a_degenerate_asset(tmp_path):
    path = _write_asset(tmp_path, "too_short", dx_m=1.0, elevations_m=[0.0])
    with pytest.raises(ValueError):
        load_heightfield_asset(path)

    path = _write_asset(tmp_path, "zero_spacing", dx_m=0.0, elevations_m=[0.0, 0.001])
    with pytest.raises(ValueError):
        load_heightfield_asset(path)


def test_every_shipped_terrain_asset_validates_clean():
    """The actual CI gate (AC3): whatever this repo ships must already pass."""
    shipped = iter_shipped_assets()
    assert shipped, "expected at least one example asset under sim/assets/terrain/"
    for path in shipped:
        violations = validate_asset_file(path)
        assert violations == [], f"{path} violates the terrain asset rule: {violations}"
