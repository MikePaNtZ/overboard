"""Guard: every scenario's rolling-radius constant must match the sim model.

Issue #24 (O1) AC3 flagged that `R_EFF_M`/`DEFAULT_R_EFF_M` (used to convert
`wheel_hinge` angular rate to a forward speed, and current to force, across
`hill.py`/`terrain.py`/`shuttle_run.py`/`rust_controller.py`) was hand-copied
as `0.14605` m in half a dozen places, one of them (`hill.py`) claiming in a
comment that it "matches the tire in the model header" -- it did not. The
model's actual `wheel_geom` radius is 0.1454 m (the mesh-derived,
enclosure-clearance figure `sim/models/overboard_onewheel.xml`'s own header
documents), and a *loaded* rolling radius larger than the tire's own unloaded
geometric radius is not physically sensible regardless of provenance.

This does not re-litigate what the real, bench-measured loaded rolling radius
should eventually be (Stage-0's job, ICD 10.5) -- it only pins the sim-side
placeholder to be internally consistent with the one model every closed-loop
metric in this repo is actually measured against, and fails loudly the next
time either side drifts.

Issue #89 found the same stale `0.14605` m independently hand-copied four
more times, in `crates/` FFI-boundary defaults (`sim-backend`, `control-ffi`
x2, `canary-ridden`, `board-app-ridden`) -- out of scope for #76, since that
is ridden/hardware-path code, not sim scenarios. Those sites now import one
Rust constant, `board_types::DEFAULT_R_EFF_M`; the last test below checks it
against this file's Python-side constant so the two cannot drift apart again.
"""

from __future__ import annotations

import re
from pathlib import Path

from sim.scenarios.hill import R_EFF_M as HILL_R_EFF_M
from sim.scenarios.plant import build_model
from sim.scenarios.rust_controller import DEFAULT_R_EFF_M
from sim.scenarios.shuttle_run import R_EFF_M as SHUTTLE_R_EFF_M
from sim.scenarios.terrain import R_EFF_M as TERRAIN_R_EFF_M

_BOARD_TYPES_SRC = (
    Path(__file__).resolve().parent.parent / "crates" / "board-types" / "src" / "lib.rs"
)


def _model_wheel_radius_m() -> float:
    """Read the tire radius the physics actually simulates, not a copy of it."""
    model = build_model(ballast_mass=0.0, ballast_height=0.0, clamp_a=None)
    return float(model.geom("wheel_geom").size[0])


def test_model_wheel_radius_is_the_documented_145_4mm():
    """Regression pin on the number every constant below is checked against."""
    assert _model_wheel_radius_m() == 0.1454


def test_default_r_eff_m_matches_the_compiled_model():
    assert DEFAULT_R_EFF_M == _model_wheel_radius_m()


def test_every_scenario_shares_the_one_r_eff_m_constant():
    """No module may hand-copy its own value -- one constant, imported everywhere."""
    assert HILL_R_EFF_M == DEFAULT_R_EFF_M
    assert TERRAIN_R_EFF_M == DEFAULT_R_EFF_M
    assert SHUTTLE_R_EFF_M == DEFAULT_R_EFF_M


def test_rust_ffi_boundary_constant_matches_the_python_side():
    """Issue #89: the four `crates/` FFI-boundary sites that independently
    hand-copied a stale `0.14605` m now import `board_types::DEFAULT_R_EFF_M`
    instead. Nothing on the Python side can read that Rust constant directly,
    so this parses the source text for the pinned literal -- the same
    grep-able, CI-enforceable shape this repo already relies on elsewhere --
    and fails loudly if either side is edited without the other.
    """
    assert _BOARD_TYPES_SRC.is_file(), f"expected {_BOARD_TYPES_SRC} to exist"
    src = _BOARD_TYPES_SRC.read_text()
    match = re.search(r"pub const DEFAULT_R_EFF_M: f32 = ([0-9.]+);", src)
    assert match, "DEFAULT_R_EFF_M declaration not found in board-types/src/lib.rs"
    rust_value = float(match.group(1))
    assert rust_value == DEFAULT_R_EFF_M
