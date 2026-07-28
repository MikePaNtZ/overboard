"""Acceptance gate for the Stage 0B bench-test runbook driver.

docs/runbook-stage0b-bench.md's dry-run acceptance criterion is that the
procedure -- step ordering, schema, pass/fail evaluation -- is debugged
against the sim before hardware exists. These tests pin the schema shape and
the determinism the doc promises, not the physical numbers themselves (those
are pinned in tests/test_bench_spinup.py and tests/test_imperfections.py,
whose functions this driver reuses read-only).
"""

import sys
from pathlib import Path

import pytest

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO / "scripts"))

from stage0b_runbook import (  # noqa: E402
    AC6_MAX_MAX_S,
    AC6_P999_MAX_S,
    run,
    step_coast_down,
    step_current_response,
    step_latency_proxy,
)

HARDWARE_ONLY_STEPS = ("continuity_insulation", "unpowered_can_roundtrip", "first_powered_spin")
SIM_STEPS = ("current_step_response", "coast_down", "command_actuation_latency")


def test_hardware_only_mode_is_not_implemented():
    """No Pi image exists yet (O3, issues #51/#52 are open) -- a --hardware
    path with nothing to execute it against would be unverifiable."""
    with pytest.raises(NotImplementedError):
        run(dry_run=False)


def test_dry_run_reports_all_six_steps_in_order():
    log = run(dry_run=True)
    names = [s["name"] for s in log["steps"]]
    assert names == [
        "continuity_insulation",
        "unpowered_can_roundtrip",
        "first_powered_spin",
        "current_step_response",
        "coast_down",
        "command_actuation_latency",
    ]


def test_hardware_only_steps_are_skipped_not_evaluated():
    log = run(dry_run=True)
    by_name = {s["name"]: s for s in log["steps"]}
    for name in HARDWARE_ONLY_STEPS:
        assert by_name[name]["result"] == "skipped"
        assert by_name[name]["falsified_by"] is None


def test_sim_representable_steps_pass_on_the_committed_defaults():
    """The whole point of a dry run: it should be green against the repo's
    own defaults before anyone points it at hardware."""
    log = run(dry_run=True)
    by_name = {s["name"]: s for s in log["steps"]}
    for name in SIM_STEPS:
        assert by_name[name]["result"] == "pass", by_name[name]
        assert by_name[name]["falsified_by"] is None


def test_every_step_conforms_to_the_log_schema():
    log = run(dry_run=True)
    assert log["schema_version"] == 1
    assert log["stage"] == "0B"
    assert log["mode"] == "dry_run"
    assert isinstance(log["git_sha"], str) and log["git_sha"]
    for step in log["steps"]:
        assert set(step) == {"name", "purpose", "result", "measured", "falsified_by"}
        assert step["result"] in {"pass", "fail", "skipped"}
        assert isinstance(step["measured"], dict)


def test_dry_run_is_deterministic():
    """No wall clock feeds any numeric result -- two runs produce identical
    step results (git_sha aside, which is real-world state by design)."""
    a = run(dry_run=True)
    b = run(dry_run=True)
    assert a["steps"] == b["steps"]


def test_current_response_step_reuses_the_stage0_placeholder_profile():
    """Same acceptance bar as bench_spinup's own STAGE0_PLACEHOLDER test
    (tests/test_bench_spinup.py::test_the_r2_diagnostic_is_asserted_and_not_merely_reported,
    R^2 > 0.95) -- not re-derived here."""
    step = step_current_response()
    assert step["measured"]["r2_bare"] > 0.95
    assert step["measured"]["r2_loaded"] > 0.95


def test_coast_down_step_defaults_to_ideal_not_stage0_placeholder():
    """spindown() has no equivalent of identify()'s settle-time windowing,
    so STAGE0_PLACEHOLDER corrupts its fit -- see the step's docstring and
    docs/runbook-stage0b-bench.md's flagged finding. IDEAL is the only
    profile it is actually validated against
    (test_bench_spinup.py::test_spindown_recovers_friction_terms_on_ideal_run)."""
    step = step_coast_down()
    assert step["measured"]["r2"] > 0.999


def test_latency_proxy_step_measures_first_departure_from_zero():
    """AC-6 is command-leaves-the-Pi to actuation-starts-changing, not the
    current loop's full settle time -- a full-settle version of this check
    would fail against AC-6's 1 ms / 2 ms thresholds even though the model's
    own actuation_delay_s (1 ms, PROVISIONAL) is right at the boundary."""
    step = step_latency_proxy()
    assert step["measured"]["first_response_s"] == pytest.approx(0.001, abs=2e-4)
    assert step["measured"]["p999_s"] <= AC6_P999_MAX_S
    assert step["measured"]["max_s"] <= AC6_MAX_MAX_S
