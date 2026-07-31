import json
import os
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

# --------------------------------------------------------------------------
# Claims manifest plumbing (issue #61, steps 1-2; design:
# docs/design-claims-manifest.md).
#
# A test that backs a public capability claim declares it with
# `@pytest.mark.claim(id, requirement=..., unit=..., gate=...)` and records
# the measured value through the `record_claim` fixture. **Both are
# required** -- a test that is marked but never calls `record_claim()` is a
# claim with a silently missing value, which is exactly the failure mode
# this mechanism exists to prevent, so it is a hard error, not a soft skip.
#
# Only tests that PASS contribute a claim. `scripts/emit_claims.py` runs the
# suite in a subprocess and reads the file this module writes at session end
# (`CLAIMS_RAW_PATH`, default `.claims_raw.json` at the repo root) to build
# the published `claims.json` -- see that script for the manifest schema.
# --------------------------------------------------------------------------

_REPO_ROOT = Path(__file__).resolve().parents[1]
CLAIMS_RAW_PATH = Path(os.environ.get("CLAIMS_RAW_PATH", _REPO_ROOT / ".claims_raw.json"))

#: nodeid -> recorded claim data, populated by `record_claim`.
_recorded_claims: dict[str, dict] = {}
#: nodeids whose "call" phase reported "passed".
_passed_nodeids: set[str] = set()


def pytest_configure(config: pytest.Config) -> None:
    config.addinivalue_line(
        "markers",
        "claim(id, requirement=None, unit=None, gate=None): declare that this "
        "test backs a claims-manifest entry. The test MUST call the "
        "record_claim fixture with the measured value, or the suite fails.",
    )


def pytest_runtest_setup(item: pytest.Item) -> None:
    """Catch the marked-but-never-requests-the-fixture case up front, before
    the test body even runs -- there is no value coming either way."""
    marker = item.get_closest_marker("claim")
    if marker is not None and "record_claim" not in item.fixturenames:
        pytest.fail(
            f"{item.nodeid} carries @pytest.mark.claim(...) but never requests "
            "the record_claim fixture -- a claim must record a measured value, "
            "it cannot be asserted silently",
            pytrace=False,
        )


@pytest.fixture
def record_claim(request: pytest.FixtureRequest):
    """Records the measured value a `@pytest.mark.claim(...)`-marked test
    pins. Fails the test if the marker is missing, or if the test finishes
    without ever calling the returned function."""
    marker = request.node.get_closest_marker("claim")
    if marker is None:
        raise RuntimeError(
            f"{request.node.nodeid} requests record_claim but carries no "
            "@pytest.mark.claim(...) -- mark the test first"
        )
    if not marker.args:
        raise RuntimeError(
            f"{request.node.nodeid}: @pytest.mark.claim(...) needs the claim "
            "id as its first argument"
        )
    claim_id = marker.args[0]
    state = {"called": False}

    def _record(value) -> None:
        state["called"] = True
        _recorded_claims[request.node.nodeid] = {
            "id": claim_id,
            "value": value,
            "unit": marker.kwargs.get("unit"),
            "requirement": marker.kwargs.get("requirement"),
            "gate": marker.kwargs.get("gate"),
            "test": request.node.nodeid,
        }

    yield _record

    if not state["called"]:
        pytest.fail(
            f"{request.node.nodeid} is marked @pytest.mark.claim({claim_id!r}, "
            "...) but never called record_claim() -- a claim with a silently "
            "missing value is not allowed",
            pytrace=False,
        )


def pytest_runtest_logreport(report: pytest.TestReport) -> None:
    if report.when == "call" and report.outcome == "passed":
        _passed_nodeids.add(report.nodeid)


def pytest_sessionfinish(session: pytest.Session, exitstatus: int) -> None:
    """Only claims from tests that actually passed make it out -- a failing
    test's claim must be absent, not stale."""
    passing = {
        nodeid: data
        for nodeid, data in _recorded_claims.items()
        if nodeid in _passed_nodeids
    }
    CLAIMS_RAW_PATH.write_text(json.dumps(passing, indent=2) + "\n")
