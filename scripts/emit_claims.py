#!/usr/bin/env python3
"""Run the pytest suite and emit `claims.json` from what actually passed.

Design: `docs/design-claims-manifest.md` (issue #61, steps 1-2). A "claim" is
a `{value, unit, requirement, test, gate}` record a test pins with
`@pytest.mark.claim(...)` and `tests/conftest.py`'s `record_claim` fixture.
**A claim with no green test is not in the file.** That absence is the whole
mechanism -- the site cannot state something engineering is not currently
proving, and a failing (or deleted, or never-existing) test drops its claim
silently rather than publishing a stale number.

    scripts/emit_claims.py                          # runs `pytest tests/`, writes sim/out/claims.json
    scripts/emit_claims.py --output claims.json      # write somewhere else
    scripts/emit_claims.py -- -k test_closed_loop    # extra args pass through to pytest

Exit status mirrors pytest's: a red suite still emits a manifest (of whatever
passed), but the script's own exit code stays non-zero so a CI step chaining
on `&&` will not treat a partially-red run as success.

CI cannot verify a `requirement` id resolves to anything -- the requirement
register lives in Notion and ADR-0003 forbids the gate from querying it. That
field is carried through as a label, not checked here.
"""

from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_OUTPUT = REPO_ROOT / "sim" / "out" / "claims.json"
DEFAULT_RAW_PATH = REPO_ROOT / ".claims_raw.json"
SCHEMA_VERSION = 1


def resolve_sha() -> str | None:
    """The commit the manifest was generated from, short form.

    Preference order matches `scripts/render_scenario.py::source_commit`: CI
    passes the head sha explicitly (a `pull_request` build's `github.sha` is
    the ephemeral merge commit, which is not an ancestor of anything), and
    only falls back to a local `git rev-parse` off that.
    """
    sha = os.environ.get("ARTIFACT_SHA") or os.environ.get("GITHUB_SHA")
    if not sha:
        try:
            sha = subprocess.run(
                ["git", "rev-parse", "HEAD"],
                cwd=REPO_ROOT,
                capture_output=True,
                text=True,
                check=True,
            ).stdout.strip()
        except Exception:
            return None
    return sha[:7] if sha else None


def run_suite(raw_path: Path, pytest_args: list[str]) -> int:
    """Runs pytest in a subprocess with CLAIMS_RAW_PATH pointed at `raw_path`,
    so `tests/conftest.py` writes the passing claims there at session end."""
    raw_path.unlink(missing_ok=True)
    env = dict(os.environ)
    env["CLAIMS_RAW_PATH"] = str(raw_path)
    result = subprocess.run(
        [sys.executable, "-m", "pytest", *pytest_args],
        cwd=REPO_ROOT,
        env=env,
    )
    return result.returncode


def build_manifest(raw_path: Path) -> dict:
    raw: dict = json.loads(raw_path.read_text()) if raw_path.exists() else {}
    claims = {}
    for data in raw.values():
        claims[data["id"]] = {
            "value": data["value"],
            "unit": data["unit"],
            "requirement": data["requirement"],
            "test": data["test"],
            "gate": data["gate"],
        }
    return {
        "schema": SCHEMA_VERSION,
        "sha": resolve_sha(),
        "generated_at": datetime.now(timezone.utc).strftime("%Y-%m-%dT%H:%M:%SZ"),
        "claims": claims,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--output", type=Path, default=DEFAULT_OUTPUT,
        help=f"where to write claims.json (default: {DEFAULT_OUTPUT.relative_to(REPO_ROOT)})",
    )
    parser.add_argument(
        "--raw-path", type=Path, default=DEFAULT_RAW_PATH,
        help="intermediate per-test claims file the pytest run writes (default: repo-root .claims_raw.json)",
    )
    parser.add_argument(
        "pytest_args", nargs=argparse.REMAINDER,
        help="extra args forwarded to pytest (default: tests/ -q)",
    )
    args = parser.parse_args()

    pytest_args = [a for a in args.pytest_args if a != "--"] or ["tests/", "-q"]

    suite_exit = run_suite(args.raw_path, pytest_args)

    manifest = build_manifest(args.raw_path)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(manifest, indent=2) + "\n")

    n = len(manifest["claims"])
    print(f"wrote {n} claim{'s' if n != 1 else ''} to {args.output}")

    return suite_exit


if __name__ == "__main__":
    raise SystemExit(main())
