#!/usr/bin/env python3
"""Prune old sha-stamped assets from the sim-preview release (issue #116).

Every PR run appends ~21 sha-stamped assets to the sim-preview release and
nothing ever removed old ones, so it hit GitHub's hard 1000-asset-per-release
cap and `publish-sim-artifact` started failing on every PR. This keeps the
most recent `--keep-runs` runs' worth of assets and deletes the rest.

Scoped by construction, not by convention: every asset this script can see
or delete comes from `gh api repos/<repo>/releases/<id>/assets`, where <id>
is resolved from the one `--release` tag passed in. There is no code path
here that can reach a different release's assets, so this cannot touch
`sim-latest` or `sim-experiments` no matter what `--release` is given.

A deletion failure exits non-zero rather than reporting success having
pruned nothing quietly -- a green publish job that silently didn't publish
(or here, didn't prune) is the exact failure class issue #116 called out.
"""
import argparse
import json
import os
import re
import subprocess
import sys
from collections import defaultdict

# ~21 assets land per PR run against GitHub's 1000-asset-per-release cap.
# Keeping the last 30 runs is ~630 assets -- real headroom below the cap
# even if the per-run asset count grows somewhat over time.
SIM_PREVIEW_KEEP_RUNS = 30

# Matches the sha-stamped suffix `publish-sim-artifact` writes:
# "<base>-<7-hex-sha>.<ext>" or "<base>-<7-hex-sha>" for extension-less files.
RUN_ID_RE = re.compile(r"-([0-9a-f]{7})(\.[^.]*)?$")


def gh_api(*args: str) -> str:
    result = subprocess.run(
        ["gh", "api", *args],
        check=True,
        capture_output=True,
        text=True,
    )
    return result.stdout


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--release", required=True, help="release TAG to prune (only this release is ever touched)"
    )
    parser.add_argument("--keep-runs", type=int, default=SIM_PREVIEW_KEEP_RUNS)
    parser.add_argument(
        "--repo", default=os.environ.get("GITHUB_REPOSITORY"), help="OWNER/REPO, defaults to $GITHUB_REPOSITORY"
    )
    args = parser.parse_args()

    if not args.repo:
        print("::error::--repo not given and GITHUB_REPOSITORY is unset", file=sys.stderr)
        return 1

    release = json.loads(gh_api(f"repos/{args.repo}/releases/tags/{args.release}"))
    release_id = release["id"]

    # --paginate concatenates every page into one JSON array, so this is the
    # full asset list for THIS release id regardless of how many pages it spans.
    assets = json.loads(gh_api("--paginate", f"repos/{args.repo}/releases/{release_id}/assets"))

    runs: dict[str, list[dict]] = defaultdict(list)
    unmatched = []
    for asset in assets:
        m = RUN_ID_RE.search(asset["name"])
        if not m:
            unmatched.append(asset["name"])
            continue
        runs[m.group(1)].append(asset)

    if unmatched:
        print(f"leaving {len(unmatched)} non-sha-stamped asset(s) alone: {unmatched}")

    # A run's recency is the newest created_at among its own assets.
    run_recency = {run_id: max(a["created_at"] for a in group) for run_id, group in runs.items()}
    ordered_runs = sorted(run_recency, key=lambda r: run_recency[r], reverse=True)

    keep_runs = set(ordered_runs[: args.keep_runs])
    prune_runs = [r for r in ordered_runs if r not in keep_runs]

    print(
        f"release '{args.release}' (id {release_id}): {len(assets)} asset(s) across "
        f"{len(ordered_runs)} run(s); keeping the {len(keep_runs)} most recent, "
        f"pruning {len(prune_runs)} older run(s)"
    )

    to_delete = [asset for run_id in prune_runs for asset in runs[run_id]]
    if not to_delete:
        print("nothing to prune")
        return 0

    failures = []
    for asset in to_delete:
        try:
            subprocess.run(
                ["gh", "api", "-X", "DELETE", f"repos/{args.repo}/releases/assets/{asset['id']}"],
                check=True,
                capture_output=True,
                text=True,
            )
        except subprocess.CalledProcessError as exc:
            failures.append((asset["name"], exc.stderr))

    deleted = len(to_delete) - len(failures)
    print(f"deleted {deleted}/{len(to_delete)} asset(s)")

    if failures:
        for name, err in failures:
            print(f"::error::failed to delete asset '{name}': {err.strip()}", file=sys.stderr)
        return 1

    return 0


if __name__ == "__main__":
    sys.exit(main())
