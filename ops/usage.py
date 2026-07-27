#!/usr/bin/env python3
"""Claude usage sensor. Reads local transcripts. Costs zero model tokens.

    python3 ops/usage.py                 # today + 7-day history + top sessions
    python3 ops/usage.py --check         # exit 1 if today is over the ceiling
    python3 ops/usage.py --ceiling 40    # ceiling in millions of weighted tokens

WHY THIS IS A SCRIPT AND NOT AN EVENT LOOP
------------------------------------------
A long-lived session that wakes on a timer to check a number pays a cache read
on its ENTIRE context window every tick -- six figures of tokens per day to
observe a number that costs nothing to read from disk. Worse, such a loop has
no actuator: it cannot pause a session, reclaim tokens, or slow another role.
The only actuator in this org is DISPATCH. So the sensor lives next to the
actuator: `ops/dispatch.md` calls this before spawning anything.

COUNTING
--------
A naive sum of `input_tokens` is wrong by about three orders of magnitude --
a cached turn reports `input_tokens: 10` next to `cache_read_input_tokens:
160000`. All four classes are summed, and a weighted total approximates
billable cost. Weights are relative to an input token; edit them here if
pricing changes.
"""

from __future__ import annotations

import argparse
import json
import os
from collections import defaultdict
from datetime import date, datetime, timedelta
from pathlib import Path

ROOT = Path.home() / ".claude" / "projects"
PROJECT_GLOB = "-Users-mike-projects-overboard*"

# Relative to one input token. Cache reads are cheap, output is expensive.
WEIGHTS = {
    "input_tokens": 1.0,
    "cache_creation_input_tokens": 1.25,
    "cache_read_input_tokens": 0.1,
    "output_tokens": 5.0,
}

# Optional: {"<session-uuid>": "Senior Controls", ...}. Absent -> unattributed.
ROLE_MAP = Path(__file__).parent / "session-roles.json"


def transcripts() -> list[tuple[Path, str, bool]]:
    """[(jsonl path, owning session id, is_subagent)]"""
    out: list[tuple[Path, str, bool]] = []
    for proj in ROOT.glob(PROJECT_GLOB):
        for f in proj.glob("*.jsonl"):
            out.append((f, f.stem, False))
        for sub in proj.glob("*/subagents/**/*.jsonl"):
            # .../<project>/<session-id>/subagents/...
            parts = sub.relative_to(proj).parts
            out.append((sub, parts[0], True))
    return out


def scan() -> tuple[dict, dict, dict]:
    by_day: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    by_session: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    sub_share: dict[str, int] = defaultdict(int)

    for path, session, is_sub in transcripts():
        try:
            text = path.read_text(encoding="utf-8", errors="ignore")
        except OSError:
            continue
        for line in text.splitlines():
            if '"usage"' not in line:
                continue
            try:
                d = json.loads(line)
            except json.JSONDecodeError:
                continue
            usage = (d.get("message") or {}).get("usage")
            if not isinstance(usage, dict):
                continue
            ts = d.get("timestamp") or ""
            day = ts[:10] or "unknown"
            for k in WEIGHTS:
                v = usage.get(k) or 0
                if isinstance(v, int):
                    by_day[day][k] += v
                    by_session[session][k] += v
                    if is_sub:
                        sub_share[day] += v
    return by_day, by_session, sub_share


def weighted(bucket: dict[str, int]) -> float:
    return sum(bucket.get(k, 0) * w for k, w in WEIGHTS.items())


def roles() -> dict[str, str]:
    if ROLE_MAP.is_file():
        try:
            return json.loads(ROLE_MAP.read_text())
        except json.JSONDecodeError:
            pass
    return {}


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="exit 1 if today is over the ceiling")
    env = os.environ.get("OPS_USAGE_CEILING_M")
    ap.add_argument("--ceiling", type=float, default=float(env) if env else None,
                    help="daily ceiling, millions of weighted tokens")
    args = ap.parse_args()

    by_day, by_session, sub_share = scan()
    today = date.today().isoformat()
    today_w = weighted(by_day.get(today, {})) / 1e6

    if args.check:
        # No invented ceiling. Until the CEO sets one from the real plan limit
        # this reports and passes -- a fake gate is worse than none, because it
        # would be tuned around rather than believed.
        if args.ceiling is None:
            print(f"usage: {today_w:.1f}M weighted today. NO CEILING SET "
                  f"(export OPS_USAGE_CEILING_M) -- reporting only, not gating.")
            return 0
        over = today_w > args.ceiling
        print(f"usage: {today_w:.1f}M weighted today, ceiling {args.ceiling:.0f}M "
              f"-- {'OVER, do not dispatch' if over else 'ok to dispatch'}")
        return 1 if over else 0

    cap = f"Ceiling {args.ceiling:.0f}M/day." if args.ceiling else "No ceiling set."
    print(f"Claude usage -- weighted tokens (millions). {cap}\n")
    print(f"{'day':<12}{'total':>9}{'output':>10}{'cache-rd':>11}{'subagent%':>11}")
    cutoff = (date.today() - timedelta(days=7)).isoformat()
    for day in sorted(d for d in by_day if d >= cutoff):
        b = by_day[day]
        tot = weighted(b) / 1e6
        raw = sum(b.get(k, 0) for k in WEIGHTS)
        pct = (sub_share.get(day, 0) / raw * 100) if raw else 0
        flag = "  <-- OVER" if (args.ceiling and tot > args.ceiling) else ""
        print(f"{day:<12}{tot:>9.1f}{b.get('output_tokens',0)/1e6:>10.2f}"
              f"{b.get('cache_read_input_tokens',0)/1e6:>11.1f}{pct:>10.0f}%{flag}")

    rmap = roles()
    print(f"\nTop sessions (weighted M):")
    top = sorted(by_session.items(), key=lambda kv: -weighted(kv[1]))[:8]
    for sid, b in top:
        role = rmap.get(sid, "unattributed")
        print(f"  {weighted(b)/1e6:>7.1f}  {sid[:8]}  {role}")

    if not rmap:
        print(f"\n  NOTE: no {ROLE_MAP.name} -- per-role attribution is NOT MEASURED.")
        print("  Add {\"<session-uuid>\": \"<role>\"} entries to attribute spend by department.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
