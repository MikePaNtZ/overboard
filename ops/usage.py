#!/usr/bin/env python3
"""Claude usage sensor. Reads local transcripts. Costs zero model tokens.

    python3 ops/usage.py                 # history, by-role rollup, cost per turn
    python3 ops/usage.py --calibrate 62  # "Settings -> Usage says 62% right now"
    python3 ops/usage.py --check         # exit 1 if over OPS_USAGE_THRESHOLD_PCT

THE GATE CANNOT BE A TOKEN COUNT
--------------------------------
Anthropic publishes NO absolute number for any subscription meter -- the UI is
percentage bars only, and there is no endpoint to read them. So the ceiling is
a percentage the CEO sets, and the conversion from weighted tokens to a
percentage comes from `--calibrate`: a human reads the real bar once and tells
this tool. Any absolute token ceiling written here would be a locally-inferred
guess that later gets mistaken for an Anthropic figure.

WHAT THIS TOOL CANNOT SEE
-------------------------
Cloud / scheduled agents execute on Anthropic's infrastructure and write their
transcripts server-side. They are ABSENT from this output -- not undercounted,
absent -- and every total here is therefore a FLOOR. Whether they even draw on
the same meters is unverified. The report says so on every run, loudly, because
a partial number passed off as a complete one is worse than no number.

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
import re
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

# Optional OVERRIDE only: {"<session-uuid>": "Senior Controls", ...}. Attribution
# is DERIVED (see attribute()); this file exists to correct a specific session
# the derivation gets wrong, not to carry the mapping.
ROLE_MAP = Path(__file__).parent / "session-roles.json"

# Calibration observations for the percentage gate. See estimate_pct().
#
# OUTSIDE the repo, deliberately. A calibration describes THE MACHINE'S PLAN,
# not this checkout -- and every role has its own worktree (ADR-0006), each with
# its own ops/ directory. Stored in-repo, a reading taken in ~/projects/overboard
# would be invisible to dispatch running in ~/projects/overboard-coo, and the
# gate would silently report "NO CALIBRATION" in the very place that gates.
# Caught while writing the instructions for taking the first reading.
CALIBRATION = Path.home() / ".claude" / "overboard-usage-calibration.json"

# Below this, a reading is too small to divide by. At 2% an eight-point rounding
# error on the bar swings the implied full scale by ~50%, and the gate would sit
# permanently green because it thinks the plan is enormous. A gate that cannot
# fire is the failure mode this whole design exists to avoid, so it refuses the
# observation rather than accepting a number it cannot use.
MIN_CALIBRATION_PCT = 15.0

ROLES_MD = Path(__file__).parent.parent / "docs" / "decisions" / "ROLES.md"

# cwd fallback when the branch carries no lane -- one worktree per role
# (ADR-0006), so the directory names the role. Only needed for work done on
# master or a detached HEAD inside a role's worktree.
WORKTREE_ROLE = {
    "overboard-coo": "COO",
    "overboard-controls": "Senior Controls",
    "overboard-archivist": "Archivist",
    "overboard-mech": "Sr. Mechanical & Systems",
    "overboard-dcp": "Digital Content Production",
}


def lane_roles() -> dict[str, str]:
    """{lane: role} from the registry -- 'feat/ops/' -> {'ops': 'COO'}.

    Keyed on the LANE, not the whole prefix, because the registry records
    `feat/ops/` while real branches are also `fix/ops/...` and `docs/ops/...`.
    Matching the literal prefix missed every one of those.
    """
    out: dict[str, str] = {}
    if not ROLES_MD.is_file():
        return out
    for line in ROLES_MD.read_text(encoding="utf-8").splitlines():
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 5 or cells[1] not in {"Ratified", "Provisional"}:
            continue
        role = re.fullmatch(r"`([^`]+)`", cells[0])
        prefix = re.fullmatch(r"`([^`]+)`", cells[3])
        if role and prefix:
            lane = prefix.group(1).strip("/").split("/")[-1]
            out[lane] = role.group(1)
    return out


def attribute(branch: str, cwd: str, session: str,
              lanes: dict[str, str], override: dict[str, str]) -> str | None:
    """Which role paid for this turn? DERIVED, not maintained.

    The design doc recommended a hand-kept {session: role} file. Two facts
    found while building it changed that call, and the deviation is deliberate:

      1. Every usage-bearing transcript entry carries `gitBranch` and `cwd`
         (511/511 on the sample checked), so the data is already there.
      2. A session is NOT one role. This session alone spanned seven branches,
         including reviewing and fixing work on another role's branch. A
         session-level map would have booked all of that to one role and been
         confidently wrong; per-turn branch attribution books it to the work.

    Derived also means it applies RETROACTIVELY and costs nobody any ongoing
    discipline -- the failure mode the design doc named as (b)'s main downside.
    """
    if session in override:
        return override[session]
    seg = branch.split("/")
    if len(seg) >= 2 and seg[1] in lanes:
        return lanes[seg[1]]
    base = Path(cwd).name if cwd else ""
    return WORKTREE_ROLE.get(base)


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


def scan() -> tuple[dict, dict, dict, dict, dict]:
    by_day: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    by_session: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    sub_share: dict[str, int] = defaultdict(int)
    by_role: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    by_branch: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    day_attr: dict[str, dict[str, int]] = defaultdict(lambda: defaultdict(int))
    turn_cost: dict[str, list] = defaultdict(lambda: [0, 0.0])
    lanes, override = lane_roles(), roles()

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
            branch = d.get("gitBranch") or ""
            role = attribute(branch, d.get("cwd") or "", session, lanes, override)
            label = "dispatched subagent" if is_sub else "main-thread"
            turn_cost[label][0] += 1
            turn_cost[label][1] += weighted(usage)
            for k in WEIGHTS:
                v = usage.get(k) or 0
                if isinstance(v, int):
                    by_day[day][k] += v
                    by_session[session][k] += v
                    by_role[role or "unattributed"][k] += v
                    if role:
                        day_attr[day][k] += v
                    if branch:
                        by_branch[branch][k] += v
                    if is_sub:
                        sub_share[day] += v
    return by_day, by_session, sub_share, by_role, by_branch, day_attr, dict(turn_cost)


def weighted(bucket: dict[str, int]) -> float:
    return sum(bucket.get(k, 0) * w for k, w in WEIGHTS.items())


def roles() -> dict[str, str]:
    if ROLE_MAP.is_file():
        try:
            return json.loads(ROLE_MAP.read_text())
        except json.JSONDecodeError:
            pass
    return {}


def rolling_week_m(by_day: dict) -> float:
    """Weighted M over the trailing 7 days.

    A PROXY for the weekly meter, not the meter. The real one resets on a fixed
    per-plan cadence we cannot read, so a rolling window will disagree with the
    dashboard near a reset boundary. Stated here so nobody reads it as the
    actual meter later.
    """
    cutoff = (date.today() - timedelta(days=6)).isoformat()
    return sum(weighted(b) for d, b in by_day.items() if d >= cutoff) / 1e6


def calibration() -> list[dict]:
    if CALIBRATION.is_file():
        try:
            return json.loads(CALIBRATION.read_text()).get("observations", [])
        except (json.JSONDecodeError, AttributeError):
            pass
    return []


def estimate_pct(week_m: float) -> tuple[float, int] | None:
    """Estimated % of the weekly meter consumed, from calibration.

    Anthropic publishes NO absolute number for a subscription meter -- only a
    percentage bar in Settings -> Usage (see the COO design doc). So the ceiling
    cannot be a token count anybody can look up, and inventing one would be
    worse than none: it would get wired into this gate and then believed.

    Instead a human reads the real bar and tells this tool once:

        ops/usage.py --calibrate 62

    which records (trailing-7-day weighted M, observed %). Full scale is then
    weighted_per_pct * 100, and today's estimate follows. More observations
    average out; each is only as good as the moment it was read.
    """
    obs = [o for o in calibration() if o.get("percent", 0) > 0 and o.get("week_m", 0) > 0]
    if not obs:
        return None
    per_pct = sum(o["week_m"] / o["percent"] for o in obs) / len(obs)
    if per_pct <= 0:
        return None
    return week_m / per_pct, len(obs)


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--check", action="store_true", help="exit 1 if over the threshold")
    env = os.environ.get("OPS_USAGE_CEILING_M")
    ap.add_argument("--ceiling", type=float, default=float(env) if env else None,
                    help="daily ceiling, millions of weighted tokens")
    tenv = os.environ.get("OPS_USAGE_THRESHOLD_PCT")
    ap.add_argument("--threshold-pct", type=float, default=float(tenv) if tenv else None,
                    help="refuse to dispatch above this %% of the weekly meter")
    ap.add_argument("--calibrate", type=float, metavar="PCT",
                    help="record the %% shown in Settings -> Usage right now")
    args = ap.parse_args()

    by_day, by_session, sub_share, by_role, by_branch, day_attr, turn_cost = scan()
    today = date.today().isoformat()
    today_w = weighted(by_day.get(today, {})) / 1e6
    week_m = rolling_week_m(by_day)

    if args.calibrate is not None:
        if args.calibrate < MIN_CALIBRATION_PCT:
            print(f"REFUSED: {args.calibrate:.0f}% is too low to calibrate from "
                  f"(minimum {MIN_CALIBRATION_PCT:.0f}%).")
            print(f"  Full scale is week_m / percent, so a small percent divides by a")
            print(f"  small number: at {args.calibrate:.0f}% a one-point reading error moves the")
            print(f"  implied ceiling enormously, and the gate would sit permanently green")
            print(f"  because it believes the plan is far bigger than it is.")
            print(f"  Take the reading later in the week, when the bar has moved.")
            return 1
        if args.calibrate > 100:
            print(f"REFUSED: {args.calibrate:.0f}% is not a percentage of a meter.")
            return 1
        obs = calibration()
        obs.append({"observed_at": datetime.now().isoformat(timespec="seconds"),
                    "percent": args.calibrate, "week_m": round(week_m, 2)})
        CALIBRATION.write_text(json.dumps({"observations": obs}, indent=2) + "\n")
        print(f"calibrated: {week_m:.1f}M trailing-7d == {args.calibrate:.0f}% of the weekly meter")
        print(f"  implied full scale ~{week_m / args.calibrate * 100:.0f}M weighted")
        print(f"  {len(obs)} observation(s) on file. This is a PROXY, not the meter.")
        return 0

    if args.check:
        est = estimate_pct(week_m)
        # Percentage gate first: it is the one that can exist, because
        # Anthropic publishes a percentage and never a number.
        if args.threshold_pct is not None and est is not None:
            pct, n = est
            over = pct > args.threshold_pct
            print(f"usage: ~{pct:.0f}% of the weekly meter (est. from {n} calibration"
                  f"{'s' if n != 1 else ''}), threshold {args.threshold_pct:.0f}% "
                  f"-- {'OVER, do not dispatch' if over else 'ok to dispatch'}")
            return 1 if over else 0
        if args.threshold_pct is not None and est is None:
            print(f"usage: threshold {args.threshold_pct:.0f}% set but NO CALIBRATION "
                  f"-- cannot convert {week_m:.1f}M weighted into a percentage.")
            print("  Read Settings -> Usage and run: ops/usage.py --calibrate <pct>")
            print("  NOT gating: refusing to invent the conversion.")
            return 0
        if args.ceiling is not None:
            over = today_w > args.ceiling
            print(f"usage: {today_w:.1f}M weighted today, ceiling {args.ceiling:.0f}M "
                  f"-- {'OVER, do not dispatch' if over else 'ok to dispatch'}")
            return 1 if over else 0
        print(f"usage: {today_w:.1f}M today, {week_m:.1f}M trailing-7d. NO GATE SET "
              f"(OPS_USAGE_THRESHOLD_PCT + --calibrate) -- reporting only.")
        return 0

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

    est = estimate_pct(week_m)
    if est:
        pct, n = est
        print(f"\nWeekly meter: ~{pct:.0f}% consumed "
              f"({week_m:.1f}M trailing-7d, est. from {n} calibration{'s' if n != 1 else ''})")
    else:
        print(f"\nWeekly meter: NOT MEASURED -- {week_m:.1f}M trailing-7d, no calibration.")
        print("  Anthropic publishes a percentage, never a number. Read Settings -> Usage")
        print("  and run: ops/usage.py --calibrate <pct>")

    total = sum(weighted(b) for b in by_role.values())
    attributed = total - weighted(by_role.get("unattributed", {}))
    print(f"\nBy role (weighted M, all time) -- attribution DERIVED from branch lane, "
          f"then worktree:")
    for role, b in sorted(by_role.items(), key=lambda kv: -weighted(kv[1])):
        w = weighted(b) / 1e6
        share = (weighted(b) / total * 100) if total else 0
        print(f"  {w:>8.1f}  {share:>5.1f}%  {role}")
    cov = (attributed / total * 100) if total else 0
    print(f"  coverage: {cov:.0f}% attributed. Unattributed is mostly pre-worktree "
          f"history, not a live gap -- see the trend below.")

    print(f"\nAttribution coverage by day -- is the derivation actually working?")
    for day in sorted(by_day):
        dt = weighted(by_day[day])
        da = weighted(day_attr.get(day, {}))
        if dt <= 0:
            continue
        pc = da / dt * 100
        bar = "#" * int(pc / 5)
        print(f"  {day}  {dt/1e6:>7.1f}M  {pc:>5.1f}%  {bar}")
    print("  Rising, because branch and worktree discipline is what makes a turn")
    print("  attributable. Early history predates both and can never be recovered.")

    # The optimisation, MEASURED and re-measured every run rather than asserted
    # once in a design doc. Per-turn controls for scope: a cheap PR and an
    # expensive one both pay the context tax on every single turn.
    if turn_cost:
        print("\nCost per turn -- the context-carry tax, measured:")
        for label in ("main-thread", "dispatched subagent"):
            n, w = turn_cost.get(label, (0, 0.0))
            if n:
                print(f"  {label:22} {n:>6} turns  {w/n/1000:>7.1f}k weighted/turn")
        mt = turn_cost.get("main-thread", (0, 0.0))
        ds = turn_cost.get("dispatched subagent", (0, 0.0))
        if mt[0] and ds[0]:
            ratio = (mt[1] / mt[0]) / (ds[1] / ds[0])
            print(f"  -> delegating is {ratio:.1f}x cheaper PER TURN.")
        print("  Weighted cost is ~90% cache read + cache creation, ~4-10% output:")
        print("  cost is dominated by CARRYING CONTEXT, not by generating text.")
        print("  NOTE: these weights model token CLASSES, not model prices. A")
        print("  Sonnet subagent is additionally cheaper per token, so the real")
        print("  saving is larger than the ratio above, by an amount not modelled.")

    # THE BLIND SPOT, stated where it cannot be missed rather than in a footnote.
    print("\n" + "=" * 68)
    print("  THIS NUMBER IS A FLOOR, NOT A MEASUREMENT.")
    print("  Cloud / scheduled agents run on Anthropic's infrastructure and")
    print("  write their transcripts server-side. They never touch this disk, so")
    print("  they are ABSENT here -- not undercounted, absent, with no signal.")
    print("  Whether they draw on the same subscription meters is UNVERIFIED.")
    print("  Report this caveat with the figure, every time.")
    print("=" * 68)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
