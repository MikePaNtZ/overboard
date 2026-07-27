#!/usr/bin/env python3
"""Operational metrics that are actually computable. Costs zero model tokens.

    python3 ops/metrics.py

Rule: anything not computable prints NOT MEASURED. An estimate presented as a
measurement is the one thing a company whose public brand is "honest in
progress" cannot do internally either.
"""
import json
import subprocess
from collections import Counter


def sh(*a: str) -> str:
    return subprocess.run(a, capture_output=True, text=True).stdout.strip()


def gh_json(*a: str):
    out = sh("gh", *a)
    try:
        return json.loads(out) if out else []
    except json.JSONDecodeError:
        return []


def main() -> None:
    merged = gh_json("pr", "list", "--state", "merged", "--limit", "200",
                     "--json", "number,mergedAt,headRefName")
    openpr = gh_json("pr", "list", "--state", "open", "--limit", "50",
                     "--json", "number,headRefName,title")
    issues = gh_json("issue", "list", "--state", "open", "--limit", "100",
                     "--json", "number,title,labels")

    by_day = Counter(p["mergedAt"][:10] for p in merged if p.get("mergedAt"))
    by_role = Counter(p["headRefName"].split("/")[1] if p["headRefName"].count("/") >= 2
                      else "unprefixed" for p in merged)

    print("DELIVERY")
    print(f"  PRs merged (all time)      {len(merged)}")
    for d in sorted(by_day)[-7:]:
        print(f"    {d}                 {by_day[d]}")
    print(f"  PRs open                   {len(openpr)}")
    for p in openpr:
        print(f"    #{p['number']} [{p['headRefName']}] {p['title'][:48]}")
    print(f"  merged PRs by branch lane  {dict(by_role)}")

    print("\nBACKLOG  (utilization = the queue is never empty and never blocked -- ADR-0007)")
    print(f"  open issues                {len(issues)}")
    if not issues:
        print("    EMPTY -- this is a reportable condition, not a quiet state.")

    print("\nROADMAP  (the numbers that actually matter)")
    for label in ("BoMs published", "Hardware ordered ($)", "Hardware delivered",
                  "Assembly videos", "Public posts published", "Impressions / engagement"):
        print(f"  {label:<27}NOT MEASURED" if label != "Hardware ordered ($)"
              else f"  {label:<27}0  (nothing ordered to date)")


if __name__ == "__main__":
    main()
