#!/usr/bin/env python3
"""Sweep the public GitHub surface for disclosure leaks and retired vocabulary.

    python3 docs/vocabulary/sweep_public.py            # report, always exit 0
    python3 docs/vocabulary/sweep_public.py --strict   # exit 1 if any `error` finding
    python3 docs/vocabulary/sweep_public.py --no-timeline   # skip per-issue history (faster)

WHY THIS EXISTS
---------------
All three repositories are public. Until now the only detector for "did private
strategy reach a public page?" was a person being asked to look. That is not a
control -- it is a hope, and it dies the first week nobody asks.

WHAT IT CHECKS THAT A NAIVE SCAN WOULD MISS
-------------------------------------------
**Renamed-issue history.** On 2026-07-28 a public issue was opened as
`EPIC: Funding pitch deck -- ...`, then renamed and closed within hours. GitHub
still served the ORIGINAL title to anyone, unauthenticated, via the issue
timeline. **A scan of live text alone reports that repository clean.** Editing a
public issue does not un-publish it; only deleting it does. The rename check is
the one that found the only real leak we have had, so it is on by default.

FALSE POSITIVES ARE THE FAILURE MODE
------------------------------------
A checker that cries wolf gets ignored, and an ignored checker is worse than
none because it launders the risk. Every pattern is word-anchored, and the
anchors are not theoretical -- they come from a manual sweep whose naive first
pass flagged `CI gates` (10x), `SoC family`, a `relative` table column, and
`@lru_cache` read as a user tag. All four are allowed for by construction.

A retired term is also permitted on any line that ALSO links the canonical
vocabulary page. That is the duplication rule enforcing itself: the only legal
way to name a retired term is to say where the current definition lives.

Data lives beside this script and is owned by the Archivist:
`retired-terms.json`, `disclosure-patterns.json`, `provenance-marks.json`.
The script is deliberately dumb about policy -- to change what is enforced,
edit the data, not this file.

PROVENANCE MARKS -- THE INVERSE CHECK
--------------------------------------
`retired-terms.json` says a pattern must NOT appear. `provenance-marks.json`
(added for the Playable Sim category, #163) says the opposite: an on-frame
mark being present creates an obligation for OTHER text -- the non-physical
channel declaration -- to also be present on the same page. Playable Sim is
the only category where a tag alone cannot tell a viewer which part of what
they are looking at is physics and which is not, so a `PLAYABLE SIM` mark
with no declaration nearby is exactly the failure this check exists to catch.
"""

from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
REPOS = ("overboard", "overboard-web", "overboard-viz")
OWNER = "MikePaNtZ"


def gh_json(endpoint: str, paginate: bool = True) -> list:
    """Call the GitHub API via `gh` and return a flat list. Never raises."""
    cmd = ["gh", "api", endpoint]
    if paginate:
        cmd += ["--paginate", "--slurp"]
    try:
        res = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
    except (OSError, subprocess.TimeoutExpired) as exc:
        print(f"  ! {endpoint}: {exc}", file=sys.stderr)
        return []
    if res.returncode != 0:
        print(f"  ! {endpoint}: {res.stderr.strip()[:160]}", file=sys.stderr)
        return []
    try:
        data = json.loads(res.stdout or "[]")
    except json.JSONDecodeError:
        return []
    # --slurp wraps each page in a list; flatten one level.
    out = []
    for item in data:
        out.extend(item) if isinstance(item, list) else out.append(item)
    return out


def compile_rules() -> tuple[list, list, re.Pattern | None]:
    """Load both data files. Vocabulary is optional; disclosure is not."""
    disc = json.loads((HERE / "disclosure-patterns.json").read_text())
    rules = [
        (p["id"], re.compile(p["pattern"]), p.get("severity", "error"))
        for p in disc["patterns"]
    ]

    vocab, exempt = [], None
    vf = HERE / "retired-terms.json"
    if vf.exists():
        v = json.loads(vf.read_text())
        for t in v.get("terms", []):
            if t.get("severity") == "warn":
                continue  # advisory heuristics stay out of a public-surface gate
            flags = re.I if t.get("flags") == "i" else 0
            vocab.append((t["id"], re.compile(t["pattern"], flags), "error"))
        if v.get("exempt_line_regex"):
            exempt = re.compile(v["exempt_line_regex"])
    return rules, vocab, exempt


def compile_marks() -> list[dict]:
    """Load provenance-marks.json. Optional -- absence just means nothing to check yet."""
    mf = HERE / "provenance-marks.json"
    if not mf.exists():
        return []
    data = json.loads(mf.read_text())
    marks = []
    for m in data.get("marks", []):
        if not m.get("requires_channel_declaration"):
            continue
        required = [
            (r["id"], re.compile(r["pattern"], re.I))
            for r in m.get("channel_declaration", {}).get("required_patterns", [])
        ]
        marks.append({
            "id": m["id"],
            "category": m["category"],
            "mark_rx": re.compile(m["mark_pattern"]),
            "required": required,
        })
    return marks


def scan_declarations(text: str, url: str, marks: list[dict]) -> list[dict]:
    """A tagged asset must carry its FULL channel declaration wherever it appears.

    Unlike `scan()` this is not line-by-line: the declaration is a list and is
    expected to span several lines, so the check is over the whole text. A mark
    with no `marks` entry produces no findings -- this only fires for categories
    that opted into the extra obligation.
    """
    hits = []
    text = text or ""
    for m in marks:
        if not m["mark_rx"].search(text):
            continue
        missing = [rid for rid, rx in m["required"] if not rx.search(text)]
        if missing:
            hits.append({
                "rule": f"{m['id']}-missing-declaration",
                "severity": "error",
                "url": url,
                "match": m["category"],
                "line": f"on-frame mark present but declaration missing: {', '.join(missing)}",
            })
    return hits


def scan(text: str, url: str, rules: list, exempt: re.Pattern | None) -> list[dict]:
    """Scan text line by line so the canonical-link exemption can apply per line."""
    hits = []
    for line in (text or "").splitlines():
        if exempt and exempt.search(line):
            continue  # names a retired term AND links canonical -- that is the correct form
        for rid, rx, sev in rules:
            m = rx.search(line)
            if m:
                hits.append(
                    {"rule": rid, "severity": sev, "url": url,
                     "match": m.group(0)[:60], "line": line.strip()[:130]}
                )
    return hits


def sweep(use_timeline: bool = True) -> list[dict]:
    """Two scopes, deliberately different -- see TWO SCOPES below.

    TWO SCOPES, AND WHY THEY DIFFER
    -------------------------------
    **Disclosure runs over everything**, including closed issues, merged pull
    requests and renamed-title history. A leak does not become harmless because
    the issue was closed; the text is still served to anyone.

    **Vocabulary runs over OPEN items only.** `retired-terms.json` says history,
    merged pull requests and closed issues keep the old terms on purpose -- the
    record of how we got here is the asset for a project whose thesis is
    building in public. Never rewrite history to match current vocabulary.

    The first version of this script ignored that and scanned everything with
    both rule sets. It produced nine vocabulary "findings" that were all correct
    history -- including the renamed-from title of an issue *this role had itself
    correctly renamed*. Every one would have been a demand to corrupt the record.
    That is exactly the cry-wolf failure the anchors exist to prevent, arriving
    by a different door.

    **Provenance-mark declarations run over OPEN items only, same reasoning.**
    A past asset's declaration is the record of what was known at the time and
    is never rewritten to match a later, longer declaration -- see
    provenance-marks.json.
    """
    disclosure, vocab, exempt = compile_rules()
    marks = compile_marks()
    findings: list[dict] = []

    for repo in REPOS:
        base = f"repos/{OWNER}/{repo}"
        print(f"  sweeping {repo} ...", file=sys.stderr)

        issues = gh_json(f"{base}/issues?state=all&per_page=100")
        for it in issues:
            url = it.get("html_url", "")
            live = it.get("state") == "open"
            rules = disclosure + vocab if live else disclosure
            findings += scan(it.get("title", ""), url, rules, exempt)
            findings += scan(it.get("body", ""), url, rules, exempt)
            if live and marks:
                whole = f"{it.get('title', '')}\n{it.get('body', '')}"
                findings += scan_declarations(whole, url, marks)

        # Comments carry no state of their own; treat them as record, not live
        # document. Disclosure still applies -- a leak in a comment is a leak.
        for ep in ("issues/comments", "pulls/comments"):
            for c in gh_json(f"{base}/{ep}?per_page=100"):
                findings += scan(c.get("body", ""), c.get("html_url", ""), disclosure, exempt)

        # The check that actually caught something. A renamed title stays public.
        # Disclosure only: a retired term in a renamed-from title is just history.
        if use_timeline:
            for it in issues:
                num = it.get("number")
                if num is None:
                    continue
                for ev in gh_json(f"{base}/issues/{num}/timeline?per_page=100"):
                    if ev.get("event") != "renamed":
                        continue
                    old = (ev.get("rename") or {}).get("from", "")
                    for f in scan(old, it.get("html_url", ""), disclosure, exempt):
                        f["rule"] += " (in RENAMED-FROM history — still public)"
                        findings.append(f)
    return findings


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--strict", action="store_true",
                    help="exit 1 if any error-severity finding (use in CI)")
    ap.add_argument("--no-timeline", action="store_true",
                    help="skip renamed-issue history (faster, misses the leak class that has actually occurred)")
    args = ap.parse_args()

    findings = sweep(use_timeline=not args.no_timeline)
    errors = [f for f in findings if f["severity"] == "error"]
    warns = [f for f in findings if f["severity"] != "error"]

    print("\n=== public surface sweep ===")
    print(f"repos: {', '.join(REPOS)}")
    print(f"errors: {len(errors)}   warnings: {len(warns)}")

    for label, group in (("ERROR", errors), ("WARN", warns)):
        if not group:
            continue
        print(f"\n--- {label} ---")
        seen = set()
        for f in group:
            key = (f["url"], f["rule"], f["match"].lower())
            if key in seen:
                continue
            seen.add(key)
            print(f"  [{f['rule']}] {f['match']!r}\n      {f['url']}\n      {f['line']}")

    if not errors:
        print("\n✅ no disclosure or retired-vocabulary findings on the public surface")
    else:
        print("\n🔴 findings above are PUBLIC. Note: renaming or editing an issue does NOT")
        print("   remove it — GitHub serves the history. Deleting the issue is the only fix.")

    return 1 if (args.strict and errors) else 0


if __name__ == "__main__":
    sys.exit(main())
