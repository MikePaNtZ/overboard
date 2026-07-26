#!/usr/bin/env python3
"""Policy gate for the Overboard repo.

Four hard checks that fail the build, plus one advisory report that does not.
See docs/decisions/ADR-0003-policy-ci-gate.md for why each one exists and why
the advisory one is deliberately not hard yet.

Run it locally before opening a PR:

    python3 .github/policy_check.py

Stdlib only, on purpose: the policy gate must not be able to fail because a
dependency moved.
"""

from __future__ import annotations

import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DECISIONS = REPO / "docs" / "decisions"
CODEOWNERS = REPO / ".github" / "CODEOWNERS"

KNOWN_ROLES = {
    "CEO",
    "COO",
    "CMO",
    "Sr. Mechanical & Systems",
    "Senior Controls",
    "Digital Content Production",
}

VALID_ADR_STATUS = re.compile(
    r"^\s*-\s*\*\*Status:\*\*\s*(Proposed|Accepted|Rejected|Superseded by ADR-\d{4})\s*$",
    re.MULTILINE,
)

# A requirement ID: UR-13, SR-WEB-4, DR-SIM-1.
REQ_ID = re.compile(r"\b(?:UR|SR|DR)-[A-Z0-9]*-?\d+\b")

# Absolutes the program committed to never saying in public. Deliberately tight:
# a broad list produces false positives, and a gate that cries wolf gets removed.
BANNED = [
    (re.compile(r"production[- ]ready", re.I), "the project is explicitly not production-ready"),
    (re.compile(r"\bcertified\b", re.I), "nothing here is certified by anyone"),
    (re.compile(r"guarantee[sd]?\s+(?:to\s+be\s+)?safe", re.I), "safety is not guaranteed"),
    (re.compile(r"safe\s+to\s+ride", re.I), "no ridden operation has cleared the D5 review"),
    (re.compile(r"perfectly\s+safe", re.I), "no."),
    (re.compile(r"crash[- ]proof", re.I), "no."),
    (re.compile(r"fully\s+autonomous", re.I), "overclaims the control stack"),
    (re.compile(r"medical[- ]grade|\bFDA\b"), "not a regulated device; do not borrow the language"),
]

# Capability language that SHOULD trace to a requirement. Advisory for now.
CAPABILITY = re.compile(
    r"self[- ]balanc\w*|balances?\s+itself|rides?\s+itself|riderless|driverless"
    r"|autonomous|\bproven\b",
    re.I,
)

failures: list[str] = []
advisories: list[str] = []


def fail(check: str, msg: str) -> None:
    failures.append(f"{check}: {msg}")


def public_markdown() -> list[Path]:
    """Markdown that forms the repo's public face.

    docs/decisions/ is excluded on purpose. ADRs are internal engineering
    records and must be able to QUOTE the banned words in order to ban them --
    ADR-0003 lists every one of them. Scanning them would make the gate
    self-defeating on its own charter.
    """
    out = [REPO / "README.md"]
    out += [p for p in sorted((REPO / "docs").rglob("*.md")) if DECISIONS not in p.parents]
    return [p for p in out if p.is_file()]


def sections(text: str) -> list[tuple[str, str, int]]:
    """Split markdown into (heading, body, heading_line_number)."""
    lines = text.splitlines()
    out: list[tuple[str, str, int]] = []
    heading, buf, start = "(preamble)", [], 1
    for i, line in enumerate(lines, 1):
        if re.match(r"^#{1,6}\s", line):
            out.append((heading, "\n".join(buf), start))
            heading, buf, start = line.lstrip("# ").strip(), [], i
        else:
            buf.append(line)
    out.append((heading, "\n".join(buf), start))
    return out


# ---------------------------------------------------------------------------
# 1. ADR index integrity
# ---------------------------------------------------------------------------
def check_adr_index() -> None:
    index = DECISIONS / "INDEX.md"
    if not index.is_file():
        fail("adr-index", "docs/decisions/INDEX.md is missing")
        return

    index_text = index.read_text(encoding="utf-8")
    on_disk = {p.name for p in DECISIONS.glob("ADR-*.md") if p.name != "ADR-TEMPLATE.md"}

    for name in sorted(on_disk):
        if name not in index_text:
            fail("adr-index", f"{name} exists but is not listed in INDEX.md")

    for ref in sorted(set(re.findall(r"ADR-\d{4}-[a-z0-9-]+\.md", index_text))):
        if not (DECISIONS / ref).is_file():
            fail("adr-index", f"INDEX.md references {ref}, which does not exist")

    for name in sorted(on_disk):
        body = (DECISIONS / name).read_text(encoding="utf-8")
        if not VALID_ADR_STATUS.search(body):
            fail("adr-index", f"{name} has no recognised '- **Status:**' line")

    numbers = sorted(re.match(r"ADR-(\d{4})", n).group(1) for n in on_disk)
    if len(numbers) != len(set(numbers)):
        fail("adr-index", "duplicate ADR numbers on disk; numbers are never reused")


# ---------------------------------------------------------------------------
# 2. Ownership coverage and role tags
# ---------------------------------------------------------------------------
def check_codeowners() -> None:
    if not CODEOWNERS.is_file():
        fail("ownership", ".github/CODEOWNERS is missing")
        return

    patterns: list[str] = []
    pending_role: str | None = None

    for lineno, raw in enumerate(CODEOWNERS.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            m = re.match(r"#\s*role:\s*(.+?)\s*$", line)
            if m:
                if m.group(1) not in KNOWN_ROLES:
                    fail("ownership", f"CODEOWNERS:{lineno} unknown role {m.group(1)!r}")
                pending_role = m.group(1)
            continue

        # A rule line.
        if pending_role is None:
            fail("ownership", f"CODEOWNERS:{lineno} rule {line.split()[0]!r} has no '# role:' tag above it")
        pending_role = None
        patterns.append(line.split()[0])

    explicit = {p.strip("/") for p in patterns if p != "*"}

    tracked = subprocess.run(
        ["git", "ls-files"], cwd=REPO, capture_output=True, text=True, check=True
    ).stdout.split()
    top_dirs = sorted({p.split("/")[0] for p in tracked if "/" in p})

    for d in top_dirs:
        if d not in explicit:
            fail(
                "ownership",
                f"top-level directory {d!r} has no explicit CODEOWNERS rule "
                f"(the '*' catch-all does not count -- assign it deliberately)",
            )


# ---------------------------------------------------------------------------
# 3 + 4 + 5. Public claim checks
# ---------------------------------------------------------------------------
def check_claims() -> None:
    for path in public_markdown():
        rel = path.relative_to(REPO)
        text = path.read_text(encoding="utf-8")

        for lineno, line in enumerate(text.splitlines(), 1):
            for pattern, why in BANNED:
                if pattern.search(line):
                    fail("banned-absolute", f"{rel}:{lineno} {pattern.pattern!r} -- {why}")

        for heading, body, lineno in sections(text):
            if "claim" in heading.lower():
                if not REQ_ID.search(body) and not REQ_ID.search(heading):
                    fail(
                        "claim-trace",
                        f"{rel}:{lineno} section {heading!r} makes claims but cites no "
                        f"requirement ID (UR-/SR-/DR-)",
                    )
            elif CAPABILITY.search(body) and not REQ_ID.search(body):
                hit = CAPABILITY.search(body)
                advisories.append(
                    f"{rel}:{lineno} section {heading!r} uses {hit.group(0)!r} "
                    f"with no requirement ID in scope"
                )


def main() -> int:
    check_adr_index()
    check_codeowners()
    check_claims()

    if advisories:
        print("Advisory -- capability language with no requirement ID in scope.")
        print("Not a failure. Input to a future ADR that tightens this into a hard check.\n")
        for a in advisories:
            print(f"  ~ {a}")
        print()

    if failures:
        print(f"POLICY FAILED -- {len(failures)} problem(s):\n")
        for f in failures:
            print(f"  x {f}")
        print("\nSee docs/decisions/ADR-0003-policy-ci-gate.md")
        return 1

    print("policy: all hard checks pass")
    return 0


if __name__ == "__main__":
    sys.exit(main())
