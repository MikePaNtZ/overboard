#!/usr/bin/env python3
"""Policy gate for the Overboard repo.

Hard checks fail the build; advisory findings are reported and do not.
See docs/decisions/ADR-0003 (the gate) and ADR-0005 (the role registry).

    python3 .github/policy_check.py              # run every check
    python3 .github/policy_check.py --who PATH   # who owns PATH?

Stdlib only, on purpose: the policy gate must not be able to fail because a
dependency moved. It also never queries Notion -- a network- or token-dependent
required check produces red builds the PR author cannot fix, which would poison
the one interrupt this org has that reliably works.
"""

from __future__ import annotations

import fnmatch
import os
import re
import subprocess
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parent.parent
DECISIONS = REPO / "docs" / "decisions"
CODEOWNERS = REPO / ".github" / "CODEOWNERS"
ROLES_MD = DECISIONS / "ROLES.md"

VALID_ADR_STATUS = re.compile(
    r"^\s*-\s*\*\*Status:\*\*\s*(Proposed|Accepted|Rejected|Superseded by ADR-\d{4})\s*$",
    re.MULTILINE,
)
REQ_ID = re.compile(r"\b(?:UR|SR|DR)-[A-Z0-9]*-?\d+\b")

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

CAPABILITY = re.compile(
    r"self[- ]balanc\w*|balances?\s+itself|rides?\s+itself|riderless|driverless"
    r"|autonomous|\bproven\b",
    re.I,
)

TURF_OVERRIDE = re.compile(r"TURF-OVERRIDE:\s*(\S.*)")
DOC_OK = re.compile(r"DOC-OK:\s*(\S.*)")

# A doc declares what it describes in an HTML comment, so it stays invisible
# when rendered:  <!-- covers: [glob, ...]  reconciled: <sha> -->
COVERS = re.compile(r"<!--\s*covers:\s*(.*?)-->", re.S)
DOC_WARN_BYTES = 20_000
DOC_FAIL_BYTES = 40_000

failures: list[str] = []
advisories: list[str] = []


def fail(check: str, msg: str) -> None:
    failures.append(f"{check}: {msg}")


def git(*args: str) -> str:
    return subprocess.run(
        ["git", *args], cwd=REPO, capture_output=True, text=True
    ).stdout.strip()


IN_CI = bool(os.environ.get("GITHUB_ACTIONS"))


def cannot_resolve(check: str, what: str) -> None:
    """A diff-based check could not find its base.

    Locally this is normal and skipping is right. **In CI it is the gate
    failing open**, which is strictly worse than having no gate: `turf` and
    `doc-drift` both skipped themselves on every PR for as long as they had
    existed -- depth-1 checkout, no merge-base -- while the run still printed
    "all hard checks pass". A skip that reports green is a lie the whole org
    then builds on, so in CI it is a hard failure.
    """
    if IN_CI:
        fail(check, f"cannot resolve {what} IN CI -- this check would have "
                    f"silently skipped and the gate would have reported green. "
                    f"Check the workflow's fetch-depth and POLICY_BRANCH/"
                    f"POLICY_BASE_REF wiring")
    else:
        print(f"{check}: cannot resolve {what} -- skipping (local run)")


# ---------------------------------------------------------------------------
# The role registry -- docs/decisions/ROLES.md is the single home.
# ---------------------------------------------------------------------------
def load_roles() -> dict[str, dict[str, str]]:
    """Parse the registry table. Returns {role: {status, prefix}}."""
    if not ROLES_MD.is_file():
        fail("roles", "docs/decisions/ROLES.md is missing -- the role registry has no home")
        return {}

    roles: dict[str, dict[str, str]] = {}
    for line in ROLES_MD.read_text(encoding="utf-8").splitlines():
        if not line.startswith("|"):
            continue
        cells = [c.strip() for c in line.strip().strip("|").split("|")]
        if len(cells) < 4:
            continue
        m = re.fullmatch(r"`([^`]+)`", cells[0])
        if not m:
            continue  # header or a non-role table
        status = cells[1]
        if status not in {"Provisional", "Ratified"}:
            continue  # a different table that happens to have a backticked col 0
        prefix = ""
        pm = re.fullmatch(r"`([^`]+)`", cells[3])
        if pm:
            prefix = pm.group(1)
        roles[m.group(1)] = {"status": status, "prefix": prefix}

    if not roles:
        fail("roles", "ROLES.md parsed to zero roles -- the registry table shape changed")
    return roles


# ---------------------------------------------------------------------------
# CODEOWNERS
# ---------------------------------------------------------------------------
def load_codeowners() -> list[tuple[str, str, int]]:
    """Returns [(pattern, role, lineno)] in file order. Last match wins."""
    if not CODEOWNERS.is_file():
        fail("ownership", ".github/CODEOWNERS is missing")
        return []

    rules: list[tuple[str, str, int]] = []
    pending: str | None = None
    for lineno, raw in enumerate(CODEOWNERS.read_text(encoding="utf-8").splitlines(), 1):
        line = raw.strip()
        if not line:
            continue
        if line.startswith("#"):
            m = re.match(r"#\s*role:\s*(.+?)\s*$", line)
            if m:
                pending = m.group(1)
            continue
        if pending is None:
            fail("ownership", f"CODEOWNERS:{lineno} rule {line.split()[0]!r} has no '# role:' tag above it")
            continue
        rules.append((line.split()[0], pending, lineno))
        pending = None
    return rules


def matches(pattern: str, path: str) -> bool:
    if pattern == "*":
        return True
    p = pattern.lstrip("/")
    if p.endswith("/"):
        return path.startswith(p)
    if "*" in p:
        return fnmatch.fnmatch(path, p)
    return path == p or path.startswith(p + "/")


def owner_of(path: str, rules: list[tuple[str, str, int]]) -> tuple[str, str, int] | None:
    hit = None
    for pattern, role, lineno in rules:
        if matches(pattern, path):
            hit = (role, pattern, lineno)
    return hit


# ---------------------------------------------------------------------------
# Checks
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
        if not re.search(r"^\s*-\s*\*\*Closes:\*\*", body, re.MULTILINE):
            fail("adr-index", f"{name} has no '- **Closes:**' line naming the row it closes")

    numbers = [re.match(r"ADR-(\d{4})", n).group(1) for n in on_disk]
    if len(numbers) != len(set(numbers)):
        fail("adr-index", "duplicate ADR numbers on disk; numbers are never reused")


def check_ownership(roles: dict, rules: list) -> None:
    for _pattern, role, lineno in rules:
        if role not in roles:
            fail(
                "ownership",
                f"CODEOWNERS:{lineno} role {role!r} is not in docs/decisions/ROLES.md. "
                f"Add a Provisional registry entry first -- see ADR-0005",
            )

    explicit = {p.strip("/") for p, _r, _l in rules if p != "*"}
    tracked = git("ls-files").split()
    for d in sorted({p.split("/")[0] for p in tracked if "/" in p}):
        if d not in explicit:
            fail(
                "ownership",
                f"top-level directory {d!r} has no explicit CODEOWNERS rule "
                f"(the '*' catch-all does not count -- assign it deliberately)",
            )

    if CODEOWNERS.is_file():
        text = CODEOWNERS.read_text(encoding="utf-8")
        for role, meta in roles.items():
            if meta["status"] == "Ratified" and role not in text:
                fail(
                    "ownership",
                    f"ratified role {role!r} appears nowhere in CODEOWNERS -- give it a rule, "
                    f"or declare that it owns nothing in this repo",
                )


def check_turf(roles: dict, rules: list) -> None:
    """Turf, checked at the moment of offence.

    Only enforced when the branch prefix resolves to a registered role. Roles
    that have not declared a prefix are skipped entirely -- failing open keeps
    the blast radius to roles that opted in, and the check tightens by itself as
    the registry fills in.
    """
    branch = os.environ.get("POLICY_BRANCH") or git("rev-parse", "--abbrev-ref", "HEAD")
    base = os.environ.get("POLICY_BASE_REF", "origin/master")

    prefixes = {m["prefix"]: r for r, m in roles.items() if m["prefix"]}
    role = next((r for p, r in prefixes.items() if branch.startswith(p)), None)
    if role is None:
        # "HEAD" means the workflow did not pass POLICY_BRANCH and we are on a
        # detached checkout -- that is the wiring bug, not an unregistered role.
        if branch == "HEAD":
            cannot_resolve("turf", "the branch name (got the literal 'HEAD')")
        else:
            print(f"turf: branch {branch!r} matches no registered branch prefix -- skipping")
        return

    merge_base = git("merge-base", base, "HEAD")
    if not merge_base:
        cannot_resolve("turf", f"base {base!r}")
        return
    changed = [p for p in git("diff", "--name-only", f"{merge_base}...HEAD").split() if p]
    if not changed:
        return

    override = os.environ.get("POLICY_PR_BODY", "") + "\n" + git("log", f"{merge_base}..HEAD", "--format=%B")
    om = TURF_OVERRIDE.search(override)

    trespass = []
    for path in changed:
        hit = owner_of(path, rules)
        if hit and hit[0] != role:
            trespass.append(f"{path} (owned by {hit[0]}, CODEOWNERS:{hit[2]})")

    if not trespass:
        return
    if om:
        print(f"turf: {len(trespass)} cross-boundary edit(s) as {role}, overridden -- {om.group(1).strip()}")
        for t in trespass:
            print(f"    ! {t}")
        return
    for t in trespass:
        fail(
            "turf",
            f"branch is {role}'s ({branch}) but edits {t}. File a row, or put "
            f"'TURF-OVERRIDE: <reason or row url>' in a COMMIT MESSAGE on this "
            f"branch if it is authorised -- CI reads commit messages, not the PR "
            f"body, so the reason lives in git history rather than an editable field",
        )


# ---------------------------------------------------------------------------
# Documentation drift
# ---------------------------------------------------------------------------
def doc_manifests() -> list[tuple[Path, list[str]]]:
    """[(doc path, covered globs)] for implementation-tier docs."""
    out = []
    for path in sorted((REPO / "docs").rglob("*.md")):
        if DECISIONS in path.parents:
            continue
        m = COVERS.search(path.read_text(encoding="utf-8"))
        if not m:
            continue
        globs = [ln.strip().lstrip("-").strip()
                 for ln in m.group(1).splitlines()
                 if ln.strip().lstrip("-").strip() and not ln.strip().startswith("reconciled:")]
        out.append((path, [g for g in globs if g]))
    return out


def check_doc_drift() -> None:
    """A change to code must carry the doc that describes it, in the same PR.

    Caught at the moment of creation rather than discovered months later. The
    check never judges CONTENT -- only that somebody looked since the code
    moved. `DOC-OK: <reason>` in a commit message is the escape hatch, same
    shape as TURF-OVERRIDE, so the reason lands in git history.
    """
    base = os.environ.get("POLICY_BASE_REF", "origin/master")
    merge_base = git("merge-base", base, "HEAD")
    if not merge_base:
        cannot_resolve("doc-drift", f"base {base!r}")
        return
    changed = set(p for p in git("diff", "--name-only", f"{merge_base}...HEAD").split() if p)
    if not changed:
        return

    override_text = (os.environ.get("POLICY_PR_BODY", "") + "\n"
                     + git("log", f"{merge_base}..HEAD", "--format=%B"))
    om = DOC_OK.search(override_text)

    for doc, globs in doc_manifests():
        rel_doc = str(doc.relative_to(REPO))
        if rel_doc in changed:
            continue  # the doc moved with the code
        hits = sorted({c for c in changed for g in globs if fnmatch.fnmatch(c, g)})
        if not hits:
            continue
        if om:
            print(f"doc-drift: {rel_doc} not updated alongside {len(hits)} covered file(s), "
                  f"overridden -- {om.group(1).strip()}")
            continue
        fail(
            "doc-drift",
            f"{rel_doc} describes {', '.join(hits[:3])}"
            f"{' and others' if len(hits) > 3 else ''}, which this PR changes, but the doc "
            f"was not touched. Update it, or put 'DOC-OK: <reason>' in a commit message",
        )


def check_doc_size() -> None:
    """Stop docs becoming historical logs.

    A 55,000-character BoM is what prompted this: excellent research, unusable
    as the thing it claimed to be. Superseded reasoning belongs in an appendix
    or deleted, not accreted at the top of the file someone has to act on.
    """
    for path in sorted((REPO / "docs").rglob("*.md")):
        n = len(path.read_text(encoding="utf-8"))
        rel = path.relative_to(REPO)
        if n > DOC_FAIL_BYTES:
            fail("doc-size", f"{rel} is {n:,} chars (limit {DOC_FAIL_BYTES:,}). "
                             f"Move superseded material to an appendix, or cut it")
        elif n > DOC_WARN_BYTES:
            advisories.append(f"{rel} is {n:,} chars -- approaching the {DOC_FAIL_BYTES:,} limit")


def public_markdown() -> list[Path]:
    """README plus docs/, excluding docs/decisions/.

    ADRs are internal records and have to be able to QUOTE the banned words in
    order to ban them -- ADR-0003 lists every one. Scanning them would make the
    gate fail on its own charter.
    """
    out = [REPO / "README.md"]
    out += [p for p in sorted((REPO / "docs").rglob("*.md")) if DECISIONS not in p.parents]
    return [p for p in out if p.is_file()]


def sections(text: str) -> list[tuple[str, str, int]]:
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


def who(path: str) -> int:
    roles, rules = load_roles(), load_codeowners()
    hit = owner_of(path.lstrip("./"), rules)
    if hit is None:
        print(f"{path}: no rule matches (this should be impossible -- '*' is a catch-all)")
        return 1
    role, pattern, lineno = hit
    status = roles.get(role, {}).get("status", "UNREGISTERED")
    print(f"{path}\n  owner : {role}  [{status}]\n  rule  : {pattern}  (CODEOWNERS:{lineno})")
    print("\n  Ownership governs WRITE, not read or import. Reading needs no row.")
    return 0


def reconcile(doc: str) -> int:
    """Stamp the current HEAD into a doc's manifest, marking it reconciled."""
    p = REPO / doc
    if not p.is_file():
        print(f"{doc}: not found")
        return 1
    text = p.read_text(encoding="utf-8")
    m = COVERS.search(text)
    if not m:
        print(f"{doc}: no <!-- covers: --> manifest to stamp")
        return 1
    head = git("rev-parse", "HEAD")[:7]
    body = m.group(1)
    body = (re.sub(r"reconciled:\s*\S+", f"reconciled: {head}", body)
            if "reconciled:" in body else body.rstrip() + f"\nreconciled: {head}\n")
    p.write_text(text[:m.start(1)] + body + text[m.end(1):], encoding="utf-8")
    print(f"{doc}: reconciled at {head}")
    return 0


def main(argv: list[str]) -> int:
    if "--reconcile" in argv:
        i = argv.index("--reconcile")
        return reconcile(argv[i + 1]) if i + 1 < len(argv) else 2
    if "--who" in argv:
        i = argv.index("--who")
        if i + 1 >= len(argv):
            print("usage: policy_check.py --who <path>")
            return 2
        return who(argv[i + 1])

    roles = load_roles()
    rules = load_codeowners()
    check_adr_index()
    check_ownership(roles, rules)
    check_turf(roles, rules)
    check_doc_drift()
    check_doc_size()
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
        print("\nSee docs/decisions/ADR-0003 and ADR-0005")
        return 1

    print(f"policy: all hard checks pass ({len(roles)} roles, {len(rules)} ownership rules)")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
