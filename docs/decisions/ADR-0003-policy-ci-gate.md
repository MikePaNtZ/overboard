# ADR-0003 — Add a `policy` CI gate for public claims and record integrity

- **Status:** Accepted
- **Date:** 2026-07-26
- **Ratified by:** COO
- **Closes:** none — directed by the CEO
- **Constrains:** anyone editing `README.md`, `docs/**.md`, `docs/decisions/`, or `.github/CODEOWNERS`
- **Enforced by:** the `policy` job in `.github/workflows/ci.yml`

## Context

The program's lock-step rule says a capability claim may not appear publicly unless a
requirement backs it, and a claim that becomes false in engineering comes off in the same
pass. Until now that rule lived only in Notion prose. This repo is public, so its `README.md`
and mirrored `docs/` **are** a public surface — the rule already applied here and nothing
checked it.

The same gap applies to the decision record itself: an index that drifts from the ADRs it
lists is worse than no index, because it is trusted.

## Decision

Add a third CI job, `policy`, running four **hard** checks and one **advisory** report.

Hard — these fail the build:

1. **ADR index integrity.** Every `docs/decisions/ADR-*.md` appears in `INDEX.md`; every ADR
   the index references exists; every ADR carries a recognised `Status`.
2. **Ownership coverage.** Every top-level directory has an explicit `CODEOWNERS` rule —
   not merely the `*` catch-all — so a new top-level directory forces an ownership decision
   instead of defaulting silently. Every rule carries a `# role:` tag naming a known role.
3. **Banned absolutes.** The words the program committed to never saying in public —
   `production-ready`, `certified`, `guaranteed`, `safe to ride`, `FDA` and similar — appear
   nowhere in public-facing markdown.
4. **Claims-section traceability.** Any section whose heading contains "claim" must cite a
   requirement ID matching `(UR|SR|DR)-[A-Z0-9-]*[0-9]`. The README's *"What the sim results
   here do and do not claim"* section already satisfies this via `UR-13`; the check stops
   that traceability from being quietly deleted.

"Public-facing markdown" means `README.md` and `docs/**.md` **excluding `docs/decisions/`**.
ADRs are internal engineering records and have to be able to quote the banned words in order
to ban them — this very document lists every one of them. Scanning them would make the gate
fail on its own charter.

Advisory — reported in the job summary, **does not fail**:

5. Capability-claim phrases (`self-balancing`, `riderless`, `autonomous`, `proven`, …) whose
   enclosing section carries no requirement ID.

**`policy` is deliberately not a required status check yet.** Per the CEO's instruction it
must pass green at least once first. Required checks stay `rust` and `sim`.

## Options considered

- **Fail on every untraced capability phrase immediately.** Rejected for v1: the README's
  opening line calls Overboard a "DIY self-balancing onewheel" in a section with no
  requirement ID. That is a description of intent, not a capability claim, and a check that
  forces either a wrong edit to a CEO-owned file or an ever-growing allowlist is a check
  that gets disabled. It is reported as advisory instead, so the decision about that line is
  made by a person with the data in front of them.
- **Advisory-only for everything.** Rejected. A check that cannot fail is theatre. The four
  hard checks were chosen precisely because they are deterministic, currently green, and
  would genuinely catch the failure modes they name.
- **The four hard checks plus an advisory report.** Chosen. Cost of the losing options:
  either the gate gets switched off within a month, or it never catches anything.

## Consequences

- Adding a top-level directory now requires a `CODEOWNERS` edit — a small, intentional tax
  on exactly the change that creates turf ambiguity.
- The advisory list is the input to a future ADR that tightens check 5 into a hard one.
- The check logic lives in `.github/policy_check.py`, inside COO-owned territory, so
  tightening it never requires editing another role's paths.

## How this is enforced

`.github/policy_check.py`, invoked by the `policy` job. It is plain-stdlib Python and runs
locally with `python3 .github/policy_check.py` — run it before opening a PR rather than
discovering it in CI.
