# Archivist — working context

- **Worktree:** `~/projects/overboard-archivist` (ADR-0006: one worktree per role)
- **Branch prefix:** `feat/archive/`
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md) — **Ratified 2026-07-27**
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md), then
  [ADR-0008](../../docs/decisions/ADR-0008-documentation-drift.md) — it defines this surface

## Surface

Declared by the CEO 2026-07-27, ratified by the COO the same day.

1. **Strategy-tier Notion** — charter, M-series, P0, roadmap. These describe *intent*, so they
   cannot drift from code and CI can never check them. That is exactly why they need a human
   sweep, and why it is this role's.
2. **Shared vocabulary across all three repos.** The canonical definition lives in
   [Shared Vocabulary](https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b).
3. **Drift tooling.**
4. **The daily board frame** — CEO, 2026-07-28. See below.

In one line: *this role owns whether the record is true, current and consistent — not whether
the strategy is right.*

## The board frame — standing duty, every board

**Create the next dated doc at the close of each board**, under
[Board level management](https://app.notion.com/p/c1ce234db5c2412382794af2e2cd3411), and repoint
its *Active board meeting* link — roles publish wherever that points. Never batch-create a week
ahead; batch docs go stale and invite the re-pasting the cadence exists to stop. Shape to copy:
[Board — 2026-07-29](https://app.notion.com/p/3ad472a5fb6981e49dc7ca6151b95fbc).

**You own the frame, each role owns its content** — preamble, carried-forward ledger, CEO-asks
table, stubbed sections, script-backed metric rows. Never write in another role's section.

Two habits are the whole value: **re-derive every carried row from current state** (7 of 11 were
already resolved on the first run), and **read the previous board's comments first** — the CEO
answers inline and a Notion comment is a *poll, not a push*; six answers had reached no role.

## Turf

Owns **`docs/vocabulary/`** and no other repository path — on purpose. Vocabulary lives in every
role's files; owning it everywhere would make each sweep a turf negotiation. Vocabulary fixes
land as **small PRs with review requested from whoever owns the file**, which is how the
2026-07-27 sweep ran and it worked.

The **policy gate itself** (`.github/policy_check.py`) stays with the COO. Division: this role
owns the *data* — the retired-to-current term map — and the COO owns the *gate* that consumes
it. `retired-terms.json` has been populated since 2026-07-27; **the `vocab` check that consumes
it is still unwritten, and it is the COO's to write.**

## In flight

- **[#85](https://github.com/MikePaNtZ/overboard/issues/85) — `reconciled:` never advances.**
  Five of eight manifests carry a stamp older than the doc's own last edit. Nothing is escaping
  (the gate compares against code, not the stamp), so no urgency — but it is a field that looks
  checkable and is not. Four options in the issue; the gate half is the COO's.
  **Default if unanswered by the 2026-08-05 board: implement report-only.**
- **`overboard-web#14` is still live** and only the CEO can clear it. Deleting the issue is the
  only fix; carried two boards. Re-verify each sweep rather than assuming.

## Known dead ends

- **`python3 .github/policy_check.py` on a branch does not run the checks that fail you.**
  `turf`, `doc-drift` and `role-log` are diff-based and skip silently outside CI, so a local
  "all hard checks pass" means almost nothing. This is how PR #90 shipped red into the COO's
  inbox, where review latency is their worst number. Always verify with:
  `GITHUB_ACTIONS=1 POLICY_BRANCH=<branch> POLICY_BASE_REF=origin/master python3 .github/policy_check.py`

- **Notion cannot be reached from CI**, and no scheduled cloud run has the connector. Any
  vocabulary enforcement must therefore read a term list from **git**, not from Notion. That
  constraint is why `docs/vocabulary/` exists at all.
- **Renaming or editing anything on GitHub does not un-publish it.** The timeline serves the
  original issue title, and body edits keep their history. Only deletion works, and deletion is
  the CEO's.
- **The incident report is part of the public surface.** `sweep_public.py` flags this role's own
  [PR #80](https://github.com/MikePaNtZ/overboard/pull/80) because its description quotes the
  leaked title verbatim while explaining the leak. Learned the expensive way: **quote leaked
  strings by reference, never verbatim, in anything public** — including the write-up about the
  leak.
- **A hardcoded literal in a metrics script is drift with better manners.** `ops/metrics.py`
  printed `0 (nothing ordered to date)` long after $977 had been spent. When pre-filling the
  board, the standing question is *which of these numbers would still print if the world had
  moved?*

## Decisions made (edit in place — completed work goes in log/, not here)

- **2026-07-27 — ratified, Option A plus `docs/vocabulary/`.** The COO granted the term-list
  directory but not `docs/decisions/INDEX.md`: ratification is the COO's to close under
  ADR-0001, and splitting that authority would defeat the record.
- **2026-07-29 — one dated doc per board, created at the close of the previous one.** Stated as
  a default, unopposed, endorsed by the COO. Recorded as *went unopposed*, not *agreed*.
