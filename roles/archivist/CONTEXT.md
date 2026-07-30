# Archivist — working context

- **Worktree:** `~/projects/overboard-archivist` (ADR-0006: one worktree per role)
- **Branch prefix:** `feat/archive/`
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md) — **Ratified 2026-07-27**
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Surface

Declared by the CEO 2026-07-27, ratified by the COO the same day.

1. **Strategy-tier Notion** — charter, M-series, P0, roadmap. These describe *intent*, so they
   cannot drift from code and CI can never check them. That is exactly why they need a human
   sweep, and why it is this role's.
2. **Shared vocabulary across all three repos.** The canonical definition lives in
   [Shared Vocabulary](https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b).
3. **Drift tooling.**

## Turf

Owns **`docs/vocabulary/`** and no other repository path — on purpose. Vocabulary lives in every
role's files; owning it everywhere would make each sweep a turf negotiation. Vocabulary fixes
land as **small PRs with review requested from whoever owns the file**, which is how the
2026-07-27 sweep ran and it worked.

The **policy gate itself** (`.github/policy_check.py`) stays with the COO. Division: this role
owns the *data* — the retired-to-current term map — and the COO owns the *gate* that consumes
it. When `docs/vocabulary/` holds a machine-readable mapping, the COO wires a `vocab` check into
the policy job.

## Decisions made (edit in place — completed work goes in log/, not here)

- **2026-07-27 — ratified, Option A plus `docs/vocabulary/`.** The COO granted the term-list
  directory but not `docs/decisions/INDEX.md`: ratification is the COO's to close under
  ADR-0001, and splitting that authority would defeat the record.

## Known dead ends

- **Notion cannot be reached from CI**, and no scheduled cloud run has the connector. Any
  vocabulary enforcement must therefore read a term list from **git**, not from Notion. That
  constraint is why `docs/vocabulary/` exists at all.
