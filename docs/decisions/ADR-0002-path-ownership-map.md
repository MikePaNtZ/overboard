# ADR-0002 — Map repository paths to roles

- **Status:** Proposed
- **Date:** 2026-07-26
- **Ratified by:** *(not yet — awaiting CEO, see Escalations row)*
- **Closes:** [Escalations — "Confirm the path ownership map"](https://app.notion.com/p/150eb337e89349948f980d2bb06bab80)
- **Constrains:** every role's "do not edit paths another role owns" rule
- **Enforced by:** `policy` CI job (coverage and role-tag validity only — it cannot check that a boundary is *correct*)

## Context

The org protocol tells each role not to edit paths another role owns, and to file a row
instead. No such map existed. Without one, "turf" is unfalsifiable: a session cannot tell
whether it is trespassing, so it either freezes or guesses.

This ADR is marked **Proposed**, not Accepted, on purpose. A path map binds every other
role, which makes it a Promise under the escalation rule — the COO does not get to set it
unilaterally just because the COO owns the file it lives in.

## Decision

The map lives in `.github/CODEOWNERS`, with the role carried in a `# role:` tag above each
rule. GitHub's format can only name real accounts and this org has one human, so
`@MikePaNtZ` is the reviewer on every rule and the tags carry the actual assignment.

Boundaries are taken **from what the handoff documents already assert**, not invented:

| Path | Role | Evidence |
|---|---|---|
| `*` (default) | CEO | Unclaimed territory is a scoping question |
| `README.md`, `CLAUDE.md`, `LICENSE` | CEO | Public claims and licence are one-way doors |
| `.github/`, `docs/`, `docs/decisions/` | COO | This role owns cross-role ops and the record |
| `crates/`, `Cargo.*`, `notebooks/`, `tests/`, `sim/`, `scripts/` | Senior Controls | The control law and its harness |
| `sim/models/`, `sim/scenarios/bench_*`, `tests/test_bench_*` | Sr. Mechanical & Systems | Handoff: owns the bench rig, the MuJoCo plant model, and the sim-to-hardware fidelity contract |
| `scripts/render_scenario.py`, `docs/web-artifact-pipeline.md` | Digital Content Production | Handoff: owns renders; explicitly does *not* own page markup or CSS |

The boundary deliberately runs **through** `sim/` rather than around it. Mechanical owns
where plant numbers come from; Controls owns the law tuned against them. Drawing the line at
the directory would have handed one role the other's work.

## Options considered

- **Map at directory granularity only.** Simpler, and wrong here: `sim/` and `tests/`
  genuinely contain two roles' work, and the bench-rig files are exactly the ones most
  likely to be edited by the wrong session.
- **Leave ownership implicit and rely on the handoff docs.** Rejected. Handoffs are prose in
  Notion; a session that did not read them cannot be caught by them, and nothing fails.
- **File-level map with a CEO default.** Chosen. Cost of the losing options: turf collisions
  that surface as merge conflicts or, worse, as a silently reverted decision.

## Open boundary questions this raises

1. **`scripts/`** is split by a single file. If the render pipeline grows, it should become
   its own directory rather than accumulating exceptions.
2. **`docs/`** defaults to COO because it is the mirror surface, but individual documents are
   authored by the role that owns the subject. Two are called out explicitly; more will need
   the same treatment as `docs/` grows.
3. **`overboard-logo.html`** is a brand asset in a repo whose own rules say brand assets live
   in `overboard-web`. It is left on the CEO default and flagged rather than moved, because
   moving it is CMO territory. **This one wants a decision.**

## Consequences

- A new top-level directory now fails `policy` until someone assigns it. That tax is the
  point: the moment a new directory appears is the cheapest moment to decide who owns it.
- If a boundary here is wrong, the correction is a row — not an edit to this file by the
  role that disagrees with it.

## How this is enforced

`policy` checks that every top-level path has an explicit rule and that every `# role:` tag
names a known role. It **cannot** check that a boundary is correct; that is what the
Proposed status and the escalation row are for.
