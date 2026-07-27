# ADR-0002 — Map repository paths to roles

- **Status:** Accepted — ⚠️ **unopposed, not agreed** (see §Ratification)
- **Date:** 2026-07-26 (Proposed) · 2026-07-26 (Accepted with corrections)
- **Ratified by:** COO
- **Closes:** [Escalations — "Confirm the path ownership map before ADR-0002 is ratified"](https://app.notion.com/p/3a9472a5fb698123862ee0c30bbc4b70)
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

## Corrections applied before acceptance

Adversarial review found the `Proposed` version **wrong in the tree as it stood**, not merely
incomplete. All of these are now in `CODEOWNERS`:

1. **`sim/scenarios/plant.py` and `imperfections.py` → Sr. Mechanical & Systems.** The
   original `bench_*` prefix seam left them with Controls, and they are the fidelity contract
   — verbatim Mechanical's per its handoff. The map assigned Mechanical's two most
   characteristic files to the wrong role. `sim/models/bench_rig.xml` landing correctly was
   luck of naming, not the seam working.
2. **Intra-`sim/models/` clause.** Mass, inertia, geometry, contact and friction are
   Mechanical's and need a row. `<sensor>` and `<actuator>` elements are Controls' and do
   not — adding observability is a routine controls act, and routing it through an escalation
   weekly would just get the protocol ignored.
3. **`.github/workflows/ci.yml` → Senior Controls.** COO owning `.github/` means owning
   governance — `CODEOWNERS`, `policy_check.py` — not the build.
4. **`sim/out/` → Senior Controls** as harness output. Media *promoted* for publication moves
   to a Content-owned path and is Content's from that point; it does not become Content's by
   being written into this directory.
5. **`.claude/` → CEO**, explicitly. It contains role definitions; it must not arrive via the
   `*` default by accident.
6. **Ownership governs write, not read or import.** Without this a cautious session files a
   row before importing a module — over-escalation freezing, which is the same failure as
   trespassing pointed the other way.
7. **CMO's null claim stated on purpose**, alongside Senior Digital Marketer and Archivist,
   so their absence reads as a decision rather than an oversight.
8. **Root-level source files are not a valid resting place** — they get a directory and an
   owner before commit. A verbatim GPLv3 `comm_can.c` once sat untracked at this root, one
   `git add .` from a public MIT repo.

**Still outstanding — the seam itself.** Review's primary finding was that a filename-prefix
seam is discoverable only from `CODEOWNERS`, whereas a directory is discoverable from `ls`.
A fresh Mechanical session naming a file `param_id_sweep.py` lands in Controls' territory
with no error, no conflict and no signal. The durable fix is
`sim/scenarios/plant/` vs `sim/scenarios/control/`, mirrored in `tests/` — roughly six file
moves **in two other roles' territory**, so it is filed to them rather than done here.

## Ratification — unopposed, not agreed

The escalation row to the CEO was open and unanswered when this was promoted. It was promoted
early rather than at its deadline because leaving a **known-wrong** binding map in force is
worse than the paperwork being untidy, and because ADR-0005's registry checks and the turf CI
check both treat `CODEOWNERS` as authoritative.

`INDEX.md` requires the record to distinguish *agreed* from *unopposed*. **This is unopposed.**
Reversing it is a one-line status change plus a revert of the corrections above.

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
