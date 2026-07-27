# ADR-0008 — Two documentation tiers; catch implementation drift at PR time

- **Status:** Accepted
- **Date:** 2026-07-27
- **Ratified by:** COO
- **Closes:** none — CEO direction, 2026-07-27
- **Constrains:** anyone editing code that a doc describes; anyone writing docs
- **Enforced by:** `policy` CI job — `doc-drift` and `doc-size` checks

## Context

The project rule is that Notion is the primary home for design docs and repo `docs/` holds
mirrors that track the implementation. In practice the mirrors drift, and the CEO's
observation is sharper than "docs get stale": pages accrete **historical editorial trails**
until the current state is buried in a log of how we got here. A 55,000-character BoM was the
proof — excellent research, and unusable as the thing it claimed to be.

The obvious fix — have a role update docs on request, triggered by an issue — does not work,
and the reason is structural: **drift is definitionally the thing nobody noticed.** A
request-based process requires someone to spot it first. That is the same failure that left
two escalation rows deadlocked for hours over a question a merged PR had already settled.

## Decision

**1. Two tiers, because docs drift for different reasons.**

| Tier | Examples | Primary home | How it stays current |
|---|---|---|---|
| **Strategy** | Charter, M0/M1, P0, roadmap | **Notion** | Describes *intent*; cannot drift from code. Swept on a cadence at the board meeting |
| **Implementation** | ICD, controls design, sim test designs, BoM | **git** | Describes *code*; drift is mechanically detectable, and CI detects it |

Notion remains primary for strategy. For the implementation tier git becomes authoritative and
Notion is a mirror — because CI cannot read Notion, and a rule CI cannot check is a rule that
depends on someone choosing to look.

**2. Every implementation doc carries a manifest** naming what it describes, in an HTML
comment so it stays invisible when rendered:

```
<!--
covers:
  - sim/scenarios/impulse_response.py
  - tests/test_impulse_response.py
reconciled: e4ddb82
-->
```

**3. The build fails when code moves without its doc.** If a PR changes a covered file and
does not touch the doc, `doc-drift` fails — unless a commit message carries
`DOC-OK: <reason>`. Same shape as `TURF-OVERRIDE`, which has worked three times in one day,
and the reason lands in git history rather than in an editable PR body.

**The check never judges content.** It asserts only that somebody *looked* since the code
moved. That is the part a machine can check, and checking it is enough — a human who opens the
doc will fix what is wrong.

**4. Docs are capped at 40,000 characters, with a warning at 20,000.** Crude, and it creates
steady pressure toward the appendix-and-prune discipline without anyone policing prose.

**5. Notion sync becomes a publish step, not a task.** Once git is authoritative for the
implementation tier, pushing to Notion is mechanical: no judgement, no drift, no owner.

## Options considered

- **Archivist updates docs on request, triggered by issues.** Rejected. Requires someone to
  notice drift first, and one role cannot know what changed *semantically* across three repos.
- **Convention: every role tidies its docs when it finishes.** Rejected as the *only*
  mechanism — it is a discipline surface with no enforcement, which is precisely what failed
  twice on 2026-07-26. Kept as a norm on top of the check.
- **A scheduled drift report.** Useful, insufficient: it tells you about drift *after* it
  exists. The PR-time check prevents it being created.
- **Manifest + PR-time check + size cap.** Chosen. Cost of the losing options: docs that are
  trusted and wrong, which is worse than docs that are obviously absent.

## Consequences

- A trivial refactor of a covered file now needs either a doc touch or one line of override.
  That is the intended tax; it is one line, and it forces a moment's thought about whether the
  doc is still true.
- The **Archivist gets a real surface** — it currently has none, which is why it is still
  `Provisional`. Not per-change updates: **own the drift tooling, and sweep the strategy tier**,
  which is exactly the part CI cannot reach because there is no code to compare against. That
  is where human judgement earns its cost.
- Adding a doc to the implementation tier is opt-in: no manifest, no check. Deliberate — it
  keeps the gate honest rather than universal on day one.

## How this is enforced

`.github/policy_check.py`:
- `doc-drift` — covered file changed, doc untouched, no `DOC-OK` → fail.
- `doc-size` — over 40k → fail; over 20k → advisory.
- `--reconcile <doc>` stamps the current HEAD into the manifest.

Verified before merge: the check fires on a covered change with the doc untouched, and clears
with `DOC-OK` in a commit message.
