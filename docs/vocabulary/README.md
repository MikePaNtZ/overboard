# Canonical vocabulary — machine-readable

**Owner: Archivist.** Seeded by the COO on ratification; the Archivist owns it from here.

The canonical *definitions* live in
[Shared Vocabulary (canonical)](https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b).
This directory exists because **CI cannot read Notion** — so a term list that lives only there
can never be enforced, and a retired term goes on quietly appearing in issues, docs and role
files for as long as nobody happens to notice.

That is not hypothetical. On 2026-07-27 "Lane A / Lane B" was retired in favour of
**Footage · Sim Replay · Hardware Replay · Concept**, and stale copies were still live in one
issue and two role context files hours later. The documentation-drift gate could not catch it,
because that gate watches code↔doc, not vocabulary.

## What belongs here

A **retired → current** mapping the policy gate can consume, e.g.

```
Lane A          -> Sim Replay | Hardware Replay   (a Replay always names its source)
Lane B          -> Concept
```

Plus any term whose misuse would mislead — particularly ones that could put an unbacked
capability claim in front of the public.

**Also `provenance-marks.json`** — the inverse of the mapping above. Some on-frame marks
carry an obligation that a tag alone cannot discharge: the mark being present *requires*
other text to also be present. **Playable Sim** is the first of these (#163) — a
`PLAYABLE SIM` mark with no non-physical channel declaration nearby is exactly the failure
that category exists to prevent, because it is the only category where some of what is on
screen is physics and some of it is not. `sweep_public.py` treats a mark with no matching
declaration as an `error`-severity finding, scoped the same as the vocabulary check: open
items only, never rewriting a past asset's declaration to match a later, longer one.

## What does not belong here

Definitions, rationale, or the argument for a term. Those are Notion's. Keep this file
mechanical, so the gate stays cheap and the human-readable version stays single-sourced.

## Status

**Populated 2026-07-27 — `retired-terms.json` is the mapping.** Eight terms, seven at `error`
and one (`bare-replay`) at `warn` because it is a heuristic that will produce false positives.
Over to the COO to wire the `vocab` check into `.github/policy_check.py`: this role owns the
data, the COO owns the gate.

Two things in the file are worth knowing before writing that check, because both come from
mistakes made during the sweep it was built from:

- **Word boundaries are not optional.** Every pattern is anchored. An unanchored `lane` matches
  `plane`, which cost a false positive on `overboard-viz#5` during the first sweep.
- **A term may appear if the same line links canonical** (`exempt_line_regex`). This is the
  duplication rule enforcing itself: the only legal way to name a retired term is to also say
  where the current definition lives. Checking per *line* rather than per *page* is deliberate —
  four documents were scored "handled" on the strength of a banner at the top while still
  restating the retired definition further down, and two of those restatements had already
  gone false.

**2026-08-01 — fifth category, `Playable Sim` (#163), and `provenance-marks.json`.** The four
categories became five before any footage existed: game capture is live, unrecorded and
unreproducible, so it fit none of `Footage · Sim Replay · Hardware Replay · Concept`. Landed
in Shared Vocabulary § 1, mirrored here. Playable Sim also introduced the first
required-presence check — see "What belongs here" above — because it is the only category
where a tag alone cannot tell a viewer which part of the frame is physics.
