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

## What does not belong here

Definitions, rationale, or the argument for a term. Those are Notion's. Keep this file
mechanical, so the gate stays cheap and the human-readable version stays single-sourced.

## Status

**Empty by design.** The COO wires a `vocab` check into `.github/policy_check.py` once a
mapping lands here — this role owns the data, the COO owns the gate.
