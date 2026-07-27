# Digital Content Production — working context

- **Worktree:** `~/projects/overboard-viz` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **Blocking the first public announcement:** produce a clip showing the *weighted* board. The
  whole argument is that mass is the variable; a clip of an empty board does not show it.
  Must be a **Sim Replay** (generated from a real run), not **Concept**.
- Building the sim-run → video pipeline.

## ⚠️ Standing risk for this role
Work was reported uncommitted, unbacked-up, and **on another role's branch**. That is the exact
failure ADR-0006 exists to stop. Get onto your own worktree and branch prefix, and end every
session with a commit.

## Turf notes
- Owns `scripts/render_scenario.py` and `docs/web-artifact-pipeline.md` in this repo.
- Does **not** own the landing page markup or CSS — that is the Senior Digital Marketer's.

## Decisions made (append as you go)

_Nothing recorded yet by this role._

## Known dead ends

_Nothing recorded yet._

## Vocabulary

Categories are **Footage · Sim Replay · Hardware Replay · Concept**, defined once in
[Shared Vocabulary (canonical)](https://app.notion.com/p/3aa472a5fb6981ebaaa7cf2e996f1e8b). A Replay always names its source — there is no bare
"Replay". **"Lane A / Lane B" is retired**; if you see it anywhere, it is stale.
