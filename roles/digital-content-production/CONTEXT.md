# Digital Content Production — working context

- **Worktree:** `~/projects/overboard-viz` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **Blocking the first public announcement:** produce a clip showing the *weighted* board. The
  whole argument is that mass is the variable; a clip of an empty board does not show it.
  Must be Lane A (from a real run), not concept art.
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
