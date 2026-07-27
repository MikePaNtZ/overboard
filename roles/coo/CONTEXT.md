# COO — working context

- **Worktree:** `~/projects/overboard-coo` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- Land the delegation layer: issues → dispatch → PR review (ADR-0007).
- Get a real daily usage number in front of the CEO. `python3 ops/usage.py`.
- Keep ops under 10% of spend and say so on every board if it is not.

## What this role must not do
- Dispatch more than 3 workers at once, or dispatch from a dirty tree (`ops/dispatch.sh`).
- Ratify a cross-role Promise without a row, or ratify one-way/public work without the Oracle.
- Build documentation where a CI check would do. Documentation is a polling surface.

## Decisions made (append as you go)

_Nothing recorded yet by this role._

## Known dead ends

_Nothing recorded yet._
