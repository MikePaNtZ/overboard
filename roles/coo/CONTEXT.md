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

- **2026-07-27 — match the agent type to the work before dispatching.** `sonnet-executor`
  carries Read/Write/Edit/Bash/Grep/Glob only. A Notion + web-research task sent to it burned
  a full agent boot to report that it had no tools. Use `general-purpose` for anything needing
  MCP or the web. Now printed by `ops/dispatch.sh`.
- **2026-07-27 — issues beat queue rows for getting work done.** Issue #21 was argued over in
  two mutually-blocking escalation rows for hours; the moment it became a GitHub issue with an
  acceptance criterion, Controls closed it unprompted and without dispatch.

## Known dead ends

_Nothing recorded yet._
