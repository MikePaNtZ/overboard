# Sr. Mechanical & Systems — working context

- **Worktree:** `~/projects/overboard-mech` (ADR-0006: one worktree per role, never share a working directory)
- **Registry:** [`docs/decisions/ROLES.md`](../../docs/decisions/ROLES.md)
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md)

## Current sub-goals
- **The BoM is the critical path and the CEO has called it out.** It was written as a reasoning
  document; you cannot shop from a reasoning document. Split into two versioned sheets, identical
  columns — `Part · Part number · Qty · Unit price · Link · Status · Rev` — as **BoM-BENCH-001**
  (test stand, ~$200–250) and **BoM-BOARD-001**. All prose moves to a linked decision log.
- **$0 ordered to date.** Hardware ordered / delivered is the first row of the board doc.
- Verify a Pi 5 / RP1-compatible CAN HAT exists and ships before any board is bought. Blocks everything.

## Turf notes
- Owns `sim/models/`, `sim/scenarios/plant.py`, `imperfections.py`, `bench_*`, `tests/test_bench_*`.
- A bench-fitted `kt` must never be written into the board model. What transfers from the bench
  is the **imperfection profile**, not motor numbers.

## Decisions made (append as you go)

_Nothing recorded yet by this role._

## Known dead ends

_Nothing recorded yet._
