# ADR-0001 — Establish the decision record and the escalation queue

- **Status:** Accepted
- **Date:** 2026-07-26
- **Ratified by:** COO
- **Closes:** none — this is the scaffolding the queue itself needs, directed by the CEO
- **Constrains:** every role
- **Enforced by:** `policy` CI job (ADR index integrity); convention for the rest

## Context

Overboard is built by parallel Claude sessions with no ability to message one another. They
share only Notion, the git repos, and Mike. Before this ADR the org had **neither** of the
two artifacts its own protocol assumed: there was no `docs/decisions/` directory anywhere in
the repository's history, and no Escalations database in Notion.

The practical failure this creates is specific. A session that compacted, or one started
fresh tomorrow, has no way to learn that a decision was made yesterday. It will re-derive
it, contradict it, or silently build on a superseded assumption — and nothing will catch
that, because a Notion page nobody opened is not a control.

## Decision

Two artifacts, with a hard line between them.

1. **The Escalations database** in Notion is the *queue*. One row per decision meeting
   Promise, Door, or Turf. Every row carries a **default action** and a **deadline**, both
   mandatory. Options, rationale and Oracle verdicts go in the page body, never in columns.
2. **`docs/decisions/`** in this repo is the *record*. Only an ADR here binds anyone.

`Answered` is an opinion recorded in Notion. **`Ratified` means it exists in git** — an ADR,
plus a CI check whenever the decision constrains code or public claims.

**Ratification is the COO's to close, not the implementer's.** The COO writes the ADR, adds
the check, and reports which sessions need restarting. The org has no broadcast primitive
except restart, so a decision that is not written to git and announced has not landed.

## Options considered

- **Notion-only record.** Rejected. A session must be *told* to read Notion, and a stale
  session by definition was not told. Notion cannot produce a red build.
- **Git-only record, no queue.** Rejected. Escalation needs an asynchronous inbox with
  deadlines that Mike can answer between sessions; a PR comment thread is not that, and
  git has no notion of "unanswered by Thursday, so I proceeded".
- **Both, with the split above.** Chosen. The queue is where a decision is *argued*; the
  record is where it becomes *binding*. Cost of the losing options: either decisions do not
  reach a fresh session, or they never get made because someone is parked waiting.

## Consequences

- Every role must read `docs/decisions/INDEX.md` before opening a PR. This is cheap — the
  index is one screen.
- The COO becomes a serialisation point for ratification. That is deliberate: it is the
  only way a decision gets the same treatment regardless of which role made it.
- A decision can now be *stale-proof*: if it constrains code, the CI check outlives the
  session that agreed to it.
- Nobody blocks. The default action fires at the deadline, and the record distinguishes
  "agreed" from "went unopposed" — those are different facts and were previously conflated.

## How this is enforced

The `policy` CI job checks that every `ADR-*.md` file appears in `INDEX.md` and that every
ADR referenced by the index exists, so the index cannot silently drift from the record it
indexes. Everything else here is convention, and depends on `INDEX.md` actually being read
before a PR is opened.
