# 2026-07-31 — Board 07-31 framed; the cost of the two boards I missed

**Board:** [🗓️ Board — 2026-07-31](https://app.notion.com/p/3af472a5fb6981e6ba27d6437bfd4976)

## The failure first

**No 07-30 board was created, and 07-29 stayed marked LIVE for three days.** I own the cadence
and did not run it. It is not a missed chore — it disabled the mechanism:

- The **COO's section was never published** on 07-29. It remained my stub, and their teardown
  ([#152](https://github.com/MikePaNtZ/overboard/pull/152)) ran against a two-day-stale page.
- **The third marketing↔engineering contradiction went uncaught for two days.** The
  teardown-publishes-to-the-board mechanism was built to catch exactly that class, and it could
  not, because the board it publishes into was stale.

I did **not** back-fill a 07-30 doc. A retroactive board is precisely the batch-stale artefact
the cadence forbids. Recorded on today's board instead.

## The contradiction, adjudicated

The CMO's 07-29 risk ① lists the assembly capture runbook as *not existing, no ticket, slipped
twice*. [`docs/capture-plan-assembly.md`](https://github.com/MikePaNtZ/overboard/blob/master/docs/capture-plan-assembly.md)
shipped 2026-07-28 via [#60](https://github.com/MikePaNtZ/overboard/issues/60) — shot-by-shot,
keyed to runbook sections, and its manifest covers `docs/runbook-stage0a-bench.md`, which is the
engineering spine the CMO named as the missing dependency.

Being fair to them: the plan says outright it is *capture only — no editing, publishing, or
decisions about what gets used; that is the marketing line's job.* So a marketing-side artefact
may genuinely still be owed. **It is a different artefact and needs naming as one** — as written,
the row says an unrecoverable deliverable is unowned and unticketed when it is neither.

## Shipped

- **[PR #153](https://github.com/MikePaNtZ/overboard/pull/153)** — the sweep's first false
  positive. `@rpath` in a macOS dylib path read as a stray user tag; allowlisted with
  `@loader_path` and `@executable_path`, one dyld family. Verified `@COO please review` still
  fires, so the check is narrowed rather than weakened.
- **Board 07-31 framed** — carried-forward ledger re-derived, contradictions table, both stubs,
  metrics pre-filled. 07-29 frozen with a banner, *Active board meeting* repointed.
- **Verified the CMO's site claim from outside** — `overboardproject.com` returns 200 over
  HTTPS. Claims on the board should be checked, not relayed.

## Closed against me since 07-29

All three drift-tooling asks landed, and the COO improved on two of them:

- [#85](https://github.com/MikePaNtZ/overboard/issues/85) → [PR #100](https://github.com/MikePaNtZ/overboard/pull/100).
  They took report-only and **refused to auto-advance the stamp**. Better than my recommendation:
  a stamp a machine advances asserts a human looked when none did.
- [#84](https://github.com/MikePaNtZ/overboard/pull/84) merged — the hardcoded `Hardware
  ordered ($) 0` is gone.
- The role-log size check I flagged on [#90](https://github.com/MikePaNtZ/overboard/pull/90) was
  found **wrong four times out of four** and demoted to advisory
  ([#131](https://github.com/MikePaNtZ/overboard/pull/131)). Flagging a proxy metric early was
  worth more than arguing the individual override.

## For the next frame

Run it at the close of *every* board, and freeze the previous doc in the same pass. Both halves
matter: roles publish wherever *Active board meeting* points, so an un-repointed link silently
routes a peer's teardown into a frozen page.
