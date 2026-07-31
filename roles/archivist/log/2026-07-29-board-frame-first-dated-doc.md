# 2026-07-29 — First dated board doc; two false-metric findings

**Board:** [🗓️ Board — 2026-07-29](https://app.notion.com/p/3ad472a5fb6981e49dc7ca6151b95fbc)

## What shipped

**The board-frame duty ran for the first time.** Created the dated doc, stubbed all three
role sections on the five standard headings, pre-filled every script-backed metric row from
`ops/metrics.py` and `ops/usage.py`, and carried forward last board's items **re-derived
rather than re-pasted**. Marked the running page frozen and repointed *Board level
management* at the dated doc, so a role that follows the teardown prompt cannot publish into
the wrong place.

The carry-forward pass is the part that paid. Of eleven items carried from 2026-07-28,
**seven were already resolved** — six by CEO comments left between 05:16 and 05:38 that
morning, which no role had seen. Re-pasting them would have spent CEO decision slots on
settled questions for the fourth consecutive board.

**Surfaced ten unanswered CEO asks.** The CEO's replies live as inline Notion comments. A
comment is a *poll, not a push*: it lands only if a role happens to re-open the page. They
are now a table in the frame, each with an owning role.

## Findings

**① `ops/metrics.py` printed a false number, and I was required to publish it.**
`Hardware ordered ($)  0  (nothing ordered to date)` was a string literal, never computed.
Purchasing then happened — $977 across 10 orders — and the literal kept printing 0. Because
the frame duty says *pre-fill metric rows from the script*, an unfixed hardcode publishes a
false figure to the board every day under the authority of "script-backed".
Fixed in [PR #84](https://github.com/MikePaNtZ/overboard/pull/84); flagged rather than
propagated on today's board.

**② `reconciled:` in the ADR-0008 manifests never advances.** Five of eight docs carry a
stamp older than the doc's own last edit. Nothing is escaping today — `check_doc_drift()`
compares against the code, not the stamp — but it is a field that *looks* checkable and is
not. Filed as [#85](https://github.com/MikePaNtZ/overboard/issues/85) with four options; the
gate half is the COO's.

**③ The write-up about the leak re-published the leak.** `sweep_public.py` now reports two
errors, not one. The second is my own [PR #80](https://github.com/MikePaNtZ/overboard/pull/80)
description, which quotes the exposed `overboard-web#14` title verbatim while explaining that
exposing it was the problem. Same phrase, same origin — nothing new is disclosed — but
editing the body will not clear it either, because GitHub serves edit history. **The incident
report is part of the public surface.** Quote leaked strings by reference from here on.

## Note for whoever runs the next frame

Both ① and ② are the same shape, and it is the shape this org keeps producing: **a value that
carries the authority of having been checked, which nothing checks.** A hardcoded `0`, a
`reconciled:` sha, a carried-forward decision row. Worth treating as a standing question at
each frame pass — *which numbers on this board would still print if the world had moved?*
