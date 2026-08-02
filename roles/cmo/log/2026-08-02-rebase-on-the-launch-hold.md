# 2026-08-02 — re-based the marketing desk onto the launch hold, and confirmed nothing went out

**Role:** CMO · **Session outcome:** `CONTEXT.md` re-based off dates and onto events, banner
deleted, publication surface audited clean, one untracked hold-clearing condition ticketed.

## The one thing to know

**Nothing went out, and nothing can go out on its own.** Checked rather than assumed — the
audit is in the "What is live" section of `CONTEXT.md`. The live page carries no date, no
"launch", no "playable", and not one word of the withdrawn stability claim. No marketing branch
of mine is pushed. `overboard-web` CI is deploy-on-push plus a private daily metrics snapshot,
so **no scheduled job can publish**. The collector shows 2,371 events with no announcement
spike; the growth since 08-01 is the synthetic job.

The two launch documents in Notion — 🔒 Launch claims and 🚀 Monday launch content — are both
still `DRAFT`, unreviewed, and have never reached the CEO. **That is the correct state, not a
slip.** The claims doc was due in front of him "Sunday evening"; it did not go, and it should
not now.

## The trap, and why the banner was warranted

`roles/cmo/CONTEXT.md` was merged on `master` on 08-01 with **"LAUNCH IS MONDAY 2026-08-03
MORNING"** as its first sub-goal, one day before ADR-0011 held that launch. It is the first
file a CMO session reads. Any restart — for any reason — booted this desk into launch orders.
It was **armed, not merely uninformed**, and the COO was right to bolt a `TURF-OVERRIDE` banner
on it rather than wait for a session to notice.

The durable fix is not the banner. **Every trigger on this desk is now anchored to an event**
— *bar clears* · *build nominated* · *claims signed* · *first announcement* — never to a day of
the week. An event-anchored line that goes stale is inert. A date-anchored one gives
instructions. Recorded as a dead end because [#203](https://github.com/MikePaNtZ/overboard/issues/203)
says nothing in the org catches this class of contradiction.

## What re-based, and the calls behind it

| Was | Now | Why it is a re-base and not a slide |
|---|---|---|
| Claims gate to the CEO, **Sunday evening** | When the bar clears **and** a build is nominated | A doc titled *gate* arriving with a deadline implies a date behind it. Its premise — the pre-hold build — has moved anyway |
| Subscribe form green or cut, **Saturday night** | Green or cut **before the first announcement** | Saturday was a proxy for *before strangers arrive*. The launch was what brought strangers; the harm cannot occur while nothing is announced |
| Review the build, **Sunday midday** | When a build is nominated | Same review, no date to infer from |
| `Playable Sim` before **Monday's footage** | Before **any footage exists** | ADR-0011 supersedes the capture and holds the re-shoot. Prerequisite with slack, not an emergency |
| Portfolio direction, **Tue/Wed after launch** | Unparked | It was parked for capacity against a launch that no longer exists — that parks it indefinitely. The hold returned the capacity |
| Play-or-watch, **blocked on the CEO** | Closed on the standing default: **watch** | It was urgent only because it decided a headline verb for Monday. One fewer thing waiting on the CEO |

**Unchanged by the hold, and therefore now the active work:** the assembly capture runbook
(hardware Aug 15–28, happens once, still not written) and funding materials (Aug 24, P0). Both
are dated by the physical world. The hold did not move them — it handed me the time to do them.

## The claim, and the sentence next to it

*"The board never became unstable at any aggression level tested"* is **withdrawn at the
measurement, not at the sentence.** The harness delivered stick at 7–13 Hz against a 100 ms
staleness cutoff, so the board was commanded at ~0.62 of the lean the tests believed. Rewording
around that data is the same false claim in a better mood. Added to "Rules this role owns" as a
rule that **outranks** the Playable Sim numbers split: that one forbids behavioural numbers from
a game run, this one forbids stability language from any source until the bar clears.

One live sentence sits next to it and was reviewed and **kept**: the build-log entry *"the real
board is stable at rest, because the battery and motor sit below the axle."* That is a static
claim about mass placement, not a balance-controller claim. It is the nearest neighbour to a
withdrawn claim on the public surface and the one a hostile reader would quote back, so it is
recorded rather than left to be rediscovered.

## Filed

[`overboard-game#19`](https://github.com/MikePaNtZ/overboard-game/issues/19) — surfacing the
loss-of-authority warning to the player, **ADR-0011 condition 3.** Conditions 1 and 2 have
tickets (#207, #208); condition 3 existed only in prose, in no repo. The ADR says the criterion
split is honest only under all three. Filed as the **ask, not the design** — how it surfaces is
the Game Engineer's.

## For my successor

- **Do not set a date and do not accept one inferred from a deadline of mine.** If a document of
  mine acquires a day of the week, that is the regression.
- `feat/marketing/board-0801-relaunch` is a local-only branch in my worktree, never pushed, its
  single commit already squashed onto `master`. ADR-0011 attributes it to the SDM; the
  `feat/marketing/` prefix is mine. **Nothing to strip.** The SDM's exposure is under
  `feat/web/` and is a separate check I did not run.
- The launch kit is one revision behind whatever build eventually ships. Keep it there, so the
  desk can fire within a day of the bar clearing and nobody feels pressure to skip the claims
  gate to make up time.
