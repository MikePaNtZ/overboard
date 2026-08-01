# 2026-08-01 — Launch re-cut around a playable game, and the claims gate

**Session:** CMO · evening · two days before launch.

## What changed under me

I had already published a board section written against *"the launch is a stripped-down page of
simulation results."* Two things then happened, in this order: the board was structurally corrupted
at ~19:40 and my section was rolled back in the CEO's restore, and the CEO inverted the strategy —
**Monday ships a playable Unreal game** driven by the real Rust control law at 500 Hz, with
post-ride reconstruction demoted from deliverable to public promise.

So this session was a re-publish plus a re-cut, not an update.

## What I produced

- **[🚀 Monday launch content — DRAFT, re-cut around a playable game](https://app.notion.com/p/3af472a5fb6981f59d3dd2cfe5320ce8)**
  — the stripped-down page rebuilt around a game. Drafted in Notion first, per the CEO.
- **[🔒 Launch claims — what we will and will not say about the game](https://app.notion.com/p/3af472a5fb6981cfb496e2444aff610e)**
  — the CEO's gate. Sentence-level say / do-not-say table, the four things that are not real, the
  promise copy. Due in front of him Sunday evening, before the readiness review.
- **[#163](https://github.com/MikePaNtZ/overboard/issues/163)** to the Archivist — a fifth
  provenance category, `Playable Sim`.
- **Board §1 re-published**, reconciled rather than retyped.

## Findings worth keeping

**The CEO's six launch elements survived the artefact change; only one moved.** Intro, repo links,
BoM, hardware-validation plan and subscribe-to-follow are unchanged in purpose. Only "pure
simulation results" became "a thing you watch someone drive". That was worth telling him first,
because the change reads bigger than it is.

**The hardware-validation plan turned out to do double duty.** It was already going on the page as a
credibility item. It is also exactly what the promise needs, because it is a published ladder with
safety gates already in it — so the promise points at something real instead of asserting good
intentions.

**The promise needed no new language at all.** "Film the CEO riding it" is **L3** in the Shared
Vocabulary launch ladder we already publish, behind L2 (riderless on real hardware) and the
ballasted-dummy bench. Naming a rung rather than a date is what stops it reading as a schedule, and
makes skipping one a visible breach rather than a quiet slip.

**Game footage cannot be filed under any existing category, and Sim Replay is the trap.** Sim Replay
means CG driven by *recorded* simulator data, and the vocabulary makes reproducibility the test —
*could someone regenerate this frame from the committed pose track plus the committed scene?*
Record-and-replay is explicitly cut from the weekend, so the answer is no. Filing it there would
break the one rule in the vocabulary that is checkable rather than a matter of opinion. Concept
fails too: the motion is not authored, and Concept may never carry an engineering number, which
would forbid saying "500 Hz" — which is true, checkable, and the most interesting thing we have.

**The style guide already solved the claims problem.** Rule 6 says concede the limit in the same
breath as the claim. Written that way the disclosure *is* the copy rather than a disclaimer under
it, and for an audience of engineers it is the part that earns anything. This turned the honesty
constraints from a compliance tax into the strongest section of the page.

**The artefact change strengthened the subreddit choice instead of upsetting it.** `r/ControlTheory`
was picked because it was the audience most likely to engage with a simulation-only result. "We put
our cascade controller under a game and let a human generate the step inputs a scripted profile
never would" is a better version of the same pitch, and this is the audience that respects the
caveats rather than punishing them.

## What I got wrong

**I corrupted the board at ~21:35 and repaired it by ~21:40.** My `update_content` edit opened a
`<table>` that the matched `old_str` had not closed, so Notion silently absorbed every following
block into it: §2 COO and §4 Archivist lost their prose entirely, and three separate metric tables
merged into one. No error was returned. I restored everything verbatim from the pre-edit fetch I
still had and verified structurally — four section headings, balanced tables, the *For the CEO*
closer — before confirming the write.

Two things follow, and both are recorded in `CONTEXT.md`:

1. **The COO's diagnosis of the earlier collapse is incomplete.** They attributed the ~19:40
   corruption to two roles writing at once. Mine was one writer and one malformed edit. Concurrency
   is not the whole mechanism; the deeper cause is that the page has **no schema validation and no
   undo a role can reach**. Their one-sub-page-per-role fix still holds, and needs one addition:
   a role must be able to verify its own write **without needing a pre-edit copy in hand**. I only
   recovered on luck.
2. **Announcing before writing is what made it survivable.** Nobody else was mid-edit, so the damage
   was mine alone and mine to fix. The interim serialise rule earned its keep on its first night.

Also learned the hard way: a full-page `replace_content` **drops existing comment anchors**, so the
reply to my own "writing now" comment 404'd and had to be posted as a new thread.

## Answered, so they do not come back as two answers

- **[#160](https://github.com/MikePaNtZ/overboard/issues/160) dashboard access** — the COO's default
  accepted as written: COO takes the auth design, I own what is displayed. Recorded as accepted
  rather than restated, because the point of the row was one answer and not two. Noted that the
  CEO's own offer to supply the token means a bookmarkable signed URL likely satisfies the whole
  requirement in an hour, and that it must not reach the launch critical path.
- **The DCP turf question** — the CEO's named default was right and I took it: convention review on
  published assets is mine. The generalising split is **what an asset claims or is labelled is
  mine; how work flows is the COO's.** And to his actual question — *is something missing in the
  process?* — yes: there was no step where a published asset is checked against the category rules
  before it goes out. The COO was covering a gap in the only way available. #163 is where that check
  gets added.

## Left open on purpose

- **Play or watch** — whether a stranger can play the game Monday. Neither build brief has
  packaging or QA in it, so the default is *watch*. It decides the headline verb, so it blocks
  finishing the copy. CEO's call.
- **No ticket for the post-capture plan.** Re-derived against the new artefact it shrank to one line
  in the W4 window with a different split. A ticket would be ceremony. Recorded as a call, not an
  omission.
