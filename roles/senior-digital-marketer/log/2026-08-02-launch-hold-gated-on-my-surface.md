# 2026-08-02 — The launch hold, gated on my surface

Session brief: ADR-0011 holds the 2026-08-03 launch. Strip or re-date
`feat/marketing/board-0801-relaunch`; the stability claim is withdrawn and may not be
reinstated in any softened form; keep the site clean; publish nothing.

**Nothing went out. Nothing needed repairing.** Everything below is prevention.

## What I found, against what I was told

Two of the three things in the brief turned out not to be true of the repo, and finding that
out cost four `git` commands. Recording it because "go and look before you ask" earned its
keep here.

**1. `feat/marketing/board-0801-relaunch` is not mine, and there is nothing on it to strip.**

- It is the **CMO's** branch: `feat/marketing/` is the CMO prefix, its one own-commit
  (`f9ef4fd`) touches only `roles/cmo/**`, and it is checked out in `~/projects/overboard-cmo`.
- It **merged as PR #164 on 2026-08-01T21:01**, the day *before* ADR-0011 was ratified.
  Squash-merge gave it a new SHA (`2c87e38`), which is why `git log master..branch` still
  lists it and it *looks* unmerged. It is not.
- Diffed against `origin/master` it is **9 insertions, 10585 deletions** — strictly behind.
  Its only delta inside `roles/cmo/` is the *absence* of the COO's hold banner.
- The dead date it carried is therefore already on `master`, and the **COO already defused it**
  on 2026-08-02 with a `TURF-OVERRIDE` banner at the top of `roles/cmo/CONTEXT.md`. That banner
  states the hold, states there is no new date, and restates the claim as withdrawn. It ends
  "Delete this banner once you have re-based the sub-goals below it" — addressed to the CMO.

So there was no strip-or-re-date for me to perform. Editing `roles/cmo/CONTEXT.md` would have
been Turf (CODEOWNERS:180) on a file another role had already deliberately handled, and
deleting a branch checked out in another role's worktree is the ADR-0006 failure verbatim.
**I left both alone.** Residual risk is low: master is strict-protected, so any PR from that
branch must be brought up to date first, which restores the banner rather than dropping it.

**2. The site was already clean, and I widened the proof.** The 2026-08-02 audit covered the
live page. I re-ran it across **every ref in `overboard-web`, merged and unmerged** — no branch
anywhere carries `2026-08-03`, a Monday launch date, or the withdrawn claim. Page status still
reads `Phase 0 · sim-first · nothing on the ground yet`, which announces no launch and is
correct under lock-step (`SR-WEB-4`).

**3. The CONTEXT sub-goal about two unpushed commits is stale.** `git ls-remote` shows
`feat/web/retire-ai-section` on the remote. Nothing is sitting locally. Removed from CONTEXT.

## What I actually built — overboard-web#55

ADR-0011 says it plainly: the `policy` gate registers the withdrawn claim but **gates the
controls repo, not `overboard-web`**. On my surface there was no check at all — the only thing
stopping the claim reaching the page was a session having read the ADR. That is the gap worth
a session, so that is what I closed.

`check_page.py` rule **6c** now fails the build on:

- the withdrawn claim **and softened forms of it**, matched by *shape* rather than by one
  exact string, because ADR-0011 forbids reinstatement "in any softened form" — denials of
  instability, stability asserted over an input range, `never flipped`, `always caught
  itself`, `handles full stick`;
- the dead date in every format the page might write it, plus dated launch announcements.
  Hard fail, not a note: there is **no new date**, so any of these is wrong *by construction*
  today rather than merely unverified.

Two scope decisions worth keeping:

- **`lab/` is covered.** `deploy.yml` copies `lab/` into `_site`, so the design harnesses are
  public and a claim parked in one is a published claim. The rule iterates everything the
  deploy actually serves, not just `index.html`.
- **Matched against raw source, not comment-stripped.** The rest of the file deliberately runs
  on stripped markup, because the source *discusses* autoplay and loop in its comments. This
  rule is the exception: comments are served to anyone who views source, so a claim in one is
  still published.

Verified before opening: passes clean as the page stands; catches all 15 claim/date variants
tested; does not trip on `stable at rest`, `its instability`, `August 30`, `30 August 2026`,
or `It falls over`. `lab/` coverage confirmed by injection.

Recorded the decision in `overboard-web/CLAUDE.md` beside the lock-step rule, per that file's
own convention. **When the hold lifts this gets narrowed, not deleted** — in the same pass as
the superseding ADR, with a measured number in place of the word "stable" (exit criterion 2).

## Open — raised to the CEO, deliberately not acted on

The `#now` heading reads **"Right now: it stays up, and it goes where it's told — as long as
it's carrying a rider's weight."**

The 08-02 audit cleared the site of *the withdrawn claim*, and this is not it. But ADR-0011
also says, more broadly, that **no public artifact may assert stability of the balance
controller** until the exit criteria are met — and "it stays up" is that assertion, in the
page's most prominent frontier line. The heading's own governing comment says it "always names
the CURRENT failure mode… and gets rewritten every time the frontier moves." The frontier moved
on 08-02: full forward stick from rest inverts the board.

Not acted on, for three reasons that all point the same way: it is live page prose, so the copy
gate applies and I am the author (CMO clears, never me); the brief was explicit that nothing
publishes; and where the frontier line sits is positioning, which is the CEO's and the CMO's
call, not a safety-net edit. **Raised in the session report for a decision.** It is not urgent
in the sense that anything false is live under the audit's scope — it is a question of whether
that scope was the right one.

The three clips the heading sits above are shove-rejection and a scripted shuttle route, both
still true of what they show. The exposure is the general present-tense framing, not the
captions.

## Handover

- **overboard-web#55** queued with `--squash --auto`. Not copy, so the copy gate does not
  apply and self-queueing is correct here.
- The gate is the durable artefact. A session that never reads ADR-0011 now cannot put the
  claim or the date on the page — which was the whole failure mode ADR-0011 warns about when
  it says the date "binds only sessions that read it".
- The `#now` heading question is the one live thread. Nothing else is parked on me.
