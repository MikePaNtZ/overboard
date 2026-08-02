# CMO — working context

- **Worktree:** `~/projects/overboard-cmo` (ADR-0006). Marketing work happens in
  `overboard-web`, `overboard-viz` and `overboard-metrics`.
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md) · [`ROLES.md`](../../docs/decisions/ROLES.md)
- **Branch prefix:** `feat/marketing/`
- In this repo I write **only** `roles/cmo/**` (ratified, CODEOWNERS:180). Everything else here is
  read-only to me.

## Current sub-goals — re-based 2026-08-02 onto the launch hold

- 🛑 **THE LAUNCH IS HELD and there is NO DATE.**
  [ADR-0011](../../docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md).
  Holding full forward stick from rest inverts the board in ~6.5 s — on the straight, at
  `steer = 0`, reachable in the playable build; the CEO hit it on first contact. **The hold is
  gated on an engineering bar, not on a calendar.** I do not set a date, I do not let one be
  inferred from a deadline of mine, and I do not run a readiness review. Every date this file
  used to carry is gone; **triggers below are events, never days of the week.**
- **What clears it is observable, so go and look rather than ask.** ADR-0011's second
  ratification respecified the exit bar; four criteria are met by
  [#205](https://github.com/MikePaNtZ/overboard/pull/205). Outstanding:
  [#207](https://github.com/MikePaNtZ/overboard/issues/207) (pin the estimator trim and the
  reserve's derivation), [#208](https://github.com/MikePaNtZ/overboard/issues/208) (encode the
  authored-world constraint as a check), and **surfacing the loss-of-authority warning to the
  player** — condition 3 of the criterion move.
- **The stability claim is WITHDRAWN, not softened.** *"The board never became unstable at any
  aggression level tested"* is **false** and may not be restated in any form, however hedged.
  The measurements behind it came through a harness delivering stick at 7–13 Hz against a
  100 ms staleness cutoff — the board was commanded at **~0.62** of the lean the tests
  believed. **Those numbers support nothing and may not be cited.** Anything I publish that
  needs them needs a re-measurement instead.
- 🔄 **The launch artefact is a playable Unreal game, not a page of simulation results.** Still
  true, and now with a caveat: the build that eventually ships is the *fixed* one, with a
  saturation warning and a constrained world. **Copy written against the pre-hold build
  describes a machine that will not ship.**
  Plan: [M3 Implementation Plan § Revision 3](https://app.notion.com/p/3af472a5fb6981f5b6e4ec038293ad6f).
- **What this desk does during the hold — three things, in order.**
  1. **Make sure nothing goes out.** Zero announced is now a *deliberate* number, not a
     backlog. It stays zero until the bar clears and the CEO signs the claims.
  2. **Spend the recovered capacity on the two real calendar deadlines** — the assembly
     capture runbook (hardware Aug 15–28) and funding materials (Aug 24). Both are dated by
     the physical world, not by the launch, so the hold does not move them; it just handed me
     the time to do them. **These are the active work.**
  3. **Keep the launch kit one revision behind the build**, so the desk can fire within a day
     of the bar clearing without anyone feeling schedule pressure to skip the claims gate.
- Answer the two big unknowns with data: do strangers care, and is *how we build it* more
  interesting than the onewheel. **We can measure the first one** — the instrument is live and
  the hold does not touch it.
- **Produced vs. seen by a stranger is still the whole game.** Six produced, five published on a
  live page, **zero announced anywhere**. The hold means that last number cannot move yet, and
  moving it early would spend the one first impression on a board that flips.

## What is live (re-verified 2026-08-02 ~08:00)

- `overboardproject.com`, HTTPS enforced, HTTP 200 in 0.19 s. Apex is DNS-only/grey-cloud so GitHub
  Pages holds its certificate; only `e.` is Cloudflare-proxied. **Do not flip the apex to orange —
  it takes the site's TLS down.**
- Collector at `e.overboardproject.com/e` → D1. `/health` reports events, last event, reject count
  and last reject reason, and is **unauthenticated** — it is the one number provable without a token.
  2,371 events, last at 07:51 UTC; 4 rejected all time, all "missing required field `site`",
  the most recent on 08-01. **No announcement spike — the growth since 08-01 is the daily
  synthetic job.**
- Dashboard at `/dashboard?k=…`, token-gated, fails closed, **404s on a bad token** (deliberate — a
  403 would confirm the route exists). Real vs. synthetic is `&site=synthetic-overboard`.
- Daily jobs: synthetic traffic, and a GitHub traffic snapshot into `overboard-metrics` (private).
- **Nothing launch-shaped is on the public surface, checked rather than assumed (08-02).** The
  live page carries no date, no "launch", no "playable", and not one word of the withdrawn
  stability claim; `overboard-web` `master` is unchanged since the policy-gate port; no
  marketing branch of mine is pushed; the three open web PRs (#41/#42/#43) are copy, not
  announcements. **No scheduled job can publish on its own** — web CI is deploy-on-push plus a
  private daily metrics snapshot, so nothing fires on a calendar.
  - The one sentence worth re-reading each time: the build-log entry *"the real board is
    stable at rest, because the battery and motor sit below the axle"*. It is a **static**
    claim about mass placement, not a balance-controller claim, so ADR-0011 does not withdraw
    it — but it is the nearest neighbour to a withdrawn claim on the live page, and it is the
    one a hostile reader would quote back. Reviewed 08-02 and kept.

## Rules this role owns

- 🛑 **No public artefact asserts stability of the balance controller** until the ADR-0011 exit
  bar is met (ADR-0011, registered with the `policy` gate). This is stronger than the existing
  Playable Sim rule and outranks it: Playable Sim forbids behavioural numbers *from a game run*;
  this forbids stability language **from any source**, including a clean scripted run, until the
  bar clears. When it does clear, the replacement is **a measured margin with its method**, never
  the word "stable".
- **A withdrawn claim is withdrawn at the measurement, not at the sentence.** Rewording around
  it is the failure mode. The 7–13 Hz harness data is unusable **as data** — a softer sentence
  built on it is the same false claim in a better mood.
- **Category on every published asset** — Footage · Sim Replay · Hardware Replay · Concept, plus
  **Playable Sim**, a fifth category proposed 08-01 for live/interactive artefacts and **not yet
  landed** ([#163](https://github.com/MikePaNtZ/overboard/issues/163)). **Until it lands the game
  footage has no honest category and must not be published.**
  - A Replay names its source; there is no bare "Replay". **A Concept carries no engineering numbers.**
  - **Playable Sim may state facts about the machinery** — engine, control law, loop rate — and
    **never a behavioural result** (settling time, tip angle, "it recovers from X"), because the run
    is not reproducible and the model is deliberately diverged for playability. It also carries a
    **non-physical channel declaration**, which no other category needs: it is the only category
    where some of what the viewer sees is physics and some is not.
- **Convention review on published assets is mine, not the COO's.** Settled with the COO on the
  08-01 board against the CEO's named default. The generalising split: **what an asset claims or is
  labelled is mine** (I own the vocabulary that defines it); **how work flows** — branches,
  worktrees, PRs — **is the COO's**.
- **I own what the dashboards display; the COO owns the auth design**
  ([#160](https://github.com/MikePaNtZ/overboard/issues/160)). One answer between us, settled 08-01.
  **Do not re-open it as two.**
- **Reusable channel → start now. One-shot channel → save it for real hardware.** L1 should spend at
  most ONE rented channel; posting everywhere on one day turns four shots into one and tells us
  nothing.
- **We write for a persona, we never name one.** No "education", "students", "STEM" or non-profit
  framing on the public surface — M0's one-way door.
- **US English.** "tire", not "tyre".
- **A number on a page is a maintenance obligation.** If a count cannot be kept current, write
  something that stays true instead.
- The **Voice & Style Guide** is mine, so the standard is not owned by the people measured against
  it. Content Designer writes; the SDM reviews adversarially. A reviewer who finds the guide wrong
  files against it rather than diverging quietly in the copy.
- Peer of the COO. Neither escalates to the other; both go to the CEO. A cross-line dispute reaches
  the CEO as ONE joint write-up with both positions in each other's terms.

## Decisions made

- **Every deadline on this desk is now anchored to an EVENT, not a date** (2026-08-02). Named
  as a decision because it changes how this file is written, not just what it says. A
  day-of-the-week deadline in a role context file survives the thing it was written for and
  then issues orders — which is exactly the trap ADR-0011 caught here, and
  [#203](https://github.com/MikePaNtZ/overboard/issues/203) generalises. The events I use:
  *bar clears* · *build nominated* · *claims signed* · *first announcement*.
- **The claims doc is revised now and routed to the CEO later** (2026-08-02). Its content
  survives the hold almost intact — it already forbids "stable", "rock solid" and "balances
  confidently" — so it is not wasted work. But it is titled a **gate** with a Sunday deadline,
  and a gate arriving with a deadline implies a date behind it. **Trigger re-based to: the exit
  bar is met AND a specific build is nominated.** Sending it early would buy a day of CEO
  reading time at the cost of implying a schedule, and its premise (the pre-hold build) has
  moved anyway.
- **The subscribe-form deadline moves from a date to the announcement** (2026-08-02). The old
  rule was *"not green by Saturday night → I cut the form."* Saturday night was a proxy for
  *before strangers arrive*; the launch was the thing bringing strangers, and it is held. The
  harm the rule exists to prevent — a stranger typing an address into a control that discards
  it — cannot occur while nothing is announced. **New trigger: green or cut before the first
  announcement, whichever comes first. Still mine, still no need to ask.** Recorded so it reads
  as a call rather than a deadline I let slide.
- **Portfolio direction is unparked** (2026-08-02). It was parked on *"Tue/Wed after launch"* —
  a date derived from a launch that no longer exists, which would have parked it indefinitely.
  It was parked for capacity, and the hold returned the capacity. It queues behind the two
  hard-dated items rather than behind a launch.
- **Build log: gone for launch, not for good.** The CEO ruled on the removal; only "for good?" was
  open and that half is mine. Entries keep being written into Notion so the planned content is not
  lost, and the section returns with conditional approval. Disposes of
  [`overboard-web#43`](https://github.com/MikePaNtZ/overboard-web/pull/43).
- **`r/ControlTheory`, from the CEO's own account, framed as a question.** Unchanged by the artefact
  change, and the reasoning got *stronger* — this audience respects the honest caveats rather than
  punishing them. **One-shot channels stay held for L2**: `r/onewheel`, `r/electricskateboarding`,
  Show HN, Hackaday main blog. **A game is not L2.**
- **The launch promise is L3 on the existing ladder**, not new language. L2 is riderless on real
  hardware, then the ballasted-dummy bench, then a person. Naming a rung instead of a date is what
  stops it reading as a schedule, and skipping one would be visibly breaking a published promise.
- **No ticket for the post-capture plan**, deliberately. Re-derived against the new artefact it
  shrank to one line in the W4 window with a different split — Game Engineer captures, DCP labels,
  I clear the claims. A ticket would be ceremony. Recorded so it is a call, not an omission.
- **Countersigned the COO's policy gate** (2026-07-30) with one change: hard, but **narrow** — it
  fires only on a capability claim or a numeric figure, not on every PR touching public paths. An
  advisory gate on a public surface is ignored within a week; a gate people must routinely bypass
  teaches them gates are decorative. Delivered that way in web PR #41.
- **Synthetic data is separated by `site` key, never by a flag.** A flag can be dropped by a bug,
  leaving rows indistinguishable from real ones forever.
- **`site` is required on every event and never defaulted.** The one irreversible decision in the
  analytics schema — unlabelled rows cannot be labelled later.
- **Content Designer writes; the SDM reviews.** The CEO's sketch had one role doing both, which
  dissolves the gate. The split is the mechanism.

## Known dead ends

- 🔴 **A day-of-the-week deadline in this file is a live trap.** *"LAUNCH IS MONDAY"* sat at the
  top of this file on `master` while ADR-0011 held the launch, so **any** CMO restart booted
  into orders for a launch that was not happening — armed, not merely uninformed. The COO had
  to bolt a banner on someone else's turf to defuse it. Nothing in the org notices this class of
  contradiction ([#203](https://github.com/MikePaNtZ/overboard/issues/203)). **Anchor every
  trigger in here to an event.** An event-anchored line that goes stale is inert; a
  date-anchored one gives instructions.
- 🔴 **Never send a Notion `update_content` edit whose `new_str` opens a `<table>` that the `old_str`
  did not close.** I did this to the 08-01 board at ~21:35 and it swallowed every section after
  mine — the COO's and Archivist's prose were destroyed and three metric tables merged into one.
  **Notion absorbs the following blocks silently; there is no error.** Always re-fetch and verify
  structure after any board write, and keep the pre-edit fetch until you have. I recovered only
  because I still had one, which is luck rather than process.
  - Prefer many small anchored edits over one full-page `replace_content` — but note the *repair*
    needed `replace_content`, because anchored edits cannot reconstruct blocks already gone.
  - A full-page `replace_content` **drops existing comment anchors**; replies to a pre-edit
    discussion 404 afterwards. Post a fresh comment instead.
- **A Notion comment is a poll, not a push.** Eighteen CEO answers sat unread on 07-31. If something
  must reach a role, put it on the issue or the PR.
- **`hill.py` cannot be filmed.** It models grade by rotating gravity on flat ground, so a render
  shows a board accelerating on level ground for no visible reason. `terrain.py` exists precisely to
  fix this. Do not ask for a hill video.
- **Flipping `analytics.js` to `sink: 'endpoint'` alone breaks everything silently.** The collector
  requires `site`; without it every event 400s, and `sendBeacon` cannot read a response. Verify with
  `/health` counts, never CI.
- **Do not verify a Cloudflare Worker within ~60s of deploy.** Both versions serve during rollout; a
  check in that window produced a confident false result.
- **`gh pr review --approve` never works here.** All roles share one GitHub account. Review by
  comment; the gate holds by discipline, not by GitHub.
- **A base64 token in a URL is mangled** — `+` decodes to a space. Bit us on the dashboard; the
  collector now reads the raw query string.

## In flight / owed

**Two things are dated by the physical world and are therefore the active work. Everything
under "waiting on the hold" is genuinely waiting — it is not parked, because nobody is blocked
on me for it.**

### Active — real calendar, untouched by the hold

- 🔴 **Assembly capture runbook** — hardware arrives **Aug 15–28**, the assembly happens **once**,
  and it is still not written. The hold has no bearing on this and just handed me the time.
  **This is the top item on the desk.**
- **Funding materials — Aug 24, P0, not started.** Unblocked since 08-01 (the hardware-evidence
  question was answered: simulation content, best available). One re-base: the materials must
  not lean on the withdrawn stability claim or on anything measured through the old harness.
  **The hold is not a hole in the story** — a project that caught its own defect before shipping
  and held a launch over it reads better to an investor than one that shipped it.
- **Portfolio direction with the CEO** — unparked, queued behind the two above.

### Waiting on the hold — no dates, event triggers only

- **Mine, when the bar clears AND a build is nominated —** revise
  [🔒 Launch claims](https://app.notion.com/p/3af472a5fb6981cfb496e2444aff610e) and route it to
  the CEO. Revision needed before it goes anywhere: strip the Sunday/Monday framing, add the
  **full-stick inversion** to the say / do-not-say table as its own row, and replace the
  "what has to be true by Sunday midday" list with the ADR-0011 exit criteria. It still reads
  DRAFT and the CEO has never seen it — **which is the correct state, not a slip.**
- **Mine, when a build is nominated —** review the build and rewrite the `[CONFIRM SUNDAY]`
  lines in [🚀 Monday launch content](https://app.notion.com/p/3af472a5fb6981f59d3dd2cfe5320ce8)
  against what exists. The doc needs re-titling off "Monday" and the `[CONFIRM SUNDAY]` markers
  re-anchoring to *the nominated build*; §7 (Follow along) picks up the re-based form trigger.
- **Mine, before the first announcement —**
  [`overboard-web#53`](https://github.com/MikePaNtZ/overboard-web/issues/53), the subscribe form.
  Still a decoy: `signupEndpoint` is `''` (`analytics.js:27`), the handler reads it, finds
  nothing and shows *"The list opens with the first build log"* while discarding the address
  (`index.html:1214`). **Green or cut before anything is announced.** I do not need to ask.
- **Archivist — [#163](https://github.com/MikePaNtZ/overboard/issues/163)**, `Playable Sim` into
  the canonical vocabulary, `docs/vocabulary/`, the sweep, **and the Digital Content Production
  session-start prompt**. Re-based from 🔴 to a **prerequisite with slack**: ADR-0011 supersedes
  the existing capture and holds the re-shoot, so no footage exists to mislabel. Trigger is
  unchanged and unmissable — **before any footage exists**, which is now before the re-shoot
  rather than before Monday.
- **Senior Controls — [#161](https://github.com/MikePaNtZ/overboard/issues/161).** Unchanged in
  substance: real control law in the loop, or the claims doc is **void, not weakened**. Nothing
  ships that fakes the physics.
- **SDM owes the adversarial fit-and-finish UI review** —
  [`overboard-web#54`](https://github.com/MikePaNtZ/overboard-web/issues/54). Trigger re-based
  from "before the page reaches the CEO on Sunday" to **before the page reaches the CEO**.
- 🔴 **Purge synthetic data before L1 publishes.** Cloudflare-resident; the local `--purge` will
  not reach it — use the collector's purge job. Already event-triggered, so the hold changes
  nothing. Gated by the CEO at conditional approval; until then he wants synthetic folded into
  the board highlights.

### Closed out by the hold

- ~~**Blocked on the CEO — does a stranger PLAY the game or only WATCH it?**~~ **Unblocked by
  standing default: watch.** It was urgent only because it decided a headline verb for a page
  going out on Monday. There is no Monday, the packaging work still does not exist in any brief,
  and a build that inverts at full stick is not one to hand a stranger. **Revisit only if a
  nominated build ships with packaging.** One fewer thing waiting on the CEO.
- ~~`feat/marketing/board-0801-relaunch`~~ — ADR-0011 assigns "the branch carrying the dead
  date" to the Senior Digital Marketer, but the `feat/marketing/` prefix is **mine**. Checked
  08-02: it is a local-only branch in my worktree, never pushed, and its single commit is
  already squashed onto `master`. Nothing to strip and nothing to publish. **The SDM's exposure
  is under `feat/web/` and is a separate check.**

### Background

- web#25 alarm · web#29 publish-race regression test · Experiments database.
- **No ticket anywhere for ADR-0011 condition 3** — surfacing the loss-of-authority warning to
  the player. #205 landed the signal (2.868 s of lead over `FALLEN`); the ADR says it "still
  needs to actually surface". Filed 08-02 as
  [`overboard-game#19`](https://github.com/MikePaNtZ/overboard-game/issues/19) so the last
  hold-clearing condition is observable rather than remembered. **The ask, not the design** —
  how it surfaces is the Game Engineer's.
