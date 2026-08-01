# CMO — working context

- **Worktree:** `~/projects/overboard-cmo` (ADR-0006). Marketing work happens in
  `overboard-web`, `overboard-viz` and `overboard-metrics`.
- **Read first:** [`docs/decisions/INDEX.md`](../../docs/decisions/INDEX.md) · [`ROLES.md`](../../docs/decisions/ROLES.md)
- **Branch prefix:** `feat/marketing/`
- In this repo I write **only** `roles/cmo/**` (ratified, CODEOWNERS:180). Everything else here is
  read-only to me.

## Current sub-goals

- **LAUNCH IS MONDAY 2026-08-03 MORNING**, readiness review Sunday night. Everything else is
  subordinate until it ships. Both of the CEO decisions that blocked this desk for three boards
  were answered on 08-01; nothing is waiting on him except the two rows below.
- 🔄 **The launch artefact is a playable Unreal game, not a page of simulation results.** It
  changed late on 08-01, after I had already written a board section against the old one.
  **If a marketing doc still says "simulation results", it predates the change and is wrong.**
  Plan: [M3 Implementation Plan § Revision 3](https://app.notion.com/p/3af472a5fb6981f5b6e4ec038293ad6f).
- Answer the two big unknowns with data: do strangers care, and is *how we build it* more
  interesting than the onewheel. **We can now measure the first one.**
- **Produced vs. seen by a stranger is still the whole game.** Six produced, five published on a
  live page, **zero announced anywhere**. Monday is the first time that last number can move.

## What is live (re-verified 2026-08-01 ~21:00)

- `overboardproject.com`, HTTPS enforced, HTTP 200 in 0.19 s. Apex is DNS-only/grey-cloud so GitHub
  Pages holds its certificate; only `e.` is Cloudflare-proxied. **Do not flip the apex to orange —
  it takes the site's TLS down.**
- Collector at `e.overboardproject.com/e` → D1. `/health` reports events, last event, reject count
  and last reject reason, and is **unauthenticated** — it is the one number provable without a token.
  1,973 events, last at 13:18 UTC; 4 rejected all time, all "missing required field `site`".
- Dashboard at `/dashboard?k=…`, token-gated, fails closed, **404s on a bad token** (deliberate — a
  403 would confirm the route exists). Real vs. synthetic is `&site=synthetic-overboard`.
- Daily jobs: synthetic traffic, and a GitHub traffic snapshot into `overboard-metrics` (private).

## Rules this role owns

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

- 🔴 **Blocked on the CEO — does a stranger PLAY the game Monday, or only WATCH it?** Neither build
  brief has packaging or QA in it, so my default is *watch*. **It decides the headline verb**, so
  the page copy cannot be finished without it.
- 🔴 **Blocked on the CEO — the claims gate.** He owns public claims personally and has not seen the
  words. Due in front of him **Sunday evening, before the readiness review**, not during it.
  [🔒 Launch claims](https://app.notion.com/p/3af472a5fb6981cfb496e2444aff610e).
- 🔴 **Blocked on the Archivist — [#163](https://github.com/MikePaNtZ/overboard/issues/163)**,
  `Playable Sim` into the canonical vocabulary, `docs/vocabulary/`, the sweep, **and the Digital
  Content Production session-start prompt** — that prompt enumerates the four categories and is what
  DCP actually reads when labelling Monday's footage. Needed **before footage exists**.
- 🔴 **Blocked on Senior Controls — [#161](https://github.com/MikePaNtZ/overboard/issues/161).** If
  the real control law is not in the loop, the claims doc is **void, not weakened**. I am not
  writing a second set of copy for the fake-physics case; there is no version of it I would sign.
- **Mine, Saturday night —** [`overboard-web#53`](https://github.com/MikePaNtZ/overboard-web/issues/53),
  the subscribe form. `signupEndpoint` is `''` (`analytics.js:27`) and the handler discards the
  address (`index.html:1216`). **Not green by Saturday night → I cut the form** rather than ship a
  decoy. I do not need to ask.
- **Mine, Sunday midday —** review the actual playable build and rewrite everything marked
  CONFIRM SUNDAY in [🚀 Monday launch content](https://app.notion.com/p/3af472a5fb6981f59d3dd2cfe5320ce8)
  against what exists rather than what was promised.
- **SDM owes the adversarial fit-and-finish UI review** before the page reaches the CEO —
  [`overboard-web#54`](https://github.com/MikePaNtZ/overboard-web/issues/54). A CEO-named gate that
  had no ticket until 08-01.
- 🔴 **Purge synthetic data before L1 publishes.** It lives in Cloudflare; the local `--purge` will
  not reach it. Use the collector's purge job. **Gated by the CEO** — he sets the date at conditional
  approval, and until then he wants synthetic folded into the board highlights.
- 🔴 **Assembly capture runbook** — hardware Aug 15–28, happens once, still not written.
- **Parked with a date, not a risk:** portfolio direction with the CEO, Tue/Wed after launch.
- Funding materials (Aug 24, P0, not started) — **unblocked as of 08-01**; the hardware-evidence
  decision was answered (simulation content, best available).
- web#25 alarm · web#29 publish-race regression test · Experiments database.
