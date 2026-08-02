# Restart briefs — the launch hold (ADR-0011), 2026-08-02

**For: the CEO.** Restart is the only broadcast primitive this org has (ADR-0001), so a
decision only reaches a role when someone starts that role's session and hands it the news.
ADR-0011 names five roles that must be told; a dead-date audit found a **sixth the ADR
missed**. This file is the thing to paste.

**Order matters.** CMO and Senior Digital Marketer first, in that order. The CMO's own
`CONTEXT.md` is merged on `master` still saying the launch ships Monday, so that role boots
into stale orders on *any* restart — it is armed rather than merely uninformed. The SDM's
branch carries the dead date, and publishing is a one-way door. Everything else can wait an
hour; those two cannot.

Each brief is self-contained on purpose. A restarted session has no memory of the day, and
three of the day's findings are the **opposite** of what the older notes in this repo say. A
brief that assumes context is a brief that gets re-derived wrongly.

---

## The three findings every brief must carry

Repeated in each brief below rather than linked, deliberately. They were each believed the
other way round for part of 2026-08-02, and the stale version is still readable in issue #190
and in older scenario-doc comments.

1. **The −10° steady-state pitch is REAL PHYSICS, not estimator error.** 5.84° of lean per
   m/s², fixed by geometry, against an 11.46° authority ceiling (`MAX_CURRENT_A·KT/KP`). Full
   stick spends 10.4° of it. The bias does not sit *near* the cliff — it **is** the cliff.
2. **"Fixing" the estimator makes the board WORSE.** The ~1° nose-down error acts as nose-up
   trim. Feed the controller MuJoCo truth and it flips **1.74 s earlier**. Issue #190 named
   the estimator as the suspect and pointed **exactly the wrong way**.
3. **The survivability boundary is the SPEED CAP, not the lean.** Saturation is survivable if
   and only if something is already unloading the board when it occurs.

And the one-line version of the defect itself: **holding full forward stick from rest inverts
the board in ~6.5 s**, on the straight, at `steer = 0`, reachable in the playable build.

---

## 0. CMO — ADR-0011 MISSED THIS ONE, and it is a live trap

**ADR-0011's "who moves" list does not name the CMO.** That is a gap in the ADR, found by a
dead-date audit rather than by reading it, and it matters more than the omission suggests:
`roles/cmo/CONTEXT.md` is **merged on `master`** (PR #164, 2026-08-01 — the day *before*
ADR-0011 was ratified) and its first sub-goal reads

> **LAUNCH IS MONDAY 2026-08-03 MORNING**, readiness review Sunday night. Everything else is
> subordinate until it ships.

That file is the first thing a CMO session reads under the session-start protocol. So **any**
CMO restart, for any reason at all, currently boots into stale launch orders and a
Sunday-evening claims-gate deadline — for a launch that does not exist. It is not waiting to
be told; it is armed.

ADR-0011 says its own enforcement is `policy` for the public-claim half, and that *"the date
itself is convention only and therefore relies on this file being read."* This is precisely
the case where that reliance fails: the role reads its context file, and the context file is
wrong.

A superseding banner has been added to the top of that file under `TURF-OVERRIDE` — defusing
it, not rewriting CMO's priorities, which remain CMO's to set. The brief:

> You are the CMO. Read `CLAUDE.md`, then `docs/decisions/INDEX.md`, then
> `roles/cmo/CONTEXT.md` — **noting the hold banner at the top of it.**
>
> **The 2026-08-03 launch is held.** The decision is
> `docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`. Holding full
> forward stick from rest inverts the board in ~6.5 s, on the straight, at `steer = 0`, and it
> is reachable in the playable build. The CEO hit it on first contact and held the launch.
>
> **Your `CONTEXT.md` sub-goals are stale and I could not fix them for you** — that file is
> your turf and your priorities are yours to set. The COO added a banner marking the
> supersession and nothing else. Everything dated to Monday or to Sunday night in that file
> needs re-basing by you: the claims gate "due Sunday evening", "Monday's footage", the
> "Monday launch content" doc, and the post-launch portfolio work.
>
> **There is no new date.** The hold is gated on a control fix, not a calendar, and it is
> open-ended. Do not set one, do not let one be inferred, and do not run a readiness review.
>
> **The stability claim is withdrawn**, not softened: "the board never became unstable at any
> aggression level tested" is false. The measurements behind it were taken through a harness
> silently delivering stick input at 7–13 Hz against a 100 ms staleness cutoff, so the board
> was being commanded at roughly 0.62 of the lean the tests believed. They support nothing and
> may not be cited in any form.
>
> Your Senior Digital Marketer is being restarted in parallel and owns the `overboard-web`
> page status under the `SR-WEB-4` lock-step. Coordinate rather than duplicating — the SDM
> brief is section 1 of this file.
>
> Report back: whether anything went out, and your re-based sub-goals.

**Good news from the same audit:** the live public `overboard-web` site is **clean** — no
2026-08-03, no Monday launch date, no stability claim. Nothing wrong is currently published.

## 1. Senior Digital Marketer — URGENT, do this one first

> You are the Senior Digital Marketer. Read `CLAUDE.md`, then `docs/decisions/INDEX.md`, then
> `roles/senior-digital-marketer/CONTEXT.md`.
>
> **The 2026-08-03 launch is held. Do not publish anything.** The decision is
> `docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`; read it before
> you touch a file.
>
> Your branch `feat/marketing/board-0801-relaunch` carries the dead date. Strip or re-date it.
> Do not publish it, and do not schedule it. There is no new date — the hold is open-ended
> because it is gated on a control fix, not on a calendar.
>
> Two things are wrong in the copy, not one:
> - **The date.** 2026-08-03 is gone.
> - **The claim.** "The board never became unstable at any aggression level tested" is
>   **false and formally withdrawn.** It must not be reinstated in any softened form. The
>   measurements it rested on were taken through a harness that was silently delivering stick
>   input at 7–13 Hz instead of 50 Hz, so the board was being commanded at ~0.62 of the lean
>   the tests thought they were applying. Those measurements support nothing and may not be
>   cited. The `policy` CI gate (ADR-0003) has the withdrawn claim registered, so it will stop
>   you — but it gates this repo, and your surface is `overboard-web`, so the gate is not your
>   safety net. You are.
>
> `overboard-web` page status moves in **lock-step** (`SR-WEB-4`) and must not announce a
> launch that is not happening. That is your call to make and your surface to fix.
>
> Report back: what you changed, what was already public, and whether anything went out.
> "Already published" is the thing I need to know first, ahead of any fix.

## 2. Digital Content Production

> You are Digital Content Production. Read `CLAUDE.md`, then `docs/decisions/INDEX.md`, then
> `roles/digital-content-production/CONTEXT.md`.
>
> **The 2026-08-03 launch is held** —
> `docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`. Holding full
> forward stick from rest inverts the board in ~6.5 s, on the straight.
>
> **The existing "Manny rips" capture is superseded. Do not publish it.** It was shot through
> the harness that was under-delivering stick input, so it does not show the board being
> driven at the lean its caption implies.
>
> **Hold the re-shoot.** Do not start it yet. It is blocked on Senior Controls choosing a
> margined lean value, which has not happened. Shooting now means shooting at a number that
> is about to change.
>
> When it is unblocked, ADR-0011 criterion (d) sets the bar: footage is re-shot through
> `sim-host --scripted-scenario` so it is **bit-identically reproducible**, and the caption
> claims **only what that run measured**. That is a stronger provenance position than anything
> the old harness could produce — the re-shoot is an upgrade, not a redo.
>
> One live risk in your area: `/tmp/overboard-media` is volatile storage and holds
> `clips.json` with the corrected caption text, plus the captures and the generated gallery.
> `ops/build-demo-gallery.py` is in-repo but the manifest is not. The COO has an agent
> archiving it now; check with the COO before assuming it is safe, and do not rely on `/tmp`
> for anything again.
>
> Report back: confirmation nothing is published, and what you need from Controls to start
> the re-shoot the moment the lean lands.

## 3. Game Engineer

> You are the Game Engineer (`overboard-game`, the Unreal client — ADR-0009). Read
> `CLAUDE.md`, then `docs/decisions/INDEX.md`.
>
> **The 2026-08-03 launch is held** —
> `docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`.
>
> **The playable build inherits the defect. No public build until ADR-0011 criterion (a) is
> met.** Holding full forward stick from rest inverts the board in ~6.5 s. It is on the
> straight with `steer = 0` — not an aggressive-manoeuvre edge case, but the first thing a new
> player does in their first ten seconds. The CEO hit it on first contact.
>
> **This is not yours to fix and you must not fix it.** The board physics live in `overboard`
> and nothing outside that repo computes them — you replay a live state stream and write
> setpoints back, and that boundary is load-bearing. Senior Controls owns the fix; the COO has
> an agent implementing it now. Your part is: do not ship, and be ready to re-verify against
> the fixed host when it lands.
>
> What is coming at you, so you can plan: commanded stick will be **scaled down at the input**
> (a derived envelope reserve, expected around 0.80 of full stick). Top speed is unchanged —
> the speed cap already governs it — but acceleration drops: ~0.93 s slower to 8 m/s and ~8%
> less distance over 15 s. If that changes how the game feels, that is your judgment to raise,
> and raise it early rather than after the fix merges.
>
> Also relevant to you and not yet done: **reset is still a no-op** (the host logs `input
> reset bit set -- not implemented yet, ignoring`), so a fallen board has no recovery path
> short of relaunching the stack. That is ADR-0011 criterion (e) and it is in flight.
>
> Report back: confirmation no public build goes out, and whether the acceleration cost above
> is acceptable for the client.

## 4. Archivist

> You are the Archivist. Read `CLAUDE.md`, then `docs/decisions/INDEX.md`, then
> `roles/archivist/CONTEXT.md`.
>
> **The 2026-08-03 launch is held** —
> `docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`. Two sweeps for
> you, both vocabulary/provenance work rather than copy edits.
>
> **Sweep 1 — the withdrawn claim.** "The board never became unstable at any aggression level
> tested" is **false and withdrawn**. It must not be reinstated in any softened form, and the
> measurements behind it may not be cited — they were taken through a harness delivering stick
> input at 7–13 Hz against a 100 ms staleness cutoff, so the stick was silently zeroed for
> part of every run. Sweep all four repos for it and for softened restatements. The `policy`
> gate has it registered for this repo; the other three repos have no such gate.
>
> **Sweep 2 — the new provenance wording.** ADR-0011 criterion (d) makes reproducibility the
> provenance basis for footage: shot through `--scripted-scenario`, bit-identically
> reproducible, captioned to only what that run measured. That is new vocabulary and it needs
> to be consistent before any footage carries it.
>
> **Loose end you are already named in:** the provenance line for the yaw change (#193) was
> adjudicated but coordination with you was never confirmed done. The agreed line is:
> *"Steering is commanded, not emergent… nothing is dead-reckoned."* Confirm it landed, or
> land it.
>
> Report back: what you found in each sweep, and confirmation on the #193 line.

## 5. Senior Controls — ⚠️ READ BEFORE RESTARTING

**Do not restart this role without talking to the COO first.** ADR-0011 makes Senior Controls
the owner of the fix and everything else queues behind it, so the instinct is to restart it
immediately. But the COO dispatched an agent on 2026-08-02 that is **implementing criteria (b)
and (c) right now** on branch `feat/controls/cmd-envelope-reserve` — the derived command
envelope reserve and the loss-of-authority warning, both in the same host file. A second
session on the same work is a merge conflict at best and duplicated measurement at worst.

Restart it once that PR is up. Everything below is filed as an issue with acceptance criteria,
so this brief is a reading order rather than the work itself:

- **[#204] Braking far too weak and coasting far too free** (CEO feedback from driving). Real
  onewheels brake hard through regen and have real rolling resistance; neither exists here —
  two missing terms, not a tuning gap. **Sequence this last.** It is the only one that changes
  what the board *does* rather than what it is *allowed* to do, and it carries a real tail:
  braking loads the board the same way gravity does during a forward stop, which is exactly
  the reverse-to-forward reversal case ADR-0011 names as the worst and untested. Stronger
  braking may create a *new* way to invert the board, in the regime the launch is held over.
- **Turn radius and reset** — the branch has now been assessed. Verdict below; it changes what
  this brief asks for, so read it rather than the older "unvalidated, verify first" note.
- **`damping="0.08"`** on `wheel_hinge` is the only load-bearing MJCF constant with no
  provenance comment, and it blocks propagating any speed-dependent number to the hardware
  spec. It belongs in the imperfection-profile conformance contract (fabe806).

### `feat/controls/turn-radius-and-reset` — verdict: NEEDS-REWORK, do not merge

Assessed 2026-08-02 against a build, a test run and a real end-to-end measurement, not by
reading the diff. It delivers **one of its two headline features and zero of the other**, and
merging it as-is would falsely close out an ADR-0011 exit criterion. Split into two issues so
the good half is not held hostage by the missing one:

- **[#202] Reset works — verified by measurement — but has zero tests.** Kick the board over,
  reset, and pitch snaps from ~3.12 rad to ~0, `fallen` clears, position returns to (0,0), and
  it stays stable through 5+ further seconds including a turn. The stub log line is genuinely
  gone. **The blocker is a comment, not the code:** `host.rs` ~line 847 claims *"asserted by
  measurement, not assumed — see this file's own reset measurements"* and **no such
  measurements exist in that diff**. That is this org's signature failure — closing on a
  plausible mechanism rather than a measurement — pre-loaded into the code where the next
  reader cannot tell. The comment becomes true or it goes; there is no softened third option.
- **[#201] Turn radius was never implemented.** `YAW_CURVATURE_PER_STEER_RAD_PER_M` is still
  `0.15`, byte-identical to `master`. What the branch actually added is *measurement
  scaffolding* — a `turnaround` scenario — and it is worth keeping. Using it: current radius
  is **6.95 m** by circle-fit over 5251 points, residual std 4.7 cm. Target ~3.5 m.

`cargo build`, `clippy --all-targets` and `cargo test --workspace` are clean on the branch
(67/67 pre-existing tests, zero warnings) — but **it adds no tests of its own**, so green CI
says nothing about either feature.

**Conflict warning, and it is the sharpest one in the repo right now:** the reset-bit block
sits inside `run()` in `crates/sim-host/src/host.rs` at ~lines 773–873, directly adjacent to
the staleness gating and the stick-tuple computation — the exact region the in-flight
`feat/controls/cmd-envelope-reserve` work is editing. Braking/coasting lands there too. All
three must be sequenced through that file, never run in parallel.

---

## What is NOT affected

Mechanical and the Pi-image work are untouched by the hold. Do not restart them for this.

## How to tell this worked

The failure this org keeps repeating is two roles waiting on each other over work that is
already finished — three times in two days. So the test is not "were they told", it is
**did each role write a teardown that the next session and its peers can read**. If a brief
above produces no log entry under `roles/<role>/log/`, the restart did not land and the same
news will need broadcasting again.
