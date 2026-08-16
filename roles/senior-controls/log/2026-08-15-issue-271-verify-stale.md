# 2026-08-15 — Issue #271: `EngageState::TailBraking` does not exist anywhere in this repo's history

Cron dispatch pass. #271 ("Allow steering during a tail brake") has no `Owner:` tag, but its
location (`crates/sim-host/src/host.rs`, the `EngageState::TailBraking` arm) is unambiguously
Controls turf, and no open PR referenced it. Picked it up as the most valuable unclaimed item —
until verification below showed there was nothing to build against.

## What #271 claims

That `EngageState::TailBraking` is a state active on the ridden host today, in which every rider
input (fore/aft, lateral, steer) is currently zeroed, and that a CEO decision on 2026-08-12
directed steering to be relaxed there as future work, per "ADR-0011's fourth ratification
(Decision 2) and its 2026-08-12 amendment."

## What I found instead

1. **No `EngageState` type exists anywhere on `master`.** `grep -rn "EngageState\|TailBraking"`
   across the full tracked tree returns nothing, confirming the same finding already logged
   independently yesterday in `roles/senior-controls/log/` on the `fix/controls/
   disturbance-force-world-frame-194` branch (#194 / PR #280) — that session flagged it as
   out-of-scope rather than picking it up.
2. **`git log --all -S"TailBraking"`** — a search of every commit that ever added or removed the
   string, across every branch and all history, not just current tree — returns exactly one
   commit: `2c88f7b`, which is that same #194 PR's log entry documenting the *absence* of the
   feature. The string has never appeared in code, only in that one prose note.
3. **`EngageState` itself has existed exactly once, on an orphaned branch.** `git log --all
   -S"EngageState"` surfaces `ffe0d84` (`feat/controls/engage-button`, authored by Mike directly,
   not a Claude session) and `940267a` (an unrelated terrain-spawn fix that also happens to use
   the English word "engage" in a commit message, no type involved). `ffe0d84` is **not an
   ancestor of `origin/master`**, and `list_pull_requests` with
   `head=MikePaNtZ:feat/controls/engage-button` (state=all) returns zero results — no PR, open or
   closed, has ever referenced that branch. It appears to be pushed, exploratory, unlanded work.
4. **On that branch, the only place `EngageState` is ever defined, the enum has exactly two
   variants:**
   ```rust
   enum EngageState {
       Disengaged,
       Engaged,
   }
   ```
   No third state, tail-braking or otherwise.
5. **ADR-0011 itself has no match for `brake`, `Decision 2`, or `2026-08-12`** in
   `docs/decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md`. The "fourth
   ratification" and "2026-08-12 amendment" the issue cites as authority for the CEO decision are
   not findable in the ratified document.

## Conclusion

Every anchor #271 gives — the enum, the variant, the ratification, the date — is unverifiable
against this repo's actual history, not just its current tree. Building the described match arm
would mean inventing an `EngageState::TailBraking` state from scratch and calling it "steering
during a tail brake," which is fabricating the very thing the issue claims already exists. That is
exactly the class of mistake `CLAUDE.md`'s "never fabricate a fact you could not verify" rule
exists to prevent. Closing rather than leaving open, on the same reasoning as the #240/#230/#169
closures: an open issue describing a match arm on a state that isn't there is a standing
invitation for a future session to build fiction around it.

**Could not verify, and not claiming it:** why the issue was filed this way, or whether a
two-state `engaged`/`disengaged` split (the actual, still-unmerged `feat/controls/engage-button`
work) was meant to land first and #271 jumped ahead of it. Both are plausible; neither is
confirmed.

## Also noticed, out of scope

`feat/controls/engage-button` (and its sibling `feat/controls/reconcile-physics`) is real,
apparently-working Controls-domain code — engage/disengage gating, yaw-aim while disengaged — sitting
on a pushed branch with no PR ever opened against it, authored directly by Mike rather than a
session. If the engage/disengage split is wanted, that branch is the closer starting point than a
fresh implementation, but landing it is a design/sequencing call (it changes wire-visible state
and the yaw law), not a one-issue pickup, so flagging rather than touching it here.

## Deliberately left out

- Did not implement any tail-brake behaviour — there is nothing here for it to modify.
- Did not open or otherwise act on `feat/controls/engage-button` — picking that up is a separate,
  larger decision this dispatch pass did not ask for.
