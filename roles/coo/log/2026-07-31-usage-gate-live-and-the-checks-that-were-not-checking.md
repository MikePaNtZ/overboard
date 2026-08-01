# 2026-07-31 — the usage gate went live, and most of my own checks turned out not to be checking

## Headline

**29 PRs merged.** `SR-SIM-5` satisfied — Rust now steps MuJoCo through the `hal` seam. The usage
gate is calibrated and **verified to refuse**, closing #38. And roughly half my own tooling was
found to be enforcing nothing, or enforcing the wrong thing on the wrong people.

## What shipped

| Thread | |
|---|---|
| **SR-SIM-5** | I1a→I1b→I1c: Rust calls `mj_step` via a C shim, proves bit-identical open-loop equivalence with Python, then hosts the real plant through the seam |
| **Torque denomination** | #137/#140 — `kt` no longer reaches the loop gain; a wrong `kt` is now a headroom error, not a gain error |
| **Usage** | derived per-role attribution, delta calibration, persisted threshold, gate proven to refuse |
| **Governance** | dispatch reaches all four repos, role labels everywhere, SDM ratified, Content Designer registered |

## The pattern worth carrying forward

**Every gate I added landed on other roles before it landed on me.**

- `role-log` size heuristic: fired four times, **wrong four times**, blocked DCP and Mechanical.
  Downgraded to advisory (#131). Worse, I told DCP they had broken a convention *without reading
  what they wrote*. They hadn't.
- Session-prep doc: I gave it a `covers:` manifest and it red-built Mechanical's PR **within the
  hour** (#144). A dated snapshot is not implementation-tier.
- `turf` matched the literal registry prefix, so every `fix/` and `docs/` branch **skipped it
  entirely** — seven live branches, three of them mine (#110).
- `--who` reported the wrong owner for every path under `.github/` — `lstrip("./")` strips
  *characters*, not a prefix (#83).

The question I kept asking was "is this check correct?". The question that would have caught all
four is **"what does this cost the person who did nothing wrong?"**

## The other recurring shape: a green light that meant nothing

`turf` and `doc-drift` had **never once executed in CI** — depth-1 checkout, no merge-base, and a
detached HEAD that made the branch name read as the literal string `"HEAD"`. The gate printed
"all hard checks pass" on every PR for its entire existence (#83).

Fixing the wiring was one line. The fix that matters is that **a diff-based check which cannot
resolve its base is now a hard failure in CI rather than a skip.** A skip that reports green is a
lie the whole org then builds on.

Same disease, three more places: `publish-sim-artifact` failed on every master push for hours
because it is not a required check (#123); I merged two PRs while it was pending and shipped the
regression twice. And I nearly queued #119 on a diff that looked right — waiting to watch the job
it fixes actually run is what caught that it was still broken for a different reason.

## Escalations that changed the answer

Two oracle calls, both of which corrected me rather than confirming me:

1. **MuJoCo binding** — I framed a third-party crate as a licensing risk. It is not; the real
   reason to reject it is version skew. *"If you write licence in the decision record as the
   reason, the record will be wrong and someone will later reverse it on correct grounds."*
2. **Gain margin / delay budget** — I asked whether to derive or measure first. False dilemma: the
   derivation is a day's arithmetic and the expensive artifacts were the sweep and the notebook.
   It also found the thing neither issue saw — **the controller's stability depends on a `kt` it
   never sees**, because gains are denominated in amps. That became #137 and it went first.

Both times the distilled packet was what made the answer useful. Both times I was wrong in a way
I could not have seen from inside the problem.

## Numbers worth keeping

- **Delegating is 3.6× cheaper per turn** — 58.1k weighted/turn main-thread vs 16.0k dispatched.
  ~90% of weighted cost is cache read + creation; **cost is carrying context, not generating text.**
- Attribution coverage rose 0% → 65% across the week as branch/worktree discipline took hold.
- Weekly meter ~44%, threshold 80%, gate verified to refuse at exit 1.

## Dead ends

- **`git reset --hard` discards uncommitted work in the same worktree** — lost a fix mid-test and
  misread the result as a code failure.
- **The Overboard guard denies the *whole* bash call.** A blocked `cp` chained after a real patch
  meant the patch never ran, and I read a later syntax check as confirmation it had.
- **Shell mangling in `gh` PR bodies.** Backticks around `kt`, `0.2/p`, `p99.9` get evaluated.
  Write bodies to a file and use `--body-file`; four PR bodies needed repair.
- **`cd` does not persist between tool calls.** Cost me four separate incidents, including patches
  applied to the primary worktree and an empty branch pushed to origin. Use `git -C` and absolute
  paths, always.
