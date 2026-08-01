# 2026-08-01 — W0: the CEO's formal approval, and the launch weekend dispatched

**Window:** W0 (Sat evening), per [M3 Implementation Plan § Revision 3].
**Exit criterion:** *both sessions running against a written brief.* Met — three, not two.

## The approval

The CEO gave **formal approval to execute the weekend plan**, in full, on 2026-08-01. Revision 3
had carried the design-gate waiver as *"conditionally approved by the CEO, formal approval pending
his read of § What the CEO must read."* That condition is now discharged.

What the approval actually licenses, stated plainly so no session over-reads it:

- **D6–D9 do not gate W1–W4.** This waiver is for **this weekend only** and expires Monday. It is
  not a precedent, and the design-gate rule in `CLAUDE.md` is untouched from Tuesday.
- **The fallback ladder is live and pre-decided**, so nobody renegotiates it at 2 a.m.:
  visuals slip → ship it ugly; roll unplayable → widen the geom further and lean on the yaw
  limiter; **host cannot be wired → do NOT fake the physics.** Fall back to existing sim content
  and hold the game. Both engineering briefs carry that ladder verbatim.
- The approval does **not** touch public claims. Those are the CEO's own reserved turf and land
  Sunday evening (W4).

## What I did before dispatching, and what it changed

**Went and looked rather than asking.** Three facts decided whether the weekend was real:

1. **The spine already exists.** `crates/sim-backend/src/lib.rs` already couples `plant-mujoco` and
   `control-ffi` in a closed loop at 500 Hz (`CYCLE_NS`, ICD §11.2), and
   `tests/test_rust_hosted_impulse_response.py` proves the Rust-hosted loop runs today. W1 is a
   real-time thread and a socket around a working system — **not new construction.** The brief says
   so explicitly, because an agent that believes it must build a simulator will build one.
2. **The toolchain is present.** UE 5.7 installed; `cargo` 1.97.1 — but **not on `PATH` in
   non-interactive shells**, which is a real trap and is stated in the brief as `~/.cargo/bin/cargo`.
3. **`overboard-game` was not cloned locally.** The repo existed on GitHub and nowhere on this
   machine, so the Game Engineer seat had no working directory. Cloned to `~/projects/overboard-game`.
   This was a hard W0 blocker and it was invisible from the issue tracker.

## The one decision I made rather than delegated

**Wire v1 is fixed by me** — [ADR-0010](../../../docs/decisions/ADR-0010-game-wire-v1-fixed-for-the-launch-weekend.md).

Both briefs instruct the two roles to *"talk continuously — this is not a handoff."* They cannot:
sessions in this org cannot message each other. Serialising costs the Game Engineer the whole
night; parallelising without a fixed contract produces a float misparse rather than a compile
error, and the plan itself names the frame transform as one of three things that *"silently poison
everything downstream."*

So both sides got the identical schema verbatim, plus the frame-ownership split: **the host emits
raw MuJoCo, the renderer converts.** ADR-0009 gives the wire to Senior Controls, so this is a
deviation — recorded, time-boxed, and reverting Tuesday. Recording it is the point; a seam quietly
re-owned is the drift the decision record exists to catch.

## Dispatched — three, which is the ADR-0007 cap

Gate run first: `ops/dispatch.sh --audit` green (22 open issues, 4 repos, exactly one `role:` label
each). All three target worktrees clean.

| Role | Issue | Branch | Brief |
|---|---|---|---|
| Senior Controls | [#161](https://github.com/MikePaNtZ/overboard/issues/161) | `feat/controls/sim-host-spine` | `sim-host`: 500 Hz dedicated thread, real control law, UDP both ways |
| Game Engineer | [#162](https://github.com/MikePaNtZ/overboard/issues/162) | `feat/game/pose-spine` | UE5 client, board actor, socket, frame transform |
| Archivist | [#163](https://github.com/MikePaNtZ/overboard/issues/163) | `feat/archive/playable-sim-category` | `Playable Sim` provenance category — launch blocker |

Two things I put in the briefs that the issues did not ask for, both to make the milestone
**checkable rather than asserted**:

- **A `wire-probe` binary** on the controls side — a headless receiver reporting measured tick
  rate, inter-packet p50/p99/max, missed-deadline count and pitch bounds. The CEO can read whether
  the board is being held up by the real controller **without Unreal building at all**, which
  decouples the milestone from the slowest and least certain toolchain in the weekend.
- **Prove the wire before generating the UE project** on the game side. Wire parsing and the
  handedness flip compile and run without the editor; the editor build is the risky part. Doing
  them in that order means a UE toolchain failure costs the schema work nothing.

## The second W0 blocker, found the hard way — and the control that worked

The Game Engineer came back inside three minutes, having written **nothing**. The write-guard
hook `~/.claude/ops/overboard-guard.py` keeps a hand-vetted `VETTED_REPOS` allowlist, and
`overboard-game` was not on it. Every `Write`/`Edit` and every Bash redirect into the repo was
denied. `git` was not guarded, which is why the branch checkout had succeeded and the block only
appeared at the first attempt to author file contents.

**The session declined to add itself to the allowlist**, citing the guard's own comment that the
`overboard-metrics` entry was *"vetted by the COO — deliberately NOT by the session that wanted
the access."* That is the correct call, and worth recording for a reason beyond politeness: the
convention was written down once, in a comment, after the fact — and it **held on first contact
with a role that had never seen it.** That is a control that actually works, unlike the two checks
this org shipped last week that reported green while enforcing nothing.

So I vetted it, to the evidence standard that entry set — checked rather than assumed:
repo is PUBLIC per ADR-0009 intent; two tracked files (`README.md`, `.github/CODEOWNERS`);
**no workflows at all**, so nothing runs on checkout or push; no exec bits; zero matches for
token/secret/key/password/PRIVATE-KEY patterns. Added it, then **verified the guard still parses
and now admits the path** rather than assuming the edit took.

Recorded a caveat with it, because this one will not stay true: the repo is **about to become an
Unreal project**, and UE drops generated build scripts and third-party binaries that none of the
above was checked against. The entry vets the repo *as it is today* and says so.

Two follow-ups this exposes, neither launch-critical:

- **`ops/dispatch.sh` cannot see this failure mode.** Its gate checks routing, cleanliness and the
  concurrency cap — not whether the target repo is writable by the session about to be dispatched
  into it. A dispatch that cannot write is indistinguishable from one that has not started yet.
  Worth a pre-flight write probe.
- **The Game Engineer's third ratification leg now exists.** `overboard-game/.github/CODEOWNERS`
  is present and correctly `# role:`-tagged, which is the exact thing ROLES.md names as the reason
  the seat is `Provisional`. Registry entry ✅, CODEOWNERS ✅, Notion select option unverified.
  Not chased tonight — `Provisional` restricts none of this weekend's work — but the seat is one
  checked box from `Ratified` and I own that paperwork.

## Housekeeping done in passing

My own worktree was sitting on `fix/ops/adr-0009-boundary-claim`, whose PR
[#158](https://github.com/MikePaNtZ/overboard/pull/158) had **already merged**. Master had since
moved on with [#164](https://github.com/MikePaNtZ/overboard/pull/164), so the stale branch's diff
against master was a **289-line revert of the CMO's launch re-cut**. Opening a PR from it would
have destroyed another role's work tonight, silently, while everyone was busy.

Reset to `origin/master`. Filing under the standing dead end *"do not trust worktree state"* — the
new edge is that **a merged branch is more dangerous than an unmerged one**, because it looks
finished and its diff has quietly inverted into a revert. Worth a check in `dispatch.sh`: refuse to
dispatch from a branch whose PR is already merged.

Also noted and **not** acted on: the primary `~/projects/overboard` worktree is stale
(`primary-current` @ 811e672, several PRs behind). It misled me once tonight — `ADR-0009` looked
missing from `INDEX.md` and was not. Not touched, per the standing rule against working there.

## Open, going into W1

- The measured host numbers do not exist yet. **No claim about 500 Hz is safe until `wire-probe`
  prints one**, and the `Playable Sim` rules make loop rate a machinery fact we are permitted to
  state — so it will be stated, and it must therefore be measured rather than inherited from
  `CYCLE_NS`.
- Whether UE 5.7 builds headlessly on this machine is unknown. The briefs are sequenced so the
  answer is cheap either way.
- W4's claims work depends on #163 landing **before footage exists**, not before launch.

[M3 Implementation Plan § Revision 3]: https://app.notion.com/p/3af472a5fb6981f5b6e4ec038293ad6f
