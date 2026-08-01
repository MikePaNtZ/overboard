# ADR-0009 — Give the Unreal client its own repo, and a Game Engineer seat to own it

- **Status:** Accepted
- **Date:** 2026-08-01
- **Ratified by:** COO
- **Closes:** none — the CEO granted the seat and chose the repo split in session on 2026-08-01, and
  directed ratification. Recorded here because a CEO decision that never reaches this directory
  binds nobody, and three sessions have already been lost to exactly that gap.
- **Constrains:** anyone adding Unreal, game or renderer code; the repo-boundary rule in
  `CLAUDE.md`; the role registry
- **Enforced by:** `policy` — `turf` (via the new `feat/game/` prefix in `ROLES.md`) and
  `ownership`. Partially convention-only in the new repo; see "How this is enforced".

## Context

The M3 scope introduces a real-time Unreal Engine 5 client: a player drives the ballasted
onewheel with a gamepad while MuJoCo remains the sole physics authority and runs the real Rust
control law at 500 Hz. Unreal renders and takes input; it computes no board physics.

`CLAUDE.md` says **this repo is controls only** and names one sibling, `overboard-web`. The
registry and `ROLES.md` describe an estate of three repos. Neither anticipated an interactive
client, and the M3 work has nowhere to legally live.

There was a real candidate already in the estate. `overboard-viz` was created on 2026-07-26 with a
charter whose *first half* is what M3 needs:

> The renderer never computes physics. It replays motion MuJoCo already computed. [...] One file
> crosses the boundary: a pose track. `overboard` writes it, `overboard-viz` reads it, and neither
> imports the other.

**Only the first sentence carries over, and an earlier draft of this ADR overstated the rest.**
"The renderer never computes physics" is a genuine inheritance. The mechanism is not: viz's
contract is **one file, one direction, offline**. M3's is **two-way and live** — pose out at render
rate, and player input in against a 500 Hz loop. A reverse channel into a real-time control loop
has no precedent anywhere in this estate, and calling it a reuse hides the one genuinely novel and
risky thing M3 introduces. It is a new versioned contract that happens to share a principle.

So the question was never "what should the boundary be" — the *principle* is settled and inherited
— but "does the Unreal client belong in the repo that already holds it", with the live two-way
contract itself needing its own design decision either way.

## Decision

**A fourth repo, `overboard-game`, owned by a new `Game Engineer` role.**

- The Game Engineer escalates to the **COO**, and is a **peer of Senior Controls, not subordinate
  to it**. Branch prefix `feat/game/`.
- It owns `overboard-game` entirely and **owns nothing in `overboard` by design**, the same way the
  CMO owns nothing here.
- Senior Controls owns everything on the `overboard` side of the seam — the plant variant, the
  real-time host, the wire crate, the observer.
- **The wire schema is the contract between them.** Neither repo imports the other. Unlike viz's
  pose track this contract is bidirectional and live, so it is versioned like the C ABI
  (`ob_abi_version()` and `size`-tagged structs) and a mismatch must fail loudly rather than
  misparse.
- **The 500 Hz host — MuJoCo plus the control law — belongs to `overboard`** and is published as a
  versioned artifact. `overboard-game` consumes it across the wire contract and **must not link
  MuJoCo or `control-core` directly**, so that "which control law was this session run against" is
  answerable from one version string rather than a build graph.

A change complies with this ADR if Unreal, game-asset and renderer-client code is in
`overboard-game`, control law and physics are in `overboard`, and the only thing crossing is a
versioned data contract.

## Options considered

**1. Extend `overboard-viz`.** Its charter already states the right *principle*, and its pose track
is the nearest thing in the estate to a live wire schema — though see the correction above: nearest
is not the same as ancestor, and the live reverse channel is new work either way. Rejected on two
counts, either of which would have been sufficient. *Lifecycle:* viz is offline, batch, Blender; the game is interactive,
real-time, multi-gigabyte UE assets and a game engine's build system. Sharing a repo means every
cinematic render carries the game's checkout weight and vice versa. *Turf:* `ROLES.md` gives
`overboard-viz` to **Digital Content Production**, which sits under the marketing line. The game is
an engineering instrument that Senior Controls depends on. Merging them puts an engineering
deliverable under the CMO's line and makes ownership ambiguous in the one place this org has
repeatedly paid for ambiguity.

**2. Put the Unreal client inside `overboard`.** Rejected. It voids the controls-only rule, and it
puts game assets in the repo whose CI gates the control law — the `sim` and `rust` checks would
start carrying engine build weight, and the margin gate is the last thing in this project that
should get slower or flakier.

**3. A fourth repo.** Chosen. Cost stated below.

## Consequences

**Easier.** Independent lifecycles and build systems. The controls repo stays controls only, so
the sim-in-the-loop gate cannot be slowed or destabilised by game work. The game cannot
accidentally contaminate a controls claim, which matters because M3 deliberately ships a
non-physical steering channel.

**Harder.** The estate grows to four repos, and `overboard-game` starts with **no CODEOWNERS**, so
path ownership there is unfalsifiable — this widens the open item already recorded in ADR-0002
rather than introducing a new class of problem. Cross-repo version pinning now applies to the wire
schema. One more session and worktree to keep in step, against an org that has already lost days
to sessions waiting on each other.

**Who moves.** The COO registers the seat and carries the registry → Notion mirror. Senior Controls
picks up the `overboard`-side seam work. The Game Engineer picks up the new repo.

## How this is enforced

- **In `overboard`:** the `ROLES.md` row declares `feat/game/`, which makes the `turf` check live
  for this role immediately. Because the Game Engineer owns no path here, any edit it makes in this
  repo is a trespass and CI will say so — which is the intended behaviour, not an oversight.
- **In `overboard-game`:** **convention only, stated honestly.** Until that repo has a CODEOWNERS
  the ownership leg does not exist, so the seat stays `Provisional` — the same reason the Senior
  Digital Marketer could not be ratified until `overboard-web` got one (ported in overboard-web#35).
  The flag follows the artefact, not the other way round.
