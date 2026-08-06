# ADR-0012 — Hand physics authority to Unreal at the moment MuJoCo stops, and carry the state to do it on wire v3

- **Status:** Accepted
- **Date:** 2026-08-06
- **Ratified by:** COO
- **Closes:** none — CEO directive of 2026-08-06 (deliberate kerb strike during the joystick
  play-test), decided in role and logged here
- **Constrains:** Senior Controls (`sim-host`, the wire encoder), Game Engineer (`BoardActor`,
  the wire decoder), Sr. Mechanical (`sim/models/`), and every future public claim made over
  footage that contains a crash
- **Enforced by:** `policy` — the provenance rule below; plus the v3 known-answer test on both
  sides of the wire, which fails if the two encoders disagree by a byte

## Context

The board cruises. The CEO wants to drive it into a kerb and watch it come apart, and the
interesting half of that is what happens *after* the strike. Four things were true when this
was asked, and all four had to move.

**ADR-0009 forbids exactly what is being asked for.** Its words are "Unreal renders and takes
input; it computes no board physics", inherited from `overboard-viz`'s renderer rule. A
post-strike tumble computed in Unreal is Unreal computing board physics. That rule is load-
bearing and the reason it exists is good: every acceptance number this project has comes out
of MuJoCo, and a second engine that quietly computes board states is how a number starts
coming from the wrong one.

**ADR-0010 expired on Tuesday 2026-08-04**, two days ago, with wire-schema ownership reverting
to Senior Controls. Nothing has been ratified in its place, so the wire is currently governed
by an expired ADR.

**The wire cannot carry a handoff even if we allowed one.** The v2 state packet has position,
orientation, wheel angle and rate, pitch, yaw, motor current and rider ballast. It has **no
linear velocity and no angular rate**. A receiving physics engine seeded without those starts
the crash from rest, which looks like the board hit a wall of treacle.

**ADR-0011 condition 2 says the authored world is constrained to what the controller
survives.** A kerb in the ridden world is, on its face, the precise thing that constraint
excludes — and ADR-0011 is explicit that dropping any of its three conditions turns the
criterion move into the softening manoeuvre it forbids by name. This is the part that needed
deciding rather than implementing, and §Decision item 3 is the answer.

## Decision

**1. Physics authority is a single latched, one-way, annunciated transfer.**

MuJoCo is the sole authority for board state from arm until it declares a *terminating event*.
On that declaration it stops propagating: it sets the handoff bit, publishes a final state, and
holds. Unreal then owns the board until the input `reset` bit clears the latch, at which point
authority returns to MuJoCo and Unreal drops back to kinematic wire-follow. There is never a
moment when both engines are integrating the board, and there is never a transfer that the
wire did not announce. The client may **not** simulate on inference — no "the board looks
fallen, I'll take over". It simulates when the bit is set and at no other time.

**2. Everything downstream of the handoff is animation, and is never evidence.**

No control decision is tuned from it. No number is quoted from it. No acceptance criterion is
evaluated on it. Footage containing a post-handoff segment carries the `Playable Sim`
provenance category and states facts about the machinery, never a result — the rule
`terrain/README.md` already applies to game runs, extended to cover the tumble. This is what
keeps item 1 from reopening the hole ADR-0009 closed: the carve-out is not "Unreal may compute
board physics", it is **"Unreal may animate a board MuJoCo has stopped simulating, and that
animation has no evidentiary standing."**

**3. ADR-0011 condition 2 stands unmodified, because a kerb is not drivable surface.**

Condition 2 constrains the *drivable corridor* — the surface the controller is asked to ride
and on which criteria are evaluated. A kerb is authored deliberately **outside** that corridor,
and striking it is a terminating event rather than a ride-through. No criterion is passed on
kerb-struck geometry; no survivability claim is made about it; the strike ends the run. The
`terraincheck` limits (0.25 mm step, 5.2° slope, 5.07 m descent) continue to bind every
surface declared drivable and are not widened by one micron here. If a future asset ever needs
a limit *widened* to pass, the answer remains the one that directory already gives: change the
asset.

The honest statement of the risk: this creates a category — "authored geometry that is
deliberately unsurvivable" — that did not exist before, and a careless author could park a kerb
inside the corridor and call it out-of-corridor. That is why item 4's handoff bit is the thing
CI keys on rather than the geometry: a strike that fires is observable in the trace, and a
corridor that contains one is a corridor that will show it.

**4. Wire v3 — 104 bytes. ADR-0010 is superseded.**

v3 appends 24 bytes to v2 and adds one flag bit. Everything at or below offset 80 is byte-
identical to v2, and v1/v2 packets **must continue to decode** on both sides — the same
forward-compatibility rule ADR-0010 set, and for the same reason: whichever side ships its
bump first must not freeze the other.

```
field           type      offset  size   since
... v1 fields (0..72) and v2 rider fields (72..80) unchanged ...
lin_vel         float[3]  80      12     v3   world frame, m/s, raw MuJoCo (Z-up, right-handed)
ang_vel         float[3]  92      12     v3   world frame, rad/s, raw MuJoCo, right-hand rule
                                  104 total (v3)
```

`flags` bit 4 = `PhysicsHandoff`. It is a new bit inside the existing field, not a wider
wire — the same call `INPUT_FLAG_KICK` and `AuthorityWarning` both made, so a host that never
sets it decodes identically to before.

**Both velocities are published in the raw MuJoCo frame, untransformed**, exactly as `pos` and
`quat` already are. `CoordinateTransform` remains the one and only place the MuJoCo→Unreal
conversion happens (ADR-0010's rule, unchanged); a velocity converted anywhere else is a bug.

Schema ownership sits with **Senior Controls** from this ADR forward. v3 is ratified here
because it is an interface between two roles and neither owns both ends; the next bump does not
need the COO unless it changes the boundary again.

**5. The terminating event is declared by the host, and the host declares it from measurement.**

The criterion is bumper contact above a force threshold, or the existing `FALLEN` pitch
condition. It is deliberately *not* "the board is near a kerb" — proximity is not contact, and
a handoff that fires on geometry rather than on physics is a handoff that fires when the board
would have missed.

## Options considered

**Model the crash in MuJoCo and keep ADR-0009 intact** — rejected, and this is the option that
looks safest and is not. It needs an articulated rider with joint limits and a contact-rich
tumble in the same engine every acceptance number in this project comes from. The cost is not
the work; it is that unvalidated tumble dynamics would then live one XML file away from the
plant that produces ADR-0011's numbers, with nothing structural stopping a criterion from
being evaluated across the boundary. Handing off to a second engine that is *definitionally*
not evidence is the stronger isolation, not the weaker one.

**A canned fall animation, no state transfer** — rejected. It needs no wire change and no ADR,
which is its whole appeal, but the tumble would not match the direction or the speed of the
strike that caused it, and on a lean-to-steer board that mismatch is the first thing a viewer
sees. The wire work it saves is ~24 bytes; the fidelity it costs is the entire point of the
exercise.

**Let the client infer the crash from `FALLEN` and take over on its own** — rejected. It is
one line of client code and no host change, and it puts the authority decision on the side of
the boundary that is not allowed to make it. Two engines would briefly integrate the same
board every time the inference disagreed with the host.

## Consequences

**Easier.** The crash the CEO asked for becomes buildable tonight against a single authored
kerb box, because nothing in items 1–5 depends on the City Park heightmap. When the heightmap
lands (`overboard-game` `feat/game/terrain-probe-tooling`, currently blocked on its own
centre-post datum assertion at 4.326 mm against a 2 mm tolerance), it swaps in as the geometry
source and **nothing downstream changes** — the handoff, the wire and the client are already
correct.

**Harder.** The wire now has three live versions, and `sim-host` and the UE client must stay
byte-identical across all three. Every role touching the packet pays a known-answer test.

**Who moves.** Sr. Mechanical adds the kerb geom and the `framelinvel`/`frameangvel`/`touch`
sensors to `sim/models/overboard_rider.xml`. Senior Controls implements the latch, the freeze
and the v3 encoder in `sim-host`, and wires the `reset` bit to clear the latch — that bit is
currently accepted and ignored (`host.rs`: "not implemented yet, ignoring"), and the handoff is
the first feature that genuinely needs it. Game Engineer implements the v3 decoder, the
takeover and the rider ragdoll, and must reconcile it with `OverboardPlayerController`'s
existing rising-edge auto-reset on `IsFallen()`, which will otherwise clear the latch a frame
after it is set and end every crash before it is visible.

## How this is enforced

- **The v3 known-answer test, both repos.** A fixed packet with known bytes, asserted in
  `wire/tests/test_wire.cpp` and in `crates/sim-host`'s wire tests. The two encoders cannot
  drift apart quietly — the same pattern `terrain/` uses for the run-out formula, and the
  reason that cross-repo constant is trustworthy.
- **`policy`** carries the provenance rule from item 2: a public claim sourced to footage
  containing a post-handoff segment is a claim about the machinery, not a result.
- **Convention only, and stated as such:** item 1's "the client may not simulate on inference"
  is not machine-checkable today. It relies on this file being read. The mitigation is that
  the takeover has exactly one call site in `BoardActor`, and a reviewer looking at that call
  site can see what gates it.
