# ADR-0010 — The game wire v1 is fixed by the COO for the launch weekend, and expires

- **Status:** Superseded by ADR-0012
- **Date:** 2026-08-01
- **Ratified by:** COO
- **Closes:** none — CEO direction, 2026-08-01 (formal approval of the M3 Revision 3 re-cut)
- **Constrains:** `sim-host` in `overboard`; the UE client in `overboard-game`; both until superseded
- **Enforced by:** a compile-time size assertion and a byte-size unit test in `sim-host`, plus
  fail-loud `magic`/`schema_version` rejection on both sides
- **Expires:** Tuesday 2026-08-04, when schema ownership reverts to Senior Controls per ADR-0009.
  It did expire, and sat unreplaced for two days. **ADR-0012 supersedes it** as of 2026-08-06:
  the packet becomes v3, and the frame-transform and fail-loud rules below are carried forward
  unchanged rather than re-decided.

## Context

M3 was re-cut on 2026-08-01 to a Monday launch ([M3 Implementation Plan § Revision 3]). The
launch artifact is a playable game. W1 — the pose spine — is Senior Controls building a 500 Hz
`sim-host` in `overboard` and the Game Engineer building a UE client in `overboard-game`, in
parallel, overnight.

[ADR-0009](ADR-0009-fourth-repo-and-game-engineer-seat.md) assigns the wire schema to Senior
Controls: *"Senior Controls owns everything on the `overboard` side of the seam — the plant
variant, the real-time host, the wire crate."* Issue [#161](https://github.com/MikePaNtZ/overboard/issues/161)
and [#162](https://github.com/MikePaNtZ/overboard/issues/162) both instruct the two roles to
*"talk continuously — this is not a handoff."*

**They cannot.** Sessions in this org cannot message each other; that is the first line of
[INDEX.md](INDEX.md). The two available orderings are both bad:

- **Serialise** — Controls defines the wire, then Game builds against it. Costs the Game
  Engineer the entire night, on the side whose W3 (*enjoyable*) has the least slack.
- **Parallelise and reconcile in the morning** — each side invents a schema. The failure is not
  a clean compile error; it is a plausible-looking float misparse. The plan names the frame
  transform as one of three things that *"silently poison everything downstream if wrong."*

A schema neither side can unilaterally change is worth more this weekend than a schema designed
by the role that will eventually own it.

## Decision

**1. The COO fixes wire v1, as a coordination act, and says so.** This is a deliberate,
time-boxed deviation from ADR-0009's ownership assignment. It is recorded rather than done
quietly, because a seam silently re-owned is exactly the drift this record exists to catch.
Ownership reverts to Senior Controls on Tuesday. Both roles received the schema below verbatim
in their dispatch brief.

**2. The schema.** Little-endian, `#[repr(C)]`, field order as listed.

**State out** — host → renderer. Host **sends** to `127.0.0.1:9601`.

| field | type | notes |
|---|---|---|
| `magic` | u32 | `0x4F425731` ("OBW1") |
| `schema_version` | u16 | `1` |
| `flags` | u16 | bit0 armed, bit1 valid, bit2 fallen |
| `seq` | u64 | monotonic tick counter |
| `sim_time_s` | f64 | MuJoCo sim time |
| `pos` | f32[3] | **raw MuJoCo world frame** — metres, Z-up, right-handed |
| `quat` | f32[4] | **w,x,y,z**, raw MuJoCo |
| `wheel_angle_rad` | f32 | |
| `wheel_rate_rad_s` | f32 | positive = forward |
| `pitch_rad` | f32 | nose-up positive (ICD §10.1) |
| `yaw_rad` | f32 | **non-physical game channel** |
| `motor_current_a` | f32 | applied current |

**Input in** — renderer → host. Host **binds** `127.0.0.1:9602`.

| field | type | notes |
|---|---|---|
| `magic` | u32 | `0x4F424931` ("OBI1") |
| `schema_version` | u16 | `1` |
| `flags` | u16 | bit0 arm, bit1 reset |
| `seq` | u64 | |
| `weight_shift_fore_aft` | f32 | clamped [-1,1] |
| `weight_shift_lateral` | f32 | clamped [-1,1] |
| `steer` | f32 | clamped [-1,1] — **non-physical** |

**3. The host emits the raw MuJoCo frame; the renderer owns the conversion.** MuJoCo is metres,
Z-up, right-handed. Unreal is centimetres, Z-up, left-handed. One side owns that transform and it
is the game side, in one named function.

The reason it goes to the renderer rather than the host: the host's output is also what
diagnostics and every later analysis path read, and those are all MuJoCo-frame consumers. A host
that pre-converted to Unreal's basis would make the renderer cheap and every controls consumer
pay a conversion back — and a controls number that had silently round-tripped through a
left-handed basis is precisely the class of error that is unfalsifiable after the fact.

**4. A mismatch fails loudly.** Wrong `magic` or `schema_version` is logged and dropped, never
parsed. This is the ABI discipline of `ob_abi_version()` and `size`-tagged structs applied to the
wire, as ADR-0009 requires.

**5. The input socket must never block the 500 Hz loop.** Non-blocking; most-recent-packet-wins;
stale input decays to zero on a documented timeout.

**6. `yaw_rad` and `steer` are non-physical and must be labelled so in code.** The simulated wheel
is a cylinder and physically cannot carve. This is not a comment-quality preference: it is the
input to the `Playable Sim` channel declaration ([#163](https://github.com/MikePaNtZ/overboard/issues/163)),
which is a launch blocker. A channel that is invented in code and unlabelled becomes a public
claim that is false.

## Consequences

**Good.** Both sides build overnight against one contract neither can drift from. The frame
transform has exactly one owner and one implementation. The non-physical channels are named in
the schema itself, so the claims work downstream has a source rather than a recollection.

**Bad, and accepted.** The COO has specified an interface it does not own and will not maintain.
That is justified for 48 hours by the no-messaging constraint and by nothing else. If v1 survives
past Tuesday unexamined, this ADR has failed and the fix is Senior Controls superseding it.

**Deliberately absent.** v1 carries no rider-ballast state, no terrain, no record/replay fields
and no MCAP mapping. All are cut from the weekend and none are cancelled. Adding a field is a
version bump, which is cheap by construction — the schema is designed to be outgrown, not to be
complete.

**The enforcement is a test, not this document.** Per the standing lesson that documentation is a
polling surface: the binding artifacts are the size assertion and the fail-loud version check in
code. If those disagree with the table above, the code is right and this ADR is stale.

[M3 Implementation Plan § Revision 3]: https://app.notion.com/p/3af472a5fb6981f5b6e4ec038293ad6f
