# ADR-0011 — Hold the launch until the board stops flipping at full stick

- **Status:** Accepted
- **Date:** 2026-08-02
- **Ratified by:** COO
- **Closes:** none — decided by the CEO directly, logged here
- **Constrains:** every role with Monday-dated work: Senior Digital Marketer, Digital Content
  Production, Game Engineer, Senior Controls, and the `overboard-web` status lock-step
- **Enforced by:** `policy` CI gate for the public-claim half (ADR-0003); the date itself is
  convention only and therefore relies on this file being read

## Context

The M3 plan shipped a playable Unreal game as the launch artifact on Monday 2026-08-03.
On the Saturday, hands-on driving by the CEO plus a root-cause investigation (issue #190,
PR #191) established a defect that the plan assumed did not exist.

**Holding full forward stick from rest flips the board.** Measured, deterministic, and
reproducible bit-for-bit via `sim-host --scripted-scenario s-curve`:

| sim time | pitch | ground speed | motor current |
|---|---|---|---|
| 4.92 s | −10.4° | 6.53 m/s | 40.00 A — envelope saturated |
| 5.87 s | −20.1° | 7.87 m/s | pinned; `FALLEN` trips |
| 6.47 s | −179.5° | 31.5 m/s | pinned; fully inverted |

`MAX_CURRENT_A` 40 A × `KT_NM_PER_A` 0.7 = 28 N·m, which is exactly the MJCF `wheel_motor`
`ctrlrange`. Both bind at the same point, so this is a **real actuator-authority limit, not a
software clamp**. Once saturated the controller has no authority left, and the board is
inverted 1.55 s later.

Four facts make this a launch blocker rather than a known rough edge:

1. **It is on the straight, not in the carve.** `steer = 0`, `yaw = 0`. It is not an
   aggressive-manoeuvre edge case; it is the first thing a new player does.
2. **It reaches the playable build.** A person holding full forward for ~5 s flips the board.
   The CEO hit exactly this and reported it; see "the misdiagnosis" below.
3. **It is a cliff, not a taper.** Stable through 0.95 of full stick (worst pitch −9.9°);
   flips at 0.97, 0.99 and 1.00. The filmed demo schedule sits at 1.00 — the wrong side, with
   no margin.
4. **The speed cap cannot help.** `MAX_GROUND_SPEED_M_S` 9.34 ramps from 8.34 m/s; authority
   runs out at 6.53 m/s. With the cap set to 1e9 the divergence times are identical to the
   millisecond.

### Why it measured as stable for two days

`send-input` paced on a wall clock and delivered stick input at **7–13 Hz instead of 50 Hz**,
straddling the host's 100 ms staleness cutoff. A run-varying fraction of every run therefore
had the stick silently zeroed: steady-state ballast 0.031–0.034 m against 0.046 m healthy, an
effective lean near 0.62 rather than 1.00. The same command flipped on one run of three.

The harness was **masking** the defect, not producing it. Every "no instability observed" note
in the scenario doc comments was measuring a de-rated delivery. Fixed in #191 (49.9 Hz
measured, with an explicit warning as the rate approaches the staleness floor).

### The misdiagnosis this supersedes

The CEO reported this defect on first contact — hold W, the rider flips and passes through the
ground. It was attributed to a missing speed cap, a cap was implemented (#186), and it was
reported fixed. The cap is correct work and stays, but it never addressed the report: the
motor gives out below the speed at which the cap begins to act. **The original report was
never fixed and remains open.** Recorded here because the failure mode — closing a report on a
plausible mechanism rather than on a measurement — is the one this org keeps repeating.

## Decision

**The launch does not ship on 2026-08-03.** It is held until the exit criteria below are met.

No public artifact may assert stability of the balance controller until then. The withdrawn
claim ("the board never became unstable at any aggression level tested") is false and must not
be reinstated in any softened form; measurements taken against the de-rated delivery support
nothing and may not be cited.

### ⚠️ SECOND RATIFICATION 2026-08-02 — the exit bar below is RESPECIFIED

**Read this before the criteria list.** The fix this ADR specified was built, measured and
merged (#205). It **met (b), (c), (a)-1 and (a)-2** — including the full reverse-to-forward
reversal at speed, which this ADR records as never having been tested and names the worst
case. It **provably cannot meet (f) or (a)-3 at any value of the constant.**

Two criteria therefore change. Adjudicated by the Oracle; the reasoning is recorded because
"the criterion was wrong" is exactly what a party that failed a criterion would say, and that
objection deserves an answer rather than a ruling.

**(f) was mis-specified, and one identity proves it.** This ADR states lean sensitivity as
**5.84° per m/s², fixed by geometry**. One radian per g is **57.2958 / 9.80665 = 5.8425° per
m/s²**. The measured `est − truth` residual slope is ~5.8° per m/s². *These are the same
number because they are the same physics:* the lean an inverted pendulum needs to sustain
acceleration `a` is `atan(a/g) ≈ a/g`, which is exactly the apparent-vertical tilt a
complementary filter reads.

So the estimator is not carrying an error that flatters the controller. It is **inadvertently
implementing the textbook balance-vehicle solution** — the same lean-into-acceleration a
Segway performs deliberately. `--pitch-source truth` does not de-bias the loop; it **deletes
the only mechanism generating the physically-required lean**, leaving a pitch-only regulator
with 4 cm of ballast trying to buy 17 cm of CoM offset. That controller inverting at every
stick fraction down to 0.05 is what the physics predicts. The criterion was measuring a
different, broken controller.

**What does NOT follow: ±0.25° does not become the new criterion.** The measured static-error
tolerance band is ±0.25° (inverts at ±0.5°). Promoting it to a bar would be **deriving the
acceptance number from what happened to pass** — the precise move (f) exists to forbid.
Record it as a *measured characterisation*, never as a threshold. A tolerance requirement
comes from the environment, not from the measurement.

**(f) is replaced by freeze-and-pin.** The pass currently rests on an unversioned accident;
promoting it to a design element is the only honest way to stand on it:

- **(f1)** Regression-test the residual: slope ≈ 1 rad/g and static offset inside a pinned
  band, so any estimator or tuning change that moves the trim **breaks CI** instead of
  silently re-flipping the board.
- **(f2)** Pin the reserve's **derivation, not just its value.** The 41.97 A/unit slope was
  measured at the current operating trim, so the constant is **trim-derived, not
  geometry-derived** — describing it otherwise is a rationalisation. Assert the measured slope
  in the harness.
- **(f3)** Longer-term, bundled with the headroom fix: make the lean setpoint explicit as
  `θ_ref = atan(a_des / g)` feedforward. At that point (f)'s original truth-fed reading
  becomes **satisfiable, and is reinstated.**

**(a)-3 and static robustness move to the hardware gate — verbatim, not deleted.** 201°/s of
imparted pitch rate against a KD channel affording ~76°/s is a geometry and actuator fact that
no software meets, and this ADR already concluded the undersized elements are **ballast stroke
and CoM height**. For hardware the mounting, calibration and thermal budget is comfortably
≥1°, and the system fails that — which confirms the hardware finding rather than excusing it.

Keeping them on the *game* gate would convert this hold into "redesign the physical board
before shipping a game", which is not what it was called for. The hold was called because the
first thing a new player does inverted the board. That is fixed and verified.

**The split is honest only under all three conditions. Drop any one and it is the softening
move this ADR forbids:**

1. **Moved, not dropped.** Both criteria land on the hardware/bench gate verbatim, cited as
   currently failing, with their measured numbers, in the same pass as this amendment.
2. **The authored world is constrained to what the controller survives, and the constraint is
   encoded as a checkable asset rule** — not a hope. The board rides out ~1 mm at a calm point,
   so terrain must be analytically smooth and authored inclines must stay well inside the
   measured static tolerance (a 0.5° slope is an effective static pitch disturbance).
3. **The loss-of-authority warning ships** as the in-game surfacing of the cliff. 2.868 s of
   lead is adequate.

**The deferred headroom-based fix is not required to clear this hold** — demanding a never-run
feedback path be designed and validated *under* a launch hold trades a measured, bounded risk
for an unmeasured one built under schedule pressure. But the 0.80 reserve is open-loop and
operating-point dependent, so it is honest **only for the frozen world and the frozen trim**.
The headroom fix is therefore a **named blocking prerequisite on the next boundary**: any world
expansion beyond the constrained terrain, any retune that moves the pinned trim band, and the
hardware gate. Named here rather than left as a deferred aspiration.

**Strongest counter-argument, recorded rather than buried:** (f) exists precisely to stop this
conversation, and accepting "the criterion was wrong" from the party that failed it is a
corrosive precedent. It was not accepted on their say-so. The 1-rad/g identity is independently
checkable arithmetic; the implementer **volunteered** the failing result under the robustness
reading the criterion was reaching for, which a party steering toward a ship would have
omitted; and the harness reproduced this ADR's own independently-established figures
(saturation 4.920 s vs 4.92 s, `FALLEN` 5.868 s vs 5.87 s). The finding survives every reading
of the criterion. The precedent worry is answered by the three conditions above being *harder*
than a quiet pass, not softer.

---

### Exit criteria — FIRST RATIFICATION 2026-08-02, after diagnosis (see respecification above)

The provisional criteria in the first revision of this ADR asked whether the −10° was real
pitch or estimator error. **It is real physics.** Estimated and true pitch never diverge by
more than ~1° (mean 0.57°). Sustaining ~2 m/s² requires ~17 cm of forward CoM offset; the
ballast stroke supplies 4 cm; the remaining 13 cm is bought by tilting a 0.67 m body, which
costs 11°. **Sensitivity is 5.84° of lean per m/s², fixed by geometry.** Total pitch authority
is `MAX_CURRENT_A·KT/KP` = 28/140 = 0.2 rad = **11.46°**, and steady full-stick acceleration
spends 10.4° of it. The bias does not sit beside the cliff; **it is the cliff**.

Two findings inverted earlier assumptions and are recorded so nobody re-derives them wrongly:

- **Correcting the estimator makes the board worse.** The ~1° nose-down error acts as nose-up
  trim. Feeding the controller MuJoCo truth makes it flip **1.74 s earlier**; measured four
  ways, monotonic in the bias. This is **accidental model error, not a designed safety
  property**, and criterion (f) exists specifically to stop it being load-bearing.
- **The survivability boundary is the speed cap, not the lean.** Every run that saturated
  above 8.34 m/s (where `SPEED_CAP_MARGIN_M_S` begins withdrawing fore/aft authority)
  survived; every run that saturated below it flipped. Once torque is clamped, `τ = −kp·θ`
  no longer holds and pitch is open-loop unstable — **saturation is survivable if and only if
  something is already unloading the board when it occurs.**

Criteria (a) and (c) below were sharpened by the Oracle; (f) and (g) were added by it.

1. **(a) The board does not invert across a named test matrix** — "from any starting state"
   was untestable as written:
   - full stick from rest;
   - **full reverse-to-forward stick reversal at speed** — the worst case, since commanded
     deceleration and gravity load the same side. This has never been tested;
   - full stick during a kerb strike (use the disturbance derivation from #147).
2. **(b)** Margin is stated as a measured number, not as "stable" — quoted in **both** current
   headroom and pitch headroom, at the worst point in the matrix.
3. **(c) A loss-of-authority warning fires before the outcome is decided, with a stated
   measured lead time.** The trigger is **saturation while below speed-cap onset** — that is
   the actual discriminator. A warning that fires on saturation above 8.34 m/s is noise.
   `FALLEN` today trips ~1 s *after* the board is committed. The signal already exists and is
   discarded at `host.rs:850` (`let (bounded_cmd, _sat) = envelope.apply(...)`); filtered
   authority utilisation over 0.85 gives a measured 2.69 s of lead.
4. **(d)** Demo footage re-shot through `--scripted-scenario` so it is bit-identically
   reproducible, captioned to only what that run measured.
5. **(e)** Reset works. Currently a no-op — the host logs `input reset bit set -- not
   implemented yet, ignoring` — so a fallen board has no recovery short of relaunching the
   stack. Snapshot preserved on `feat/controls/turn-radius-and-reset` (unvalidated).
6. **(f) Every pass must hold with the estimator bias removed** — acceptance runs are fed
   MuJoCo truth, the measured-worse case. **No criterion may be satisfied on borrowed
   margin.** This is the most important addition: without it, (a) can pass only because of an
   accident nobody designed.
7. **(g) Every pass must hold across a damping sweep of 0.5×–2×** on the wheel-hinge
   `damping="0.08"` (see below). Cheap in sim, and it converts a soft cliff into a bounded one.

### The fix, and why 0.80 is not a magic number

**Cap commanded lean, changing nothing else.** Input shaping upstream of the estimator, the
regulator and the envelope — standard fly-by-wire practice and the only zero-risk move
available. Measured cost: **top speed unchanged** (the speed cap governs it), 0.93 s slower to
8 m/s, 8% less distance over 15 s.

The constant must be **derived, not tuned to the cliff**: peak demand is linear at 42 A per
unit stick, so 0.80 × 42 = 33.6 A ≈ **84% of envelope**, leaving steady lean at 8.3° of the
11.46° ceiling ≈ **28% pitch reserve**. Name it for the reserve it expresses (e.g.
`CMD_ENVELOPE_RESERVE`) and cite the 42 A/unit measurement as provenance. Tuning to the
measured 0.96/0.97 cliff is forbidden — that boundary rests partly on the accidental
estimator bias and on an undocumented damping constant.

Rejected alternatives: lowering `MAX_GROUND_SPEED_M_S` to 6–7 m/s is causally proven but costs
top speed while the stick cap costs none — dominated. Retuning `KP`/`KD` moves every
gain-related acceptance criterion in the repo and is refused. Raising `MAX_CURRENT_A` is a
claim about hardware, not a tuning knob.

**The permanent fix is different and is deliberately not shipped here:** derive fore/aft
authority from remaining **current headroom** rather than from speed. It is the runtime form
of the spec rule below, protects at any speed and any lean, and needs no arbitrary top-speed
number — but it is a new feedback path that has never been run, and there is no launch
pressure to justify shipping it unvalidated.

### What propagates to the hardware spec

The sim-first premise obliges a hardware finding here. **The supported finding is not "the
motor is undersized"** — writing that into the spec would aim the hardware effort at the wrong
lever, because more torque raises the pitch ceiling but does nothing about the 11° of lean
that 2 m/s² geometrically requires. What propagates:

- **Command-map derivation rule.** The stick→setpoint map SHALL be derived from the actuator
  envelope minus a stated disturbance reserve. The present 5% over-command at full stick is a
  **normalisation defect, not a sizing gap**, and a spec rule prevents it recurring on hardware.
- **The geometry, which is gain-independent and damping-independent** — the truest hardware
  findings available: 5.84° of lean per m/s², and 17 cm of CoM offset required at 2 m/s²
  against 4 cm of ballast stroke. These say the **ballast stroke**, not the motor, is the
  undersized element if acceleration without deep lean is wanted. The three levers are torque
  envelope, ballast stroke and CoM height; the spec should trade them with these sensitivities
  in hand.
- **The coupling identity** `pitch ceiling = τ_max / KP` — record the identity, not today's
  11.46°, because any hardware gain change moves the cliff.

**Explicitly does NOT propagate yet:** any speed-dependent number, including the 8.34 m/s
survivability boundary and top-speed envelope fractions. Those rest on `damping="0.08"`.

### The undocumented constant

`damping="0.08"` on `wheel_hinge` is the only load-bearing constant in the MJCF with **no
provenance comment**, in files where every other constant carries a paragraph of derivation.
It costs 0.55 N·m (0.79 A) per m/s of ground speed — **18% of the envelope at the speed cap** —
and it sets the entire speed-dependence of this failure. It does not block launch *provided
criterion (g) holds*, but it blocks propagating any speed-dependent number to the hardware
spec, and it blocks the eventual headroom-based fix, which would inherit its speed dependence.
It belongs in the imperfection-profile conformance contract (fabe806), which exists for exactly
this class of silent constant.

## Options considered

**Ship Monday with a documented limit.** Caption the known limit, cap commanded lean below
the cliff in the shipped build, and fix properly afterwards. Cost: the launch artifact is a
*playable* game, and the defect is reachable by the most obvious input a new player supplies
in their first ten seconds. A disclosure that reads "holding forward flips the board" is not a
limitation, it is the review. Rejected by the CEO.

**Hold for the fix.** Cost: the date slips, Monday-dated work across four roles is stranded
mid-flight, and the slip is open-ended because the mechanism is not yet understood — a
diagnosis could return "estimator error, one day" or "balance-loop retune, several". Accepted,
on the basis that the failure is trivially reachable and a first impression is not reversible.

**Tune the gains now and ship Monday anyway.** Rejected without escalation: retuning a balance
controller against an unexplained −10° bias, the day before launch, converts one measured
defect into an unmeasured number of new ones. The diagnosis gates any tuning.

## Consequences

**Easier.** The stability claim problem disappears rather than needing careful wording. The
re-shoot can be bit-identically reproducible, which is a stronger provenance position than
anything achievable under the old harness.

**Harder.** Four roles have Monday-dated work in flight and must be told; restart is the only
broadcast primitive this org has (ADR-0001). The `overboard-web` page status must move in
lock-step (`SR-WEB-4`) and must not announce a launch that is not happening. Any dated public
copy is now wrong.

**Who moves:**

- **CMO** — ⚠️ **added by amendment 2026-08-02; the first revision of this list missed it.**
  `roles/cmo/CONTEXT.md` was merged on 2026-08-01 (#164), the day before this ADR was
  ratified, and its first sub-goal still read "LAUNCH IS MONDAY 2026-08-03 MORNING, readiness
  review Sunday night." Under the session-start protocol that file is the first thing a CMO
  session reads, so the role is not merely uninformed — **any** restart, for any reason, boots
  into stale launch orders. A supersession banner was added under `TURF-OVERRIDE` to defuse
  it; the sub-goals themselves remain CMO's to re-base.
- **Senior Controls** — owns the fix and the diagnosis. Everything else queues behind it.
- **Digital Content Production** — the existing "Manny rips" capture is superseded; hold the
  re-shoot until a margined lean is chosen.
- **Senior Digital Marketer** — `feat/marketing/board-0801-relaunch` carries the date. Strip
  or re-date it; do not publish.
- **Game Engineer** — the playable build inherits the defect. No public build until item 1.
- **Archivist** — the withdrawn claim and the provenance wording both need sweeping.

## How this is enforced

The public-claim half is enforced: `policy` (ADR-0003) already gates public claims in this
repo, and the withdrawn stability claim is registered with it.

The date itself is **convention only**. There is no CI check that can tell whether a launch
happened, so this ADR binds only sessions that read it — which is precisely why the COO owes
the CEO a list of sessions to restart, and why that list is part of closing this decision
rather than a follow-up to it. That list is
[`roles/coo/restart-briefs-2026-08-02-launch-hold.md`](../../roles/coo/restart-briefs-2026-08-02-launch-hold.md).

### The gap this ADR shipped with, found the same day

"Binds only sessions that read it" is weaker than it sounds, and the CMO amendment above is
the proof. **A role's standing context file can contradict a ratified ADR and nothing
notices.** The ADR is read by sessions that go looking for it; `roles/<role>/CONTEXT.md` is
read by every session of that role automatically, at start, before anything else. When the two
disagree, the context file wins on reach — so the weaker document is the one this ADR was
relying on.

The CMO omission was found by a dead-date audit across all four repos, **not** by reading this
ADR's consequences list. That is the honest provenance and it is the finding: the consequences
list was not sufficient to identify who the decision bound, and there is no check that would
have caught the difference. Tracked as a work request rather than solved here, because the
obvious fix — a `policy` check comparing role context files against ratified decisions — is a
new gate, and a gate whose cost to the person who did nothing wrong has not been thought
through is how four of the COO's checks landed on other roles first.

One thing this did **not** cost, checked rather than assumed: the live public `overboard-web`
site carries no 2026-08-03, no Monday launch date and no stability claim. Nothing wrong was
published.
