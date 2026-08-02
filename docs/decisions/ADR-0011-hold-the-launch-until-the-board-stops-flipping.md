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

### Exit criteria — RATIFIED 2026-08-02, after diagnosis

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
rather than a follow-up to it.
