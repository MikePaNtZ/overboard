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

### Exit criteria — provisional

Marked provisional because the mechanism is not yet understood: the board holds roughly −10°
of steady-state pitch under sustained acceleration, which is what parks it beside the cliff,
and it is not established whether that is real physical pitch or an attitude-estimator error.
A diagnosis is in flight. **If it is estimator error, the controller is balancing against a
lie and these criteria are wrong** — they will be revised on that finding and this ADR
superseded or amended.

1. Holding full stick from rest, from any starting state, does not invert the board.
2. Whatever margin is chosen is stated as a measured number, not as "stable".
3. A saturation / loss-of-authority signal exists and fires **before** the outcome is decided.
   `FALLEN` currently trips ~1 s after the board is already committed to going over.
4. Demo footage is re-shot through `--scripted-scenario` so it is bit-identically
   reproducible, and its caption claims only what that run measured.
5. Reset works. It is currently a no-op — the host logs `input reset bit set -- not
   implemented yet, ignoring` — so a fallen board has no recovery path short of relaunching
   the stack. Tracked with the in-flight yaw epic.

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
