# 2026-08-02 — Launch held: the board flips at full stick, and we found out why

**Role:** COO · **Session outcome:** launch date abandoned, root cause established,
fix specified down to the constant. No fix implemented yet.

## The one thing to know

**Holding full forward stick from rest inverts the board in ~6.5 s.** It is on the straight
with `steer = 0`, it reaches the playable build, and it is the first thing a new player does.
The CEO decided to hold the launch. Ratified as **ADR-0011**, with exit criteria ratified in
**PR #195** (queued at teardown — check it merged).

## What was landed

| PR | What |
|---|---|
| #185 | `ops/drive-game.sh` — one command to drive the board yourself |
| #186 | Steering sign, speed-proportional yaw, speed cap (CEO's first driving feedback) |
| #187 | Soft corridor so the board stops running through walls |
| #189 | `--kick-at` inducible falls; verified `FALLEN` actually trips |
| #191 | s-curve flip root cause + the harness pacing fix + `--scripted-scenario` |
| #192 | ADR-0011 — hold the launch |
| #193 | **Kinematic in-plant yaw injection — MuJoCo's pose is now authoritative** |
| #195 | ADR-0011 exit criteria ratified after diagnosis *(queued at teardown)* |
| #188 | Oracle routing moves to Fable 5 *(queued at teardown)* |

## The three findings that inverted earlier assumptions

Written out because each one was believed the other way round for part of the day, and a
successor working from the older notes will get them wrong.

**1. The −10° steady-state pitch is real physics, not estimator error.** Estimated and true
pitch never diverge by more than ~1°. Sustaining ~2 m/s² requires ~17 cm of forward CoM
offset; the ballast stroke supplies 4 cm; the other 13 cm is bought by tipping a 0.67 m body,
costing 11°. **5.84° of lean per m/s², fixed by geometry.** Authority ceiling is
`MAX_CURRENT_A·KT/KP` = 11.46°, and full stick spends 10.4° of it. The bias is not *near* the
cliff — it **is** the cliff.

**2. "Fixing" the estimator makes the board worse.** The ~1° nose-down error acts as nose-up
trim, shallowing the lean. Feeding the controller MuJoCo truth makes it flip **1.74 s
earlier** — measured four ways, monotonic. Issue #190 named the estimator as the suspect and
pointed **exactly the wrong way**. This protection is *accidental model error, not a designed
property*, which is why exit criterion (f) forbids any criterion passing on it.

**3. The survivability boundary is the speed cap, not the lean.** Every run that saturated
above 8.34 m/s survived; every run below it flipped. Once torque is clamped, `τ = −kp·θ` no
longer holds and pitch is open-loop unstable. **Saturation is survivable iff something is
already unloading the board when it happens.** Today the speed cap is the only such mechanism
and at full stick it arrives 1.8 m/s too late.

## Why it measured as stable for two days

`send-input` paced on a wall clock, delivering **7–13 Hz instead of 50 Hz** against a 100 ms
staleness cutoff — so the stick was silently zeroed for part of every run (effective lean
~0.62). The harness was **masking** the defect, not causing it. Fixed in #191. Every
"no instability observed" note written before that was measuring a de-rated delivery and
supports nothing.

**Process lesson, recorded in the ADR:** the CEO reported this defect on first contact. It was
attributed to a missing speed cap, the cap shipped, and it was reported fixed — but the motor
gives out *below* the speed the cap acts at, so the report was never addressed. Closing a
report on a plausible mechanism rather than on a measurement is the recurring failure here.

## What remains to clear the hold

All specified, none implemented:

1. **Derived command-envelope reserve** (~0.80 stick). Must be *derived, not tuned*: 0.80 ×
   42 A/unit = 33.6 A ≈ 84% of envelope, 28% pitch reserve. Name it for the reserve. **Tuning
   to the measured 0.96/0.97 cliff is forbidden** — that boundary rests on the accidental
   estimator bias and an undocumented damping constant. Zero-risk change: input scaling
   upstream of estimator, regulator and envelope. Cost: top speed unchanged, 0.93 s slower to
   8 m/s, 8% less distance over 15 s.
2. **Loss-of-authority warning.** The signal already exists and is thrown away at
   `host.rs:850` (`let (bounded_cmd, _sat) = envelope.apply(...)`). Filtered authority
   utilisation > 0.85 gives **2.69 s of lead**; `FALLEN` today gives −0.95 s. Trigger on
   *saturation while below speed-cap onset* — that is the discriminator.
3. **Reset** — still a no-op (`input reset bit set -- not implemented yet, ignoring`).
4. **Turn radius** ~2× tighter (CEO drove it).
5. **Re-shoot** demo footage through `--scripted-scenario` at the margined lean.
6. **Braking and coasting** — CEO reports braking far too weak and coasting far too free. Real
   onewheels brake hard through regen and have real rolling resistance. Neither exists. This is
   the fore-aft path plus a drag term. NOT started.

⚠️ **`feat/controls/turn-radius-and-reset` holds 240 lines of UNVALIDATED work** across six
files — items 3 and 4, snapshotted when an agent died on a session limit. **Do not merge.**
Verify before trusting any of it.

## Hardware finding — state it precisely

The Oracle explicitly rejected "the motor is undersized": more torque raises the pitch ceiling
but does nothing about the 11° of lean geometry requires, and writing it into the spec would
aim the hardware effort at the wrong lever. What propagates:

- **Command-map derivation rule** — the stick→setpoint map SHALL be derived from actuator
  envelope minus a stated disturbance reserve. The 5% over-command at full stick is a
  **normalisation defect, not a sizing gap**.
- **The geometry** (gain- and damping-independent, so the truest finding available): 5.84°
  per m/s², 17 cm required vs 4 cm of ballast stroke. Says the **ballast stroke** is the
  undersized element. Three levers: torque envelope, ballast stroke, CoM height.
- **The identity** `pitch ceiling = τ_max/KP` — record the identity, not today's 11.46°.

**Does NOT propagate:** any speed-dependent number, including the 8.34 m/s boundary. Those
rest on `damping="0.08"`, the only load-bearing MJCF constant with **no provenance comment**,
worth 18% of the envelope at the speed cap. Belongs in the imperfection-profile contract
(fabe806).

## Roles that must be restarted

Restart is the only broadcast primitive this org has (ADR-0001), so this is owed to the CEO:

- **Senior Digital Marketer** — most urgent. `feat/marketing/board-0801-relaunch` carries the
  dead date. **Must not publish.** `overboard-web` status moves in lock-step (`SR-WEB-4`).
- **Digital Content Production** — "Manny rips" is superseded; hold the re-shoot until the
  margined lean is chosen.
- **Game Engineer** — playable build inherits the defect. No public build until criterion (a).
- **Archivist** — withdrawn stability claim + new provenance wording both need sweeping.

Mechanical and the Pi-image work are unaffected.

## Loose ends

- **`/tmp/overboard-media` is volatile.** It holds `clips.json` (with the corrected claim
  text), the captures, and the generated gallery served over Tailscale. `ops/build-demo-gallery.py`
  is in-repo but the manifest is not. A reboot loses the corrected captions. Worth moving
  somewhere durable.
- The **provenance line** for the yaw change was adjudicated but coordination with the
  Archivist (issue #163) was not confirmed done: *"Steering is commanded, not emergent… nothing
  is dead-reckoned."*
- **Reverse-to-forward stick reversal at speed has never been tested** and is the worst case
  (commanded decel and gravity on the same side). It is now exit criterion (a).
- Two agents hit **session limits** today. Three concurrent agents plus a long driver session
  is roughly the ceiling.
