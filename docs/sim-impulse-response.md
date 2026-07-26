# Sim Test — Impulse Disturbance Response

Implementation mirror of the Notion mini design doc
[Overboard — Sim Test: Impulse Disturbance Response](https://app.notion.com/p/3a8472a5fb6981d8b9e6f749517498dd)
(GitHub issue [#2](https://github.com/MikePaNtZ/overboard/issues/2)).
Notion holds the intent; **this file tracks what actually shipped**, including
the two places the implementation had to depart from the design.

| | |
|---|---|
| Status | Implemented — open-loop milestone |
| Code | `sim/scenarios/impulse_response.py`, `tests/test_impulse_response.py`, `scripts/render_scenario.py` |
| Model | `sim/models/overboard_onewheel.xml` |
| CI | `.github/workflows/ci.yml` → jobs `sim` (gate) and `publish-sim-artifact` (film) |

## 1. What the test does

Kick a driverless onewheel with a known horizontal impulse and measure the
response. The canonical controls bring-up experiment.

1. **Initial condition** — board upright, at rest, no rider, no proxy mass.
2. **Stimulus** — a single horizontal impulse, forward, at t₀ = 0.5 s.
3. **Open-loop** (today, `Command::ZERO`) — the board rolls away, pitches over,
   and noses into the ground. This is the honest checkpoint.
4. **Closed-loop** (once the controller exists) — arrests the pitch and
   recovers. Same scenario, so it doubles as the regression/margin gate.

The board is **stable at rest** — CoM 30 mm below the axle, because the battery
and hub motor sit low. It does not fall over on its own and a build where it
does is a bug. That is the entire point of the rewrite: the previous model
faked instability with an 8 kg rider mass on a mast (the red "ball on a stick"),
which made every result about the stand-in rather than the vehicle.

## 2. Two departures from the design doc

### 2.1 The topple criterion is nose strike, not 45° of tilt

**The design doc's "pitch exceeds ~45°" is not reachable by this vehicle.**

With the axle held at the 145.4 mm tire radius, the underside of a bumper
reaches the ground after **18.57°** of pitch. The board physically cannot tilt
further while upright — it noses in first. A 45° gate would have been
unfalsifiable: never satisfied, no matter how hard the board is hit.

So `nose_strike` is a real contact between a bumper collision hull and the
ground plane. The angle is *computed from the hulls at runtime*
(`nose_strike_angle_deg()`), never hardcoded, so it tracks any change to the
meshes or the tire radius, and a test asserts the documented value.

> **Do not re-derive this from the STL bounding box.** That gives 14.9°, by
> assuming the extreme −X and extreme −Z coordinates meet at a single corner.
> They do not — the bumper sweeps upward toward its tip, so the vertex that
> lands first is the underside heel at x = −381 mm, ~90 mm inboard of the
> 469 mm tip. This was wrong in an earlier pass of the model header.

18.6° is therefore the real margin the balance controller has to hold, and a
much more useful number than 45°.

### 2.2 The impulse acts through the CoM

The design doc lists application point as "board CoM / deck". It is a
parameter (`application_height_m`), but it **defaults to the CoM**, because a
raised application point injects an angular impulse `r × J` that swamps the
linear one: at deck height (0.15 m) a 20 N·s push adds 3 N·m·s about a frame
pitch inertia of only 0.40 kg·m², a ~430 °/s kick that slams the nose down
within a quarter of a metre — before the vehicle dynamics express themselves at
all. The outcome then depends almost entirely on an arbitrary lever arm.

Pushing through the CoM keeps the disturbance a pure linear impulse, so the
pitch response is produced by the vehicle's own dynamics. Deck-height
application is retained for a follow-on "shove"/curb-strike scenario.

### 2.3 Sign conventions across the sim/HAL seam

The BoardIo ICD calls a sign error across this seam the most dangerous bug in
the system, and says not to implement it from memory. Reconciling the docs
against the model found one:

**Fixed — the motor was inverted.** ICD §7.3 mandates `amps > 0 ⇒ forward wheel
acceleration ⇒ nose pitches up`. With the wheel hinge on `+Y` the model did the
exact opposite: `+6 A` drove the board *backwards* 2.1 m and pitched the nose
*down*, while its own comment claimed the reverse. Nothing depended on it —
the open-loop scenario never actuates — but it would have inverted the balance
law the moment a controller was attached. The hinge is now on `-Y`, and
`test_motor_sign_matches_icd` asserts both the forward case and its mirror.

**Decided — pitch reporting disagreed; the sim moves.** This scenario reports
pitch **nose-down-positive** (so the strike is at +18.6°, the natural reading
here); the ICD is **nose-up-positive**. They are exact negations, so the same
physical law is written `current ≈ +K·pitch` here and `−K·pitch` in the ICD.
Measured, not argued: `+K·pitch` holds the board upright through the nominal
impulse (peak 0.21°, no strike), while the ICD's literal `−K·pitch` drives it
to 180°.

This was left open as "which document moves". **It is now closed: the sim
moves.** ICD §10 is normative and derives the convention from a free-body
argument rather than asserting it — and it has already been wrong once (v0.2
inverted the polarity gate), which is precisely why §10.3 makes the sim the
arbiter and requires the gate to be asserted in CI rather than documented.

⚠️ **Decided, not yet done.** The flip lands in increment I1 of the seam PR,
not here, because it has to move together with the MJCF actuator comments and
`test_motor_sign_matches_icd` — left behind, those become stale-and-wrong,
which is worse than a documented inconsistency. It carries a test asserting
the trajectory is bit-identical with the pitch series exactly negated, proving
it is a reporting change and not a physics change.

Converting at the seam only, and letting the two conventions coexist, was
considered and rejected: one repo, one convention. That is the trap ICD §10
exists to prevent.

## 3. Measured behaviour

Impulse sweep, open-loop, forward, through the CoM:

| Impulse (N·s) | Peak pitch | Nose strike | t_strike | Speed at strike | Travel |
|---:|---:|:---:|---:|---:|---:|
| 0 | 0.00° | no | — | — | 0.00 m |
| 6 | 9.04° | no | — | — | 1.34 m |
| 10 | 15.11° | no | — | — | 2.23 m |
| 12 | 18.18° | no | — | — | 2.68 m |
| **12.5** | 18.55° | **yes** | 2.02 s | 0.57 m/s | 2.73 m |
| **20** | 18.64° | **yes** | 1.65 s | 1.00 m/s | 4.03 m |
| 30 | 18.66° | yes | 1.51 s | 1.48 m/s | 6.10 m |

The threshold is a **sharp knee at ~12.5 N·s**, and right at it the nose grazes
the ground — the strike boolean genuinely flickers there (12.5 strikes, 13.0
misses by micrometres, 13.5 strikes). A gate parked on that knee would be
flaky, so the chosen constants sit well clear of it:

- `NOMINAL_IMPULSE_NS = 20.0` — ~60% above the knee. On 12.5 kg that is a
  1.6 m/s Δv, a firm shove.
- `SUBTHRESHOLD_IMPULSE_NS = 6.0` — peaks at 9.0°, under half the strike angle.

Peak pitch saturates at the strike angle by construction, so **severity** is
carried by `speed_at_strike_ms` and `pitch_rate_at_strike_dps`, which grow
monotonically with impulse. Those are the metrics to watch, not peak pitch.

## 4. Acceptance criteria and how they are enforced

| Criterion | Enforced by |
|---|---|
| No rider mass / mast | `test_no_rider_proxy_bodies` |
| Real onewheel geometry, believable mass | `test_mass_is_a_plausible_driverless_board` |
| Stable at rest — CoM below the axle | `test_centre_of_mass_sits_at_or_below_the_axle` |
| Undisturbed board does not topple | `test_board_rests_upright_with_no_disturbance` |
| Nominal impulse causes a nose strike | `test_nominal_impulse_causes_a_nose_strike` |
| **The impulse is what causes it** | `test_subthreshold_impulse_does_not_topple` |
| Deterministic and repeatable | `test_repeat_runs_are_bit_identical` |
| Strike angle is geometry-derived | `test_strike_angle_is_geometry_derived_and_documented` |
| Closed-loop seam exists | `test_controller_hook_is_wired_to_the_motor` |
| Scripted, produces logged artifacts | `scripts/render_scenario.py` + CI |

The positive case is always paired with a negative one. A suite that only
checked "big kick → falls over" would still pass against the old
falls-over-by-itself model — which is the exact bug being retired.

## 5. Determinism

MuJoCo is bit-reproducible for a fixed version on a fixed platform, so
`requirements-sim.txt` pins the toolchain exactly; the version is part of the
fixture, not an implementation detail. Repeatability is asserted as
**bit-identical same-machine repeat runs**. The CI gate asserts **metrics
against thresholds with margin**, not against stored baselines, because MuJoCo
is *not* bit-portable across platforms — a baseline would be a cross-platform
flake generator.

## 6. Artifacts and CI

The physics gate (`sim` job) runs on every push and PR, needs no GL, and takes
~2 s. Rendering is a separate mainline-only job so a broken graphics stack can
never change a pass/fail.

`scripts/render_scenario.py` **replays the captured trajectory** rather than
re-stepping the physics, so the film is provably of the same run the metrics
describe, and GL stays out of the physics path entirely.

| Artifact | Purpose |
|---|---|
| `impulse_open_loop.mp4` | H.264, 1280×720 @ 30 fps — what the landing page embeds |
| `impulse_open_loop.webm` | VP9 fallback |
| `impulse_open_loop.gif` | short loop for the README |
| `impulse_poster.jpg` | video poster (clean frame, no callouts) |
| `impulse_pitch.png` | pitch + travel plot; needs no GL |
| `impulse_metrics.json` | full metrics — the CI gate reads this |
| `sim-run.json` | flat provenance for the landing-page caption |

CI publishes these to a rolling **`sim-latest`** GitHub Release, giving one
stable URL per artifact. The landing page fetches from there at deploy time, so
no binary ever enters the web repo's git history. Publishing is gated on the
physics gate passing — a clip is never published for a commit whose physics
failed.

## 7. Out of scope

- Ridden / rider-coupled dynamics → D4.
- Lateral (roll) disturbances and turn-rate response → follow-on.
- The controller itself — this defines the **test**, not the control law.
- MCAP logging → deferred; the trajectory is currently carried in
  `ImpulseResult` and summarised to JSON. Adding an MCAP writer is a
  self-contained follow-on and does not change the gate.
