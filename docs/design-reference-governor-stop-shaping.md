# Reference governor — making the stop command executable

<!--
covers:
  - crates/sim-host/src/host.rs (CMD_ENVELOPE_RESERVE, CMD_ENVELOPE_RESERVE_BRAKING)
  - sim/models/overboard_rider.xml (wheel_hinge frictionloss/damping)
  - sim/scenarios/plant.py (drag model, Crr)
-->

- **Status:** Design spec — CEO directive. **No implementation in this document.** Written to be
  handed to whoever builds the mechanism; every acceptance number is stated so a reviewer can
  re-take it, not just read it.
- **Owner:** Senior Controls (design); COO for the ADR-0011 exit-criteria bookkeeping this
  eventually feeds.
- **Closes:** none yet — this is the design that would close the "reference governor" line item
  ADR-0011's third ratification names as a candidate remedy, not a decision record itself.
- **Reads:** [ADR-0011](decisions/ADR-0011-hold-the-launch-until-the-board-stops-flipping.md)
  (**especially its 🛑 THIRD RATIFICATION, 2026-08-05** — every pre-08-05 number in that document
  is superseded and none is cited below), `crates/sim-host/src/host.rs` §"ADR-0011: the
  command-envelope reserve", [#235](https://github.com/MikePaNtZ/overboard/issues/235) (regen is
  SOC-limited).
- **Depends on / does not resolve:** [#218](https://github.com/MikePaNtZ/overboard/pull/218)/[#219](https://github.com/MikePaNtZ/overboard/pull/219)'s
  open question of what `CMD_ENVELOPE_RESERVE_BRAKING` should be relative to
  `CMD_ENVELOPE_RESERVE`; [#235](https://github.com/MikePaNtZ/overboard/issues/235) (no SOC/battery
  model exists in this repo — this spec names the interface, not the model); the `KT_NM_PER_A`
  refit in progress on `feat/controls/realistic-motor-torque` (this spec is written entirely as
  formulas over that constant, never a literal, for exactly that reason).

## 0. The problem, in one line

A step command through zero wheel speed is not a hard case for this controller — it is **not an
executable command at all**. Coulomb rolling resistance reverses sign at the zero crossing,
producing a torque discontinuity of `2 × frictionloss` = `2 × 2.368` = **4.74 N·m**
(`sim/models/overboard_rider.xml`'s `wheel_hinge`, corrected-drag-model value, ADR-0011 third
ratification) against a worst-matrix-point headroom of **2.39 N·m** (3.41 A). The jolt is
~2× the entire available headroom, and it lands exactly at the instant a full reverse-to-forward
stick reversal asks the actuator to do it. No amount of P/D retuning fixes an actuator being
asked to jump a hole twice its own width — the fix is upstream, in what gets asked for.

**This document specifies a reference governor**: a stage that synthesises an achievable
deceleration/reversal trajectory from the rider's raw stick, in place of passing that stick
straight to the controller. It does not increase the board's stopping power — the motor's torque
and the battery's regen headroom are whatever they are. It converts an infeasible instantaneous
demand into a feasible time-varying one, and tells the rider it is doing so.

## 1. Intent detection

### 1.1 Reframe: this is feasibility detection, not intent classification

The naive framing — "detect that the rider is asking to stop, as distinct from ordinary
modulation" — invites building a classifier for something psychological (what does the rider
*mean*), which is untestable and the kind of thing that quietly grows a threshold nobody can
defend. The reframe that IS testable: **the governor is always active, and "detecting a stop"
reduces to detecting when the raw command, executed as given, would demand more deceleration
than the actuator can currently deliver.** That is a comparison between two numbers computed
every cycle, not a discrete yes/no gate on a stick shape.

This also means the mechanism generalises for free: a merely-firm brake (not a full reversal)
gets the same treatment if and only if it is itself infeasible. An achievable brake, however
hard, passes through unshaped — matching the CEO's own framing of the ask ("you should be able
to stop faster by leaning back... but we don't need to overdo it", quoted in
`CMD_ENVELOPE_RESERVE_BRAKING`'s doc comment) and avoiding a governor that second-guesses every
touch of the stick.

### 1.2 The signal, and its state dependence

Reuse the opposition test the braking reserve already defines rather than inventing a second
one — `stick × last_forward_speed_m_s < 0`, gated above `BRAKING_RESERVE_MIN_SPEED_M_S` (0.25
m/s) so the launch-rock transient (a full-stick standing start rocks the board to −0.085 m/s for
~0.6 s before it moves off) never gets misread as a stop. Three states, not two:

| State | Condition | Governor action |
|---|---|---|
| **Standing start** | `\|v\| < BRAKING_RESERVE_MIN_SPEED_M_S` | Not this mechanism. Ordinary `CMD_ENVELOPE_RESERVE` (accelerating) path, unchanged — there is no stop to make, the board is already stopped. |
| **Achievable modulation** | `stick × v < 0` (braking), and the raw command's implied deceleration `a_raw` ≤ `a_ceiling(v, grade, SOC)` (§2) | Pass through, currently-shaped by `CMD_ENVELOPE_RESERVE_BRAKING` as today. The governor computed the ceiling and found the ask inside it — nothing to synthesise. |
| **Infeasible stop/reversal** | `stick × v < 0`, and `a_raw > a_ceiling(v, grade, SOC)` by more than a stated margin `ε` | **Governed.** The raw command is replaced by the synthesised profile (§2) until the trajectory converges back to the raw ask or the rider releases the stick. |

`a_raw` is the deceleration the raw stick asks for, read off the same
`PEAK_DEMAND_A_PER_UNIT_STICK`-style mapping the command path already measures (a stick fraction
maps to a peak current demand, which maps to a torque, which maps to a deceleration through
`m_total`) — no new measurement is introduced to get it, only a comparison against the
ceiling this document derives in §2.

The margin `ε` exists so the governor does not chatter at the boundary; a value is a bench/sim
tuning question, not asserted here.

### 1.3 False positive

The governor engages on a command that was, in fact, achievable — because `a_ceiling` was
under-estimated (e.g. a conservative regen-headroom placeholder, §2.4) or because of measurement
noise near the boundary. **Cost:** the rider's brake feels slightly sluggish for the duration of
the jerk-limited ramp-in (§2), instead of the instantaneous response an unshaped command would
have given, for an input that turns out to have been fine. Bounded and falsifiable — AC-G5 (§6).
This is the safe direction to be wrong in.

### 1.4 False negative

The governor fails to engage on a genuinely infeasible full-reversal-at-speed — detector bug, a
stale `a_ceiling` computation, or an over-generous regen-headroom placeholder. **This is the
dangerous direction**, and it is the one ADR-0011 exists to close. The mitigation is not "make
the detector more conservative" (that just moves the false-positive/false-negative trade,
doesn't remove the risk) — it is that a false negative here reproduces exactly today's already-
analysed, already-measured risk (ADR-0011's own criterion (a)-2 failure), **not something new**,
because the governor sits upstream of the static reserve and the hard envelope clamp, both of
which still apply unconditionally regardless of whether the governor fired (§3, §7). A false
negative is a regression to the known baseline, not a new failure mode.

## 2. The feasible profile

### 2.1 Shape: jerk-limited, with a deliberate near-zero dwell at the crossing

A trapezoidal-acceleration (S-curve) profile — ramp the commanded deceleration from whatever is
currently in force toward `a_ceiling(t)` at a bounded jerk `j_max`, track the (falling) ceiling
as speed drops, and ramp the commanded torque itself toward **zero, not toward the reversed
sign**, as `v → 0`.

That last clause is the load-bearing design choice and is worth stating plainly, because "S-curve
to the target" alone does not fix the defect. The discontinuity is not merely large — it is a
**sign flip in what the wheel's own friction is doing**, independent of what the controller
commands. Ramping a smooth curve *through* zero still asks the actuator to supply a specific
signed torque at the exact instant friction flips underneath it; the 4.74 N·m jump is a property
of the plant, not of how smoothly the reference approaches it, and no jerk limit on the reference
removes it by itself.

What removes the *demand* on the actuator at that instant is commanding **near-zero net torque**
in a narrow dwell band around the crossing (a scale to be set at the bench, comparable to the
0.25 m/s used elsewhere in this pipeline to define "not moving"), letting the board coast through
on residual momentum plus whatever the (now near-zero-torque) friction does on its own, and only
resuming a driven torque command — in the new direction, as its own jerk-limited ramp, subject to
the *accelerating* envelope reserve like any standing start — once `|v|` is back below the dwell
threshold. This turns one infeasible instantaneous command into two sequential, ordinary,
already-analysed manoeuvres: **a deceleration to rest, then a standing start in the opposite
direction** — the second of which is exactly the case `CMD_ENVELOPE_RESERVE` (not the braking
sibling) already governs.

### 2.2 The deceleration ceiling is computed, not fixed

`a_ceiling` is a force balance at the wheel, evaluated every cycle from the current speed,
grade and (regen-headroom) state — not a constant, and not the comparable-board figure
(3.15–3.37 m/s², §6) baked in as a target. That figure is a *cross-check on the arithmetic
below*, not an input to it.

```
tau_brake_avail(SOC)  = KT_NM_PER_A · MAX_CURRENT_A · R_regen(SOC)         [N·m, motor/battery-limited braking torque]
F_brake_avail(SOC)    = tau_brake_avail(SOC) / r_wheel                      [N]

F_roll(v)             = Crr · W · sign(v)                                  [N, Coulomb — CONSTANT in magnitude, discontinuous in sign at v = 0]
F_aero(v)             = 0.5 · rho_air · CdA · v² · sign(v)                  [N, quadratic — vanishes as v -> 0]
F_grade(theta)         = m_total · g · sin(theta)                          [N, +downhill assists forward motion, i.e. OPPOSES deceleration]

a_ceiling(v, theta, SOC) = [ F_brake_avail(SOC) + F_roll(v) + F_aero(v) − F_grade(theta) ] / m_total
```

`KT_NM_PER_A`, `MAX_CURRENT_A`, `Crr`, `r_wheel` (`DEFAULT_R_EFF_M`), `CdA` and `m_total` are the
same named constants the rest of this codebase already carries (`board-types::Params`,
`sim/models/overboard_rider.xml`'s `wheel_hinge` comment, ADR-0011 third ratification's corrected
drag model). **None is restated as a literal in this document** — `KT_NM_PER_A` in particular is
mid-correction downward from 0.70 on `feat/controls/realistic-motor-torque`, and a formula is
right regardless of which fitted value lands; a hardcoded number in a design doc would not be.

Two consequences worth stating because they are counter-intuitive:

- **The ceiling itself shrinks as the board slows**, on top of the rider's raw ask usually
  shrinking too (a rider easing off as they approach a stop). Near `v = 0`, `F_aero` vanishes
  (it is exactly the term that "helps" least when help is needed least, per the third
  ratification's own finding about the old drag model) and `F_roll`'s *sign* is about to flip —
  the ceiling is not a flat number the profile decays gently toward, it is falling out from
  under the profile at the same time, and §2.1's dwell exists because of that, not despite it.
- **`F_grade` can make the ceiling negative** — no amount of motor torque decelerates a board on
  a downhill steep enough that gravity alone exceeds the available braking force. That is a real
  physical limit, not a defect in this formula, and the governor's job in that regime is to
  report it (§4), not to promise a stop it cannot deliver.

### 2.3 `j_max` is internally derived, not borrowed

There is no published jerk limit for a standing rider on a self-balancing platform. The bus/
shuttle figures (0.3–0.9 m/s³) are for passengers who are a passive load holding a rail; here the
rider **is** the balance loop — a jerk in commanded deceleration is a jerk in required lean rate,
which is a demand on the same pitch-rate channel (`KD`) ADR-0011 already found undersized
(criterion (a)-3, 201°/s of imparted pitch rate against a ~76°/s KD channel — moved to the
hardware gate, not solved). **`j_max` must be set by sim sweep against that KD channel and
validated on the bench once hardware exists; dressing a shuttle number as authority here would
repeat exactly the mistake ADR-0011's own second ratification found in the estimator bias — a
number from an unrelated domain, accepted because it was the only one available.** No value is
proposed in this document.

### 2.4 Regen headroom: named as an input, not modelled

`R_regen(SOC) ∈ [0, 1]` above is a **placeholder interface, not a function this repo can write
today.** There is no battery or state-of-charge model anywhere in this codebase (confirmed:
no `SOC`/`state_of_charge`/`regen` symbol exists outside issue #235 itself). Per this task's own
instruction and this repo's standing rule: where a number is unknown, say so and name the
measurement, don't invent one.

- **What is documented, not measured on our hardware:** production Onewheel-class boards ship
  "Full-Battery Pushback" and warn against descending a long hill on a fresh charge — the
  manufacturer's own evidence that `R_regen` is not 1 near full charge.
- **What this spec requires of the interface, regardless of the function's shape:** `R_regen`
  must be a first-class input to `a_ceiling`, not folded into a fixed torque number, so that when
  a SOC (or SOC-proxy) signal exists the governor consumes it without a redesign. Until then,
  **`R_regen ≡ 1` is the only honest default** — it is not a safety margin, it is an admission
  that the model does not yet know better, and it must be documented at every call site as
  optimistic, not conservative. A governor computing an achievable profile from a torque-only
  ceiling is *wrong in the unsafe direction* on a fresh-charge downhill — issue #235's own
  words — and this spec does not pretend otherwise.
- **What would settle it:** a bench measurement of regen current acceptance vs. pack voltage/SOC
  on the actual pack, or, short of that, adopting the manufacturer's documented ~90–100% SOC
  threshold as a stand-in curve — flagged there as *their* measurement, not ours, if adopted
  before ours exists.

## 3. Relationship to the existing reserve — the governor sits UPSTREAM, and does not replace it

`CMD_ENVELOPE_RESERVE`/`CMD_ENVELOPE_RESERVE_BRAKING` are, in the Oracle's own framing, the
**zeroth-order version of this idea**: a single static scalar multiply on the stick, applied at
the point where a stick value enters the host, before anything else touches it
(`crates/sim-host/src/host.rs`, "Deliberately placed HERE... upstream of everything"). The Oracle
ruled the mechanism **KEPT**, invariant restated as **peak demand ≤ stated fraction × envelope**
(ADR-0011 third ratification, "What survives from the second ratification"). This spec does not
propose deleting it, and could not honestly claim to improve on a mechanism by removing its own
backstop.

**Composition: the governor sits upstream of that multiply, replacing the raw stick with a
synthesised reference during a governed episode (§1.2); the static reserve then applies to
WHATEVER the governor outputs, exactly as it applies to the raw stick today.** Concretely, in
`host.rs`'s own pipeline terms: `shape_fore_aft_command_directional` still runs, unconditionally,
on the value the governor produces, not on the raw wire value it produces during a governed
episode. Two consequences:

1. **The governor does not get its own escape from the invariant.** If the governor's own
   arithmetic (§2.2) is wrong — a bad `R_regen` default, a stale speed reading, a bug — the
   output still cannot exceed `stated fraction × envelope`, because that clamp is unconditional
   and does not know or care that a governor exists upstream of it. This is the concrete
   mechanism behind §1.4's false-negative claim: a broken governor degrades to today's baseline,
   not below it.
2. **The static reserve is not made redundant, even where it currently saturates to a no-op.**
   At the 60 A / 42 N·m envelope, `CMD_ENVELOPE_RESERVE` is currently 1.00 — the founding premise
   (full stick over-commands the actuator) is gone at that envelope, so today the multiply does
   nothing on the accelerating side. That is a fact about the CURRENT fitted constants, not a
   property of the mechanism, and `KT_NM_PER_A`'s in-flight downward correction could reinstate
   a sub-1.00 value at any time — the governor's design must not assume the static reserve is
   inert. `CMD_ENVELOPE_RESERVE_BRAKING` (0.90) is unaffected by any of this and continues to
   apply on top of the governor's output during braking, per its own doc comment.

**This is wrap, not subsume.** The governor is the dynamic, state-aware layer that decides *what
trajectory to ask for*; the static reserve remains the always-on, state-blind layer that bounds
*how much of the envelope any single instant's ask may spend*, regardless of which upstream
mechanism produced that ask. Whether `CMD_ENVELOPE_RESERVE_BRAKING` should itself be retired,
raised to 1.0, or left as is once a governor exists is the open question already on record at
#218/#219 and is explicitly **not resolved by this document** — the governor's composition rule
above is correct at any value that constant ends up taking.

## 4. Rider feedback

Silent shaping is the documented failure. The manufacturer's own guidance for these boards is
that the rider's correct response to a limit is to **lean back harder**; the fatal mistake filed
against the 2023 recall (~300,000 units, at least 4 deaths, MDL 3087 — allegations, not
adjudicated fact) is leaning forward to fight it. A governor that quietly re-shapes the rider's
command without telling them *manufactures the exact ambiguity that produces the wrong reflex*:
the board feels less responsive than the rider asked for, and an unadvised rider's instinct is to
push harder into the direction that isn't working.

**Requirement, not a nicety:** whenever the governor is actively de-rating the raw ask — `a_raw −
a_commanded > δ` for some stated `δ`, i.e. §1.2's "governed" state — the board must surface that
fact through some channel (audible, haptic, visual; the modality is a hardware-gate decision, not
this document's) for as long as the episode lasts, and the signal must be measured for lead time
the same way criterion (c)'s loss-of-authority warning is (§5).

### The shared-fate rule, addressed directly

The litigation's central engineering allegation is that the warning mechanism on the recalled
boards draws on the same depleting motor/battery resource whose exhaustion causes the nosedive —
so it degrades exactly when the rider needs it, and the 2023 remedy allegedly inherits the same
flaw. **Design rule this spec adopts: the governor's rider-facing feedback signal's own
actuation must not share a resource ceiling with the thing it is warning about.**

- **Today, in sim, this is close to free to satisfy**: the feedback signal is a log/telemetry
  event with no torque or current cost of its own, so there is no shared resource to speak of.
  That is a property of the sim being software-only, not a design achievement — flagged so it is
  not mistaken for one.
- **At the hardware gate this becomes a real constraint, not solved here**: whatever produces
  the rider-facing signal (a light, a buzzer, a haptic pulse) must be verified to remain
  functional down to whatever SOC/voltage floor the DRIVE electronics become unusable at — ideally
  on a separate, low-current rail from the traction pack's discharge path, or independently
  proven to survive to a lower floor than the motor does. Naming this as a hardware requirement is
  as far as this document goes; it does not pick the modality or the rail.

### Two distinct signals, not one

The governor's feedback (`a_raw − a_commanded`, "how much I am de-rating your ask") is not the
same measurement as criterion (c)'s loss-of-authority warning (filtered `|proposed current| /
MAX_CURRENT_A`, "how close the actuator is to its own ceiling"). They correlate but are not
interchangeable, and §5 depends on keeping both.

## 5. Instrumentation — do not quietly re-break criterion (c)

Criterion (c)'s warning is filtered `|proposed_amps| / MAX_CURRENT_A` crossing
`AUTHORITY_UTILISATION_WARN` (0.85), gated below `SPEED_CAP_ONSET_M_S`
(`crates/sim-host/src/host.rs`, `authority_warning_active`). Critically, `proposed_amps` is read
**downstream of whatever shaping already ran on the stick** — it is the regulator's response to
the ALREADY-SHAPED command, not to the raw wire value. A governor that succeeds at its own job —
never letting the commanded reference exceed `a_ceiling` — will, by construction, keep
`proposed_amps` away from `MAX_CURRENT_A` and therefore keep filtered utilisation away from 0.85.
**A working governor turns off the only instrument this repo currently has on the approach to the
cliff**, which is precisely the risk this task calls out: preventing saturation is not the same
thing as removing the need to see how close saturation was.

**Requirement: the governor must emit its own utilisation-style signal, symmetric in form to the
existing one, so the same discipline (filtered, thresholded, edge-triggered, lead-time-measured)
applies to it.** Concretely:

- **`governor_active` (bool, rising/falling-edge logged)** — §1.2's "governed" state, the same
  discipline `prev_authority_warning` already uses so the log announces state changes rather than
  spamming every tick.
- **`governor_margin` = `(a_ceiling − a_commanded) / a_ceiling`**, filtered the same way
  `utilisation_filtered` is (§ AUTHORITY_UTILISATION_TAU_S is a reasonable starting point for the
  time constant; re-derive if the governor's own dynamics warrant a different one) — this is the
  "how close to the actuator's real edge is the SHAPED trajectory running" signal that
  `utilisation_filtered` used to be the only proxy for. `governor_margin → 0` is the governor
  itself approaching saturation of the ceiling it computed, which is the new "cliff" once the old
  one stops firing.
- **`governor_derate` = `a_raw − a_commanded`** — the §4 rider-feedback quantity, logged
  regardless of whether it crosses any threshold, so a post-hoc trace can show exactly how much
  any given episode was shaped by.

None of this replaces criterion (c) — `authority_warning_active` still runs, unconditionally, on
whatever the governor outputs, for the same reason §3's static reserve still runs on it. If the
actuator ever does approach saturation despite the governor (a false negative, an underestimated
disturbance, a wrong `R_regen` default), criterion (c) must still fire. The two signals answer
different questions — "is the actuator near its ceiling" vs. "is the governor near the ceiling it
computed" — and a review that only reads one after this ships has lost information the other
used to carry alone.

## 6. Acceptance criteria — numeric and falsifiable

Every criterion below names the scenario/harness that re-takes it. Per this repo's standing rule,
an acceptance number nobody else can re-take is not a measurement.

- **AC-G1 (feasibility held).** Over ADR-0011's test-matrix entry (a)-2 (full reverse-to-forward
  stick reversal at speed), run through `--scripted-scenario` with the governor engaged: at every
  tick, `a_commanded ≤ a_ceiling(v, grade, R_regen≡1)` computed independently by the check script
  from the same trace. Falsifiable by re-running the scenario and re-computing `a_ceiling` from
  the logged `v`/`grade` columns against the logged commanded torque.
- **AC-G2 (the board does not invert).** Re-run the `Crr` sweep ADR-0011's third ratification
  used to show criterion (g) fails at every swept value (§"Criterion (g): FAILS") with the
  governor engaged, same sweep band. Falsifiable: pass/fail is "does the board invert", exactly as
  measured before — this is the same harness, governor on vs. off.
- **AC-G3 (stopping performance, quoted not claimed).** From a stated cruise speed (name it at
  measurement time; ADR-0011's own saturation point, ~6.5 m/s, is a defensible choice for
  comparability), measured mean deceleration to rest. **Reported against the comparable-board
  band (3.15–3.37 m/s²) as a cross-check on the arithmetic, not asserted as a target the design
  promises to hit** — `a_ceiling` is whatever the real torque/drag/grade/regen numbers say it is;
  if the measured figure lands outside that band, that is a finding about this board's actual
  authority, not a governor defect, and must be reported as such.
- **AC-G4 (criterion (c) survives).** Same scenario as AC-G1, with `AUTHORITY_UTILISATION_WARN`
  instrumentation left running unmodified. Either it fires with lead time ≥ the pre-governor
  measured figure (re-measure against the third-ratification-corrected model; the old 2.69 s/
  2.868 s figures are superseded per §0), **or**, if the governor keeps utilisation below 0.85
  entirely, `governor_margin` (§5) must cross an equivalently-defensible threshold with a
  reported, re-derivable lead time of its own. A run with neither signal firing anywhere near the
  reversal is a fail regardless of whether the board stayed upright.
- **AC-G5 (false-positive cost bounded).** For a braking input measured to be inside `a_ceiling`
  at every tick (an "achievable" brake, §1.4), the governed and ungoverned traces' time-to-
  target-speed differ by no more than a stated bound (bench/sim tuning value, named at
  measurement time, not asserted here) — falsifiable by running the same input through both
  paths and diffing the traces.
- **AC-G6 (false-negative backstop, fault-injected).** With the governor's intent detector
  disabled or forced to `governor_active = false` unconditionally (a deliberate fault injection,
  not a normal run), peak commanded current over the reversal-at-speed scenario must still not
  exceed `stated fraction × envelope` any more than it does on the current, un-governed baseline
  — i.e. breaking the governor must be provably unable to make the actuator demand worse than
  today's already-measured (and already-failing, per ADR-0011 third ratification) baseline.
  Falsifiable: same scenario, governor forced off, compare peak current against the pre-governor
  trace.
- **AC-G7 (regen/SOC — explicitly deferred, not claimed).** No acceptance number is stated for
  downhill/high-SOC braking performance. This is named as a **blocking prerequisite** on any
  public claim about stopping distance under those conditions, per issue #235, until a measured
  or manufacturer-sourced `R_regen(SOC)` curve replaces the `R_regen ≡ 1` placeholder (§2.4).

## 7. Failure modes and explicit scope

**What this design does NOT do.** Shaping the command does not make the board stop faster. The
motor's torque and the battery's regen headroom are what they are; the governor spends the same
budget more honestly instead of demanding it all at once through a discontinuity. If `a_ceiling`
itself is inadequate for the disturbance encountered — a steep enough downhill on a full pack, a
kerb strike mid-stop — the board can still invert, exactly as ADR-0011's criterion (a)'s
kerb-strike entry already establishes independent of this mechanism. **The rider must still be
told** (§4) that the board is at its ceiling, not led to believe a governed stop means margin
exists when it does not.

**If the governor itself misbehaves:** fail toward the known baseline, not toward an unanalysed
new behaviour. Concretely, on detector faults, stuck state, or a trajectory that has diverged from
the raw command by more than a stated bound for more than a stated duration, the governor must
disengage (`governor_active = false`, pass the raw — still statically-reserved — stick straight
through) rather than continue commanding a trajectory nobody is validating in real time. §3's
composition rule is what makes this safe rather than merely convenient: disengaging returns
exactly to today's already-measured, already-documented risk profile, not to an unknown one.

**Explicitly out of scope for this document:**

- **A battery/SOC model.** `R_regen(SOC)` is named as an interface (§2.4); building it is
  issue #235's, not this design's.
- **What `CMD_ENVELOPE_RESERVE_BRAKING` should be once a governor exists.** Open at #218/#219;
  §3's composition rule holds at any value.
- **Setting `j_max`, `ε` (§1.2), `δ` (§4) or the dwell-band threshold (§2.1) to actual numbers.**
  Named as sim-sweep-then-bench-validate quantities; no value is proposed here, per §2.3's own
  argument against borrowing a number from an unrelated domain.
- **Lateral/steering shaping.** Fore/aft only, for the same reason `CMD_ENVELOPE_RESERVE` is
  fore/aft only — this is a pitch-authority failure, and lateral stick consumes no wheel torque
  on this geometry.
- **The wheel-odometry estimator path's coverage of any of this** (#227) and the tilted-ground
  vs. rotated-gravity grade-formulation disagreement (#232, 3.5%) — both pre-existing open
  questions this design inherits rather than resolves; `theta` in §2.2's `F_grade` term is only as
  trustworthy as whichever hill formulation feeds it.
- **The hardware realisation of the §4 feedback channel** — modality, power rail, shared-fate
  verification against the drive electronics' own floor. Named as a hardware-gate requirement,
  not designed here.
