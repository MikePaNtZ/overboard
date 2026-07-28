# Runbook — Stage 0B: the procedure the Pi executes, and the data it returns

<!--
covers:
  - scripts/stage0b_runbook.py
  - tests/test_stage0b_runbook.py
reconciled: 49a2cfc
-->

**Owned by: Senior Controls** (issue #27 — O4 of the interim roadmap; explicit assignment,
see "Turf" below). **Executed by: the Pi**, not a human at the bench. Stage 0A
(`docs/runbook-stage0a-bench.md`) is a checklist a person follows with a multimeter and a
kill switch. This is its Stage 0B counterpart: the Pi image (O3, issues #51/#52) is the
vehicle, and this is the payload it runs once it exists — an ordered, machine-executed
procedure that produces one log per run instead of one person's notes.

**Same physical rig as Stage 0A** — the 6374 bench motor, CANable, and the Pi as the Linux
host. Stage 0A proves the rig can spin and be measured by hand; this procedure is what the Pi
runs automatically once it can, and it adds the one measurement Stage 0A cannot make:
command→actuation latency, gated by real-time scheduling this rig does not yet have running.

---

## Why a procedure, not a script dump

Each step below states three things:

- **Purpose** — what the step is trying to establish.
- **Falsifies** — the specific observation that would prove the step did NOT establish it.
  A step with no falsifying observation is theatre, not a test.
- **Pass / fail** — the numeric or boolean condition evaluated against the step's result.

Steps run in the stated order because each one is a precondition for the next: there is no
value in measuring current-step response through a CAN link that has not been proven to
round-trip a frame, and no value in doing that unpowered check on wiring that has not passed
continuity.

---

## ⚠️ Abort criteria — stated here, before step 4 (the first powered step)

Per the project safety rule (hardware deadman in series with motor power), the following abort
the run **immediately**, regardless of which step is in progress:

- The deadman is opened, at any point, by anyone. This is a hardware interrupt, not a
  procedure step, and it pre-empts everything below.
- Step 1 (continuity/insulation) fails. Powering a shorted or leaky circuit is not
  diagnosable from software.
- Step 2 (unpowered CAN round-trip) fails. A bus that cannot be trusted unpowered cannot be
  trusted with a motor attached.
- Motor detection (Stage 0A §5) has not previously completed with halls found on this exact
  hardware assembly. This procedure does not re-run detection; it assumes Stage 0A's gate
  already passed and refuses to proceed if the log for that gate is missing.
- Any commanded current step produces a measured current that does not settle within
  `2 × current_loop_tau_s` of commanding zero. An actuator that will not release the disc on
  command is the one failure this project cannot tolerate quietly.

An aborted run is still **logged** (see §Log schema) with `"aborted_at_step"` set — a run that
stopped is data, and silently discarding it hides exactly the failure the abort exists to
catch.

---

## The procedure

### Step 1 — Continuity and insulation check
**Purpose:** confirm the assembly matches Stage 0A §4 (no phase-to-phase or phase-to-frame
short) before any power is applied.
**Falsifies:** measured resistance below the insulation-tester's fault threshold on any
phase-to-phase or phase-to-frame pair.
**Pass/fail:** pass iff every pair reads open circuit (phase-to-phase) or above the tester's
insulation-fault threshold (phase-to-frame).
**Hardware-only** — no sim equivalent. A simulated harness has no frame to short against.

### Step 2 — Unpowered CAN round-trip
**Purpose:** confirm the CAN transport (CANable, wiring, termination) carries a frame
correctly before anything downstream depends on it.
**Falsifies:** a frame sent is not received, or is received corrupted (wrong ID, wrong DLC,
wrong payload bytes).
**Pass/fail:** pass iff a loopback frame round-trips byte-identical.
**Covered separately, not re-derived here:** the round-trip fidelity claim (including sign
and scaling survival) is the acceptance target of the `can-harness` crate (issue #52), which
proves it on a virtual CAN bus with no hardware required. This step is that same check, run
against the real bus. This runbook does not re-implement it — it depends on `can-harness`
passing first, once merged.

### ⚠️ Abort gate — see "Abort criteria" above. Do not proceed past this line without it read.

### Step 3 — First powered spin (deadman in hand)
**Purpose:** confirm the assembly survives being powered and commanded at all, before trusting
any measurement it produces.
**Falsifies:** controller does not power up on deadman-close, does not respond to a CAN
command, or does not die on deadman-open (re-run of Stage 0A §2's three-cycle test, because a
harness change since 0A invalidates that gate).
**Pass/fail:** pass iff power-up, command-response and deadman-kill all confirm.
**Hardware-only.**

### Step 4 — Current-step response
**Purpose:** characterise the current loop's step response — first-order lag time constant —
against the sim's prediction, on the real drive.
**Falsifies:** measured current does not settle monotonically toward the commanded value, or
settles to a time constant more than 3× the sim's `current_loop_tau_s` prediction (a gross
mismatch means the sim's imperfection profile is not representative, which is exactly the
kind of gap this runbook exists to catch before it reaches the board).
**Pass/fail:** pass iff settle time is within the stated band at each of three commanded
amplitudes, both directions (mirrors Stage 0A §6c).
**Dry-runnable against the sim** — see §Dry run.

### Step 5 — Coast-down
**Purpose:** measure bearing/seal/cogging friction via speed decay, as an independent check
that nothing changed mechanically since Stage 0A.
**Falsifies:** decay parameters (`b`, `tau_c`) drift by more than the Stage 0A measurement's
stated fit uncertainty — a drift means something in the assembly moved or degraded between
sessions.
**Pass/fail:** pass iff fit `R²` clears the Stage 0A acceptance threshold and both fitted
parameters fall within tolerance of the Stage 0A baseline.
**Dry-runnable against the sim** — see §Dry run.

### Step 6 — Command→actuation latency measurement (the Stage-0 go/no-go)
**Purpose:** the reason Stage 0B exists — measure the time from a CAN command frame leaving
the Pi to the drive actually changing current, under representative CPU and network load.
**Falsifies:** p99.9 latency exceeding 1 ms, or any single sample exceeding 2 ms, over the
pre-registered ≥10⁵-cycle run.
**Pass/fail:** this project's own AC-6 (`docs/design-pi-image-stage0b.md`, §"Numeric
acceptance criteria"), reused verbatim here rather than re-derived: **Go** if p99.9 ≤ 1 ms
*and* max ≤ 2 ms over ≥10⁵ cycles under representative load; any max > 2 ms escalates as an
architecture finding rather than being tuned away.
**Hardware measurement — a real jitter distribution needs a real RT kernel and a real bus.**
**Dry-runnable against the sim only as a determinism/schema check** — see §Dry run for exactly
what that does and does not prove.

---

## Log schema

One JSON document per run, uploaded and traceable to the exact code that produced it. Fields:

| Field | Type | Meaning |
|---|---|---|
| `schema_version` | int | `1`. Bump on any incompatible field change. |
| `stage` | string | `"0B"`. |
| `git_sha` | string | Short SHA of the tree that produced the run, `-dirty` suffixed if uncommitted changes were present — same convention as `scripts/experiment.py`. |
| `mode` | string | `"hardware"` or `"dry_run"`. A dry run against the sim must never be mistaken for a hardware result downstream. |
| `aborted_at_step` | string \| null | Step name if the run aborted, else `null`. |
| `steps` | array | One entry per step actually run, in order. |

Each entry in `steps`:

| Field | Type | Meaning |
|---|---|---|
| `name` | string | e.g. `"current_step_response"`. |
| `purpose` | string | One-line restatement, for a log read without this doc open. |
| `result` | string | `"pass"`, `"fail"`, or `"skipped"` (skipped = hardware-only step, not run in `dry_run` mode). |
| `measured` | object | Step-specific numeric results (e.g. `{"tau_s": 0.00098}`). |
| `falsified_by` | string \| null | If `result == "fail"`, which falsifying condition fired. |

This schema is deliberately small. It is the shape a log needs to be re-derivable and
attributable to code, not a full telemetry format — the high-rate signal (current, speed,
CAN frames) belongs in the project's MCAP log per the engineering conventions, referenced by
run ID from this document, not duplicated into it.

---

## Dry run against the sim

`scripts/stage0b_runbook.py` runs steps 4, 5 and 6 against the sim so the procedure — the
step ordering, the schema, the pass/fail evaluation — is debugged before any hardware exists,
per the acceptance criterion. It reuses the Stage 0A bench-rig identification code
(`sim.scenarios.bench_spinup.identify` / `.spindown`, Sr. Mechanical & Systems' turf — read
only, no changes made here) rather than re-deriving the physics, and the Stage 0B
imperfection profile (`sim.scenarios.imperfections.STAGE0_PLACEHOLDER`) for the latency
proxy.

**What the dry run does NOT prove:** step 6's sim proxy finds the first instant current departs
from zero after a step command, under `STAGE0_PLACEHOLDER`'s `actuation_delay_s` (a pure
transport delay, no jitter source). It is a schema/plumbing check — does the pass/fail logic
correctly evaluate a `(p99.9, max)` pair against AC-6's thresholds — and explicitly not a
substitute for a real jitter distribution, which needs a real RT scheduler, a real bus and a
real load generator. `STAGE0_PLACEHOLDER`'s own docstring already carries this caveat: its
delay and current-loop-lag constants are marked `PROVISIONAL`, no Stage-0A current-loop data
exists yet to fit them against. The dry run inherits that honesty rather than hiding it.

**Finding, flagged rather than fixed here:** step 5's dry run runs the `IDEAL` profile, not
`STAGE0_PLACEHOLDER`. `bench_spinup.spindown()` (Sr. Mechanical & Systems' turf) fits its
decay window starting the instant current is commanded to zero, with no equivalent of
`identify()`'s data-driven `settle_time_s` to skip the actuation-delay + current-loop-lag
transient at the start of the decay. Under `STAGE0_PLACEHOLDER` that transient corrupts the
least-squares fit across the whole window (R² collapsed to ~0.002 in testing — noise, not a
curve). `IDEAL` is the only profile `spindown()` is actually validated against today
(`tests/test_bench_spinup.py::test_spindown_recovers_friction_terms_on_ideal_run`, R² > 0.999).

Steps 1–3 are hardware-only and report `"skipped"` with no falsifying claim in dry-run mode.

```sh
python3 scripts/stage0b_runbook.py --dry-run --out sim/out/stage0b/dry-run.json
```

Determinism: like every scenario run in this project, no wall clock is read for the numeric
result — the sim steps are seeded and reproducible bit-for-bit (`tests/test_stage0b_runbook.py`
pins this). The JSON's `git_sha` field is the one piece of real-world state the dry run
records, exactly as `scripts/experiment.py` does.

---

## Turf

`docs/` is COO turf (`CODEOWNERS`). Issue #27 explicitly assigns this runbook to Senior
Controls ("Owner: Senior Controls", "Dispatch: cron OK") — same shape as issue #32's override
for `docs/design-pi-image-stage0b.md`.

TURF-OVERRIDE: docs/ is COO turf; issue #27 assigns this runbook to Senior Controls.

## What this runbook does NOT establish

Same discipline as Stage 0A §7: this is the **bench** motor's current loop and the **bench**
rig's friction, not the board's. What transfers to the board is the imperfection profile
(actuation delay, current-loop lag) and the CAN transport's proven behaviour — properties of
the shared controller + CAN + host path, not of the machine at either end.

**Next:** once AC-6 passes on real hardware under this procedure, Stage 0B's architecture
go/no-go is decided — see `docs/design-pi-image-stage0b.md`.
