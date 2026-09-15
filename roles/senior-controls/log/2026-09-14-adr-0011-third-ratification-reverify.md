# 2026-09-14 — Re-verify ADR-0011's (f1)/(f2) trim pin and condition-2 inputs against the corrected drag model

Cron dispatch pass. No open issue carries a literal `Owner: Senior Controls` or `Dispatch: cron
OK` tag (confirmed by reading the body of all 23 currently-open issues, not by assumption — the
only explicit `Dispatch:` marker in the queue is `Dispatch: COO only` on #61, reserved). Went by
turf + open-PR-check instead, the convention prior cron passes (#245, #276, the 2026-08-16
pass logged in this same directory) used.

## Why this, not one of the 23 open issues

Every `role:senior-controls`-labelled / clearly-controls-turf open issue was either already
covered by an open PR, blocked, or not cleanly dispatchable:

- **#286, #285, #284, #261, #202, #216, #182, #168, #132** — each already has an open PR
  (#289, #288, #287, #263, #239, #238, #290, #291, #293 respectively). Skipped per instruction.
- **#204** (braking/regen model) — its own text imposes a sequencing constraint: it must land
  *after* the command-envelope reserve and the reset/turn-radius work settle, "because it wants a
  settled baseline to be judged against." The reserve landed (#218/#219) and turn radius landed
  (#269), but reset (#202) is still an **open, unmerged** PR (#239) touching the same
  `host.rs` region. Starting #204 now would run in parallel with #239 against #204's own explicit
  instruction not to. Left for a session after #239 merges.
- **#235** (battery/SOC-dependent regen ceiling) — no numeric acceptance criteria; its four
  "Asks" are themselves open design questions ("decide whether the sim needs a battery model at
  all"), and it depends on #204's regen model existing first (there is currently no regen braking
  in the sim at all for a SOC ceiling to bound). Goal not yet decided, not just the solution —
  correctly a design question, not a dispatchable task yet.
- **#208** (the world-authoring asset-validator, ADR-0011 condition 2) — turf-mine per
  `policy_check.py` (see below), **not blocked**, and has written acceptance criteria. This is
  the closest thing to workable in the queue.
- **#61**, **#33**, **#151** — `Dispatch: COO only` / not dispatchable (a conversation) /
  `role:coo`, resolved (CEO said "Turn it on" in #151's own comment thread; recorded there
  2026-08-01, not yet promoted to an ADR, but explicit and unambiguous).
- **#265, #217, #203, #159, #156, #160, #162, #163** — Digital Content Production / Game
  Engineer / COO / Sr. Mechanical & Systems turf respectively (checked, not assumed — #203 in
  particular asks for a `policy_check.py` change, which CLAUDE.md reserves to the COO).

## The finding that changed the plan

`roles/senior-controls/CONTEXT.md`'s own standing context — read by every Senior Controls
session automatically, before anything else — still frames **ADR-0011's SECOND ratification**
as current, and calls #208 "Not my turf." Both are wrong, and the second one has already cost a
pickup once: the 2026-08-16 log in this directory names "#208 — flagged unowned... explicitly
'Not my turf'" as its own reason to skip it.

**#208 is turf-mine.** `python3 .github/policy_check.py --who sim/scenarios/terrain_validate.py`
and `--who sim/assets/terrain` both return `Senior Controls [Ratified]` via the generic `/sim/`
rule (`CODEOWNERS:78`) — only `sim/models/` geometry/mass/contact/friction and
`plant.py`/`imperfections.py` carve out to Mechanical, and #208 asks for a *validator script*,
not authored geometry.

**More importantly: ADR-0011 has a THIRD ratification (2026-08-06, PR #237)** that
`roles/senior-controls/CONTEXT.md` never mentions. It replaced the drag model (a pure-viscous
term wrong *in shape* — no Coulomb rolling resistance, no quadratic aero) and its own text is
explicit: *"Every quantitative claim in this ADR that predates 2026-08-05 is superseded...
Not 'approximately still valid' — superseded... may not be cited... until re-measured against
the corrected model."* That list names the (f1)/(f2) trim pin outright. #208's condition-2 inputs
(the ~1 mm / 20 mm kerb figures, the incline-tolerance sweep) are exactly the kind of number this
blocks from being cited — so #208, even once correctly identified as my turf, could not be
honestly built on the numbers `tests/test_incline_tolerance.py`'s docstring currently states,
without first checking whether the third ratification actually changed them.

**Nobody had done that check.** `git log` shows 30+ PRs merged since #237 landed, none of them
touching this. The `sim` CI gate (a required check) has in fact been re-running
`tests/test_cmd_envelope_reserve.py` and `tests/test_incline_tolerance.py` on every one of those
PRs — so the re-measurement the ADR demands has mechanically been happening continuously — but
nobody had looked at the result and written down that it still holds. That is the gap this pass
closes: not a code defect, a traceability one, on the decision record for a launch-hold whose
whole premise is "no claim stands until it is verified against the current model."

## What I actually did, not assumed

Built the exact pinned toolchain (`requirements-sim.txt` needs Python >=3.12 for
`numpy==2.5.1`; this sandbox's default `python3` is 3.11, so used `/usr/bin/python3.12` in a
fresh venv — `mujoco==3.10.0`, `numpy==2.5.1`, `pytest==8.4.2`, matching the pins exactly).
`cargo build --release -p sim-host --bin sim-host` with `MUJOCO_DIR` pointed at the venv's
`mujoco` package (the build script's own documented fallback).

Ran `tests/test_cmd_envelope_reserve.py` and `tests/test_incline_tolerance.py` against current
master (`787c664`, after #237). Every test passed or xfailed exactly as it already asserts:

```
tests/test_cmd_envelope_reserve.py: 14 passed, 4 xfailed
  (f1) pinned trim: -2.5009 deg measured, -2.501 +/- 0.10 deg pinned
  (f2) peak-demand slope: 41.967 A/unit (host.rs: 42.03)
  (a)1 peak demand 33.41 A of 40 A (headroom 6.59 A, 16.5%); peak lean 7.96 of 11.46 deg (30.5%)
  (a)2 peak demand 36.59 A of 40 A (headroom 3.41 A, 8.5%); peak lean 10.99 of 11.46 deg (4.1%)
  (c) warning 3.000 s | saturation 4.920 s | FALLEN 5.868 s -- warning leads FALLEN by 2.868 s
  residual ratio (atan(a/g), the 1 rad/g identity (f) depends on): 1.0280
  (a)3 kerb 20 mm at 6.75 m/s: 95.6 N*s, 201 deg/s imparted -- still xfail (moved to hardware gate)

tests/test_incline_tolerance.py: 5 passed
  full-stick and stick-reversal: held at every swept incline, -12..+12 deg / -8..+8 deg
  free-roll ratio to g*sin(phi): 0.70-0.71 across a 40x angle range (unchanged shape)
  corridor-brake arrest boundary: arrested through 6.5 deg, outrun at 7.0 deg (exact match
  to the docstring's own prior figures)
```

Then built the full workspace (`cargo build --release --workspace`) and ran the entire suite for
a clean baseline: **348 passed, 7 xfailed, 0 failed** (`pytest tests/ -q`, 257 s). (A first pass
with only `sim-host` built showed 72 failures, all `FileNotFoundError` against binaries the other
crates produce — an incomplete local build, not a regression; resolved by building the whole
workspace before drawing any conclusion from it.)

## Conclusion

**The re-measurement the third ratification demanded has been done, for real, and the numbers do
not move.** The residual slope (the physical quantity (f) depends on: lean-per-acceleration is
geometry, not drag) measured 1.028 rad/g, matching the pre-ratification value to three
significant figures — consistent with the physical read that the corrected drag model is a
velocity-dependent correction, and every one of these criteria is measured at low speed / short
duration (full stick from rest, a reversal, a kerb strike) where velocity-dependent drag has not
yet had time to matter. Per ADR-0011's own branch for this case ("if the slope moved... needs
re-derivation... if not...") **no re-derivation is triggered.** The (f1)/(f2) pins and the
condition-2 input numbers stand as they were pinned, now with a recorded, dated re-verification
against the model that superseded them, rather than a silent assumption that CI passing meant the
same thing.

Updated `roles/senior-controls/CONTEXT.md`'s "ADR-0011 balance loop" section in place (standing
context, not this log) to: point at the third ratification instead of the second; record this
re-verification and its numbers; and correct the "#208 — not my turf" line, which the
`policy_check.py` evidence above shows was simply wrong and had already caused one prior pickup
to skip it. Added a short dated pointer to this file at the top of both re-verified test files'
module docstrings, so the record is discoverable from the pinned evidence itself, not only from
role context that will eventually scroll out of relevance.

## Deliberately left out

- **Did not implement #208 itself** (the terrain/asset validator script). Its inputs are now
  confirmed current, so it is unblocked, but writing the validator + a deliberately-violating
  fixture asset + wiring it into CI is its own scoped unit of work and would have pushed this PR
  past "one thing." Next Senior Controls session should pick it up directly rather than
  re-flagging it as unowned.
- **Did not touch #204, #235, or #232** (tilted-ground vs. rotated-gravity 3.5% disagreement,
  also raised by the third ratification's "Open work this raises" list) — each blocked or
  underspecified for the reasons stated above, not re-litigated here.
- **Did not attempt to bring the rest of `roles/senior-controls/CONTEXT.md` current.** Its
  Stage-0B section in particular describes PR #197 as "DOES NOT BUILD YET" against a master that
  has since shipped a working Pi image, cross-built binaries, and hardware-verified AC-5 — that
  section is significantly stale and is its own follow-up, out of scope for a pass about the
  ADR-0011 balance-loop thread specifically.
- **Did not promote #151's "Turn it on" comment to an ADR.** Noted only as the reason this pass
  proceeded as a cron dispatch at all; ratifying it is the COO's, not mine.

No issue number to close — this is a decision-record correctness finding raised directly by
ADR-0011's own third ratification, not a numbered issue. Referenced in the PR against the ADR
itself.
