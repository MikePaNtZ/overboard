# 2026-08-16 — Issue #190: the s-curve motor-saturation flip is already fixed

Cron dispatch pass. No open issue carries a literal `Owner: Senior Controls` or
`Dispatch: cron OK` tag — confirmed by reading the body of all 39 open issues, not by
assumption. The only explicit `Dispatch:` marker anywhere in the queue is `Dispatch: COO only`
on #61 (reserved). Went by the repo's actual routing convention instead: the `role:` label plus
turf paths (`crates/`, `tests/`, `sim/scenarios/*` estimator/actuator elements), same convention
prior cron passes (PR #245, #276) used and documented.

Of the `role:senior-controls`-labelled and clearly-controls-turf issues, every one already has
an open PR against it except:
- **#132** — blocked on a bench-fitted `kt` Sr. Mechanical & Systems owns; not independently
  actionable without hardware that does not exist yet.
- **#235** — no numeric acceptance criteria, four "decide whether..." asks about battery/SOC
  modelling scope; the goal itself is undecided, not just the solution.
- **#61** — `Dispatch: COO only`, reserved.
- **#208** — flagged unowned in this role's own standing context (`roles/senior-controls/CONTEXT.md`):
  the world-authoring asset rule, explicitly "Not my turf."
- **#203** — its own options section requires a `policy_check.py` change, which is COO turf.

That left #190 as the highest-value unlabelled-but-clearly-controls issue with no open PR:
filed 2026-08-01, it is the original finding behind the entire ADR-0011 launch-hold thread
(`host.rs`'s own header cites it eight times), and closing it if it no longer reproduces removes
a standing invitation to re-diagnose a problem the subsequent ADR-0011 work already solved.

## What #190 asked for

Motor-current saturation at ~6.5 m/s under full-stick fore/aft lean flips the board in ~1.55 s,
reproduced deterministically via `send-input --scenario s-curve` / `sim-host
--scripted-scenario s-curve`. Three asks:

1. A COO/CEO call on whether to back off the filmed schedule's build lean, shorten it, or accept
   and re-shoot.
2. Either derive `MAX_GROUND_SPEED_M_S` from available torque instead of a marketing figure, or
   limit fore/aft authority by remaining current headroom.
3. Add a saturation / impending-loss-of-authority signal — `FALLEN` alone trips ~1 s after the
   outcome is already decided.

## What I verified, not assumed

**(1) is not a controls call and was never mine to close** — it is explicitly a COO/CEO
decision in the issue's own text, and ADR-0011's own ratifications (its second and later
rounds) are the record of that call having since been made at the launch-hold level. Nothing
here re-opens it.

**(2) was considered and explicitly rejected in writing, in favour of a better fix.**
`crates/sim-host/src/host.rs:709` (`MAX_GROUND_SPEED_M_S`'s own doc comment): *"`MAX_GROUND_SPEED_M_S`
does not move (rejected as dominated -- it costs top speed and this does not)"*, in favour of
`CMD_ENVELOPE_RESERVE` (ADR-0011 criterion (b)) — an upstream multiply on the fore/aft stick
that caps authority without touching top speed. This is a documented engineering decision, not
an oversight, and it is the mechanism that actually fixes the flip (below).

**(3) is implemented and tested, with wire-visibility correctly split into its own issue.**
`AUTHORITY_UTILISATION_WARN` / `authority_warning_active()` (`host.rs:1756`) compute the signal
every cycle; it is traced to CSV (`authority_warning` column) and unit-tested
(`the_authority_warning_fires_only_on_high_utilisation_below_the_speed_cap_onset`,
`host.rs:3104`). It is not yet on the wire to a renderer — that is issue #216, already tracked
separately with its own open PR (#238). Splitting "the signal exists and is tested" from "the
signal reaches a client" is correct issue hygiene, not a reason to keep #190 open.

**The core defect — full-stick motor-current-saturation flip — is fixed and tested.**
`tests/test_cmd_envelope_reserve.py` proves both halves: the underlying saturation still exists
unshaped (`test_the_unshaped_command_map_still_inverts_the_board`) and the shaped board does not
invert from full stick at rest (`test_criterion_a1_full_stick_from_rest_does_not_invert`). Ran
both directly against current master (`2e41fc7`):

```
tests/test_cmd_envelope_reserve.py::test_the_unshaped_command_map_still_inverts_the_board PASSED
tests/test_cmd_envelope_reserve.py::test_criterion_a1_full_stick_from_rest_does_not_invert PASSED
```

**Re-ran #190's own exact scenario**, not a proxy for it — `cargo run -p sim-host --release
--bin sim-host -- --scripted-scenario s-curve --trace-csv <path>`, the same schedule #190's
table walked tick-by-tick to a 179.5° flip:

| metric (this run, current master) | value |
|---|---|
| max \|truth_pitch_deg\| | 7.96° |
| `fallen` at any tick | never |
| `saturated` at any tick | never |
| `authority_warning` at any tick | never |
| max `shaped_fore_aft` | exactly **0.80** |
| max `forward_speed_m_s` | 10.13 |

`shaped_fore_aft` capping at exactly 0.80 across the whole run is `CMD_ENVELOPE_RESERVE` visibly
doing the work: the schedule still commands full (1.0) stick, but the reserve delivers at most
0.80 of it, safely under the measured 0.95–0.97 flip cliff #190 itself found. Pitch never leaves
single digits; nothing in the original 20°/90°/170°/179.5° table appears.

**Full suite, clean build:** `cargo build --workspace --release` clean; `pytest tests/ -q` →
**333 passed, 5 xfailed, 0 failed** — matching the baseline PR #244 and PR #276 measured for
#240/#270 on this same tree.

## Conclusion

Closing rather than leaving open. #190's core defect is fixed (`CMD_ENVELOPE_RESERVE`,
re-verified above by re-running its own scenario), its rejected alternative is documented in
place, its instrumentation ask is implemented and tested with the remaining wire-visibility work
correctly forked into #216 (open PR #238), and its one ask that was never controls' to close
belongs to a decision record (ADR-0011) that has already run its course. Leaving #190 open
describes a flip that no longer happens as still unsolved.

## Deliberately left out

- **Did not touch #216** (wire the authority-warning bit) — it already has an open PR (#238);
  redoing it would be exactly the "already in flight" duplication this pass is told to skip.
- **Did not re-verify the corridor-exit messages** in the s-curve run
  (`LEFT THE DRIVABLE CORRIDOR`) — that is `CORRIDOR_X_MAX_M`/`CORRIDOR_BRAKE_LEAN` behaviour,
  unrelated to #190's saturation-flip claim, and expected at this schedule's travel distance.
- **Did not spot-check the other four `role:senior-controls` issues with open PRs** (#182, #169,
  #168, #161, #142, #133) — each already has a PR or a stated blocker from a prior pass; re-auditing
  them is not this pass's job.

Closes #190.
