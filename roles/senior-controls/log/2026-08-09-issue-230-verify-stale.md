# 2026-08-09 — Issue #230: AC-5's thermal-contamination gate is already closed

Cron dispatch pass. Open issues whose body contains `Owner: Senior Controls`: #261 (PR #263
open), #255 (PR #266 open), #226 (PR #234 open), #61 (`Dispatch: COO only`, reserved), #230.
Of that set, #230 is the only one without a PR already in flight — and it turned out to
already be resolved.

## What #230 asked for

Filed 2026-08-05: a 30-minute loaded `cyclictest` run on the CEO's Pi 5 Rev 1.1 hit the soft
thermal limit and throttled after ~4m20s (`throttled=0xe0000` — bits 17/18/19 set), which
poisons AC-5's number rather than merely degrading it, per the issue's own reasoning (a
throttling CPU's clock changes *during* the measurement, so the result is uninterpretable, not
pessimistic-but-usable). The ask: fit an active cooler, then re-run. The issue states its own
acceptance condition explicitly — "`get_throttled` returning `0x0` after a full 30-minute
loaded run is the acceptance condition for the *rig*, before AC-5 is an acceptance condition
for the *platform*."

## What I verified, not assumed

`docs/design-pi-image-stage0b-reference.md`'s AC-5 row and Q1, landed in `cdaed8d` (#254,
2026-08-07 — two days after #230 was filed, no `Closes #230` in the commit), record a clean
30-minute run on the Stage-0B image:

```
p99.9   72 us   (limit 150)   2.1x margin
max    113 us   (limit 500)   4.4x margin
n = 2,699,993 samples, 30 min, throttled=0x0
```

`throttled=0x0` is exactly the rig acceptance condition #230 wrote down — all four sticky
bits clear, including the two (arm-frequency-capping, throttling) that were set in #230's
contaminated run. AC-5 passed on this same data with real margin (2.1x and 4.4x), so the
platform acceptance condition is also met, not just the rig one. Confirmed by reading the
commit (`cdaed8d`) and the current doc table directly, not inferring from the issue thread —
#230 has zero comments, so there's no discussion trail to lean on instead.

The two caveats #254 recorded alongside this pass (isolated core not measured, network stressor
loopback-only) are #255 and unrelated respectively — #255 already has an open PR (#266) and is
not this issue's concern. #230 is specifically and only about thermal contamination, and that
question has a clean, measured, non-ambiguous answer: fixed.

## Conclusion

Closing rather than leaving open. An open issue reporting a throttled rig with no valid AC-5
number is a standing invitation for a future session to re-litigate a cooling fix that already
shipped, or to distrust an AC-5 PASS that is now backed by a clean thermal reading right next
to it in the same table.

## Deliberately left out

- Did not touch `crates/loop-profiler` to add an automated `throttled` precondition check
  (e.g. failing the profiler run outright if any sticky bit is set) even though #230's own
  argument would support one. That's #226's territory (loop-profiler can't tell a live run
  from a contaminated one, print a bare verdict, etc.) and #226 already has an open PR (#234).
  Bundling it here would be scope creep into a PR someone else is already landing.
- Did not verify whether an active cooler was physically fitted, or whether the clean
  2026-08-07 run used some other mitigation (shorter thermal soak before the run started,
  different case, ambient temperature). Out of reach for this session either way — no hardware
  access — and irrelevant to closing this issue, since #230 defines its own acceptance
  condition purely in terms of the measured `get_throttled` output, which is satisfied.
