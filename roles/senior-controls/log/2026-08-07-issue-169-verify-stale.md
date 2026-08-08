# 2026-08-07 — Issue #169: the outer-velocity-loop station-keeping gap is already closed

Cron dispatch pass. `role:senior-controls` queue this run: #182 (already has five open PRs —
#236/#231/#225/#224/#223 — skipped, in flight), #169, #168, #161 (the W1–W3 launch-weekend
umbrella issue #169/#168 were split from; itself not a single dispatchable unit), #142 (blocked
on Sr. Mechanical & Systems input per its own AC2 — cannot be obtained in an unattended run),
#133 (explicitly blocked on #142's answer), #132 (needs Mechanical's fitted `kt`; the fix is a
gain-margin/delay-margin tradeoff call, not a clean execution task), #61 (`Dispatch: COO only`,
reserved). Of that set, #169 is the one that turned out to already be done.

## What #169 asked for

Filed 2026-08-01: `sim-host` had no outer velocity loop, so the board balanced but coasted away
indefinitely (10.8 m in 10 s) instead of station-keeping, while `host.rs`'s own comment claimed
the opposite ("W1 is pure station-keeping balance"). Four asks, in priority order:

1. Decide deliberately whether the outer velocity loop is on for the game, and write down which.
2. Fix the comment either way.
3. Reconsider the unconditional startup kick for the game path (it was sliding the board away
   before the player touched anything).
4. If the loop goes on, wire `v_ref` from the weight-shift channel.

## What I verified, not assumed

All four are already in the tree, landed same-day (2026-08-01) in `f49fcba` ("feat(controls):
W2 -- ridden rider model, lean-to-drive, roll-shaped yaw limiter (#170)") — a PR that fixed
#169 without a `Closes #169` in its message, so the issue never auto-closed:

1. **Decided, and written down.** `crates/sim-host/src/host.rs:10-31`'s module docstring: *"The
   board does not station-keep, and is not supposed to. A real onewheel does not either -- lean
   forward to accelerate, level off to coast... An earlier revision of this file's
   controller-config comment called this loop 'pure station-keeping balance', which was true of
   W1's driverless-plant, `pitch_ref`-only inner loop but became actively wrong the moment this
   file switched to a ridden plant."* `control_core::VelocityLoop` exists and stays deliberately
   unused for the ridden game path.
2. **Comment fixed.** The corrected docstring above replaces the inverted claim #169 quoted.
3. **Startup kick gated off by default.** `crates/sim-host/src/bin/sim-host.rs:24`: *"
   `--startup-kick` is OFF by default (issue #169): a normal run must not..."* — it's an opt-in
   CLI flag (`--startup-kick`, line 61-62 of that file), `HostConfig::default().startup_kick =
   false` (`host.rs:1245`), and the one-time kick only fires when explicitly enabled
   (`host.rs:1898`, itself commented `"only when explicitly enabled (issue #169)"`).
4. **Moot.** The loop stays off per (1), so there's no `v_ref` wiring to do.

Confirmed by reading `crates/sim-host/src/host.rs` and `crates/sim-host/src/bin/sim-host.rs` at
current master (`e163414`) directly — not inferring from commit messages. All four citations
above name issue #169 explicitly in the source, so this isn't a coincidental resemblance.

## Conclusion

Closing rather than leaving open. An open issue describing a station-keeping gap that was fixed
six days ago is a standing invitation for a future session to re-derive a fix for a problem that
no longer exists, or to distrust a docstring that is now telling the truth.

## Deliberately left out

- **#168** (`Pacer` catch-up-burst comment, filed the same day from the same PR's predecessor
  state) looks like the same situation on a first read — its comment fix is confirmed landed in
  the same commit (`f49fcba`, `crates/sim-host/src/pacer.rs:62-77`) — but its ask 3 ("report
  jitter percentiles, not just a miss count") is not implemented; only a raw `missed_deadlines`
  count is written to `/tmp/overboard-sim-host-stats.txt`. Not closing it in this PR — "do
  exactly one issue" — but it's the same stale-tracking pattern and worth a look next pass.
- **The `Dispatch:` marker this run's instructions describe** ("`Dispatch: cron OK` means it is
  yours to take") does not exist as a convention anywhere in the 33 open issues checked, except
  two explicit reservations (#61 `Dispatch: COO only`, #33 not-dispatchable). The actual routing
  mechanism, per ADR-0007 and `ops/dispatch.sh`, is the `role:` GitHub label alone, with
  `--audit` treating a missing label as a hard routing error. Worth reconciling the dispatch
  instructions against `ops/dispatch.sh` so a future cron pass doesn't stall looking for a
  marker that was never adopted.
