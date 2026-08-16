# Issue #168, ask 3 — jitter percentiles instead of a bare miss count

**PR:** `fix/controls/pacer-jitter-percentiles-168` (references #168, does not close it).

## What was done

Ask 1 (fix the inverted `Pacer` comment) and the underlying bursty-not-late finding were already
landed and correctly described in `crates/sim-host/src/pacer.rs`'s `CORRECTION (issue #168)`
comment block. Asks 2 and 4 are explicitly deferred by the issue itself ("Tuesday work, not
weekend work" / "Fine for W1"). Ask 3 — "report jitter percentiles, not just a miss count" — was
the one remaining, in-scope, unimplemented item, and is what this PR does:

- `Pacer` now keeps a bounded (2,000-sample, ~4 s at 500 Hz) ring of per-tick lateness and exposes
  `jitter_percentiles()` — nearest-rank p50/p99/max, same convention as
  `crates/loop-profiler/src/stats.rs` (duplicated, not depended on — the algorithm is ~10 lines and
  this crate has no other reason to pull in `loop-profiler`).
- `host.rs`'s stats file (`/tmp/overboard-sim-host-stats.txt`) gained `jitter_p50_ns` /
  `jitter_p99_ns` / `jitter_max_ns` alongside the existing `missed_deadlines`.
- `wire-probe`'s report line changed from a bare "missed-deadline count from the host: N" to
  "host-side jitter (ms, recent window): p50=... p99=... max=... -- N/M ticks missed overall" when
  the fields are present, falling back to the old bare-count line for a stats file written before
  this change (best-effort internal tooling, not the wire — no version bump needed).

## Verification

- `cargo test -p sim-host --lib pacer` — 9/9 (5 pre-existing + 4 new, deterministic, no real
  sleeps, matching the module's existing style).
- `cargo test --workspace`, `cargo clippy --workspace --all-targets -- -D warnings`,
  `cargo fmt --all -- --check`, `cargo run -p xtask -- gate` — all clean.
- Manual end-to-end smoke test: ran `sim-host` and `wire-probe` against each other locally,
  confirmed the stats file carries the new fields and `wire-probe` prints the new percentile line
  (`host-side jitter (ms, recent window): p50=0.0000 p99=0.0000 max=8.5193 -- 5/1500 ticks missed
  overall`).
- `python3 .github/policy_check.py` — all hard checks pass (pre-existing advisories only, none
  touched by this change).

## Deliberately left out

- Ask 2 (real-time thread policy on macOS) and ask 4 (promote the miss count to a wire field on
  the next version bump) — the issue itself defers both.
- `RunSummary`/`bin/sim-host.rs`'s final one-line summary print was **not** touched: it prints once
  at process exit over the whole run, and `jitter_percentiles()` is deliberately a *recent* window,
  not a whole-run statistic — mixing the two into one line would misrepresent a multi-hour run's
  jitter as "the last 4 seconds." The stats file / `wire-probe` path this PR touches is the
  mechanism the issue's own report was actually built from.
- Did not widen `JITTER_WINDOW` or otherwise validate it against a real macOS/Linux run — 2,000
  samples (4 s at 500 Hz) was sized to keep the periodic in-loop sort cheap, not measured against
  hardware.

## Also found, out of scope, flagged rather than fixed

- **Duplicate PRs on the same issue.** #242 and #253 are both open, both closing #222, both adding
  the same `pins.env` fields. Not this session's turf to resolve (would mean closing someone else's
  PR), but worth a COO/CEO look — this run's dispatch check ("skip any issue that already has an
  open PR against it") should have prevented a second one and evidently didn't catch this pair.
- **The `Dispatch: cron OK` marker this run's own instructions describe as the gating field still
  does not exist as a convention anywhere in the 37 open issues checked** (confirmed again this
  run; PR #245 flagged the same thing on 2026-08-07). Only two explicit reservations exist in the
  whole set: `Dispatch: COO only` on #61, and #33's "not dispatchable." The actual routing
  mechanism is the `role:` GitHub label (ADR-0007, `ops/dispatch.sh`). Flagging the mismatch again
  rather than fixing it — reconciling the dispatch prompt isn't controls turf.
- Every other issue whose body says `Owner: Senior Controls` already has an open PR against it
  (#261→#263, #255→#266, #226→#234, #222→#242/#253). #230 (thermal throttling) is a hardware/BoM
  ask with nothing to code. #194 ("Controls, with Mechanical to weigh in") is a genuine open design
  question across two roles' turf, not a default-action item for an unattended run.
