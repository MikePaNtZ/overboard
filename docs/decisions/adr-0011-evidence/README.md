# ADR-0011 evidence — the measurement harnesses, rescued

**These eleven scripts are the derivations behind numbers ADR-0011 and its two ratifications
already treat as settled.** They lived only in a session-scoped scratchpad under
`/private/tmp/claude-501/…`, which nothing preserves. The numbers were merged; the working
that produced them was not. That asymmetry is the reason this directory exists.

Rescued verbatim by the COO on 2026-08-04, from the 2026-08-02 Senior Controls session.

## They do not run as-is, and were not fixed

Each script hardcodes the absolute scratchpad path it wrote to, and several hardcode
constants of the day — `onewheel_compare.py` opens with `MAX_A, KT, R_EFF, MASS = 40.0, 0.7,
0.1454, 82.5`, which is the **pre-60 A** figure now under review on
`feat/controls/realistic-motor-torque`.

They were copied **unmodified on purpose.** Repointing the paths or refreshing the constants
would be re-deriving another role's evidence under the guise of preserving it, and a rescued
measurement that has been quietly edited is worth less than no measurement at all. Anyone
re-running these owns the port, and should say in the commit which constants moved.

## What each one asks

| Script | The question it was written to answer |
|---|---|
| `incline_sweep.py` | The authored-terrain incline the board tolerates — **ADR-0011 (f)/condition 2** directly |
| `incline_wide.py` | The same sweep, widened |
| `characterise_trim.py` | How far the residual slope/offset move across operating points — what honestly sets a **pinned band** (#207) |
| `settled.py` | The two candidate instruments for that band, and their spread |
| `shape.py` | Whether a quasi-steady window exists where residual/unaided specific force settles |
| `slope_vs_trim.py` | Whether the 42 A/unit peak-demand slope actually depends on the trim |
| `brake_sweep.py` | What braking authority buys (stop time/distance) and what it costs — the **reserve** derivation (#218, #219) |
| `hold_limit.py` | Where the speed cap stops bounding a downhill roll, at zero stick |
| `grade.py` | Free-roll downhill-forward, and the steepest grade held at full stick |
| `onewheel_compare.py` | *"How far are we from the CEO's cited Onewheel figure, and what sets the gap?"* — **#204** |
| `analyse.py` | General reader for `sim-host --trace-csv` output |

## `onewheel_compare.py` is the one to read first

It is timestamped **16:06 on 2026-08-02 — after that session's teardown commit**, so it is
the only artefact of work that was never written up anywhere. It sweeps braking against the
CEO's cited Onewheel stopping figure. Minutes later the working tree gained an uncommitted
40 A → 60 A change across every `MAX_CURRENT_A` site, and the session ended.

The apparent chain — *braking is short of the cited figure → the gap is motor authority →
raise the current limit* — is **inference from timestamps, not a recorded conclusion.** It
is banked on `feat/controls/realistic-motor-torque` and is unvalidated. 28 → 42 N·m is a 50%
authority increase against a margin ADR-0011 measured at approximately zero, so if the chain
holds it is a decision-record question rather than a parameter change.

## The 178 CSVs were deliberately left behind

262 MB of trace output, regenerable by re-running the scripts above. Nothing is checked into
git that a build can reproduce — the same rule that keeps `sim/out/` out of the repo. The
naming is systematic and worth knowing if the corpus is ever regenerated:

- `i_*`, `w_*`, `t_*` — estimator trim and window sweeps (`i_hold`, `i_idle`, `i_rev`, …)
- `s_*`, `b_*`, `g_*` — command-envelope reserve, braking, and grade sweeps
- `h_*`, `x_*` — hold-limit and the final 16:08 two-parameter sweep
- `*_shipped` — the as-merged configuration, for comparison against a candidate

Suffixes are parameter values (`b_rev0.9.csv` = reverse case at reserve 0.90).

## Not rescued

Draft PR bodies, issue write-ups and board-doc drafts from 2026-07-31 (`c122.md`, `i137.md`,
`prgate.md`, `board-coo.md`, `teardown_msg.txt`, …). Every one of them is superseded by the
merged artefact it was a draft of.
