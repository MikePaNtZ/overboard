# Monte Carlo results

One CSV row per run, written by `sim/carve/monte_carlo.py`; summarise with
`sim/carve/mc_summary.py`.

**Current baseline (sim-host c28e529 or later):** the X7 build plant
(`--plant-x7`), tail braking on, 200 runs, seed 1.

| File | Set |
|---|---|
| `x7_tail_brake.csv` | No rider warning: 200 PASS |
| `x7_warn_no_reaction.csv` | Warning on, rider does not react: 200 PASS, 12 runs warned |
| `x7_warn_rider_reacts.csv` | Warning on, rider reacts: 197 PASS, 3 DISMOUNT (20-25 % climbs) |

**Superseded.** Before c28e529 the backend capped the motor at 40 A
whatever `--max-current` said, so every run with a limit above 40 A ran at
40 A. These files are kept for the record only:
`baseline.csv`, `grade_ff.csv` (also an older plane plant),
`tail_brake.csv`, `warn_no_reaction.csv`, `warn_rider_reacts.csv` (13 kg board,
30-60 A), `b18_*.csv` (17.9 kg board, 30-60 A). Runs at or below 40 A in them
are unaffected. `xcheck40_*.csv` compares ground models and is unaffected in
its conclusion.
