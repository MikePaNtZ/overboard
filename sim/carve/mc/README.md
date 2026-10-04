# Monte Carlo results

One CSV row per run, written by `sim/carve/monte_carlo.py`; summarise with
`sim/carve/mc_summary.py`.

**Current baseline (sim-host b03fece or later; the end boxes kicked 4.2 deg, strike 20.4 deg nominal):** the X7 build plant
(`--plant-x7`) with the build's meshes and pads, the deck strike angle spread
over 18-21 deg (pad_z), tail braking on, 200 runs, seed 1.

| File | Set |
|---|---|
| `x7_tail_brake.csv` | No rider warning: 200 PASS (closest: 2.1 deg from a deck strike) |
| `x7_warn_no_reaction.csv` | Warning on, rider does not react: 200 PASS, 11 runs warned |
| `x7_warn_rider_reacts.csv` | Warning on, rider reacts: 197 PASS, 3 DISMOUNT (20-25 % climbs) |

**The deployed balance law (what a rider rides; sim-host 74490a4 or later):**
a rider model leans (`--rider-law`), the deployed pitch law with grade
compensation balances (`--balance-comp`), X7 plant, tail braking on.

| File | Set |
|---|---|
| `x7_rider_law_comp.csv` | No warning: 159 PASS, 37 STALL (32 on climbs >= 15 %: the rider model's 3 cm lean bound), 2 RUNAWAY (23-24 % descents, 5.4-6.3 m/s), 2 nose strikes (110 kg, 20-24 % climbs) |
| `x7_rider_law_comp_warn_reacts.csv` | Warning, rider reacts: 158 PASS, 36 STALL, 2 RUNAWAY, 4 DISMOUNT, 0 nose strikes |

**Rider reach 10 cm** (`--rider-reach 0.10`; sim-host 880a2b2 or later). The
rider moves the body up to 6 cm over the feet (60 % of reach), not 3 cm.

| File | Set |
|---|---|
| `x7_rider_law_reach10.csv` | No warning: 191 PASS, 3 STALL, 6 FALL (5 nose strikes: riders of 100 kg or more on 18-24 % climbs at the current limit; 1 tail-down at the brake limit, -22 %) |
| `x7_rider_law_reach10_warn_reacts.csv` | Warning, rider reacts (and steps off when stopped on a climb under the warning): 185 PASS, 2 STALL, 7 DISMOUNT, 4 EASED STOP, 2 FALL (tail-down at the brake limit, 20-22 % descents), 0 nose strikes |

**Superseded.** Before c28e529 the backend capped the motor at 40 A
whatever `--max-current` said, so every run with a limit above 40 A ran at
40 A. These files are kept for the record only:
`baseline.csv`, `grade_ff.csv` (also an older plane plant),
`tail_brake.csv`, `warn_no_reaction.csv`, `warn_rider_reacts.csv` (13 kg board,
30-60 A), `b18_*.csv` (17.9 kg board, 30-60 A). Runs at or below 40 A in them
are unaffected. `xcheck40_*.csv` compares ground models and is unaffected in
its conclusion.
