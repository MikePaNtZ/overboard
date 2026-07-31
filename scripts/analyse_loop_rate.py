#!/usr/bin/env python3
"""Where does 500 Hz come from? — the measurement behind issue #113.

`docs/design-pi-image-stage0b.md` sources the control rate to **ICD §11.2** and
says outright that *"500 Hz is an ICD number, not physics"*. Stage 0B's AC-5
and AC-6 are then written as fractions of that 2 ms period. This script
produces every number needed to say whether that period is the right one, and
`notebooks/05-control-loop-rate.ipynb` reads what it writes.

Five blocks, in the order the argument runs:

1.  **The plant.** Linearise the compiled MuJoCo model and get the unstable
    pole out of it — for the ridden vehicle, not for a point mass.
2.  **The budget.** Delay margin of the loop as actually gained, continuous and
    sampled. This is the number AC-6 should be judged against, because AC-6
    measures *latency* and latency is what the budget is spent on.
3.  **The sweep.** Rate x delay in the simulator, where the 40 A cap, the
    estimator and the nose strike all exist and the linear model's do not.
4.  **Jitter**, one-sided and late, as a real-time scheduler produces it.
5.  **Noise sensitivity**, because a rate answer that depends on an unstated
    noise model is worth nothing. If the assumptions dominate, that is the
    result and this script says so.

    scripts/analyse_loop_rate.py --out-dir sim/out/experiments
    scripts/analyse_loop_rate.py --quick        # coarse grid, for a smoke run

Deterministic and seeded throughout: re-running reproduces every number here
bit-for-bit, on the pinned MuJoCo in `requirements-sim.txt`.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
import time
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.imperfections import STAGE0_PLACEHOLDER  # noqa: E402
from sim.scenarios.loop_rate import (  # noqa: E402
    SWEEP_DT_S,
    SWEEP_RATES_HZ,
    Gains,
    delay_margin,
    linearise,
    loop_margins,
    sampled_delay_budget,
    simulate_point,
)
from sim.scenarios.plant import build_model  # noqa: E402

INK, AMBER, MINT, MUTED, RED = "#16232E", "#F2A24A", "#2AAE97", "#96A8B0", "#C2513B"

#: The ridden vehicle: 70 kg of rider proxy 0.75 m above the axle. Identical to
#: `tests/test_closed_loop.py`'s cascade fixture on purpose -- a rate derived
#: against a different plant from the one the gates use would prove nothing.
BALLAST_KG, BALLAST_M = 70.0, 0.75

RIDDEN_GAINS = dict(kp_a_per_rad=200.0, kd_a_per_rad_s=30.0, max_current_a=40.0,
                    kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True)
DRIVERLESS_GAINS = dict(kp_a_per_rad=80.0, kd_a_per_rad_s=11.0, max_current_a=40.0)

#: What Stage 0B's AC-6 asks the transport to hold, seconds. Reused verbatim
#: from the ratified design doc; this script's job is to say what fraction of
#: the real budget they are, not to re-pick them.
AC6_P999_S, AC6_MAX_S = 0.001, 0.002

#: The Pi 5 RP1 SPI tail spike that prompted the whole question, seconds.
SPI_SPIKE_S = (0.0015, 0.002)

#: Phase margin the delay budget is quoted at, as well as at zero. Stating one
#: is the difference between "the loop is on the edge" and "the loop has room".
PM_TARGETS_DEG = (45.0, 30.0, 10.0)


# ---------------------------------------------------------------------------
def analytic(plant, gains: Gains, tau_i: float, rates) -> dict:
    margins = loop_margins(plant, gains)
    binding = delay_margin(plant, gains)
    return {
        "plant": plant.summary(),
        "gains": {
            "kp_a_per_rad": gains.kp_a_per_rad,
            "kd_a_per_rad_s": gains.kd_a_per_rad_s,
            "kp_v_rad_per_m_s": gains.kp_v_rad_per_m_s,
            "ki_v_rad_per_m": gains.ki_v_rad_per_m,
        },
        "crossovers": [
            {
                "omega_rad_s": round(m.crossover_rad_s, 5),
                "phase_margin_deg": round(m.phase_margin_deg, 3),
                "delay_margin_ms": round(m.delay_margin_s * 1e3, 3),
            }
            for m in margins
        ],
        "delay_margin_ms": round(binding.delay_margin_s * 1e3, 3),
        "delay_at_phase_margin_ms": {
            f"{t:g}": round(binding.delay_at_phase_margin_s(t) * 1e3, 3)
            for t in PM_TARGETS_DEG
        },
        "sampled": [
            {
                "rate_hz": r,
                "zoh_half_period_ms": round(0.5e3 / r, 4),
                "added_delay_budget_ms": round(
                    sampled_delay_budget(plant, gains, r, tau_i) * 1e3, 3
                ),
                "total_equivalent_ms": round(
                    (0.5 / r + sampled_delay_budget(plant, gains, r, tau_i)) * 1e3, 3
                ),
            }
            for r in rates
        ],
    }


def timestep_control(model, gains, out: list) -> list:
    """Is the 0.5 ms sweep timestep telling the truth about the 2 ms model?

    The sweep runs the physics 4x finer than the pinned model so that rates
    above 500 Hz are representable at all. That is a fidelity change, and an
    unchecked fidelity change is how a sweep produces a confident answer to a
    different question.
    """
    for dt in (0.002, 0.001, 0.0005, 0.00025):
        p = simulate_point(model, gains, rate_hz=500.0, delay_s=0.001,
                           dt_s=dt, phases=1)
        out.append({"physics_dt_ms": dt * 1e3, **p.to_dict()})
    return out


def grid(model, gains, rates, delays_ms, *, jitter_cycles=0.0, noise_scale=1.0,
         phases, seconds, label) -> list[dict]:
    rows = []
    t0 = time.time()
    for rate in rates:
        for d in delays_ms:
            pt = simulate_point(
                model, gains, rate_hz=rate, delay_s=d * 1e-3,
                jitter_s=jitter_cycles / rate, noise_scale=noise_scale,
                sim_seconds=seconds, phases=phases,
            )
            rows.append(pt.to_dict())
    print(f"  {label}: {len(rows)} points in {time.time()-t0:.1f} s")
    return rows


def first_failure(rows: list[dict], key: str = "delay_ms") -> dict:
    """Lowest value of `key`, per rate, at which the board struck.

    Reported as the boundary rather than a bisection to it: `sweep_closed_loop`
    already found that outcomes near the strike angle are not monotonic in
    disturbance magnitude, and bisecting a non-monotonic boundary converges on
    an arbitrary knife edge that moves with any upstream change.
    """
    out: dict[str, dict] = {}
    for r in rows:
        rate = f"{r['rate_hz']:g}"
        cur = out.setdefault(rate, {"first_strike": None, "last_survivor": None})
        if r["struck"]:
            if cur["first_strike"] is None or r[key] < cur["first_strike"]:
                cur["first_strike"] = r[key]
        else:
            if cur["last_survivor"] is None or r[key] > cur["last_survivor"]:
                cur["last_survivor"] = r[key]
    return out


# ---------------------------------------------------------------------------
def plot(summary: dict, out_path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.6))

    # -- 1. the budget against what is being spent ---------------------------
    a = axes[0][0]
    s = summary["analytic"]["ridden"]["sampled"]
    rates = [row["rate_hz"] for row in s]
    a.semilogx(rates, [row["zoh_half_period_ms"] for row in s], "o-",
               color=AMBER, lw=1.8, label="what the RATE costs (Ts/2)")
    a.semilogx(rates, [row["total_equivalent_ms"] for row in s], "s-",
               color=MINT, lw=1.8, label="total delay budget (ridden cascade)")
    a.semilogx(rates, [summary["analytic"]["driverless"]["delay_margin_ms"]] * len(rates),
               "--", color=INK, lw=1.4, label="total delay budget (driverless inner)")
    a.axhline(AC6_MAX_S * 1e3, ls=":", lw=1.4, color=RED)
    a.annotate("AC-6 max (2 ms)", (rates[0], AC6_MAX_S * 1e3), fontsize=8,
               color=RED, textcoords="offset points", xytext=(4, 4))
    a.set_yscale("log")
    a.set_xlabel("control rate (Hz)")
    a.set_ylabel("delay (ms)")
    a.set_title("The rate buys you Ts/2 of a ~65 ms budget", color=INK)
    a.grid(alpha=0.25, which="both")
    a.legend(fontsize=8)

    # -- 2. simulated rate x delay ------------------------------------------
    b = axes[0][1]
    rows = summary["sweep"]["ridden_rate_delay"]
    for rate in sorted({r["rate_hz"] for r in rows}):
        sel = sorted([r for r in rows if r["rate_hz"] == rate], key=lambda r: r["delay_ms"])
        b.plot([r["delay_ms"] for r in sel], [r["peak_abs_pitch_deg"] for r in sel],
               "o-", lw=1.5, ms=3.5, label=f"{rate:g} Hz")
        for r in sel:
            if r["struck"]:
                b.plot(r["delay_ms"], min(r["peak_abs_pitch_deg"], 30.0), "x",
                       color=RED, ms=8, mew=2)
    b.axhline(summary["strike_angle_deg"], ls="--", color=RED, lw=1.3)
    b.annotate("nose strike, 18.6 deg", (0, summary["strike_angle_deg"]),
               fontsize=8, color=RED, textcoords="offset points", xytext=(4, 4))
    b.set_ylim(0, 30)
    b.set_xlabel("transport delay (ms)")
    b.set_ylabel("peak |pitch| (deg)")
    b.set_title("Ridden cascade: delay is what bites, not rate", color=INK)
    b.grid(alpha=0.25)
    b.legend(fontsize=8, ncol=2)

    # -- 3. jitter -----------------------------------------------------------
    c = axes[1][0]
    rows = summary["sweep"]["ridden_jitter"]
    for rate in sorted({r["rate_hz"] for r in rows}):
        sel = sorted([r for r in rows if r["rate_hz"] == rate], key=lambda r: r["jitter_ms"])
        c.plot([r["jitter_ms"] for r in sel], [r["peak_abs_pitch_deg"] for r in sel],
               "o-", lw=1.5, ms=3.5, label=f"{rate:g} Hz")
        for r in sel:
            if r["struck"]:
                c.plot(r["jitter_ms"], min(r["peak_abs_pitch_deg"], 30.0), "x",
                       color=RED, ms=8, mew=2)
    c.axhline(summary["strike_angle_deg"], ls="--", color=RED, lw=1.3)
    c.set_ylim(0, 30)
    c.set_xlabel("worst-case scheduler lateness (ms)")
    c.set_ylabel("peak |pitch| (deg)")
    c.set_title("Jitter costs milliseconds, not cycles", color=INK)
    c.grid(alpha=0.25)
    c.legend(fontsize=8)

    # -- 4. noise sensitivity ------------------------------------------------
    d = axes[1][1]
    rows = summary["sweep"]["ridden_noise"]
    for rate in sorted({r["rate_hz"] for r in rows}):
        sel = sorted([r for r in rows if r["rate_hz"] == rate], key=lambda r: r["noise_scale"])
        d.semilogx([r["noise_scale"] for r in sel],
                   [min(r["peak_abs_pitch_deg"], 40.0) for r in sel],
                   "o-", lw=1.5, ms=3.5, label=f"{rate:g} Hz")
    d.axhline(summary["strike_angle_deg"], ls="--", color=RED, lw=1.3)
    d.axvline(1.0, ls=":", color=MUTED, lw=1.2)
    d.annotate("ICD §12 placeholder", (1.0, 2.0), fontsize=8, color=MUTED,
               textcoords="offset points", xytext=(5, 0), rotation=90)
    d.set_xlabel("white-noise multiplier on the ICD §12 profile")
    d.set_ylabel("peak |pitch| (deg)")
    d.set_title("It takes 10-30x the modelled noise before rate matters", color=INK)
    d.grid(alpha=0.25, which="both")
    d.legend(fontsize=8)

    fig.suptitle("Deriving the control loop rate from the plant (issue #113)",
                 color=INK, fontsize=13)
    fig.tight_layout(rect=(0, 0, 1, 0.97))
    fig.savefig(out_path, dpi=140)
    plt.close(fig)


# ---------------------------------------------------------------------------
def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim" / "out" / "experiments")
    ap.add_argument("--quick", action="store_true",
                    help="coarse grid and one phase per point -- a smoke run, "
                         "not a result: the numbers it prints are not the "
                         "published ones")
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    from sim.scenarios.impulse_response import nose_strike_angle_deg

    ridden = build_model(BALLAST_KG, BALLAST_M, 40.0, "figure")
    driverless = build_model(0.0, 0.0, 40.0, "figure")

    rp, dp = linearise(ridden), linearise(driverless)
    gr = Gains(200.0, 30.0, 0.05, 0.02, True)
    gd = Gains(80.0, 11.0)
    tau_i = STAGE0_PLACEHOLDER.current_loop_tau_s

    print("1. the plant")
    print(f"   ridden unstable pole   {rp.unstable_pole:.4f} rad/s "
          f"({rp.unstable_pole/2/math.pi:.4f} Hz), doubles in "
          f"{rp.doubling_time_s*1e3:.0f} ms")
    print(f"   the point-mass figure this was filed with: "
          f"{rp.point_mass_pole():.4f} rad/s -- {rp.unstable_pole/rp.point_mass_pole():.2f}x low")
    print(f"   pitch feedback alone needs kp > {rp.critical_kp_a_per_rad():.1f} A/rad "
          f"(in use: 200)")

    print("2. the delay budget")
    an = {"ridden": analytic(rp, gr, tau_i, SWEEP_RATES_HZ),
          "driverless": analytic(dp, gd, tau_i, SWEEP_RATES_HZ)}
    for name, block in an.items():
        print(f"   {name:11s} delay margin {block['delay_margin_ms']:7.2f} ms  "
              f"(crossover {block['crossovers'][0]['omega_rad_s']:.2f} rad/s, "
              f"PM {block['crossovers'][0]['phase_margin_deg']:.1f} deg)")

    print("3-5. the simulator")
    phases = 1 if args.quick else 3
    seconds = 4.0 if args.quick else 5.0
    rates = (125.0, 500.0) if args.quick else SWEEP_RATES_HZ
    fast = (250.0, 500.0) if args.quick else (125.0, 250.0, 500.0, 1000.0)
    delays = (0, 16, 48) if args.quick else (0, 4, 8, 16, 24, 32, 40, 48)

    sweep = {
        "ridden_rate_delay": grid(ridden, RIDDEN_GAINS, rates, delays,
                                  phases=phases, seconds=seconds,
                                  label="ridden rate x delay"),
        "driverless_rate_delay": grid(driverless, DRIVERLESS_GAINS, fast,
                                      (0, 16, 32, 48, 64, 80),
                                      phases=1, seconds=seconds,
                                      label="driverless rate x delay"),
    }
    jit, noi = [], []
    for rate in fast:
        for cycles in (0.0, 0.5, 1.0, 2.0, 4.0):
            jit.append(simulate_point(ridden, RIDDEN_GAINS, rate_hz=rate,
                                      delay_s=0.001, jitter_s=cycles / rate,
                                      sim_seconds=seconds, phases=1).to_dict())
        for scale in (1.0, 3.0, 10.0, 30.0, 100.0):
            noi.append(simulate_point(ridden, RIDDEN_GAINS, rate_hz=rate,
                                      delay_s=0.001, noise_scale=scale,
                                      sim_seconds=seconds, phases=1).to_dict())
    sweep["ridden_jitter"] = jit
    sweep["ridden_noise"] = noi
    print(f"  jitter: {len(jit)} points, noise: {len(noi)} points")

    control = timestep_control(ridden, RIDDEN_GAINS, [])
    peaks = [c["peak_abs_pitch_deg"] for c in control]
    spread = (max(peaks) - min(peaks)) / np.mean(peaks)
    print(f"   physics-timestep control: peak pitch spread {spread*100:.1f}% "
          f"across 2.0 -> 0.25 ms")

    budget = an["driverless"]["delay_margin_ms"]
    summary = {
        "note": "Deterministic and seeded; re-running reproduces this bit-for-bit.",
        "quick": args.quick,
        "strike_angle_deg": round(nose_strike_angle_deg(ridden), 3),
        "sweep_physics_dt_ms": SWEEP_DT_S * 1e3,
        "imperfection_profile": STAGE0_PLACEHOLDER.to_dict(),
        "analytic": an,
        "physics_timestep_control": control,
        "sweep": sweep,
        "boundaries": {
            "ridden_rate_delay": first_failure(sweep["ridden_rate_delay"]),
            "ridden_jitter": first_failure(sweep["ridden_jitter"], "jitter_ms"),
        },
        "ac6_against_the_budget": {
            "binding_delay_margin_ms": budget,
            "p999_threshold_ms": AC6_P999_S * 1e3,
            "p999_as_fraction_of_budget": round(AC6_P999_S * 1e3 / budget, 5),
            "max_threshold_ms": AC6_MAX_S * 1e3,
            "max_as_fraction_of_budget": round(AC6_MAX_S * 1e3 / budget, 5),
            "spi_spike_ms": [s * 1e3 for s in SPI_SPIKE_S],
            "spi_spike_as_fraction_of_budget": [
                round(s * 1e3 / budget, 5) for s in SPI_SPIKE_S
            ],
        },
    }

    plot(summary, args.out_dir / "loop-rate.png")
    (args.out_dir / "loop-rate.json").write_text(json.dumps(summary, indent=2) + "\n")
    np.savez_compressed(
        args.out_dir / "loop-rate.npz",
        **{
            f"{name}_{field}": np.array([row[field] for row in rows])
            for name, rows in sweep.items()
            for field in ("rate_hz", "delay_ms", "jitter_ms", "noise_scale",
                          "struck", "peak_abs_pitch_deg", "late_band_deg",
                          "travel_m", "control_effort_a_s", "saturated_cycles")
        },
    )
    print(f"\nwrote {args.out_dir/'loop-rate.json'}, .npz and .png")
    print(f"AC-6's 2 ms ceiling is {summary['ac6_against_the_budget']['max_as_fraction_of_budget']*100:.1f}% "
          f"of the {budget:.0f} ms budget; the 1.5-2 ms SPI spike is "
          f"{summary['ac6_against_the_budget']['spi_spike_as_fraction_of_budget'][1]*100:.1f}% of it")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
