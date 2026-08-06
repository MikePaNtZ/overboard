#!/usr/bin/env python3
"""Is a burst of missed deadlines worse than the same delay held constantly?

Issue #130. The delay budget (#122, corrected in #134) measured the loop
against a **pure, fixed** transport delay and said so in its own limits:

    "AC-6's original framing conflated an isolated tail spike with a sustained
     shift -- this analysis only speaks to the latter."

That gap matters because the two are not the same event. A single 2 ms stale
sample against a plant whose unstable pole is 5.24 rad/s is nothing. What
threatens a balancer is **consecutive** misses -- the command frozen for
several cycles while the pendulum keeps falling. `hal::Observation` already
carries `missed_cycles`, so the runtime can report the quantity. **Nothing
pre-registers what value is acceptable.** This produces that number.

## Where this is modelled, and why not in ImperfectionProfile

#130's AC1 asks for burst structure on `ImperfectionProfile`. It is modelled
here instead, at the **controller seam**, for a reason that is not turf
convenience:

`ImperfectionProfile` models the **plant and its hardware** -- sensor noise,
quantisation, actuator lag, the current loop. A missed control deadline is
none of those. It is a **compute/scheduling** failure: the control task did
not run, so no new command was produced and the actuator holds the last one.
The plant is behaving perfectly; the computer is not. Modelling it as a plant
imperfection would put a scheduler property inside the physics contract that
Sr. Mechanical & Systems owns and that `sim-backend` must conform to.

So a miss is modelled as exactly what it is: the controller returning its
previous output. If Sr. Mechanical & Systems would rather it live in the
profile, that is their call to make -- flagged, not assumed.

## The pattern: deterministic worst case, not a distribution

AC1 asks which, and why. **Deterministic**, for two reasons:

1.  **A distribution cannot be pre-registered honestly today.** We have no
    measured jitter distribution from the Pi -- AC-9's `cyclictest` and SPI
    runs have not happened, there is no hardware. Inventing a burst
    distribution and then deriving a threshold from it would be exactly the
    failure #113 was filed about: a decision judged against a number nobody
    derived.
2.  **The bound wanted is a worst case, not an average.** "No more than K
    consecutive misses" is a statement the runtime can check per-cycle against
    `missed_cycles`. A distributional bound is not checkable that way.

So: one burst of K consecutive misses, placed at the worst instant, and the
boundary is bisected in cycles.

    .venv/bin/python scripts/analyse_deadline_bursts.py
"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

from sim.scenarios.imperfections import STAGE0_PLACEHOLDER, ImperfectionProfile  # noqa: E402
from sim.scenarios.impulse_response import (  # noqa: E402
    NOMINAL_IMPULSE_NS,
    ImpulseParams,
    run,
)
from sim.scenarios.plant import build_model  # noqa: E402
from sim.scenarios.rust_controller import RustController  # noqa: E402

BALLAST_KG, BALLAST_M, CLAMP_A = 70.0, 0.75, 40.0
#: Torque-denominated since #137/#140: 140 N*m/rad and 21 N*m/(rad/s) are the
#: same design point as the old 200 A/rad and 30 A/(rad/s) at kt = 0.7. Kept
#: identical to `tests/test_delay_budget_stage0b.py:41` so this is measured
#: against the same loop the delay budget was.
GAINS = dict(kp_nm_per_rad=140.0, kd_nm_per_rad_s=40.0, max_current_a=40.0,
             kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True)

#: Control period, s. The burst length is expressed in these.
DT_S = 0.002

#: The disturbance fires at t0 = 0.5 s (`ImpulseParams.t0_s`).
T0_S = 0.5


class BurstHolder:
    """Wraps a controller and freezes its output for K consecutive cycles.

    That IS a missed deadline, modelled honestly: the control task did not
    run, so the actuator keeps driving the last command it was given. No new
    sample is consumed and nothing is interpolated -- a held command is not a
    delayed command, which is the whole point of separating this from
    `actuation_delay_s`.
    """

    def __init__(self, inner, start_s: float, cycles: int,
                 period_s: float | None = None):
        self.inner = inner
        self.start_s = start_s
        self.cycles = cycles
        self.period_s = period_s
        self.last = 0.0
        self.n_held = 0

    def _in_burst(self, t_s: float) -> bool:
        if self.cycles <= 0 or t_s < self.start_s:
            return False
        since = t_s - self.start_s
        if self.period_s is None:
            return since < self.cycles * DT_S
        # Repeating: a burst at the head of every period. This is the shape a
        # correlated stall actually has -- a GC pause, a thermal throttle or a
        # CAN retry storm recurs, it does not happen once.
        return (since % self.period_s) < self.cycles * DT_S

    def __call__(self, t_s, *a, **kw):
        if self._in_burst(t_s):
            self.n_held += 1
            return self.last
        self.last = self.inner(t_s, *a, **kw)
        return self.last


def _profile(delay_s: float | None) -> ImperfectionProfile:
    p = STAGE0_PLACEHOLDER
    if delay_s is None:
        return p
    return ImperfectionProfile(
        f"probe-{delay_s}", actuation_delay_s=delay_s,
        current_loop_tau_s=p.current_loop_tau_s,
        gyro_noise_rad_s=p.gyro_noise_rad_s,
        gyro_bias_rad_s=p.gyro_bias_rad_s,
        wheel_rate_quantum_rad_s=p.wheel_rate_quantum_rad_s,
        wheel_rate_update_hz=p.wheel_rate_update_hz,
    )


def strikes(model, *, burst_cycles: int = 0, burst_start_s: float = T0_S,
            delay_s: float | None = None,
            burst_period_s: float | None = None) -> tuple[bool, float]:
    """One run. Returns (nose_strike, peak_abs_pitch_deg)."""
    with RustController(**GAINS) as c:
        ctrl = (BurstHolder(c, burst_start_s, burst_cycles, burst_period_s)
                if burst_cycles else c)
        r = run(ImpulseParams(magnitude_ns=NOMINAL_IMPULSE_NS, sim_seconds=8.0),
                model=model, controller=ctrl, profile=_profile(delay_s))
    return bool(r.metrics.nose_strike), float(r.metrics.peak_abs_pitch_deg)


def bisect(model, probe, lo: int, hi: int) -> tuple[int, int]:
    """Largest surviving value and smallest fatal one, by bisection."""
    while hi - lo > 1:
        mid = (lo + hi) // 2
        if probe(mid):
            hi = mid
        else:
            lo = mid
    return lo, hi


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim/out/experiments")
    args = ap.parse_args()

    model = build_model(BALLAST_KG, BALLAST_M, CLAMP_A)

    # 1. Where does a burst hurt most? A miss only matters while the loop has
    #    work to do, so placement is swept before the boundary is bisected --
    #    bisecting at an arbitrary instant would report a boundary for that
    #    instant and quietly call it "the" boundary.
    placements = {}
    for offset_ms in (0, 10, 25, 50, 100, 200):
        t = T0_S + offset_ms / 1000.0
        struck, peak = strikes(model, burst_cycles=20, burst_start_s=t)
        placements[f"+{offset_ms} ms"] = {"struck": struck, "peak_deg": peak}
    worst_off = max(placements, key=lambda k: placements[k]["peak_deg"])
    worst_start = T0_S + float(worst_off.strip("+ ms")) / 1000.0

    # 2. The boundary, in cycles, at the worst placement.
    safe, fatal = bisect(
        model,
        lambda k: strikes(model, burst_cycles=k, burst_start_s=worst_start)[0],
        0, 400,
    )

    # 2b. Bisection ASSUMES the outcome is monotone in burst length. On a
    #     saturating, nonlinear plant that is an assumption, not a given, and
    #     an unchecked bisection on a non-monotone metric returns an arbitrary
    #     crossing dressed up as "the" boundary. Scanned linearly around the
    #     answer so the claim is falsifiable rather than asserted.
    scan = {}
    transitions = 0
    prev = None
    for k in range(max(0, safe - 14), safe + 27, 2):
        struck, peak = strikes(model, burst_cycles=k, burst_start_s=worst_start)
        scan[k] = {"struck": struck, "peak_deg": peak}
        if prev is not None and struck != prev:
            transitions += 1
        prev = struck

    # 3. AC4: is a burst of K cycles worse than K x 2 ms of CONSTANT delay?
    #    The useful output is the ratio, so both are measured the same way.
    delay_safe_s, delay_fatal_s = None, None
    lo, hi = 0.0, 0.200
    for _ in range(12):
        mid = 0.5 * (lo + hi)
        if strikes(model, delay_s=mid)[0]:
            hi = mid
        else:
            lo = mid
    delay_safe_s, delay_fatal_s = lo, hi

    burst_equiv_s = safe * DT_S
    ratio = burst_equiv_s / delay_safe_s if delay_safe_s else float("nan")

    # 4. The case the single-burst number does NOT cover, and the one a real
    #    stall actually has: a burst that RECURS. #130 names the mechanisms --
    #    a GC pause, a CAN retry storm, thermal throttling. None of them happen
    #    once. Measured at three repetition periods so the trend is visible
    #    rather than a single point being over-read.
    #    Periods are chosen LONGER than the single-burst limit (114 cycles =
    #    228 ms). A shorter period cannot answer the question: the burst can
    #    never exceed its own period, so the period itself becomes the binding
    #    cap and the run reports that cap rather than any property of the
    #    plant. A first pass at 20/50/100 ms did exactly that -- every answer
    #    came back at one cycle short of 100% duty, which is the cap talking,
    #    not the physics.
    repeating = {}
    for period_ms in (300, 500, 1000):
        cycles_in_period = int(round(period_ms / 1000.0 / DT_S))
        r_safe, r_fatal = bisect(
            model,
            lambda k, p=period_ms: strikes(
                model, burst_cycles=k, burst_start_s=worst_start,
                burst_period_s=p / 1000.0)[0],
            0, cycles_in_period,
        )
        repeating[f"every {period_ms} ms"] = {
            "last_safe_cycles": r_safe,
            "first_fatal_cycles": r_fatal,
            "cycles_in_period": cycles_in_period,
            "cap_bound_the_answer": r_fatal >= cycles_in_period,
            "vs_single_burst_cycles": r_safe - safe,
        }

    report = {
        "control_period_s": DT_S,
        "placement_sweep_20_cycles": placements,
        "worst_placement": worst_off,
        "burst_boundary_cycles": {"last_safe": safe, "first_fatal": fatal},
        "burst_boundary_ms": {"last_safe": safe * DT_S * 1e3,
                              "first_fatal": fatal * DT_S * 1e3},
        "constant_delay_boundary_ms": {"last_safe": delay_safe_s * 1e3,
                                       "first_fatal": delay_fatal_s * 1e3},
        "burst_over_constant_ratio": ratio,
        "monotonicity_scan": scan,
        "monotone": transitions == 1,
        "monotonicity_transitions": transitions,
        "repeating_bursts": repeating,
        # A repeating burst holds a SUPERSET of the cycles a single burst of
        # the same K holds, so it can never survive where the single one dies.
        # Where this reports otherwise, the failure is not "too many
        # consecutive misses" -- it is something the controller does AFTER a
        # long hold, which the next hold then interrupts. Reported, not
        # smoothed over: it means a bound pre-registered off the single-burst
        # number alone would be describing the wrong mechanism.
        "repeating_survives_where_single_dies": any(
            v["last_safe_cycles"] > safe for v in repeating.values()
        ),
        "verdict": (
            "a held burst is CHEAPER than the same duration of constant delay"
            if ratio > 1.0 else
            "a held burst is WORSE than the same duration of constant delay"
        ),
    }

    print(json.dumps(report, indent=2))
    args.out_dir.mkdir(parents=True, exist_ok=True)
    out = args.out_dir / "deadline_bursts.json"
    out.write_text(json.dumps(report, indent=2) + "\n")
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
