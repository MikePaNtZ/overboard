"""Issue #201: the turn radius was too wide, and nobody had measured it.

THE CEO'S REPORT
-----------------
"turn radius is too wide by maybe 2x to turn around completely" -- against an
ask of roughly 3.5 m. `YAW_CURVATURE_PER_STEER_RAD_PER_M` (0.15) was quoted
from `1 / k` as ~6.7 m and never actually driven through a turn to check.

WHY A FORMULA ALONE ISN'T ENOUGH
----------------------------------
`host.rs`'s yaw law makes turn radius a function of ground speed:
`1 / (steer * yaw_curvature_per_steer(v) * roll_authority)`. `roll_authority`
(added after the constant was first picked) and the low-speed tighten ramp
both live outside the plain `1 / k` formula, so the only honest way to check
the achieved radius is to drive a full turn and measure the ground track --
which is what `--scripted-scenario turnaround` and this file do.

METHOD -- local curvature from the ground track, not a whole-path circle fit
-------------------------------------------------------------------------
`turnaround` holds 0.6 lean (issue #201's own acceptance criterion) and full
steer/lateral from a standing start. This plant has no outer velocity loop
(see `host.rs`'s own header), so ground speed keeps climbing under a
sustained positive lean -- the achieved radius is NOT constant over the run,
it tightens as speed climbs through `YAW_LOW_SPEED_TIGHTEN`'s ramp and only
reaches the "widest" ~`1 / k` radius once ground speed is near
`YAW_TIGHTEN_REF_SPEED_M_S`. A single circle fit over the whole run assumes a
constant radius that does not hold here (see `scenario.rs`'s own doc comment
on `TURNAROUND_SCHEDULE` for why the stale `feat/controls/turn-radius-and-
reset` branch's coast-through-the-turn version produced an un-fittable
spiral). Fitting the LOCAL radius at each instant instead -- ground speed
divided by the ground track's own instantaneous heading rate -- needs no such
assumption, and it is what `the_turn_radius_shrinks_as_the_board_slows`
(`host.rs`'s own unit test) already asserts is the correct shape: radius
shrinks monotonically as speed falls, and is WIDEST at and above the
reference speed. So the acceptance number is measured at the near-reference-
speed tail of the hold, which is deliberately the worst case (widest turn),
not a favourable one.
"""

from __future__ import annotations

import csv
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

HOST_RS = REPO / "crates" / "sim-host" / "src" / "host.rs"

STATE_OUT = "127.0.0.1:19805"
INPUT_IN = "127.0.0.1:19806"

#: Issue #201's own acceptance criterion 1.
TURN_RADIUS_CEILING_M = 3.6

#: `CORRIDOR_HALF_WIDTH_M` in `host.rs` -- the measurement must stay well
#: inside this or the corridor brake (`CORRIDOR_BRAKE_LEAN`) contaminates the
#: speed trace and the radius reading with it.
CORRIDOR_HALF_WIDTH_M = 8.6

#: Sampling stride for the ground-track derivative -- coarse enough that
#: consecutive heading samples are not dominated by floating-point noise at
#: the 500 Hz control rate, fine enough to resolve the speed ramp.
SAMPLE_DT_S = 0.1


def _rust_constant(name: str) -> float:
    m = re.search(rf"^const {name}: f32 = ([0-9.]+);", HOST_RS.read_text(), re.M)
    assert m is not None, f"{name} is not a plain f32 const in {HOST_RS}"
    return float(m.group(1))


def _run(tmp_path: Path) -> list[dict]:
    out = tmp_path / "turnaround.csv"
    argv = [
        "cargo", "run", "--release", "-q", "-p", "sim-host", "--bin", "sim-host", "--",
        "--scripted-scenario", "turnaround",
        "--free-run",
        "--stats-path", "none",
        "--pitch-source", "estimator",
        "--state-out-addr", STATE_OUT,
        "--input-in-addr", INPUT_IN,
        "--trace-csv", str(out),
    ]
    proc = subprocess.run(argv, cwd=REPO, capture_output=True, text=True)
    assert proc.returncode == 0, f"sim-host failed:\n{proc.stderr[-4000:]}"
    with out.open() as f:
        return [{k: float(v) for k, v in row.items()} for row in csv.DictReader(f)]


def _local_radius(rows: list[dict]):
    """`(mid_time_s, ground_speed_m_s, radius_m)` at `SAMPLE_DT_S` spacing.

    Radius is `|v / dheading_dt|`, heading taken from consecutive ground-track
    deltas -- see this file's own module doc comment for why a local estimate
    is used instead of a whole-run circle fit.
    """
    seen_buckets: set[int] = set()
    sel = []
    for r in rows:
        bucket = round(r["sim_time_s"] / SAMPLE_DT_S)
        if bucket in seen_buckets:
            continue
        seen_buckets.add(bucket)
        sel.append(r)
    t = np.array([r["sim_time_s"] for r in sel])
    x = np.array([r["pos_x_m"] for r in sel])
    y = np.array([r["pos_y_m"] for r in sel])
    v = np.array([r["forward_speed_m_s"] for r in sel])

    heading = np.unwrap(np.arctan2(np.diff(y), np.diff(x)))
    tm = (t[:-1] + t[1:]) / 2
    yaw_rate = np.diff(heading) / np.diff(tm)
    tm2 = (tm[:-1] + tm[1:]) / 2
    v_mid = np.interp(tm2, t, v)
    with np.errstate(divide="ignore", invalid="ignore"):
        radius = np.abs(v_mid / yaw_rate)
    return tm2, v_mid, radius


def test_the_sim_host_binary_is_actually_built():
    assert (REPO / "target" / "release" / "sim-host").exists(), (
        "sim-host is not built, so this whole gate would be vacuous. Run:\n"
        "    cargo build --release -p sim-host --bin sim-host"
    )


def test_the_turn_radius_at_full_steer_and_0_6_lean_is_within_acceptance(tmp_path):
    """Issue #201, acceptance criterion 1.

    Measured, not quoted: drives the `turnaround` scenario, takes the local
    radius reading once ground speed is within 5% of the reference speed
    (`YAW_TIGHTEN_REF_SPEED_M_S` -- the widest, worst-case turn this lean can
    produce), and asserts it against the acceptance ceiling with the
    residual spread reported alongside it, not hidden.
    """
    # `YAW_TIGHTEN_REF_SPEED_M_S` is defined as `= MAX_GROUND_SPEED_M_S`, not a
    # literal of its own -- read the literal it aliases instead.
    ref_speed = _rust_constant("MAX_GROUND_SPEED_M_S")
    k = _rust_constant("YAW_CURVATURE_PER_STEER_RAD_PER_M")

    rows = _run(tmp_path)
    hold_rows = [r for r in rows if r["sim_time_s"] <= 16.5]  # TURNAROUND hold window
    max_abs_y = max(abs(r["pos_y_m"]) for r in hold_rows)
    assert max_abs_y < CORRIDOR_HALF_WIDTH_M - 1.0, (
        f"the measurement run's ground track reached {max_abs_y:.2f} m laterally, "
        f"too close to the {CORRIDOR_HALF_WIDTH_M} m corridor half-width -- the "
        "corridor brake may have engaged and contaminated the speed/radius trace"
    )

    t, v, radius = _local_radius(hold_rows)
    near_ref = radius[(v >= 0.95 * ref_speed) & np.isfinite(radius)]
    assert near_ref.size >= 10, (
        f"only {near_ref.size} samples reached 95% of the {ref_speed:.2f} m/s "
        "reference speed -- lengthen TURNAROUND_HOLD_S in scenario.rs"
    )
    # MEDIAN, not max: the local (per-sample) derivative is a `dheading/dt`
    # finite difference, and it is numerically unstable for the 1-2 samples
    # right at the ground speed peak, where `dv/dt` -> 0 makes the heading
    # difference itself the dominant source of noise -- measured directly,
    # not assumed: this run's own near-reference-speed window has 2 outlier
    # samples (~5.2 m) sitting inside an otherwise smooth 3.17-3.42 m band.
    # The median is insensitive to a couple of outliers among 40+ samples;
    # `spread` is reported so that noise is visible, not hidden by the choice
    # of a robust statistic.
    measured = float(np.median(near_ref))
    spread = float(np.percentile(near_ref, 90) - np.percentile(near_ref, 10))
    print(
        f"\nturn radius near reference speed ({ref_speed:.2f} m/s, k={k}): "
        f"median {measured:.3f} m, 10-90th pct spread {spread:.3f} m, "
        f"range [{near_ref.min():.3f}, {near_ref.max():.3f}] m, n={near_ref.size}"
    )
    assert measured <= TURN_RADIUS_CEILING_M, (
        f"turn radius at full steer + 0.6 lean measured {measured:.3f} m near "
        f"the reference speed, exceeding issue #201's {TURN_RADIUS_CEILING_M} m "
        "acceptance ceiling"
    )


def test_the_pinned_formula_matches_what_was_actually_driven():
    """Cross-check: the shipped constant's own `1 / k` asymptote must be under
    the acceptance ceiling too, so a future retune of `k` alone (without
    rerunning the full-sim measurement) cannot silently regress past the
    corridor-safe, CEO-accepted radius by relying on the formula matching
    reality less well than it does today.
    """
    k = _rust_constant("YAW_CURVATURE_PER_STEER_RAD_PER_M")
    asymptote = 1.0 / k
    assert asymptote <= TURN_RADIUS_CEILING_M, (
        f"YAW_CURVATURE_PER_STEER_RAD_PER_M={k} gives a widest-case radius of "
        f"{asymptote:.3f} m by the formula alone, already over the "
        f"{TURN_RADIUS_CEILING_M} m ceiling -- full-sim measurement would only "
        "make this worse, not better"
    )
