"""Electrical post-processing of sim runs: motor and battery load, for sizing.

The sim gives the motor current i (A, torque = KT*i) and the wheel speed.
This adds a hub motor and a pack:

  omega      = v / R_WHEEL
  V_motor    = KE*omega + R_PHASE*i            (q-axis, steady state)
  P_mech     = KT*i*omega                      (negative = regeneration)
  i         += (Crr*m*g + 0.5*rho*CdA*v**2)*R_WHEEL/KT   (losses the sim omits)
  P_copper   = 1.5*R_PHASE*i**2                 (three-phase, i is the peak phase current)
  P_battery  = P_mech + P_copper + P_IDLE      (regeneration flows back, at REGEN_EFF)
  I_battery  = P_battery / V_pack,  V_pack = OCV(SoC) - R_PACK*I_battery

Defaults are typical, not measured: a Onewheel-class hub motor (KT = KE =
0.7, the value the controller uses), and a 20S2P pack of 21700 cells
(Samsung 40T class: 4.0 Ah, 15 mOhm). Change them with flags when the kit is
chosen.

    $PY sim/carve/battery.py RUN.csv [RUN2.csv ...]     # one line per run
"""
import argparse
import csv

import numpy as np

R_WHEEL = 0.1454


def load(path, s_range=None):
    rows = list(csv.DictReader(open(path)))
    g = lambda k: np.array([float(r[k]) for r in rows])
    t, i, v = g('sim_time_s'), g('applied_amps'), g('forward_speed_m_s')
    s = 90.0 - g('pos_x_m')
    fallen = g('fallen') > 0
    n = np.nonzero(np.diff(t) < 0)[0]
    n = n[0] + 1 if len(n) else len(t)
    # Stop at the first fall: after the hand-off the trace is not a ride.
    f = np.nonzero(fallen[:n])[0]
    n = f[0] if len(f) else n
    ok = np.ones(n, bool)
    if s_range:
        ok &= (s[:n] >= s_range[0]) & (s[:n] <= s_range[1])
    return t[:n][ok], i[:n][ok], v[:n][ok]


def analyse(t, i, v, a):
    w = v / R_WHEEL
    v_motor = a.ke * w + a.r_phase * i
    # The MuJoCo contact has no rolling resistance (condim 3) and the model
    # has no air drag, so the motor never pays for them. Add them here.
    f_loss = a.crr * a.mass * 9.81 + 0.5 * 1.2 * a.cda * v ** 2
    i = i + f_loss * R_WHEEL / a.kt * np.sign(v)
    v_motor = a.ke * w + a.r_phase * i
    p_mech = a.kt * i * w
    p_cu = 1.5 * a.r_phase * i ** 2
    p_raw = p_mech + p_cu + a.p_idle
    p_batt = np.where(p_raw < 0, p_raw * a.regen_eff, p_raw)
    ocv = a.series * 3.6
    r_pack = a.series * a.r_cell / a.parallel
    # V = OCV - R*I, I = P/V  ->  R*I^2 - OCV*I + P = 0
    disc = np.maximum(ocv ** 2 - 4 * r_pack * p_batt, 0.0)
    i_batt = (ocv - np.sqrt(disc)) / (2 * r_pack)
    v_pack = ocv - r_pack * i_batt
    dt = np.diff(t, prepend=t[0])
    dt[dt > 0.01] = 0.0  # a window that skips samples must not count the gap
    dist_km = np.sum(np.abs(v) * dt) / 1000.0
    wh = np.sum(p_batt * dt) / 3600.0
    pack_wh = a.series * a.parallel * a.cell_ah * 3.6
    cell_a = i_batt / a.parallel
    return dict(
        dist_km=dist_km, wh=wh, wh_per_km=wh / dist_km if dist_km > 0 else np.nan,
        p_peak_w=p_batt.max(), p_regen_w=p_batt.min(),
        i_batt_peak=i_batt.max(), cell_a_peak=cell_a.max(),
        headroom=np.min(v_pack * a.max_duty - np.abs(v_motor)),
        range_km=0.8 * pack_wh / (wh / dist_km) if wh > 0 and dist_km > 0 else np.inf,
        pack_wh=pack_wh,
    )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('runs', nargs='+')
    ap.add_argument('--kt', type=float, default=0.7)
    ap.add_argument('--ke', type=float, default=0.7)
    ap.add_argument('--r-phase', type=float, default=0.12)
    ap.add_argument('--p-idle', type=float, default=15.0, help='controller + electronics, W')
    ap.add_argument('--regen-eff', type=float, default=0.8)
    ap.add_argument('--series', type=int, default=20)
    ap.add_argument('--parallel', type=int, default=2)
    ap.add_argument('--cell-ah', type=float, default=4.0)
    ap.add_argument('--r-cell', type=float, default=0.015)
    ap.add_argument('--max-duty', type=float, default=0.9)
    ap.add_argument('--mass', type=float, default=83.0, help='board + rider, kg (the sim value)')
    ap.add_argument('--crr', type=float, default=0.015, help='tyre rolling resistance')
    ap.add_argument('--cda', type=float, default=0.5, help='rider drag area, m2')
    ap.add_argument('--s-range', type=float, nargs=2, help='course window, m (s = 90 - x)')
    a = ap.parse_args()
    print(f"pack {a.series}S{a.parallel}P, {a.series * a.parallel * a.cell_ah * 3.6:.0f} Wh nominal")
    for p in a.runs:
        name = p.rsplit('/', 1)[-1].removesuffix('.csv')
        run = load(p, a.s_range)
        if len(run[0]) < 2:
            print(f"{name:14s} no samples in the window (fell first?)")
            continue
        r = analyse(*run, a)
        print(f"{name:14s} {r['wh_per_km']:6.1f} Wh/km  P peak {r['p_peak_w']:6.0f} W  "
              f"P min {r["p_regen_w"]:6.0f} W  cell peak {r['cell_a_peak']:5.1f} A  "
              f"voltage headroom {r['headroom']:5.1f} V  range(80%) {r['range_km']:5.1f} km")


if __name__ == '__main__':
    main()
