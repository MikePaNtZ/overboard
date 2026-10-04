"""HUD fields for a recorded pose track: battery charge and voltage, the motor
current limit, Kt, torque, speed in mph, and the rider-warning margin.

Adds these columns to a track.npz (500 Hz, the track's own t), so a render or
the game can draw a HUD without a physics model of its own:

    speed_mph     forward speed from the wheel rate x 0.146 m, mph
    torque_nm     Kt x motor current, N.m (the plant's true Kt)
    torque_lim_nm Kt x the current limit, N.m (constant)
    i_limit       motor current limit, A (constant)
    kt            plant Kt, N.m/A (constant)
    batt_soc      battery state of charge, 0..1
    batt_v        pack voltage under load, V
    batt_i        pack current, A (negative = charging, regeneration)
    margin        rider-warning authority margin, 0..1 (control_core::AuthorityMargin)
    margin_level  0 none, 1 pulsed, 2 solid

The battery is the build's 20S2P pack of Molicel P50B cells (10 Ah): a
generic Li-ion open-circuit curve per cell, pack resistance 0.15 ohm, and the
power path of battery.py (motor copper loss, 15 W idle, 80 % regeneration
efficiency; rolling and air losses are already in the sim's current only for
battery.py's energy figures, so they are added here too). Charging current is limited
to 45 A (the pack's limit after the P50B rebuild), and the values hold after
the handoff latch (flags bit 4). Typical values, not measurements.

    $PY sim/carve/hud_fields.py RUN_DIR KT_NM_PER_A I_LIMIT_A [--soc0 0.9]
    $PY sim/carve/hud_fields.py --kinematic KIN_TRACK.npz OUT.npz [--soc0 0.9]
"""
import argparse
import csv
from pathlib import Path

import numpy as np

R_WHEEL = 0.146
SERIES, PARALLEL, CELL_AH = 20, 2, 5.0
R_PACK = 0.15
R_PHASE = 0.0525
P_IDLE, REGEN_EFF = 15.0, 0.8
MPH_PER_M_S = 2.23694
# Generic Li-ion cell OCV against state of charge (V).
SOC_PTS = np.array([0.0, 0.05, 0.1, 0.2, 0.4, 0.6, 0.8, 0.9, 1.0])
OCV_PTS = np.array([3.00, 3.30, 3.45, 3.58, 3.72, 3.87, 4.02, 4.10, 4.20])


REGEN_LIMIT_A = 45.0   # pack charge-current limit (P50B rebuild; stock 50S cells: 12 A)


def battery(t, amps, wheel_rate, kt, soc0, mass_kg=113.4, frozen=None):
    """Pack charge, voltage and current along a track. From the first sample
    where `frozen` is true (the handoff latch: the sim no longer rides the
    board) the values hold. Charging current is limited to REGEN_LIMIT_A."""
    v = wheel_rate * R_WHEEL
    i = amps + (0.015 * mass_kg * 9.81 + 0.5 * 1.2 * 0.5 * v ** 2) * R_WHEEL / kt * np.sign(v)
    p = kt * i * wheel_rate + 1.5 * R_PHASE * i ** 2 + P_IDLE
    p = np.where(p < 0, p * REGEN_EFF, p)
    soc = np.empty_like(t)
    vb = np.empty_like(t)
    ib = np.empty_like(t)
    q = soc0
    stop = int(np.argmax(frozen)) if frozen is not None and frozen.any() else len(t)
    for k in range(len(t)):
        if k >= stop:
            soc[k], vb[k], ib[k] = soc[stop - 1], vb[stop - 1], ib[stop - 1]
            continue
        ocv = SERIES * np.interp(q, SOC_PTS, OCV_PTS)
        # V = OCV - R I,  I = P / V  ->  R I^2 - OCV I + P = 0
        disc = max(ocv ** 2 - 4 * R_PACK * p[k], 0.0)
        ib[k] = max((ocv - np.sqrt(disc)) / (2 * R_PACK), -REGEN_LIMIT_A)
        vb[k] = ocv - R_PACK * ib[k]
        soc[k] = q
        dt = t[k + 1] - t[k] if k + 1 < len(t) else 0.0
        q = min(1.0, max(0.0, q - ib[k] * dt / 3600.0 / (CELL_AH * PARALLEL)))
    return soc, vb, ib


def augment(track_path, kt, i_limit, soc0, trace=None, out_path=None):
    z = dict(np.load(track_path))
    t, amps, w = z['t'], z['current'], z['wheel_rate']
    handoff = (z['flags'].astype(int) & 0x10) > 0
    soc, vb, ib = battery(t, amps, w, kt, soc0, frozen=handoff)
    n = len(t)
    z.update(speed_mph=w * R_WHEEL * MPH_PER_M_S, torque_nm=kt * amps,
             torque_lim_nm=np.full(n, kt * i_limit), i_limit=np.full(n, float(i_limit)),
             kt=np.full(n, float(kt)), batt_soc=soc, batt_v=vb, batt_i=ib)
    if trace is not None:
        rows = list(csv.DictReader(open(trace)))
        tt = np.array([float(r['sim_time_s']) for r in rows])
        j = np.nonzero(np.diff(tt) < 0)[0]
        m = j[0] + 1 if len(j) else len(tt)
        mg = np.array([float(r.get('margin') or 0.0) for r in rows])[:m]
        lv = np.array([float(r.get('margin_level') or 0.0) for r in rows])[:m]
        z['margin'] = np.interp(t, tt[:m], mg)
        z['margin_level'] = np.round(np.interp(t, tt[:m], lv)).astype(np.float64)
    else:
        z['margin'] = np.zeros(n)
        z['margin_level'] = np.zeros(n)
    np.savez(out_path or track_path, **z)
    return soc[0], soc[-1], vb.min(), vb.max()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('args', nargs='+')
    ap.add_argument('--kinematic', action='store_true')
    ap.add_argument('--soc0', type=float, default=0.9)
    a = ap.parse_args()
    if a.kinematic:
        src, dst = a.args
        r = augment(src, 0.658, 90.0, a.soc0, out_path=dst)
    else:
        d, kt, il = Path(a.args[0]), float(a.args[1]), float(a.args[2])
        for name in ('track.npz', 'track_full.npz'):
            if (d / name).exists():
                r = augment(d / name, kt, il, a.soc0, trace=d / 'trace.csv')
    print('soc %.3f -> %.3f, pack %.1f..%.1f V' % r)


if __name__ == '__main__':
    main()
