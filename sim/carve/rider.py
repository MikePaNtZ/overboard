"""Closed-loop rider for the downhill carve.

Reads sim-host state (wire v3) on STATE_PORT, sends stick input (InputIn v1)
on INPUT_PORT at 50 Hz, and records every state packet to an .npz track.

The rider only moves sticks. MuJoCo (inside sim-host) computes all motion.

    python rider.py OUT.npz [v_ref] [amp_m] [wavelength_m]
"""
import json, socket, struct, sys, time
import numpy as np

STATE_PORT, INPUT_PORT = 9711, 9712
out = sys.argv[1]
V_REF = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
LOOKAHEAD = 4.0       # long enough for smooth arcs, short enough to reach the line
YAW_K_TOP = 1 / 6.67  # rad/m per unit steer at top speed (host.rs)
V_TOP = 9.34
RAMP_S = 4.0          # target speed rises over this many seconds
BRAKE_X = -46.0       # brake to a stop past this x (weak brake on this grade: ~0.7 m/s^2)
LEAN_RATIO = 0.8      # lateral weight shift per unit steer: lean into the turn

ST = struct.Struct('<IHHQd3f4f5f2f3f3f')
IN = struct.Struct('<IHHQfff')
COLS = 'flags seq t px py pz qw qx qy qz wheel_angle wheel_rate pitch yaw current rider_fa rider_lat vx vy vz wx wy wz'.split()

lane = np.array(json.load(open('/tmp/carve/lane.json')))  # x, ymin, ymax (x descending)
lx = lane[::-1, 0]
lmid = ((lane[:, 1] + lane[:, 2]) / 2)[::-1]
lhalf = ((lane[:, 2] - lane[:, 1]) / 2 - 1.0)[::-1]  # usable half-width, 1 m clear of each edge

# The line, as (distance downhill m, offset as a fraction of the usable half-width).
# The road bends left (+y), so +y is the inside of the bend.
# Enter wide, three long linked carves, apex the inside of the bend, run out.
LINE = [(0, 0.0), (4, -0.1), (14, -0.95), (27, 0.95), (40, -0.8), (51, 0.7), (60, 0.3), (80, 0.3)]
_ls = np.array([p[0] for p in LINE], float)
_lo = np.array([p[1] for p in LINE], float)


def _smooth_offset(s):
    """Monotone cubic (PCHIP) through the line points: no overshoot past a carve apex."""
    from scipy.interpolate import PchipInterpolator
    return PchipInterpolator(_ls, _lo)(np.clip(s, _ls[0], _ls[-1]))


_grid = np.linspace(0, 80, 801)
_off = _smooth_offset(_grid)


def y_ref(x):
    s = -x
    f = np.interp(s, _grid, _off)
    return np.interp(x, lx, lmid) + f * np.interp(x, lx, lhalf)


def kmax(v):
    """Curvature at full steer: 1x at top speed, 2x at standstill (host.rs)."""
    return YAW_K_TOP * (2.0 - min(abs(v), V_TOP) / V_TOP)


rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
rx.bind(('127.0.0.1', STATE_PORT))
rx.setblocking(False)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rows, last, seq = [], None, 0
integ = 0.0
t0 = None
period = 0.02
next_send = time.perf_counter()
idle_since = time.perf_counter()
while True:
    while True:
        try:
            b, _ = rx.recvfrom(256)
        except BlockingIOError:
            break
        if len(b) == 104:
            last = ST.unpack(b)
            rows.append(last[2:])
            idle_since = time.perf_counter()
    now = time.perf_counter()
    if rows and now - idle_since > 2.0:
        break
    if now < next_send:
        time.sleep(min(0.002, next_send - now))
        continue
    next_send += period  # absolute pacing: no drift from sleep granularity
    fa = lat = steer = 0.0
    if last is not None:
        _, _, flags, _, t, px, py, pz, qw, qx, qy, qz, *_r = last
        vx, vy = last[19], last[20]
        # board forward is body -X; rotate into world
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        fwd = R @ np.array([-1.0, 0.0, 0.0])
        fwd = fwd[:2] / np.linalg.norm(fwd[:2])
        v = float(fwd @ np.array([vx, vy]))  # signed: +ve = forward
        if t > 0.0:
            # speed: PI on fore/aft lean; brake hard after the end of the run
            # A soft speed hold: the speed rises down the fall line and falls
            # across it, as a carving rider's does. Firm braking only at the end.
            vr = V_REF * min(1.0, t / RAMP_S) if px > BRAKE_X else 0.0
            err = vr - v
            integ = np.clip(integ + err * period, -3, 3)
            kp, ki = (0.18, 0.05) if vr > 0 else (0.35, 0.12)
            fa = float(np.clip(kp * err + ki * integ - 0.45, -0.85, 0.25))
            # steer: pure pursuit to the reference path
            tx_ = px - LOOKAHEAD
            tgt = np.array([tx_, y_ref(tx_)]) - np.array([px, py])
            ang = np.arctan2(fwd[0] * tgt[1] - fwd[1] * tgt[0], fwd @ tgt)  # +ve = target to the left
            k = 2 * np.sin(ang) / LOOKAHEAD
            steer = float(np.clip(-k / kmax(max(v, 0.5)), -1, 1))  # +ve steer turns right
            lat = LEAN_RATIO * steer
    tx.sendto(IN.pack(0x4F424931, 1, 0, seq, fa, lat, steer), ('127.0.0.1', INPUT_PORT))
    seq += 1

a = np.array(rows, dtype=np.float64)
np.savez(out, **{c: a[:, i] for i, c in enumerate(COLS)})
s = a[:, 1]
fell = (a[:, 0].astype(int) & 4) > 0
print(f"{len(a)} packets, dropped {int(s[-1] - s[0] + 1 - len(a))}, "
      f"fell {'at %.2fs' % a[fell, 2][0] if fell.any() else 'never'}, max speed {np.hypot(a[:, 17], a[:, 18]).max():.2f}")
