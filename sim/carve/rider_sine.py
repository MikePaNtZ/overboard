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
V_REF = float(sys.argv[2]) if len(sys.argv) > 2 else 3.0
AMP = float(sys.argv[3]) if len(sys.argv) > 3 else 1.5
WAVE = float(sys.argv[4]) if len(sys.argv) > 4 else 16.0
RUN_IN_M = 6.0        # straight distance before the weave starts
END_X = -66.0         # stop the weave and brake past this x
LOOKAHEAD = 3.5
YAW_K_TOP = 1 / 6.67  # rad/m per unit steer at top speed (host.rs)
V_TOP = 9.34

ST = struct.Struct('<IHHQd3f4f5f2f3f3f')
IN = struct.Struct('<IHHQfff')
COLS = 'flags seq t px py pz qw qx qy qz wheel_angle wheel_rate pitch yaw current rider_fa rider_lat vx vy vz wx wy wz'.split()

lane = np.array(json.load(open('/tmp/carve/lane.json')))  # x, ymin, ymax (x descending)
lx, lmid = lane[::-1, 0], ((lane[:, 1] + lane[:, 2]) / 2)[::-1]


def y_ref(x):
    """Lane centre plus a sine weave that starts after the run-in."""
    s = -x  # distance downhill
    w = 0.0 if s < RUN_IN_M else AMP * np.sin(2 * np.pi * (s - RUN_IN_M) / WAVE)
    if x < END_X:
        w = 0.0
    return np.interp(x, lx, lmid) + w


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
        v = float(np.hypot(vx, vy))
        # board forward is body -X; rotate into world
        R = np.array([
            [1 - 2 * (qy * qy + qz * qz), 2 * (qx * qy - qz * qw), 2 * (qx * qz + qy * qw)],
            [2 * (qx * qy + qz * qw), 1 - 2 * (qx * qx + qz * qz), 2 * (qy * qz - qx * qw)],
            [2 * (qx * qz - qy * qw), 2 * (qy * qz + qx * qw), 1 - 2 * (qx * qx + qy * qy)]])
        fwd = R @ np.array([-1.0, 0.0, 0.0])
        fwd = fwd[:2] / np.linalg.norm(fwd[:2])
        if t > 0.0:
            # speed: PI on fore/aft lean; brake hard after the end of the run
            vr = V_REF if px > END_X else 0.0
            err = vr - v
            integ = np.clip(integ + err * period, -3, 3)
            fa = float(np.clip(0.35 * err + 0.12 * integ - 0.45, -0.85, 0.35))
            # steer: pure pursuit to the reference path
            tx_ = px - LOOKAHEAD
            tgt = np.array([tx_, y_ref(tx_)]) - np.array([px, py])
            ang = np.arctan2(fwd[0] * tgt[1] - fwd[1] * tgt[0], fwd @ tgt)  # +ve = target to the left
            k = 2 * np.sin(ang) / LOOKAHEAD
            steer = float(np.clip(-k / kmax(max(v, 0.5)), -1, 1))  # +ve steer turns right
            lat = 0.6 * steer
    tx.sendto(IN.pack(0x4F424931, 1, 0, seq, fa, lat, steer), ('127.0.0.1', INPUT_PORT))
    seq += 1

a = np.array(rows, dtype=np.float64)
np.savez(out, **{c: a[:, i] for i, c in enumerate(COLS)})
s = a[:, 1]
fell = (a[:, 0].astype(int) & 4) > 0
print(f"{len(a)} packets, dropped {int(s[-1] - s[0] + 1 - len(a))}, "
      f"fell {'at %.2fs' % a[fell, 2][0] if fell.any() else 'never'}, max speed {np.hypot(a[:, 17], a[:, 18]).max():.2f}")
