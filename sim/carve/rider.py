"""Closed-loop rider for the downhill carve.

Reads sim-host state (wire v3) on STATE_PORT, sends stick input (InputIn v1)
on INPUT_PORT at 50 Hz, and records every state packet to an .npz track.

The rider only moves sticks. MuJoCo (inside sim-host) computes all motion.

    python rider.py OUT.npz [v_ref] [amp_m] [wavelength_m]
"""
import json, os, socket, struct, sys, time
import numpy as np

STATE_PORT, INPUT_PORT = 9711, 9712
out = sys.argv[1]
V_REF = float(sys.argv[2]) if len(sys.argv) > 2 else 4.0
LOOKAHEAD = 4.0       # long enough for smooth arcs, short enough to reach the line
YAW_K_TOP = 1 / 6.67  # rad/m per unit steer at top speed (host.rs)
V_TOP = 9.34
RAMP_S = 8.0          # target speed rises over this many seconds
BRAKE_X = float(os.environ.get("BRAKE_X", "-46.0"))       # brake to a stop past this x (weak brake on this grade: ~0.7 m/s^2)
LEAN_RATIO = 0.8      # lateral weight shift per unit steer: lean into the turn

ST = struct.Struct('<IHHQd3f4f5f2f3f3f')
IN = struct.Struct('<IHHQfff')
COLS = 'flags seq t px py pz qw qx qy qz wheel_angle wheel_rate pitch yaw current rider_fa rider_lat vx vy vz wx wy wz'.split()

# Course inputs (defaults: the City Park carve road). LANE_JSON is the clean lane,
# COURSE_X0 the x where the path starts (s = X0 - x), LINE_JSON an optional line
# as [[s, offset fraction], ...].
lane = np.array(json.load(open(os.environ.get('LANE_JSON', '/tmp/carve/lane.json'))))  # x, ymin, ymax (x descending)
COURSE_X0 = float(os.environ.get('COURSE_X0', '0'))
lx = lane[::-1, 0]
lmid = ((lane[:, 1] + lane[:, 2]) / 2)[::-1]
lhalf = ((lane[:, 2] - lane[:, 1]) / 2 - 1.0)[::-1]  # usable half-width, 1 m clear of each edge

# The line, as (distance downhill m, offset as a fraction of the usable half-width).
# The road bends left (+y), so +y is the inside of the bend.
# Enter wide, three long linked carves, apex the inside of the bend, run out.
LINE = [(0, 0.0), (4, -0.1), (14, -0.95), (27, 0.95), (40, -0.8), (51, 0.7), (60, 0.3), (80, 0.3)]
if os.environ.get('LINE_JSON'):
    LINE = [tuple(p) for p in json.loads(os.environ['LINE_JSON'])]
_ls = np.array([p[0] for p in LINE], float)
_lo = np.array([p[1] for p in LINE], float) * float(os.environ.get("LINE_SCALE", "1.0"))


def _smooth_offset(s):
    """Monotone cubic (PCHIP) through the line points: no overshoot past a carve apex."""
    from scipy.interpolate import PchipInterpolator
    return PchipInterpolator(_ls, _lo)(np.clip(s, _ls[0], _ls[-1]))


_grid = np.linspace(0, max(80.0, _ls[-1]), 1601)
_off = _smooth_offset(_grid)


def y_ref(x):
    s = COURSE_X0 - x
    f = np.interp(s, _grid, _off)
    return np.interp(x, lx, lmid) + f * np.interp(x, lx, lhalf)


LEAN_KMAX = float(os.environ.get("LEAN_KMAX", "0"))  # set for sim-host --lean-steer
LOOK_TIME_S = float(os.environ.get("LOOK_TIME_S", "2.0"))
LEAN_BOUND = float(os.environ.get("LEAN_BOUND", "0.6"))
# A rider knows the grade under them and leans back by the amount that
# balances the holding torque: d = m g sin(a) r / (M g) = 0.154 sin(a) m,
# i.e. -3.08 sin(a) of the +-5 cm stick (grade + = downhill). Without it the
# speed loop over-braked by lean on a 6 % descent and the board tipped back.
_course = json.load(open(os.environ['COURSE_JSON'])) if os.environ.get('COURSE_JSON') else None
def grade_at(s_path):
    if not _course:
        return 0.0
    pr = _course['profile']
    return float(np.interp(s_path, pr['s_m'], pr['grade_pct'])) / 100.0


def kmax(v):
    """Curvature at full steer. With --lean-steer, `steer` is the rider's
    curvature intent scaled by LEAN_KMAX (lean_steer.rs kappa_max_per_m).
    Otherwise the commanded-yaw law: 1x at top speed, 2x at standstill."""
    if LEAN_KMAX > 0:
        return LEAN_KMAX
    return YAW_K_TOP * (2.0 - min(abs(v), V_TOP) / V_TOP)


rx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
rx.setsockopt(socket.SOL_SOCKET, socket.SO_RCVBUF, 8 << 20)
rx.bind(('127.0.0.1', STATE_PORT))
rx.setblocking(False)
tx = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

rows, last, seq = [], None, 0
integ = 0.0
v_prev = None
acc_f = 0.0
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
            # Lean-to-steer needs a firmer speed hold: a soft one let the speed
            # run to 6.6 m/s on the 6.5 % grade (measured).
            kp, ki = (0.35, 0.12) if (vr == 0 or LEAN_KMAX > 0) else (0.18, 0.05)
            # The -0.45 brake bias holds speed on the 6.5 % grade. It fades in
            # with speed so the rider pushes off forward from a flat start.
            bias = -0.45 * min(1.0, max(v, 0.0) / 2.0)
            # Push off gently: forward lean past ~0.1 saturates the motor at low
            # speed (the full-stick flip). On the hill gravity supplies the speed.
            if LEAN_KMAX > 0:
                # A balancing board brakes with a lag (it must pitch nose-up
                # first), so a plain PI rings: brake too late, then push
                # forward again (measured). Damp on acceleration, keep the
                # integral small, and never push forward once rolling.
                acc = (v - v_prev) / period if v_prev is not None else 0.0
                acc_f = acc_f + 0.1 * (acc - acc_f)
                # Forward lean is how a rider drives on the flat and up a climb;
                # a 0.1 cap stalled the board on a 2 % run-in (measured). The
                # acceleration damping keeps the push-off from surging.
                fwd_cap = float(os.environ.get("FWD_CAP", "0.6"))
                # Feel the motor working: as the current passes ~20 A the rider
                # stops asking for more forward lean, and past ~32 A leans back.
                # Without this, forward lean in the sag at the foot of the 12 %
                # descent drove the motor to 40 A and the nose into the ground
                # (measured) -- the onewheel "overpower nosedive".
                i_abs = abs(last[14])
                fwd_cap = min(fwd_cap, max(-0.3, fwd_cap * (1.0 - (i_abs - 20.0) / 12.0)))
                # No fixed brake bias here: it assumed the 6.5 % City Park grade
                # and stalled the board on a 2 % run-in (measured). The integral
                # finds the lean any grade needs.
                # Lean bounded to 3 cm (0.6 of the 5 cm range): the motor can
                # hold the rider's centre of mass at most ~3.7 cm off the
                # axle (28 N*m / (78 kg g)); asking for more tips the board.
                alpha = np.arctan(grade_at(COURSE_X0 - px + 0.5 * max(v, 0.0)))
                ff = -3.08 * np.sin(alpha)
                raw = ff + 0.30 * err + 0.10 * integ - 0.45 * acc_f
                fa = float(np.clip(raw, -LEAN_BOUND, min(fwd_cap, LEAN_BOUND)))
                if fa != raw:  # anti-windup: do not integrate into a stop
                    integ -= err * period
            else:
                fa = float(np.clip(kp * err + ki * integ + bias, -0.85, 0.10))
            v_prev = v
            # steer: pure pursuit to the reference path
            # Look further ahead at speed: a lean-steered board takes ~0.5 s to
            # build a turn, and a short look-ahead then weaves (measured).
            look = max(LOOKAHEAD, LOOK_TIME_S * abs(v)) if LEAN_KMAX > 0 else LOOKAHEAD
            tx_ = px - look
            tgt = np.array([tx_, y_ref(tx_)]) - np.array([px, py])
            ang = np.arctan2(fwd[0] * tgt[1] - fwd[1] * tgt[0], fwd @ tgt)  # +ve = target to the left
            k = 2 * np.sin(ang) / look
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
