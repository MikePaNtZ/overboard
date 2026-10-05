"""Physics-informed KINEMATIC pose track for the Overboard valley render.

Blog render content: this does NOT run the MuJoCo controller. It builds a believable
ride (carving line, speed from a point-mass energy balance, bank from v^2*kappa/g) over a
course made by course.py, and writes an npz in the carve-lab track format that
overboard-game-render/tools/replay/npz_replay.py turns into a wire-v3 .bin.

    python kinematic_track.py COURSE_DIR [OUT.npz] [--amp 0.8 ...]

Conventions (measured on a05_track.npz, all float64, 500 Hz, t starts at 0.002):
  * Board forward is -X. The body +X axis points REARWARD (world +X at yaw 0).
  * yaw = atan2(-dy, -dx) of the travel direction (heading + pi), so yaw = 0 when riding -X.
  * q = Rz(yaw) Ry(pitch) Rx(roll) (ZYX). pitch = elevation of the FORWARD direction
    (positive = nose up = riding uphill). yaw = atan2(R10, R00), pitch = -asin(R20).
  * roll = +atan(v * dyaw/dt / g), a05 regression slope 0.121 (1/g = 0.102), corr 0.93.
  * wx, wy, wz = d(roll)/dt, d(pitch)/dt, d(yaw)/dt. vx, vy, vz = d(px, py, pz)/dt (world).
  * wheel_angle grows when riding forward: wheel_rate = v / 0.1454.
  * current, rider_fa are positive with forward acceleration, rider_lat with roll.
Height: heightfield[row = y, col = x], post spacing and centre from metadata.json
(x = (col - (ncol-1)/2) * spacing).
"""
import argparse, json, subprocess, sys
from pathlib import Path
import numpy as np
from scipy import ndimage as nd
from scipy.interpolate import CubicSpline

G = 9.81
R_W = 0.1454          # rolling radius, m
R_T = 0.099           # tyre crown radius, m
KT = 0.7              # N.m/A
X0 = 90.0             # path start, MuJoCo x (course.py); s = X0 - px
DT = 0.002            # 500 Hz, as a05
REPLAY = "/Users/mike/projects/overboard-game-render/tools/replay/npz_replay.py"


def smoothstep(x):
    x = np.clip(x, 0.0, 1.0)
    return x * x * x * (x * (6 * x - 15) + 10)


def load_course(cdir):
    meta = json.loads((cdir / "metadata.json").read_text())
    h = np.load(cdir / "course_height.npy")
    course = json.loads((cdir / "course.json").read_text())
    lane = json.loads((cdir / "lane.json").read_text())
    return meta, h, course, lane


def make_terrain(meta, h):
    c = (meta["ncol"] - 1) / 2.0
    sp = meta["spacing_m"]

    def terrain(x, y):
        return nd.map_coordinates(h, [np.asarray(y) / sp + c, np.asarray(x) / sp + c],
                                  order=1, mode="nearest")
    return terrain


def build_path(course, lane, a):
    """Lateral offset y(s) as a smooth S-curve line; returns arclength-gridded path."""
    segs = course["segments"]
    bounds = np.cumsum([0.0] + [sg["length_m"] for sg in segs])
    total = bounds[-1]
    s = np.arange(0.0, total + 1e-9, 0.05)
    # Envelope: straight at the sag/valley floor and on the climb, full on the descent.
    names = [sg["name"] for sg in segs]
    b = dict(zip(names, zip(bounds[:-1], bounds[1:])))
    d0, d1 = b["descent"]
    f0, f1 = b["valley floor"]
    c0, c1 = b["climb"]
    env = np.ones_like(s)
    env *= smoothstep((s - 4.0) / 10.0)                               # straight start
    env *= 1.0 - (1.0 - a.floor_amp) * smoothstep((s - (f0 - 8.0)) / 10.0)   # into the sag
    env += (a.climb_amp - a.floor_amp) * smoothstep((s - (c0 - 2.0)) / 14.0) \
        * smoothstep((s - 4.0) / 10.0)                                # climb: a little weave
    env *= 1.0 - smoothstep((s - (total - 3.0 - 12.0)) / 10.0)        # straight for the stop
    # Wavelength drifts 25..35 m; phase is its integral.
    lam = 30.0 + 5.0 * np.sin(2 * np.pi * s / 70.0)
    phase = np.cumsum(2 * np.pi / lam) * 0.05
    y = a.amp * env * np.sin(phase)
    y = nd.gaussian_filter1d(y, 1.5 / 0.05, mode="nearest")           # 1.5 m smoothing
    half = min(abs(l[1]) for l in lane)
    assert np.abs(y).max() < half - 0.5, "path leaves the lane"
    sp = CubicSpline(s[::10], y[::10], bc_type="natural")             # C2: continuous kappa
    yy, y1, y2 = sp(s), sp(s, 1), sp(s, 2)
    dl = np.sqrt(1 + y1 ** 2) * 0.05
    L = np.concatenate([[0.0], np.cumsum(0.5 * (dl[1:] + dl[:-1]))])  # arclength at each s
    yaw = -np.arctan(y1)                                              # atan2(-dy, -dx), dx<0
    dyaw_ds = -y2 / (1 + y1 ** 2)
    dyaw_dl = dyaw_ds / np.sqrt(1 + y1 ** 2)
    return dict(s=s, L=L, px=X0 - s, py=yy, yaw=yaw, dyaw_dl=dyaw_dl, total_l=L[-1])


def simulate(P, grade_dl, a):
    """Point-mass ride along arclength l. Returns time series (500 Hz) of l, v, a_long, a_drive."""
    m, crr, rho, cda = a.mass, a.crr, 1.2, a.cda
    l0 = float(np.interp(a.start_s, P["s"], P["L"]))
    l_stop = P["total_l"] - a.end_margin
    l_grid = P["L"]
    l, v, ad = l0, a.v0, 0.0
    A_END = a.stop_decel
    L_, V_, A_, F_ = [], [], [], []
    ending = False
    hold = int(1.5 / DT)
    while True:
        g = np.interp(l, l_grid, grade_dl)            # downhill-positive slope dz/dl
        sina = g / np.sqrt(1 + g * g)
        cosa = 1.0 / np.sqrt(1 + g * g)
        a_res = -G * crr * cosa - 0.5 * rho * cda * v * v / m
        a_g = G * sina
        rem = l_stop - l
        if not ending and np.sqrt(2 * A_END * max(rem, 0)) <= v and rem < 40:
            ending = True
        if ending:
            vn = np.sqrt(2 * A_END * max(rem, 0.0))
            vn = min(vn, v)
            acc = (vn - v) / DT
            v_new = vn
            ad_cmd = 0.0
            F_.append(m * acc - m * (a_g + a_res))    # wheel force that realises it
        else:
            # drive towards cruise; accel and power limited, jerk limited
            ad_cmd = np.clip(a.kp * (a.cruise - v), 0.0, a.drive_acc)
            ad_cmd = min(ad_cmd, a.power / (m * max(v, 0.5)))
            ad += np.clip(ad_cmd - ad, -a.jerk * DT, a.jerk * DT)
            ab = np.clip(8.0 * (v - (a.vmax - 0.3)), 0.0, a.brake_decel)
            acc = a_g + a_res + ad - ab
            if ad > 0:
                acc = min(acc, a.drive_acc) if acc > a.drive_acc else acc
            v_new = v + acc * DT
            F_.append(m * (ad - ab))
        L_.append(l); V_.append(v); A_.append(acc)
        l += 0.5 * (v + v_new) * DT
        v = max(v_new, 0.0)
        if ending and (v < 0.003 or rem <= 0.0):
            v = 0.0
            for _ in range(hold):
                L_.append(l); V_.append(0.0); A_.append(0.0); F_.append(0.0)
            break
        if len(L_) > 400000:
            raise RuntimeError("did not stop")
    return np.array(L_), np.array(V_), np.array(A_), np.array(F_)


def quat_zyx(yaw, pitch, roll):
    cy, sy = np.cos(yaw / 2), np.sin(yaw / 2)
    cp, sp = np.cos(pitch / 2), np.sin(pitch / 2)
    cr, sr = np.cos(roll / 2), np.sin(roll / 2)
    return (cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy)


def build(cdir, a):
    meta, h, course, lane = load_course(cdir)
    terrain = make_terrain(meta, h)
    P = build_path(course, lane, a)
    # terrain profile along the path, slope from a lightly smoothed profile
    zt = terrain(P["px"], P["py"]).astype(float)
    zt_s = nd.gaussian_filter1d(zt, 0.5 / 0.05, mode="nearest")
    dzdl = np.gradient(zt_s, P["L"])
    grade_dl = -dzdl                                    # downhill positive
    Ls, v, acc, F = simulate(P, grade_dl, a)
    n = len(Ls)
    t = DT * (np.arange(n) + 1)

    s_t = np.interp(Ls, P["L"], P["s"])
    px = X0 - s_t
    py = np.interp(Ls, P["L"], P["py"])
    yaw = np.interp(Ls, P["L"], P["yaw"])
    pitch = np.arctan(np.interp(Ls, P["L"], dzdl))      # elevation of the forward direction
    dyaw_dl = np.interp(Ls, P["L"], P["dyaw_dl"])

    # bank: phi = atan(v^2 kappa / g), kappa signed by yaw rate; previewed then low-passed
    raw = np.arctan(v * v * dyaw_dl / G)
    k = int(round(a.preview / DT))
    raw_p = np.concatenate([raw[k:], np.full(k, raw[-1])]) if k > 0 else raw
    alpha = DT / (a.bank_tau + DT)
    roll = np.empty(n)
    roll[0] = raw_p[0]
    for i in range(1, n):
        roll[i] = roll[i - 1] + alpha * (raw_p[i] - roll[i - 1])
    pitch = nd.gaussian_filter1d(pitch, 0.15 / DT, mode="nearest")

    # axle height of a banked crowned tyre: h = r_w - r_t * (1 - cos(phi))
    axle = R_W - R_T * (1.0 - np.cos(roll))
    ground = terrain(px, py).astype(float)
    pz = ground + axle

    # derivatives
    vx, vy, vz = (np.gradient(q, DT) for q in (px, py, pz))
    wz = np.gradient(np.unwrap(yaw), DT)
    wx = np.gradient(roll, DT)
    wy = np.gradient(pitch, DT)
    vs = nd.gaussian_filter1d(v, 0.05 / DT)
    a_long = np.gradient(vs, DT)
    wheel_rate = v / R_W
    wheel_angle = np.concatenate([[0.0], np.cumsum(0.5 * (wheel_rate[1:] + wheel_rate[:-1]) * DT)])
    current = nd.gaussian_filter1d(F, 0.05 / DT) * R_W / KT
    rider_fa = np.clip(0.04 * a_long, -0.05, 0.05)
    rider_lat = np.clip(1.0 * np.tan(roll), -0.25, 0.25)
    qw, qx, qy, qz = quat_zyx(yaw, pitch, roll)

    out = dict(flags=np.zeros(n), seq=np.arange(n, dtype=float), t=t, px=px, py=py, pz=pz,
               qw=qw, qx=qx, qy=qy, qz=qz, wheel_angle=wheel_angle, wheel_rate=wheel_rate,
               pitch=pitch, yaw=yaw, current=current, rider_fa=rider_fa, rider_lat=rider_lat,
               vx=vx, vy=vy, vz=vz, wx=wx, wy=wy, wz=wz)
    out = {k_: np.asarray(x, dtype=np.float64) for k_, x in out.items()}
    # clearance: axle vs the highest terrain within one tyre radius around the contact
    ring = [terrain(px + dx, py + dy) for dx in (-.15, 0, .15) for dy in (-.15, 0, .15)]
    info = dict(s=s_t, v=v, roll=roll, ground=ground, pz=pz, py=py, grade=grade_dl[0],
                clearance=float((pz - np.max(ring, axis=0)).min()),
                s_bounds=np.cumsum([0.0] + [sg["length_m"] for sg in course["segments"]]),
                names=[sg["name"] for sg in course["segments"]])
    return out, info


def plot(info, path):
    import matplotlib; matplotlib.use("Agg"); import matplotlib.pyplot as plt
    s = info["s"]
    fig, ax = plt.subplots(4, 1, figsize=(12, 10), dpi=100, sharex=True)
    ax[0].plot(s, info["v"]); ax[0].set_ylabel("v m/s")
    ax[1].plot(s, np.degrees(info["roll"])); ax[1].set_ylabel("bank deg")
    ax[2].plot(s, info["ground"], label="terrain"); ax[2].plot(s, info["pz"], label="pz (axle)")
    ax[2].set_ylabel("z m"); ax[2].legend(loc="upper right")
    ax[3].plot(s, info["py"]); ax[3].set_ylabel("lateral y m"); ax[3].set_xlabel("course progress s m")
    for x in ax:
        x.grid(alpha=.3)
        for b in info["s_bounds"]:
            x.axvline(b, color="k", alpha=.15)
    fig.tight_layout(); fig.savefig(path); plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("course_dir")
    ap.add_argument("out", nargs="?", help="output npz (default COURSE_DIR/kin_track.npz)")
    ap.add_argument("--amp", type=float, default=0.8, help="lateral amplitude on the descent, m")
    ap.add_argument("--floor-amp", type=float, default=0.1, help="amplitude factor at the sag")
    ap.add_argument("--climb-amp", type=float, default=0.4, help="amplitude factor on the climb")
    ap.add_argument("--mass", type=float, default=100.0)
    ap.add_argument("--crr", type=float, default=0.015)
    ap.add_argument("--cda", type=float, default=0.5)
    ap.add_argument("--v0", type=float, default=2.5)
    ap.add_argument("--vmax", type=float, default=5.5)
    ap.add_argument("--cruise", type=float, default=4.5)
    ap.add_argument("--power", type=float, default=1500.0, help="motor power limit, W")
    ap.add_argument("--drive-acc", type=float, default=1.0)
    ap.add_argument("--brake-decel", type=float, default=2.0)
    ap.add_argument("--stop-decel", type=float, default=0.6, help="gentle final braking, m/s^2")
    ap.add_argument("--kp", type=float, default=1.5, help="cruise gain, 1/s")
    ap.add_argument("--jerk", type=float, default=1.5, help="drive accel slew, m/s^3")
    ap.add_argument("--start-s", type=float, default=2.0)
    ap.add_argument("--end-margin", type=float, default=3.0)
    ap.add_argument("--bank-tau", type=float, default=0.25, help="bank low-pass lag, s")
    ap.add_argument("--preview", type=float, default=0.25, help="curvature look-ahead cancelling the lag, s")
    a = ap.parse_args()
    cdir = Path(a.course_dir)
    npz = Path(a.out) if a.out else cdir / "kin_track.npz"
    out, info = build(cdir, a)
    np.savez(npz, **out)
    png = npz.with_suffix(".png")
    plot(info, png)
    r = subprocess.run([sys.executable, REPLAY, str(npz), "--bin", str(npz.with_suffix(".bin"))],
                       capture_output=True, text=True)
    print(r.stdout.strip(), r.stderr.strip())
    s, v, roll = info["s"], info["v"], info["roll"]
    names, sb = info["names"], info["s_bounds"]
    print(f"duration {out['t'][-1]:.2f} s, {len(out['t'])} samples, s {s[0]:.1f} -> {s[-1]:.2f}")
    print(f"v max {v.max():.2f} min {v.min():.2f}; max |bank| {np.degrees(np.abs(roll)).max():.2f} deg")
    for nm, lo, hi in zip(names, sb[:-1], sb[1:]):
        mk = (s >= lo) & (s < hi)
        if mk.any():
            print(f"  {nm:13s} s[{lo:5.0f},{hi:5.0f}) v {v[mk].min():.2f}..{v[mk].max():.2f}")
    print(f"max |rider_fa| {np.abs(out['rider_fa']).max():.3f}, max |rider_lat| {np.abs(out['rider_lat']).max():.3f}, "
          f"current {out['current'].min():.1f}..{out['current'].max():.1f} A")
    print(f"min axle clearance over local terrain {info['clearance']:.4f} m; max |y| {np.abs(info['py']).max():.2f} m")
    print(npz, npz.with_suffix(".bin"), png)


if __name__ == "__main__":
    main()
