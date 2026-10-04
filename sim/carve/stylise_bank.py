"""Add a stylised bank to a recorded carve track.

    python stylise_bank.py [IN.npz OUT.npz]

The recorded board does not lean: its turn is a commanded yaw. This script
adds a roll about the board's forward axis, through the tyre contact point,
from the steady-turn bank angle atan(v^2 * kappa / g). The motion is not
re-simulated. Output keys: every input key, plus `bank_deg`.
"""
import sys
import numpy as np
from scipy.spatial.transform import Rotation as Rot

DATA = '/Users/mike/projects/overboard-viz/out/carve-lab/data/'
G = 9.81
CENTRE_H = 0.1454          # axle height above the tyre contact point (m)
MAX_BANK = np.radians(30.0)


def smooth(a, seconds, dt):
    k = max(1, int(round(seconds / dt)))
    pad = np.concatenate([np.full(k, a[0]), a, np.full(k, a[-1])])
    return np.convolve(pad, np.ones(2 * k + 1) / (2 * k + 1), 'valid')


def to_rot(d):
    q = np.stack([d['qx'], d['qy'], d['qz'], d['qw']], axis=1)   # scipy: x y z w
    return Rot.from_quat(q)


def bank_angle(d):
    """Steady-turn bank in rad, + = left."""
    t = d['t']
    dt = float(np.median(np.diff(t)))
    R = to_rot(d).as_matrix()
    fwd = -R[:, :, 0]                                  # board forward = body -X
    psi = np.unwrap(np.arctan2(fwd[:, 1], fwd[:, 0]))
    psi_dot = smooth(np.gradient(psi, t), 0.5, dt)
    v = smooth(np.hypot(d['vx'], d['vy']), 0.5, dt)
    kappa = psi_dot / np.maximum(v, 0.5)
    phi = np.arctan(v ** 2 * kappa / G)
    phi = np.clip(phi, -MAX_BANK, MAX_BANK)
    return smooth(phi, 0.3, dt)


def apply_bank(d, phi):
    """Return (pos, quat wxyz) with the roll applied about the contact point."""
    R = to_rot(d)
    Rm = R.as_matrix()
    pos = np.stack([d['px'], d['py'], d['pz']], axis=1)
    up = np.array([0.0, 0.0, CENTRE_H])
    contact = pos - Rm @ up
    # Rx(+phi) sends body z toward -Y (left) for phi > 0.
    R2 = R * Rot.from_rotvec(np.stack([phi, 0 * phi, 0 * phi], axis=1))
    pos2 = contact + R2.as_matrix() @ up
    q = R2.as_quat()
    q = q * np.where(q[:, 3:4] < 0, -1.0, 1.0)         # keep w >= 0
    return pos2, q[:, [3, 0, 1, 2]]


def main():
    src = sys.argv[1] if len(sys.argv) > 2 else DATA + 'a03_track.npz'
    dst = sys.argv[2] if len(sys.argv) > 2 else DATA + 'a04_track.npz'
    d = dict(np.load(src))
    phi = bank_angle(d)
    pos2, q = apply_bank(d, phi)
    out = dict(d)
    out['px'], out['py'], out['pz'] = pos2[:, 0], pos2[:, 1], pos2[:, 2]
    out['qw'], out['qx'], out['qy'], out['qz'] = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    for k in ('px', 'py', 'pz', 'qw', 'qx', 'qy', 'qz'):
        out[k] = out[k].astype(d[k].dtype)
    out['bank_deg'] = np.degrees(phi)
    np.savez(dst, **out)
    deg = np.degrees(phi)
    i = int(np.argmax(np.abs(deg)))
    print(f'bank min {deg.min():.1f} deg, max {deg.max():.1f} deg; '
          f'largest |bank| {deg[i]:.1f} deg at t = {d["t"][i]:.2f} s')
    print('wrote', dst)


def selftest():
    """For a left turn, the body z-axis must lean toward the left direction."""
    d = dict(np.load(DATA + 'a03_track.npz'))
    phi = bank_angle(d)
    pos2, q = apply_bank(d, phi)
    R = Rot.from_quat(q[:, [1, 2, 3, 0]]).as_matrix()
    z = R[:, :, 2]
    fwd = -R[:, :, 0]
    left = np.stack([-fwd[:, 1], fwd[:, 0], 0 * fwd[:, 0]], axis=1)
    lean = np.sum(z * left, axis=1)
    L, Rr = phi > np.radians(5), phi < -np.radians(5)
    assert L.any() and Rr.any()
    assert np.all(lean[L] > 0) and np.all(lean[Rr] < 0), 'bank sign wrong'
    print('selftest ok: left turns lean left, right turns lean right')


if __name__ == '__main__':
    if '--selftest' in sys.argv:
        selftest()
    else:
        main()
