"""Speed-hold LQR for the ridden board, designed on the planar linear model.

The authority sweep needs the board to hold a speed by itself, so that the
result measures the motor and not rider skill. A cascade of a slow PI speed
loop on a fixed PD pitch loop overshoots by 2-3 m/s on grade transitions.
This designs one full-state law instead:

    i = -K [theta, theta_dot, v - v_ref, integral(v - v_ref)]

States the firmware can measure: pitch, pitch rate (estimator), wheel speed.
The rider's fore/aft slide is NOT measured, so the law is designed on the
rigid-rider model and then checked on the model that includes the slide
(the rider is a 70 kg mass on a spring-damper, a real low-frequency mode).

Planar model, generalised coordinates q = [x, theta, b]:
  x      wheel centre along the ground (forward positive)
  theta  body pitch, positive = top moves forward (nose DOWN on this board)
  b      rider slide along the deck, forward positive
Motor torque tau acts on the wheel against the body: Q_x = tau/r, Q_theta = -tau.

Parameters are those of sim/models/overboard_rider.xml with the --lean-steer
splice (ballast_fa kp 12000, damping 600).

    $PY sim/carve/lqr_design.py            # prints gains and margins
"""
import json
import sys

import numpy as np
import scipy.linalg as sl
import sympy as sp

R = 0.1454                       # rolling radius
M_W, I_W = 4.5, 0.0595           # wheel
M_F, I_F, L_F = 8.0, 0.400, -0.03  # frame: mass, pitch inertia about own CoM, CoM height
M_R, I_R, L_R = 70.5, 10.5, 0.75   # rider + carrier, slide height above the axle
K_B, C_B = 12000.0, 600.0        # rider slide servo (lean-steer splice)
KT = 0.7                         # N*m per A
G = 9.81
DT = 0.002


def linear_model(with_slide):
    x, th, b = sp.symbols('x theta b')
    xd, thd, bd = sp.symbols('xd thetad bd')
    q, qd = [x, th, b], [xd, thd, bd]
    if not with_slide:
        b, bd = sp.Integer(0), sp.Integer(0)
        q, qd = [x, th], [xd, thd]
    n = len(q)

    def pos(h, along):
        return sp.Matrix([x + h * sp.sin(th) + along * sp.cos(th),
                          R + h * sp.cos(th) - along * sp.sin(th)])

    def vel(p):
        return p.jacobian(q) * sp.Matrix(qd)

    pf, pr = pos(L_F, 0), pos(L_R, b)
    vf, vr = vel(pf), vel(pr)
    T = (sp.Rational(1, 2) * M_W * xd**2 + sp.Rational(1, 2) * I_W * (xd / R)**2
         + sp.Rational(1, 2) * M_F * vf.dot(vf) + sp.Rational(1, 2) * I_F * thd**2
         + sp.Rational(1, 2) * M_R * vr.dot(vr) + sp.Rational(1, 2) * I_R * thd**2)
    V = G * (M_F * pf[1] + M_R * pr[1]) + sp.Rational(1, 2) * K_B * b**2
    Lg = T - V
    tau = sp.symbols('tau')
    Q = sp.Matrix([tau / R, -tau, -C_B * bd][:n])
    qdv = sp.Matrix(qd)
    Mm = sp.Matrix(n, n, lambda i, j: sp.diff(Lg, qd[i], qd[j]))
    # Euler-Lagrange: M qdd + (dM/dq qd) qd - dL/dq = Q
    h = sp.Matrix([sum(sp.diff(sp.diff(Lg, qd[i]), q[k]) * qd[k] for k in range(n))
                   - sp.diff(Lg, q[i]) for i in range(n)])
    qdd = Mm.LUsolve(Q - h)
    f = sp.Matrix([qdv, qdd])
    s = list(q) + list(qd)
    zero = {v: 0 for v in s + [tau]}
    A = np.array(f.jacobian(s).subs(zero), dtype=float)
    B = np.array(f.jacobian([tau]).subs(zero), dtype=float) * KT  # input in amps
    # Drop position x (index 0): nothing depends on it.
    A, B = A[1:, 1:], B[1:]
    return A, B  # rigid: [theta, xd, thetad]; slide: [theta, b, xd, thetad, bd]


def design(q_diag, r_amps):
    A, B = linear_model(False)
    # reorder to [theta, thetad, v] and append the speed-error integral
    P = [0, 2, 1]
    A, B = A[np.ix_(P, P)], B[P]
    Aa = np.zeros((4, 4)); Aa[:3, :3] = A; Aa[3, 2] = 1.0
    Ba = np.vstack([B, [[0.0]]])
    Ad = sl.expm(Aa * DT)
    Bd = np.linalg.solve(Aa, (Ad - np.eye(4))) @ Ba if np.linalg.matrix_rank(Aa) == 4 else _zoh(Aa, Ba)
    Qm, Rm = np.diag(q_diag), np.array([[r_amps]])
    Pm = sl.solve_discrete_are(Ad, Bd, Qm, Rm)
    K = np.linalg.solve(Rm + Bd.T @ Pm @ Bd, Bd.T @ Pm @ Ad)
    return K.ravel(), Aa, Ba


def _zoh(A, B):
    n, m = A.shape[0], B.shape[1]
    E = sl.expm(np.block([[A, B], [np.zeros((m, n + m))]]) * DT)
    return E[:n, n:]


def check_with_slide(K):
    """Closed loop on the model with the rider slide; returns the eigenvalues."""
    A, B = linear_model(True)            # [theta, b, v, thetad, bd]
    n = 6
    Aa = np.zeros((n, n)); Aa[:5, :5] = A; Aa[5, 2] = 1.0
    Ba = np.vstack([B, [[0.0]]])
    # measured states map: theta=0, thetad=3, v=2, integral=5
    Kf = np.zeros(n); Kf[[0, 3, 2, 5]] = K
    return np.linalg.eigvals(Aa - Ba @ Kf[None, :]), Aa, Ba, Kf


def loop_margins(Aa, Ba, Kf):
    """Gain and phase margin of the loop broken at the current input."""
    w = np.logspace(-2, 3, 4000)
    L = np.array([(Kf @ np.linalg.solve(1j * wi * np.eye(len(Aa)) - Aa, Ba)).item() for wi in w])
    mag, ph = np.abs(L), np.unwrap(np.angle(L))
    pm, gm = [], []
    for i in range(len(w) - 1):
        if (mag[i] - 1) * (mag[i + 1] - 1) < 0:
            pm.append((w[i], np.degrees(ph[i]) % 360 - 180))
    return pm, float(np.min(np.abs(1 + L)))


if __name__ == '__main__':
    q = json.loads(sys.argv[1]) if len(sys.argv) > 1 else [400.0, 10.0, 40.0, 10.0]
    r = float(sys.argv[2]) if len(sys.argv) > 2 else 0.02
    A, B = linear_model(False)
    print('open loop rigid eig', np.round(np.linalg.eigvals(A), 3))
    K, Aa, Ba = design(q, r)
    print('K [A/rad, A/(rad/s), A/(m/s), A/m] =', np.round(K, 3).tolist())
    eig, A6, B6, Kf = check_with_slide(K)
    print('closed loop with slide eig', np.round(np.sort_complex(eig), 2))
    pm, smin = loop_margins(A6, B6, Kf)
    print('crossovers (w rad/s, phase margin deg):', [(round(a, 2), round(b, 1)) for a, b in pm])
    print('min |1+L| (disk margin proxy):', round(smin, 3))
