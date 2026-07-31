"""Where the 500 Hz control rate comes from — issue #113.

`docs/design-pi-image-stage0b.md` sources the control rate to **ICD §11.2**, an
interface constant, and says so in as many words: *"500 Hz is an ICD number, not
physics."* Stage 0B's AC-5 and AC-6 are then expressed as fractions of that
2 ms period, so a pre-registered architecture decision was about to be judged
against a threshold nobody had derived. This module derives it.

WHAT THE USEFUL NUMBER ACTUALLY IS
----------------------------------
Not a rate. **A delay budget.** Sample rate and loop delay are different
quantities, and AC-6 measures the second one. They are related by exactly one
term: a zero-order hold at period `Ts` costs `Ts/2` of equivalent transport
delay, and nothing else in the loop cares what `Ts` was. So the honest question
is "how much total loop delay does this plant tolerate", and the rate's whole
contribution to the answer is half a period — 1 ms at 500 Hz.

`delay_margin()` computes that budget. `sampled_delay_budget()` computes the
same thing exactly, in discrete time, and the two agree to the `Ts/2` term,
which is how each checks the other.

THE PLANT IS NOT A PENDULUM ON A FIXED PIVOT
--------------------------------------------
It is a wheeled inverted pendulum, and the difference is not cosmetic:

    fixed pivot, real inertia       sqrt(mgl / J_axle)            3.19 rad/s
    point mass at the CoM height    sqrt(mgl / m*h^2)             3.94 rad/s
    THIS VEHICLE                    the eigenvalue of `A` below   5.24 rad/s

The axle is free to accelerate, which removes `(m*l)^2 / M_t` from the
effective inertia and makes the vehicle diverge **64% faster** than the
point-mass approximation suggests. 3.94 rad/s is the figure quoted when this
work was requested; it is reproduced by `point_mass_pole()` so the comparison
stays checkable, and it is not the right number.

THE ACTUATOR REACTION HELPS, AND GETTING ITS SIGN WRONG COSTS A FACTOR OF 1.6
-----------------------------------------------------------------------------
Driving the wheel forward does two things to the body: it accelerates the axle
out from under the centre of mass (a large nose-up moment, `m*l/(M_t*r)` = 4.21
here), and the hinge reaction torques the body directly (a smaller nose-up
moment, 1.0). **They add.** An earlier draft of this derivation had the hinge
reaction opposing, giving 3.21 instead of 5.21, and the linear model then
under-predicted MuJoCo's torque-step response by a factor of 1.62 — which is
precisely 5.21/3.21. The sign was fixed because the model disagreed, not
because the algebra was re-read; `tests/test_loop_rate.py` pins the agreement
so it cannot silently invert again.

VALIDATED, NOT ASSERTED
-----------------------
Every constant is read out of the **compiled** MuJoCo model rather than retyped
from a document, and the resulting linearisation is checked against that same
model's free divergence and torque-step response inside the small-angle band it
claims. See `tests/test_loop_rate.py`.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import numpy as np

from .plant import KT_NM_PER_A

G = 9.81

#: Body parts rigidly attached to the frame. The wheel is NOT one of them: it
#: turns on the hinge, which is the whole point of the vehicle.
_RIGID_WITH_FRAME = ("frame", "ballast")


# ---------------------------------------------------------------------------
# Linear algebra this project does not have scipy for
# ---------------------------------------------------------------------------
def expm(M: np.ndarray, terms: int = 30) -> np.ndarray:
    """Matrix exponential by scaling-and-squaring on a Taylor series.

    Written out rather than adding scipy to `requirements-sim.txt`: that file
    is a pinned CI fixture whose whole job is that the physics reproduces, and
    a new transitive dependency tree is a poor trade for one function. Accuracy
    is checked against a known-answer case in `tests/test_loop_rate.py`.
    """
    M = np.asarray(M, dtype=float)
    nrm = float(np.abs(M).sum(axis=1).max())
    s = max(0, int(math.ceil(math.log2(nrm))) + 1) if nrm > 0 else 0
    A = M / (2.0**s)
    E = np.eye(A.shape[0])
    T = np.eye(A.shape[0])
    for k in range(1, terms + 1):
        T = T @ A / k
        E = E + T
    for _ in range(s):
        E = E @ E
    return E


def integrated_expm(A: np.ndarray, h: float) -> np.ndarray:
    """``int_0^h e^{As} ds``, via the block-matrix exponential identity."""
    n = A.shape[0]
    M = np.zeros((2 * n, 2 * n))
    M[:n, :n] = A
    M[:n, n:] = np.eye(n)
    return expm(M * h)[:n, n:]


# ---------------------------------------------------------------------------
# The plant
# ---------------------------------------------------------------------------
@dataclass(frozen=True, eq=False)
class LinearPlant:
    """The linearised wheeled inverted pendulum, about upright and at rest.

    State is ``[x, theta, xdot, thetadot]``: axle position along the direction
    of travel (m, forward positive), body pitch (rad, **nose-up positive**, ICD
    §10.1), and their rates. Input is motor torque at the hinge, N*m.

    VALIDITY. Small angle, no slip, no contact loss, rigid body. `sin` is
    replaced by its argument, which is 1% wrong at 14° and 3% at 24°; the
    vehicle noses in at 18.6°, so the linear model is inside a few per cent
    everywhere the board is still upright. It says nothing at all about the
    40 A current cap, the cutback layers, or the nose strike itself — those are
    what the simulated sweep is for.
    """

    m_body_kg: float
    """Mass rigidly attached to the frame. Excludes the wheel."""

    com_above_axle_m: float
    """`l` — height of that mass's centre above the axle."""

    inertia_about_axle: float
    """`J` — its pitch inertia about the axle, parallel-axis included."""

    wheel_mass_kg: float
    wheel_inertia: float
    wheel_radius_m: float
    hinge_damping: float
    """Viscous damping on the hinge, N*m*s/rad. Small, and it is the only
    dissipation in the model."""

    A: np.ndarray = field(repr=False)
    B: np.ndarray = field(repr=False)

    # -- derived ------------------------------------------------------------
    @property
    def mgl(self) -> float:
        """`m*g*l`. **Positive means gravity destabilises** — an inverted
        pendulum. Negative means it restores, which is the driverless board."""
        return self.m_body_kg * G * self.com_above_axle_m

    @property
    def translating_mass(self) -> float:
        """`M_t` — mass the motor has to shove, including the wheel's own
        rotational inertia reflected through the rolling constraint."""
        return (
            self.m_body_kg
            + self.wheel_mass_kg
            + self.wheel_inertia / self.wheel_radius_m**2
        )

    @property
    def effective_pitch_inertia(self) -> float:
        """`J - (m*l)^2 / M_t`. The axle running away under the centre of mass
        removes real inertia from the pitch mode; this is what is left."""
        return (
            self.inertia_about_axle
            - (self.m_body_kg * self.com_above_axle_m) ** 2 / self.translating_mass
        )

    @property
    def torque_leverage(self) -> float:
        """`m*l/(M_t*r) + 1` — nose-up moment per N*m of motor torque.

        The `+1` is the hinge reaction on the body and the first term is the
        axle accelerating out from under the mass. They ADD; see the module
        docstring for what happens when they do not.
        """
        return (
            self.m_body_kg * self.com_above_axle_m
            / (self.translating_mass * self.wheel_radius_m)
            + 1.0
        )

    @property
    def unstable_pole(self) -> float:
        """Largest real part of `A`'s spectrum, rad/s. Zero or negative means
        the open-loop plant does not diverge."""
        return float(max(np.linalg.eigvals(self.A).real))

    @property
    def doubling_time_s(self) -> float | None:
        p = self.unstable_pole
        return math.log(2.0) / p if p > 1e-9 else None

    def point_mass_pole(self) -> float:
        """The approximation this work was asked to check: all the mass as a
        point at the CoM height, pivoting about a pinned axle. Kept so the
        comparison in the module docstring stays reproducible."""
        total = self.m_body_kg + self.wheel_mass_kg
        h = self.m_body_kg * self.com_above_axle_m / total
        return math.sqrt(max(0.0, self.mgl / (total * h * h)))

    def fixed_pivot_pole(self) -> float:
        """Real inertia, but a pinned axle. Still not this vehicle."""
        return math.sqrt(max(0.0, self.mgl / self.inertia_about_axle))

    def critical_kp_a_per_rad(self) -> float:
        """Proportional gain at which pitch feedback ALONE stops the divergence.

        Below it the inner loop cannot hold the board up on its own however
        much damping it has, and the cascade's outer loop is doing the
        stabilising. Negative means the plant was never unstable.
        """
        if self.mgl <= 0.0:
            return -abs(self.mgl / self.effective_pitch_inertia)
        accel_per_amp = self.torque_leverage / self.effective_pitch_inertia * KT_NM_PER_A
        return (self.mgl / self.effective_pitch_inertia) / accel_per_amp

    def summary(self) -> dict:
        return {
            "m_body_kg": round(self.m_body_kg, 4),
            "com_above_axle_mm": round(self.com_above_axle_m * 1000.0, 2),
            "inertia_about_axle_kg_m2": round(self.inertia_about_axle, 5),
            "translating_mass_kg": round(self.translating_mass, 4),
            "effective_pitch_inertia_kg_m2": round(self.effective_pitch_inertia, 5),
            "torque_leverage_per_n_m": round(self.torque_leverage, 5),
            "mgl_n_m_per_rad": round(self.mgl, 3),
            "unstable_pole_rad_s": round(self.unstable_pole, 5),
            "unstable_pole_hz": round(self.unstable_pole / (2 * math.pi), 5),
            "doubling_time_ms": (
                None if self.doubling_time_s is None
                else round(self.doubling_time_s * 1000.0, 2)
            ),
            "point_mass_pole_rad_s": round(self.point_mass_pole(), 5),
            "fixed_pivot_pole_rad_s": round(self.fixed_pivot_pole(), 5),
            "critical_kp_a_per_rad": round(self.critical_kp_a_per_rad(), 3),
        }


def linearise(model) -> LinearPlant:
    """Read the plant out of a **compiled** MuJoCo model and linearise it.

    Nothing here is retyped from a document. Masses, inertias, the tire radius
    and the hinge damping all come from the model object, so a change to the
    plant moves this derivation with it instead of silently invalidating it.
    """
    import mujoco

    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    bid = {
        mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, i): i
        for i in range(model.nbody)
    }
    axle_z = float(data.xpos[bid["frame"]][2])
    wheel_gid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, "wheel_geom")
    r = float(model.geom_size[wheel_gid][0])
    hinge = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, "wheel_hinge")
    b = float(model.dof_damping[model.jnt_dofadr[hinge]])

    m_w = float(model.body_mass[bid["wheel"]])
    i_w = float(model.body_inertia[bid["wheel"]][1])

    m_b = ml = j = 0.0
    for name in _RIGID_WITH_FRAME:
        if name not in bid:
            continue
        i = bid[name]
        mi = float(model.body_mass[i])
        zi = float(data.xipos[i][2]) - axle_z
        m_b += mi
        ml += mi * zi
        j += float(model.body_inertia[i][1]) + mi * zi * zi
    l = ml / m_b
    m_t = m_b + m_w + i_w / r**2

    # Mass matrix of [x, theta] and the generalised forces, linearised:
    #     M_t*xdd  -  m*l*tdd            =  u/r
    #     J*tdd    -  m*l*xdd  -  mgl*t  =  u
    #     u = tau - b*(xdot/r + thetadot)          (hinge damping)
    mass = np.array([[m_t, -ml], [-ml, j]])
    minv = np.linalg.inv(mass)
    gravity = np.array([[0.0, 0.0], [0.0, m_b * G * l]])
    b_u = np.array([[1.0 / r], [1.0]])
    damp = np.array([[-b / r, -b]])

    A = np.zeros((4, 4))
    B = np.zeros((4, 1))
    A[0, 2] = A[1, 3] = 1.0
    A[2:, 0:2] = minv @ gravity
    A[2:, 2:4] = minv @ (b_u @ damp)
    B[2:, 0] = (minv @ b_u).ravel()

    return LinearPlant(
        m_body_kg=m_b,
        com_above_axle_m=l,
        inertia_about_axle=j,
        wheel_mass_kg=m_w,
        wheel_inertia=i_w,
        wheel_radius_m=r,
        hinge_damping=b,
        A=A,
        B=B,
    )


# ---------------------------------------------------------------------------
# The controller, as the loop sees it
# ---------------------------------------------------------------------------
@dataclass(frozen=True)
class Gains:
    """The gains actually in use, in the units the FFI takes.

    `kp_v`/`ki_v` zero means inner loop only, which is the driverless impulse
    gate's configuration. Non-zero adds the cascade's outer velocity loop —
    which for the ridden board is not optional: `critical_kp_a_per_rad` is
    140 A/rad against the 200 A/rad in use, so pitch feedback alone is only
    just sufficient and it is the outer loop that regulates where the board
    ends up.
    """

    kp_a_per_rad: float
    kd_a_per_rad_s: float
    kp_v_rad_per_m_s: float = 0.0
    ki_v_rad_per_m: float = 0.0
    com_above_axle: bool = True

    @property
    def coupling_sign(self) -> float:
        """`PlantCoupling`'s sign. Inverting it is positive feedback in
        velocity — see `control_core::PlantCoupling`."""
        return 1.0 if self.com_above_axle else -1.0


def _controller_rows(plant: LinearPlant, gains: Gains):
    """`(D_row, C_integrator, v_row)` — the controller as a row on the state.

    The measured speed is the WHEEL's, which is the hinge rate, so it carries a
    pitch-rate term: `v = r*(xdot/r + thetadot) = xdot + r*thetadot`. That term
    is real and it is easy to leave out; the outer loop sees the board's
    forward speed *plus* the tangential speed of the axle due to pitching.
    """
    r = plant.wheel_radius_m
    e_theta = np.array([0.0, 1.0, 0.0, 0.0])
    e_theta_dot = np.array([0.0, 0.0, 0.0, 1.0])
    v_row = np.array([0.0, 0.0, 1.0, r])
    sign = gains.coupling_sign
    d_row = KT_NM_PER_A * (
        -gains.kp_a_per_rad * e_theta
        - gains.kd_a_per_rad_s * e_theta_dot
        + gains.kp_a_per_rad * sign * gains.kp_v_rad_per_m_s * v_row
    )
    c_int = KT_NM_PER_A * gains.kp_a_per_rad * sign * gains.ki_v_rad_per_m
    return d_row, c_int, v_row


def loop_transfer(plant: LinearPlant, gains: Gains, omega: np.ndarray) -> np.ndarray:
    """`L(jw)` for the loop broken at the plant input (the motor).

    Broken at the input on purpose: that is where every delay in the system
    actually sits — transport to the drive, the drive's own current loop, and
    the scheduler's lateness are all in series there, and a margin measured at
    that point is the margin against all of them at once.
    """
    d_row, c_int, v_row = _controller_rows(plant, gains)
    n = plant.A.shape[0]
    out = np.empty(len(omega), dtype=complex)
    eye = np.eye(n)
    for i, w in enumerate(omega):
        s = 1j * w
        g = np.linalg.solve(s * eye - plant.A, plant.B)[:, 0]
        k_row = d_row + (c_int / s) * v_row
        out[i] = -(k_row @ g)
    return out


@dataclass(frozen=True)
class Margin:
    """One gain crossover and what it implies."""

    crossover_rad_s: float
    phase_margin_deg: float
    delay_margin_s: float
    """Extra pure transport delay that drives this crossing's phase margin to
    zero. `phase_margin_deg / (crossover * 180/pi)`, exactly — a pure delay is
    all-pass, so it cannot move the crossover frequency, only the phase there.
    """

    def delay_at_phase_margin_s(self, target_deg: float) -> float:
        """Delay that leaves `target_deg` of phase margin. Negative means the
        loop does not have that much margin even at zero delay."""
        return (self.phase_margin_deg - target_deg) * math.pi / 180.0 / self.crossover_rad_s


def loop_margins(
    plant: LinearPlant,
    gains: Gains,
    omega_max: float = 400.0,
    points: int = 200_000,
) -> list[Margin]:
    """Every gain crossover, ordered by the delay each one tolerates.

    A list rather than a single number because a cascade can cross unity more
    than once, and the binding constraint is the *smallest* delay margin among
    them, not the one at the highest frequency.
    """
    w = np.linspace(1e-4, omega_max, points)
    mag = np.abs(loop_transfer(plant, gains, w))
    out: list[Margin] = []
    crossings = np.flatnonzero(np.sign(mag[:-1] - 1.0) != np.sign(mag[1:] - 1.0))
    for i in crossings:
        frac = (1.0 - mag[i]) / (mag[i + 1] - mag[i])
        wc = float(w[i] + frac * (w[i + 1] - w[i]))
        phase = float(np.angle(loop_transfer(plant, gains, np.array([wc]))[0]))
        pm = math.degrees(phase) + 180.0
        delay = (phase + math.pi) / wc
        while delay <= 0.0:
            delay += 2 * math.pi / wc
        out.append(Margin(wc, pm, delay))
    return sorted(out, key=lambda m: m.delay_margin_s)


def delay_margin(plant: LinearPlant, gains: Gains, **kw) -> Margin:
    """The binding crossover. Raises if the loop never reaches unity gain."""
    margins = loop_margins(plant, gains, **kw)
    if not margins:
        raise ValueError(
            "no gain crossover found -- the loop gain never reaches unity, so "
            "a phase margin is not defined for it"
        )
    return margins[0]


# ---------------------------------------------------------------------------
# The same question in discrete time, where the rate is actually a variable
# ---------------------------------------------------------------------------
def sampled_closed_loop_poles(
    plant: LinearPlant,
    gains: Gains,
    rate_hz: float,
    delay_s: float,
    current_loop_tau_s: float = 0.0,
) -> np.ndarray:
    """Exact discrete closed-loop eigenvalues for a ZOH loop with input delay.

    Not an approximation of the sampling: the plant is integrated exactly
    across the sample with the input piecewise-constant, and a delay that is
    not a whole number of samples is split between the two commands that
    straddle it. Rounding a fractional delay to whole cycles is how this
    project once deleted the ICD's own 1 ms figure entirely (see
    `ImperfectionState.apply_current`), and the same trap applies here.
    """
    ts = 1.0 / rate_hz
    a, b = plant.A, plant.B
    n = a.shape[0]
    d_row, c_int, v_row = _controller_rows(plant, gains)

    if current_loop_tau_s > 0.0:
        a2 = np.zeros((n + 1, n + 1))
        b2 = np.zeros((n + 1, 1))
        a2[:n, :n] = a
        a2[:n, n] = b[:, 0]
        a2[n, n] = -1.0 / current_loop_tau_s
        b2[n, 0] = 1.0 / current_loop_tau_s
        d_row = np.append(d_row, 0.0)
        v_row = np.append(v_row, 0.0)
        a, b, n = a2, b2, n + 1

    cycles = delay_s / ts
    whole = int(math.floor(cycles))
    frac = cycles - whole
    phi = expm(a * ts)
    g_new = integrated_expm(a, (1.0 - frac) * ts) @ b
    g_old = expm(a * (1.0 - frac) * ts) @ integrated_expm(a, frac * ts) @ b

    # state: [plant (+ current-loop lag), outer integrator, u[k-1] .. u[k-1-whole]]
    n_hist = whole + 1
    size = n + 1 + n_hist
    m = np.zeros((size, size))
    m[:n, :n] = phi
    k_row = np.zeros(size)
    k_row[:n] = d_row
    k_row[n] = c_int
    if whole == 0:
        m[:n, :] += np.outer(g_new[:, 0], k_row)
        m[:n, n + 1] = g_old[:, 0]
    else:
        m[:n, n + whole] = g_new[:, 0]
        m[:n, n + 1 + whole] = g_old[:, 0]
    m[n, n] = 1.0
    m[n, :n] = ts * v_row
    m[n + 1, :] = k_row
    for i in range(1, n_hist):
        m[n + 1 + i, n + i] = 1.0
    return np.linalg.eigvals(m)


def is_sampled_stable(*args, tol: float = 1e-9, **kw) -> bool:
    return bool(max(abs(sampled_closed_loop_poles(*args, **kw))) <= 1.0 + tol)


def sampled_delay_budget(
    plant: LinearPlant,
    gains: Gains,
    rate_hz: float,
    current_loop_tau_s: float = 0.0,
    hi_s: float = 0.5,
    tol_s: float = 1e-6,
) -> float:
    """Largest *additional* transport delay this rate survives, seconds.

    Additional to the sampling itself. Add `1/(2*rate)` to compare it with the
    continuous `delay_margin()`; that they agree to that term across three
    decades of rate is the cross-check that neither derivation is wrong.
    """
    if not is_sampled_stable(plant, gains, rate_hz, 0.0, current_loop_tau_s):
        return 0.0
    lo = 0.0
    hi = hi_s
    while hi - lo > tol_s:
        mid = 0.5 * (lo + hi)
        if is_sampled_stable(plant, gains, rate_hz, mid, current_loop_tau_s):
            lo = mid
        else:
            hi = mid
    return lo


# ---------------------------------------------------------------------------
# The same question in the simulator, where the things the linear model cannot
# express actually happen
# ---------------------------------------------------------------------------
#: Physics timestep for every swept run, seconds.
#:
#: 4x finer than the model's own 2 ms, because the model's timestep IS 500 Hz
#: and a rate sweep that cannot represent a rate above the one under test is
#: not a sweep. Every rate in `SWEEP_RATES_HZ` divides 2 kHz exactly, so no
#: point in the grid is silently rounded to a different rate than its label.
#:
#: Choosing it is a fidelity change and was checked rather than assumed:
#: `analyse_loop_rate.py`'s control block re-runs one configuration at 2, 1,
#: 0.5 and 0.25 ms and the peak pitch moves by ~2%, which is smaller than any
#: effect this study reports.
SWEEP_DT_S = 0.0005

SWEEP_RATES_HZ = (31.25, 62.5, 125.0, 250.0, 500.0, 1000.0)

#: Where the impulse lands inside the control period. **Not a detail.**
#:
#: At low rates the peak pitch depends strongly on where the kick falls
#: relative to the next control cycle -- a 15 Hz run measured 5.15 deg and a
#: 50 Hz run 8.70 deg with the same delay, which reads as "50 Hz is worse than
#: 15 Hz" and is nothing of the sort. Each grid point is therefore run at
#: several sub-period offsets and reported by its WORST outcome, which is the
#: number a safety case wants anyway.
SWEEP_PHASES = 3


@dataclass(frozen=True)
class RunPoint:
    """One simulated configuration and what it did."""

    rate_hz: float
    delay_s: float
    jitter_s: float
    noise_scale: float
    struck: bool
    peak_abs_pitch_deg: float
    late_band_deg: float
    """Largest |pitch| over the final 1.5 s. Separates "recovered" from "still
    ringing": a plant that is stable open-loop (the driverless board) answers a
    lost delay margin with a limit cycle rather than a fall, so `struck` alone
    would call an oscillating loop a pass."""
    travel_m: float
    control_effort_a_s: float
    saturated_cycles: int
    pitch_est_rms_deg: float
    control_cycles: int

    def to_dict(self) -> dict:
        return {
            "rate_hz": self.rate_hz,
            "delay_ms": round(self.delay_s * 1e3, 4),
            "jitter_ms": round(self.jitter_s * 1e3, 4),
            "noise_scale": self.noise_scale,
            "struck": self.struck,
            "peak_abs_pitch_deg": round(self.peak_abs_pitch_deg, 4),
            "late_band_deg": round(self.late_band_deg, 4),
            "travel_m": round(self.travel_m, 4),
            "control_effort_a_s": round(self.control_effort_a_s, 4),
            "saturated_cycles": self.saturated_cycles,
            "pitch_est_rms_deg": round(self.pitch_est_rms_deg, 4),
            "control_cycles": self.control_cycles,
        }


def simulate_point(
    model,
    gains: dict,
    rate_hz: float,
    delay_s: float = 0.001,
    jitter_s: float = 0.0,
    noise_scale: float = 1.0,
    sim_seconds: float = 5.0,
    phases: int = SWEEP_PHASES,
    dt_s: float = SWEEP_DT_S,
    late_window_s: float = 1.5,
) -> RunPoint:
    """One grid point: the impulse scenario at a given rate, delay and jitter.

    Reported by the WORST of `phases` runs whose kicks land at evenly spaced
    offsets inside one control period — see `SWEEP_PHASES`. `gains` is passed
    straight to `RustController`, so this exercises the shipped control law
    over the FFI rather than a Python restatement of it.
    """
    import dataclasses

    import numpy as np  # noqa: F811  (module-level import is the same object)

    from .imperfections import STAGE0_PLACEHOLDER
    from .impulse_response import NOMINAL_IMPULSE_NS, ImpulseParams, run
    from .rust_controller import RustController

    profile = dataclasses.replace(
        STAGE0_PLACEHOLDER,
        actuation_delay_s=delay_s,
        gyro_noise_rad_s=STAGE0_PLACEHOLDER.gyro_noise_rad_s * noise_scale,
        gyro_bias_rad_s=STAGE0_PLACEHOLDER.gyro_bias_rad_s * noise_scale,
        accel_noise_m_s2=STAGE0_PLACEHOLDER.accel_noise_m_s2 * noise_scale,
        profile_id=(
            f"{STAGE0_PLACEHOLDER.profile_id}+loop-rate"
            f"(delay={delay_s*1e3:g}ms,noise=x{noise_scale:g})"
        ),
    )
    worst: RunPoint | None = None
    period = 1.0 / rate_hz
    for k in range(max(1, phases)):
        params = ImpulseParams(
            magnitude_ns=NOMINAL_IMPULSE_NS,
            t0_s=0.5 + k * period / max(1, phases),
            sim_seconds=sim_seconds,
            control_period_s=period,
            control_jitter_s=jitter_s,
            physics_timestep_s=dt_s,
        )
        with RustController(**gains) as ctrl:
            result = run(params, model=model, controller=ctrl, profile=profile)
        late = result.pitch_deg[result.t >= result.t[-1] - late_window_s]
        point = RunPoint(
            rate_hz=rate_hz,
            delay_s=delay_s,
            jitter_s=jitter_s,
            noise_scale=noise_scale,
            struck=bool(result.metrics.nose_strike),
            peak_abs_pitch_deg=float(result.metrics.peak_abs_pitch_deg),
            late_band_deg=float(np.abs(late).max()) if late.size else float("nan"),
            travel_m=float(result.metrics.travel_m),
            control_effort_a_s=float(result.metrics.control_effort_a_s),
            saturated_cycles=int(ctrl.saturated_cycles),
            pitch_est_rms_deg=float(result.metrics.pitch_est_rms_deg),
            control_cycles=int(result.metrics.control_cycles),
        )
        if worst is None or (point.struck, point.peak_abs_pitch_deg) > (
            worst.struck, worst.peak_abs_pitch_deg
        ):
            worst = point
    assert worst is not None
    return worst
