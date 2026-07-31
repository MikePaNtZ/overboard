#!/usr/bin/env python3
"""Why does the estimator destabilise a loop it tracks to 0.65 deg?

The estimator is accurate in shadow and fatal in the loop. Accuracy alone does
not explain that: the same loop tolerates 40 ms of pure actuation delay, so a
sub-degree error should be nothing. Something about *how* the error is shaped
must be doing the damage, and this script is the measurement that says what.

Three experiments, in order of how much they assume:

1.  **Which path.** Switching the estimator on changes two things at once --
    the P term starts using fused pitch, and the D term starts using the raw
    gyro. Feed them independently and see which one is fatal. Costs four runs
    and rules out half the hypotheses before any spectral work.

2.  **The frequency response** theta_hat/theta, measured with a swept-sine,
    read at the ~12 rad/s crossover. Run under TWO excitations -- a force
    disturbance on the frame, and a commanded-velocity sweep -- because the
    accelerometer is corrupted by the board's own forward acceleration, and
    those two excite that coupling very differently. If the two responses
    disagree, "transfer function" is the wrong model for the error and that is
    itself the finding.

3.  **The discriminator.** Take the measured defect and reproduce it
    synthetically on top of TRUTH: a pure lag matched to the phase at
    crossover, and separately a constant offset matched to the measured bias.
    Whichever reproduces the strike is the mechanism. If neither does, the
    hypothesis is wrong and I would rather find that out in an afternoon.

The estimator runs as a Python replica here rather than through the FFI,
because the FFI exposes one switch and this needs the paths separated. The
replica is therefore CHECKED against the real Rust filter in shadow mode before
any of its output is believed -- see `validate_replica`. A replica that has
drifted from the shipped filter would make every number below fiction.

    scripts/analyse_estimator_phase.py --out-dir sim/out/experiments
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass
from pathlib import Path

import numpy as np

REPO = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(REPO))

import mujoco  # noqa: E402

from sim.scenarios.imperfections import STAGE0_PLACEHOLDER, ImperfectionState  # noqa: E402
from sim.scenarios.impulse_response import (  # noqa: E402
    frame_pitch_rad,
    nose_strike_angle_deg,
)
from sim.scenarios.plant import KT_NM_PER_A, build_model, imu_readings  # noqa: E402
from sim.scenarios.rust_controller import DEFAULT_R_EFF_M, RustController  # noqa: E402
from sim.scenarios.shuttle_run import ShuttleParams, VelocityProfile  # noqa: E402

INK, AMBER, MINT, MUTED, RED = "#16232E", "#F2A24A", "#2AAE97", "#96A8B0", "#C2513B"

#: Where the inner loop crosses over, rad/s. From the gain derivation in
#: `rust_controller.DEFAULT_KP_NM_PER_RAD`'s docstring, at the ridden inertia.
OMEGA_C = 12.0

# 200 A/rad, 30 A/(rad/s) at kt = 0.7 N*m/A, re-denominated in torque (#137).
GAINS = dict(kp_nm_per_rad=200.0 * KT_NM_PER_A, kd_nm_per_rad_s=30.0 * KT_NM_PER_A,
             max_current_a=40.0,
             kp_v_rad_per_m_s=0.05, ki_v_rad_per_m=0.02, com_above_axle=True)

#: Beyond this the nose is on the ground. Taken from the MODEL rather than
#: picked: the geometry says 18.57 deg, and an invented 25 deg threshold
#: silently reclassified two runs in this very script as survivors when the
#: board had already crashed.
STRIKE_DEG = nose_strike_angle_deg(build_model(70.0, 0.75, 40.0))


# ---------------------------------------------------------------------------
# Python replicas of the two shipped estimator pieces.
#
# Line-for-line ports of `control_core::ComplementaryFilter` and
# `WheelAccelEstimator`. Kept deliberately literal -- including the awkward
# first-sample snap and the dt<=0 guards -- because the point is fidelity to
# the shipped code, not elegance. `validate_replica` is what enforces that.
# ---------------------------------------------------------------------------

class PyComplementary:
    def __init__(self, tau_s: float) -> None:
        self.tau_s = tau_s
        self.pitch = 0.0
        self.rate = 0.0
        self.last_t_ns: int | None = None
        self.initialised = False

    def update(self, gyro, accel, t_ns: int, forward_accel: float) -> tuple[float, float]:
        theta_accel = math.atan2(accel[0] - forward_accel, -accel[2])
        if not self.initialised:
            self.pitch = theta_accel
            self.initialised = True
            self.last_t_ns = t_ns
            self.rate = gyro[1]
            return self.pitch, self.rate
        dt = (t_ns - self.last_t_ns) * 1e-9
        self.last_t_ns = t_ns
        self.rate = gyro[1]
        if dt > 0.0:
            alpha = self.tau_s / (self.tau_s + dt)
            self.pitch = alpha * (self.pitch + self.rate * dt) + (1.0 - alpha) * theta_accel
        return self.pitch, self.rate


class PyWheelAccel:
    def __init__(self, tau_s: float) -> None:
        self.tau_s = tau_s
        self.last_v: float | None = None
        self.accel = 0.0

    def update(self, v: float, dt: float) -> float:
        if self.last_v is None:
            self.last_v = v
            return 0.0
        prev, self.last_v = self.last_v, v
        if dt <= 0.0:
            return self.accel
        raw = (v - prev) / dt
        self.accel += (dt / (self.tau_s + dt)) * (raw - self.accel)
        return self.accel


# ---------------------------------------------------------------------------
# One stepping loop, with the sensing paths pulled apart
# ---------------------------------------------------------------------------

@dataclass
class Run:
    t: np.ndarray
    truth: np.ndarray          # rad, nose-up positive
    est: np.ndarray            # rad, the replica's estimate (always computed)
    fed: np.ndarray            # rad, what the P term actually received
    theta_acc: np.ndarray      # rad, raw accel-derived pitch (the LP branch input)
    aiding: np.ndarray         # m/s^2, the forward-accel compensation
    v: np.ndarray              # m/s
    current: np.ndarray        # A
    travel: np.ndarray         # m
    struck: bool
    strike_t: float


def simulate(
    *,
    seconds: float,
    p_source: str = "truth",
    d_source: str = "gyro",
    v_ref_fn=None,
    disturb_fn=None,
    lag_tau_s: float = 0.0,
    bias_rad: float = 0.0,
    r_eff_m: float = DEFAULT_R_EFF_M,
    est_tau_s: float = 1.0,
    aiding: str = "wheel",
) -> Run:
    """Step the ridden board, choosing independently what each term is fed.

    `p_source`: truth | estimate | truth_lagged | truth_biased
    `d_source`: gyro (the sensed rate, as every scenario uses) | truth
    `aiding`:   wheel (differentiated wheel speed) | none | torque
    """
    model = build_model(70.0, 0.75, 40.0)
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    mujoco.mj_forward(model, data)          # sensordata before the first read

    # Torque aiding needs the mass it is pushing: a = kt*I / (r_eff * m).
    total_mass = float(sum(model.body_mass))

    imp = ImperfectionState(profile=STAGE0_PLACEHOLDER, dt_s=dt)
    est = PyComplementary(est_tau_s)
    wheel_accel = PyWheelAccel(0.05)
    lagged = 0.0
    last_amps = 0.0
    x0 = float(data.xpos[frame][0])

    cols = {k: [] for k in
            ("t", "truth", "est", "fed", "theta_acc", "aiding", "v", "current", "travel")}
    struck, strike_t = False, float("nan")

    with RustController(**GAINS, v_ref_fn=v_ref_fn, use_estimator=0,
                        r_eff_m=r_eff_m) as ctl:
        for _ in range(int(seconds / dt)):
            now = float(data.time)
            data.xfrc_applied[frame] = 0.0
            if disturb_fn is not None:
                data.xfrc_applied[frame][:3] = [disturb_fn(now), 0.0, 0.0]

            truth = frame_pitch_rad(model, data)
            true_rate = float(data.qvel[4])
            sensed_rate = imp.gyro(true_rate)
            sensed_wheel = imp.wheel_rate(float(data.qvel[6]), now)

            true_gyro, true_accel = imu_readings(model, data)
            gyro_v = imp.gyro_vec(true_gyro)
            accel_v = imp.accel_vec(true_accel)

            if aiding == "wheel":
                a_fwd = wheel_accel.update(sensed_wheel * r_eff_m, dt)
            elif aiding == "torque":
                # Feed-forward from the command we just issued. Noise-free and
                # lag-free, unlike differentiating a quantised 100 Hz wheel
                # speed -- at the cost of being open-loop: it is blind to
                # slope, drag and slip, which the wheel-derived one does see.
                a_fwd = KT_NM_PER_A * last_amps / (r_eff_m * total_mass)
            else:
                a_fwd = 0.0
            e_pitch, _ = est.update(gyro_v, accel_v, int(now * 1e9), a_fwd)
            theta_acc = math.atan2(accel_v[0] - a_fwd, -accel_v[2])

            # A first-order lag on TRUTH, for the synthetic-defect experiment.
            lagged += (dt / (lag_tau_s + dt)) * (truth - lagged) if lag_tau_s > 0 else 0.0

            fed = {
                "truth": truth,
                "estimate": e_pitch,
                "truth_lagged": lagged if lag_tau_s > 0 else truth,
                "truth_biased": truth + bias_rad,
            }[p_source]
            rate_in = sensed_rate if d_source == "gyro" else true_rate

            amps = float(ctl(now, fed, rate_in, sensed_wheel,
                             gyro_rad_s=gyro_v, accel_m_s2=accel_v))
            last_amps = amps
            data.ctrl[0] = imp.apply_current(amps) * KT_NM_PER_A
            mujoco.mj_step(model, data)

            for k, val in (("t", float(data.time)), ("truth", truth), ("est", e_pitch),
                           ("fed", fed), ("theta_acc", theta_acc), ("aiding", a_fwd),
                           ("v", float(data.qvel[6]) * r_eff_m), ("current", amps),
                           ("travel", float(-(data.xpos[frame][0] - x0)))):
                cols[k].append(val)

            if not struck and abs(frame_pitch_rad(model, data)) > math.radians(STRIKE_DEG):
                struck, strike_t = True, float(data.time)
                break

    return Run(**{k: np.asarray(v) for k, v in cols.items()},
               struck=struck, strike_t=strike_t)


# ---------------------------------------------------------------------------
# 0. Is the replica actually the shipped filter?
# ---------------------------------------------------------------------------

def validate_replica() -> dict:
    """Run the REAL Rust filter in shadow alongside the replica, same trajectory.

    Shadow mode fuses and reports without driving, so both see an identical
    truth-driven run and any divergence is the replica's fault. Nothing further
    down is trustworthy if this does not come out at machine precision.
    """
    model = build_model(70.0, 0.75, 40.0)
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    frame = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, "frame")
    mujoco.mj_forward(model, data)

    imp = ImperfectionState(profile=STAGE0_PLACEHOLDER, dt_s=dt)
    est = PyComplementary(1.0)
    wheel_accel = PyWheelAccel(0.05)
    r_eff = DEFAULT_R_EFF_M
    rust, mine = [], []

    prof = VelocityProfile(ShuttleParams())
    with RustController(**GAINS, v_ref_fn=prof.v_ref, use_estimator=2) as ctl:
        for _ in range(int(min(prof.duration_s, 20.0) / dt)):
            now = float(data.time)
            truth = frame_pitch_rad(model, data)
            sensed_rate = imp.gyro(float(data.qvel[4]))
            sensed_wheel = imp.wheel_rate(float(data.qvel[6]), now)
            gyro_v = imp.gyro_vec(imu_readings(model, data)[0])
            accel_v = imp.accel_vec(imu_readings(model, data)[1])

            a_fwd = wheel_accel.update(sensed_wheel * r_eff, dt)
            e_pitch, _ = est.update(gyro_v, accel_v, int(now * 1e9), a_fwd)

            amps = float(ctl(now, truth, sensed_rate, sensed_wheel,
                             gyro_rad_s=gyro_v, accel_m_s2=accel_v))
            rust.append(ctl.pitch_used_rad)
            mine.append(e_pitch)
            data.ctrl[0] = imp.apply_current(amps) * KT_NM_PER_A
            mujoco.mj_step(model, data)

    d = np.degrees(np.abs(np.asarray(rust) - np.asarray(mine)))
    return {"max_abs_deg": float(d.max()), "rms_deg": float(np.sqrt((d**2).mean())),
            "samples": len(d),
            "faithful": bool(d.max() < 1e-3)}


# ---------------------------------------------------------------------------
# 1. Which path is fatal
# ---------------------------------------------------------------------------

def path_factorial() -> dict:
    prof = VelocityProfile(ShuttleParams())
    rows = []
    for p in ("truth", "estimate"):
        for dsrc in ("truth", "gyro"):
            r = simulate(seconds=prof.duration_s, p_source=p, d_source=dsrc,
                         v_ref_fn=prof.v_ref)
            rows.append(dict(
                p_source=p, d_source=dsrc, struck=r.struck,
                strike_t_s=None if math.isnan(r.strike_t) else round(r.strike_t, 3),
                peak_abs_pitch_deg=round(float(np.degrees(np.abs(r.truth)).max()), 3),
                est_rms_deg=round(float(np.degrees(
                    np.sqrt(((r.est - r.truth) ** 2).mean()))), 4),
                est_mean_err_deg=round(float(np.degrees((r.est - r.truth).mean())), 4),
            ))
    return {"rows": rows}


# ---------------------------------------------------------------------------
# 2. Frequency response, two excitations
# ---------------------------------------------------------------------------

def welch_tf(x, y, dt, nperseg=8192, overlap=0.75):
    """Cross-spectral estimate of Y/X, plus coherence.

    Welch rather than a single FFT ratio: it averages, and the coherence it
    yields is the honest statement of where the estimate means anything. A
    swept sine puts all its energy in one band per segment, and summing the
    spectra before dividing weights each frequency by the energy actually
    present there -- which is what we want.
    """
    x, y = np.asarray(x, float), np.asarray(y, float)
    nperseg = min(nperseg, len(x))
    step = max(1, int(nperseg * (1 - overlap)))
    win = np.hanning(nperseg)
    Pxx = Pyy = Pxy = 0.0
    for s in range(0, len(x) - nperseg + 1, step):
        xs = (x[s:s + nperseg] - x[s:s + nperseg].mean()) * win
        ys = (y[s:s + nperseg] - y[s:s + nperseg].mean()) * win
        X, Y = np.fft.rfft(xs), np.fft.rfft(ys)
        Pxx = Pxx + (X.conj() * X).real
        Pyy = Pyy + (Y.conj() * Y).real
        Pxy = Pxy + X.conj() * Y
    w = 2 * np.pi * np.fft.rfftfreq(nperseg, dt)
    H = Pxy / np.maximum(Pxx, 1e-30)
    coh = np.abs(Pxy) ** 2 / np.maximum(Pxx * Pyy, 1e-30)
    return w, H, coh


def chirp(t, f0, f1, dur, amp):
    """Log sweep. Phase is the integral of the instantaneous frequency."""
    if t >= dur:
        return 0.0
    k = (f1 / f0) ** (t / dur)
    phase = 2 * np.pi * f0 * dur * (k - 1) / math.log(f1 / f0)
    return amp * math.sin(phase)


def at_omega(w, H, coh, target):
    i = int(np.argmin(np.abs(w - target)))
    return dict(omega_rad_s=round(float(w[i]), 3),
                gain=round(float(np.abs(H[i])), 4),
                gain_db=round(float(20 * np.log10(max(abs(H[i]), 1e-12))), 3),
                phase_deg=round(float(np.degrees(np.angle(H[i]))), 3),
                coherence=round(float(coh[i]), 4))


def frequency_response() -> tuple[dict, dict]:
    """theta_hat/theta under a force sweep and a velocity-command sweep."""
    dur = 60.0
    cases = {
        # Amplitude is set so the board stays UP for the whole sweep. A run
        # that strikes stops early, which both truncates the record (coarsening
        # the frequency grid) and contaminates it with the fall -- and the
        # resulting Bode plot looks perfectly respectable. `struck` below is
        # the guard; if it ever trips, the numbers from that case are void.
        "force_sweep": dict(
            disturb_fn=lambda t: chirp(t, 0.5, 25.0, dur, 35.0), v_ref_fn=None),
        "vref_sweep": dict(
            disturb_fn=None,
            v_ref_fn=lambda t: chirp(t, 0.15, 6.0, dur, 0.45)),
    }
    summary, arrays = {}, {}
    for name, kw in cases.items():
        r = simulate(seconds=dur + 2.0, p_source="truth", **kw)
        dt = float(np.mean(np.diff(r.t)))
        w, H, coh = welch_tf(r.truth, r.est, dt)
        _, Hacc, cacc = welch_tf(r.truth, r.theta_acc, dt)

        # Peak gain is only reported where the estimate is CONDITIONED. An
        # unconstrained argmax over |H| finds wherever Pxx is smallest, which
        # is a statement about the excitation, not about the estimator.
        band = (w > 0.5) & (w < 200.0) & (coh > 0.8)
        summary[name] = {
            "struck": r.struck,
            "excitation_peak_pitch_deg": round(float(np.degrees(np.abs(r.truth)).max()), 3),
            "est_rms_deg": round(float(np.degrees(
                np.sqrt(((r.est - r.truth) ** 2).mean()))), 4),
            "at_crossover": at_omega(w, H, coh, OMEGA_C),
            "at_crossover_accel_branch": at_omega(w, Hacc, cacc, OMEGA_C),
            "at_1_rad_s": at_omega(w, H, coh, 1.0),
            "at_40_rad_s": at_omega(w, H, coh, 40.0),
            "peak_gain_in_band": round(float(np.abs(H[band]).max()), 4) if band.any() else None,
            "peak_gain_omega": round(float(w[band][np.argmax(np.abs(H[band]))]), 3)
            if band.any() else None,
            "peak_gain_coherence": round(float(coh[band][np.argmax(np.abs(H[band]))]), 3)
            if band.any() else None,
            "coherent_bandwidth_rad_s": [round(float(w[coh > 0.8].min()), 3),
                                         round(float(w[coh > 0.8].max()), 3)]
            if (coh > 0.8).any() else None,
        }
        arrays[name] = dict(w=w, H_real=H.real, H_imag=H.imag, coh=coh,
                            Hacc_real=Hacc.real, Hacc_imag=Hacc.imag, coh_acc=cacc,
                            t=r.t, truth=r.truth, est=r.est, theta_acc=r.theta_acc,
                            aiding=r.aiding, v=r.v)
    return summary, arrays


# ---------------------------------------------------------------------------
# 3. Reproduce the defect synthetically on top of truth
# ---------------------------------------------------------------------------

def discriminate(phase_deg_at_wc: float, bias_deg: float) -> dict:
    """Can a pure lag -- or a pure offset -- of the measured size kill it?

    The lag constant is chosen to match the MEASURED phase at crossover:
    a first-order lag contributes -atan(w*tau), so tau = tan(|phi|)/w_c.
    """
    prof = VelocityProfile(ShuttleParams())
    tau = math.tan(math.radians(min(abs(phase_deg_at_wc), 85.0))) / OMEGA_C
    rows = []

    base = simulate(seconds=prof.duration_s, p_source="truth", v_ref_fn=prof.v_ref)
    rows.append(dict(case="truth (baseline)", detail="-", struck=base.struck,
                     peak_abs_pitch_deg=round(float(np.degrees(np.abs(base.truth)).max()), 3)))

    r = simulate(seconds=prof.duration_s, p_source="truth_lagged",
                 lag_tau_s=tau, v_ref_fn=prof.v_ref)
    rows.append(dict(case="truth + matched lag",
                     detail=f"tau={tau*1000:.1f} ms ({phase_deg_at_wc:.1f} deg at w_c)",
                     struck=r.struck,
                     peak_abs_pitch_deg=round(float(np.degrees(np.abs(r.truth)).max()), 3)))

    # A lag ten times worse, to find where phase alone DOES break it -- a
    # negative result is only worth something if the test can produce a
    # positive one.
    r10 = simulate(seconds=prof.duration_s, p_source="truth_lagged",
                   lag_tau_s=tau * 10, v_ref_fn=prof.v_ref)
    rows.append(dict(case="truth + 10x lag", detail=f"tau={tau*10000:.1f} ms",
                     struck=r10.struck,
                     peak_abs_pitch_deg=round(float(np.degrees(np.abs(r10.truth)).max()), 3)))

    for mult in (1.0, 3.0):
        rb = simulate(seconds=prof.duration_s, p_source="truth_biased",
                      bias_rad=math.radians(bias_deg * mult), v_ref_fn=prof.v_ref)
        rows.append(dict(case=f"truth + {mult:g}x measured offset",
                         detail=f"{bias_deg*mult:+.3f} deg constant",
                         struck=rb.struck,
                         peak_abs_pitch_deg=round(
                             float(np.degrees(np.abs(rb.truth)).max()), 3)))

    real = simulate(seconds=prof.duration_s, p_source="estimate", v_ref_fn=prof.v_ref)
    rows.append(dict(case="the estimator itself", detail="-", struck=real.struck,
                     peak_abs_pitch_deg=round(float(np.degrees(np.abs(real.truth)).max()), 3)))

    return {"lag_tau_s": round(tau, 5), "rows": rows,
            "_traces": {"baseline": base, "estimator": real}}


# ---------------------------------------------------------------------------
# 4. The fix the diagnosis points at
# ---------------------------------------------------------------------------

def fix_sweep() -> dict:
    """If the accelerometer's corruption is the problem, lean on it less.

    Two knobs, from the two terms of the filter. theta_hat = HP(tau)*theta_gyro
    + LP(tau)*theta_accel, so a LONGER tau shrinks the accelerometer's weight
    at every frequency that matters -- the cost is gyro-bias drift, which is
    what tau was short for in the first place. The other knob is to make the
    accelerometer branch less wrong by cancelling the forward acceleration
    better. Sweep both, report survival AND error, because a configuration
    that survives with a useless estimate is not a fix.
    """
    prof = VelocityProfile(ShuttleParams())
    rows = []
    for aid in ("wheel", "torque", "none"):
        for tau in (1.0, 2.0, 5.0, 10.0, 20.0):
            r = simulate(seconds=prof.duration_s, p_source="estimate",
                         v_ref_fn=prof.v_ref, est_tau_s=tau, aiding=aid)
            err = np.degrees(r.est - r.truth)
            # Score the estimate only while the board is still upright: after a
            # strike the trajectory is a different one and the error is a
            # consequence of the fall rather than a cause of it.
            live = np.abs(np.degrees(r.truth)) < 12.0
            rows.append(dict(
                aiding=aid, tau_s=tau, struck=r.struck,
                strike_t_s=None if math.isnan(r.strike_t) else round(r.strike_t, 3),
                peak_abs_pitch_deg=round(float(np.degrees(np.abs(r.truth)).max()), 3),
                est_rms_deg=round(float(np.sqrt((err[live] ** 2).mean())), 4)
                if live.any() else None,
                est_max_abs_deg=round(float(np.abs(err[live]).max()), 4)
                if live.any() else None,
                travel_m=round(float(r.travel[-1]), 4),
            ))
    survivors = [r for r in rows if not r["struck"]]
    best = min(survivors, key=lambda r: r["est_rms_deg"]) if survivors else None
    return {"rows": rows, "survivors": len(survivors), "best": best}


# ---------------------------------------------------------------------------

def plot(fr_summary, fr_arrays, disc, out):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, axes = plt.subplots(2, 2, figsize=(13.5, 8.0))

    colours = {"force_sweep": AMBER, "vref_sweep": MINT}
    labels = {"force_sweep": "force disturbance sweep",
              "vref_sweep": "velocity-command sweep"}

    for name, arr in fr_arrays.items():
        w = arr["w"]
        H = arr["H_real"] + 1j * arr["H_imag"]
        ok = (arr["coh"] > 0.5) & (w > 0.3) & (w < 300)
        axes[0][0].semilogx(w[ok], 20 * np.log10(np.abs(H[ok])),
                            color=colours[name], lw=1.6, label=labels[name])
        axes[0][1].semilogx(w[ok], np.degrees(np.angle(H[ok])),
                            color=colours[name], lw=1.6, label=labels[name])
        axes[1][0].semilogx(w[(w > 0.3) & (w < 300)], arr["coh"][(w > 0.3) & (w < 300)],
                            color=colours[name], lw=1.2, label=labels[name])

    for a, ylab, title in ((axes[0][0], "|theta_hat/theta| (dB)", "Estimator gain"),
                           (axes[0][1], "phase (deg)", "Estimator phase"),
                           (axes[1][0], "coherence", "Coherence (below 0.5, ignore the plot above)")):
        a.axvline(OMEGA_C, ls="--", lw=1.2, color=INK, alpha=0.7)
        a.set_title(title, color=INK)
        a.set_xlabel("omega (rad/s)")
        a.set_ylabel(ylab)
        a.grid(alpha=0.25, which="both")
        a.legend(fontsize=8)
    axes[0][0].axhline(0, ls=":", lw=1.0, color=MUTED)
    axes[0][1].axhline(0, ls=":", lw=1.0, color=MUTED)
    axes[1][0].axhline(0.5, ls=":", lw=1.0, color=RED)
    axes[0][1].annotate("crossover", (OMEGA_C, 0), textcoords="offset points",
                        xytext=(6, 8), fontsize=8, color=INK)

    b = axes[1][1]
    base, real = disc["_traces"]["baseline"], disc["_traces"]["estimator"]
    b.plot(base.t, np.degrees(base.truth), color=INK, lw=1.6, label="truth pitch (baseline)")
    b.plot(real.t, np.degrees(real.truth), color=RED, lw=1.6,
           label="truth pitch (estimator in the loop)")
    b.plot(real.t, np.degrees(real.est - real.truth), color=MUTED, lw=1.0,
           label="estimator error")
    b.axhline(STRIKE_DEG, ls="--", lw=1.0, color=RED, alpha=0.6)
    b.set_ylim(-30, 30)
    b.set_title("The failure, in the time domain", color=INK)
    b.set_xlabel("time (s)"); b.set_ylabel("deg")
    b.legend(fontsize=8, loc="lower left"); b.grid(alpha=0.25)

    fig.suptitle("Estimator in the loop: locating the mechanism", color=INK, fontsize=13)
    fig.tight_layout()
    fig.savefig(out, dpi=130)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("--out-dir", type=Path, default=REPO / "sim" / "out" / "experiments")
    args = ap.parse_args()
    args.out_dir.mkdir(parents=True, exist_ok=True)

    print("validating the replica against the shipped Rust filter ...")
    val = validate_replica()
    print(f"  max |rust - replica| = {val['max_abs_deg']:.3e} deg over {val['samples']} samples"
          f"   {'OK' if val['faithful'] else 'DIVERGED -- everything below is suspect'}")

    print("\n1. which sensing path is fatal")
    fac = path_factorial()
    for r in fac["rows"]:
        print(f"  P={r['p_source']:<9} D={r['d_source']:<6} strike={str(r['struck']):<5}"
              f" peak {r['peak_abs_pitch_deg']:7.2f} deg"
              f"  est RMS {r['est_rms_deg']:.3f} mean {r['est_mean_err_deg']:+.3f}")

    print("\n2. frequency response")
    fr, arrays = frequency_response()
    for name, s in fr.items():
        c = s["at_crossover"]
        print(f"  {name:<12} at {c['omega_rad_s']:5.1f} rad/s:"
              f" gain {c['gain_db']:+6.2f} dB  phase {c['phase_deg']:+7.2f} deg"
              f"  (coherence {c['coherence']:.2f})")
        print(f"  {'':<12} peak gain {s['peak_gain_in_band']:.2f} at "
              f"{s['peak_gain_omega']:.1f} rad/s")

    # Take the phase from whichever VALID excitation is better conditioned at
    # w_c. A case that struck is excluded outright rather than down-weighted:
    # its record is a truncated fall, and coherence cannot tell you that.
    valid = {k: v for k, v in fr.items() if not v["struck"]}
    if not valid:
        print("  every sweep struck -- no usable frequency response")
        return 1
    for k, v in fr.items():
        if v["struck"]:
            print(f"  {k}: STRUCK during the sweep, excluded -- lower its amplitude")
    best = max(valid, key=lambda k: valid[k]["at_crossover"]["coherence"])
    phase = valid[best]["at_crossover"]["phase_deg"]
    # From a run that did NOT strike. Taking it from the diverged run would be
    # circular -- that mean error is mostly the fall, so the "is a constant
    # offset of this size fatal?" test would be asking about its own outcome.
    bias = next(r["est_mean_err_deg"] for r in fac["rows"]
                if r["p_source"] == "truth" and r["d_source"] == "gyro")

    print(f"\n3. synthetic defects (phase {phase:.1f} deg from {best}, "
          f"offset {bias:+.3f} deg)")
    disc = discriminate(phase, bias)
    for r in disc["rows"]:
        print(f"  {r['case']:<28} {r['detail']:<28} strike={str(r['struck']):<5}"
              f" peak {r['peak_abs_pitch_deg']:7.2f} deg")

    print("\n4. does leaning on the accelerometer less fix it?")
    fix = fix_sweep()
    for r in fix["rows"]:
        rms = "n/a" if r["est_rms_deg"] is None else f"{r['est_rms_deg']:6.3f}"
        print(f"  aiding={r['aiding']:<7} tau={r['tau_s']:5.1f}s  strike={str(r['struck']):<5}"
              f" peak {r['peak_abs_pitch_deg']:7.2f} deg  RMS {rms} deg"
              f"  travel {r['travel_m']:+7.3f} m")
    if fix["best"]:
        b = fix["best"]
        print(f"  -> best survivor: aiding={b['aiding']} tau={b['tau_s']}s, "
              f"{b['est_rms_deg']:.3f} deg RMS, returned to {b['travel_m']:+.3f} m")
    else:
        print("  -> NOTHING in the sweep survives; the fix is not in these two knobs")

    traces = disc.pop("_traces")
    plot(fr, arrays, {"_traces": traces}, args.out_dir / "estimator-phase.png")

    summary = {
        "note": "Deterministic and seeded; re-running reproduces these bit-for-bit.",
        "omega_crossover_rad_s": OMEGA_C,
        "replica_validation": val,
        "path_factorial": fac,
        "frequency_response": fr,
        "discriminator": disc,
        "fix_sweep": fix,
    }
    (args.out_dir / "estimator-phase.json").write_text(json.dumps(summary, indent=2) + "\n")
    np.savez_compressed(
        args.out_dir / "estimator-phase.npz",
        **{f"{n}_{k}": v for n, d in arrays.items() for k, v in d.items()},
        **{f"{n}_{k}": getattr(tr, k) for n, tr in traces.items()
           for k in ("t", "truth", "est", "fed", "current", "travel", "v")},
    )
    print(f"\nwrote {args.out_dir/'estimator-phase.json'}, .npz and .png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
