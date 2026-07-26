"""Bench-rig identification scenarios — Wave-0 "does the method work at all".

Three independent jobs against `sim/models/bench_rig.xml` (read its header
first; it states what does and does not transfer to the board):

    identify   two-run kt / rotor-inertia fit, bare vs flywheel-loaded
    spindown   viscous (b) / Coulomb (tau_c) friction fit from a decay
    replay     residual of the sim against a measured-hardware CSV

None of this predicts anything about the board. It proves the *identification
method* -- constant-current ramp, spin-down decay, residual-against-hardware --
on a rig small enough to sit on a desk, before it is pointed at a motor twenty
times the size that we do not yet have a bench for.

THE TWO-RUN kt / J_rotor FIT
-----------------------------
Held current, near w=0, friction is a small correction rather than the whole
story:

    kt*i - b*w - tau_c*sign(w) = J*dw/dt

so the initial slope of w(t) is alpha ~= kt*i/J. Two runs at the SAME current
-- bare rotor, then with the flywheel disc added -- give two equations in two
unknowns once `J_disc` is treated as known (it is a machined aluminium disc,
weighed and measured, not fitted):

    kt*i = J_bare*alpha_1
    kt*i = (J_bare + J_disc)*alpha_2
    =>  kt = J_disc / (i * (1/alpha_2 - 1/alpha_1))
        J_bare = kt*i/alpha_1

The bare variant is built by REGEX-STRIPPING the `flywheel` geom out of the
XML string and compiling with `mujoco.MjModel.from_xml_string` -- `MjsGeom`
has no `.delete()` and hand-rolling `MjSpec` surgery for one geom is not worth
the fragility it buys. The strip is verified, not trusted: `build_bare_model`
asserts the geom is actually gone and that the resulting joint inertia is
lower than the loaded rig's. A silent failure to strip would make the two
runs identical, which is a divide-by-zero waiting to happen rather than a
loud one.

kt IS BIASED BY A SMALL, KNOWN AMOUNT. The algebra above assumes zero
friction at the initial instant; Coulomb does not vanish at w=0, so the fit
actually recovers `kt - tau_c/i`, not kt exactly. Choosing i large enough that
`tau_c/i` is a small fraction of kt (see `IdentifyParams.commanded_current_a`)
keeps that bias under the acceptance threshold rather than pretending it does
not exist. `J_bare`, by contrast, comes out UNBIASED by this same algebra --
the friction term cancels between the two runs exactly, which is a genuine
property of the method and not a coincidence of these particular numbers.

SPIN-DOWN: b AND tau_c ARE FIT SEPARATELY, NOT LUMPED
------------------------------------------------------
A single lumped friction coefficient fits a decelerating flywheel badly:
Coulomb dominates at low speed, viscous at high speed, and a lumped term is
neither. It shows up as a residual that changes SIGN across the speed range --
the diagnostic this module reports (`lumped_residual_changes_sign`) rather
than asserts and trusts. The two-term fit instead regresses the logged
acceleration directly against `[w, sign(w)]`:

    J*dw/dt = -b*w - tau_c*sign(w)
    =>  qacc = -(b/J)*w - (tau_c/J)*sign(w)

`J` is read directly off the compiled model (`_joint_inertia`) rather than
re-derived here -- that is mode 1's job, not this one's.

CATEGORY ERROR GUARD
---------------------
This bench motor's kt is ~0.05 N*m/A (a 190 kv 6374). The onewheel hub
motor's is `plant.KT_NM_PER_A` = 0.7 (an unfitted guess, an order of magnitude
different machine). If a fitted bench kt ever lands within ~20% of
`plant.KT_NM_PER_A` that is a coincidence, not a validation -- nothing in this
module writes a bench-fitted value into `plant.py`, and nothing should.

WHAT ImperfectionProfile MEANS HERE
-------------------------------------
`STAGE0_PLACEHOLDER` (the default) degrades the COMMANDED current through the
same actuation delay / current-loop lag the board's own drive has, and
degrades the LOGGED shaft rate through the same quantisation / update rate --
because a real bench identification only ever sees the sensor's version of
events, never MuJoCo truth. `IDEAL` is for bring-up and for the numeric
acceptance tests, where the method itself is what is being checked and the
signal path must not be the thing under test.
"""

from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import re
from dataclasses import asdict, dataclass, field
from pathlib import Path

import mujoco
import numpy as np

from .imperfections import IDEAL, STAGE0_PLACEHOLDER, ImperfectionProfile, ImperfectionState

REPO = Path(__file__).resolve().parents[2]
MODEL_PATH = REPO / "sim" / "models" / "bench_rig.xml"
OUT_DIR = REPO / "sim" / "out" / "bench"

#: Nameplate kt for the bench motor (190 kv 6374): kt = 9.5493 / kv. This is
#: the GROUND TRUTH the sim uses to turn commanded current into the shaft
#: torque `data.ctrl` actually wants -- i.e. it stands in for the bench
#: motor's real electromagnetic torque constant, which `identify` then tries
#: to recover from the response, exactly as a real bench would from measured
#: current and speed.
#:
#: NOT plant.KT_NM_PER_A. That is the onewheel hub motor's (guessed) kt,
#: ~0.7 N*m/A -- a different machine by more than an order of magnitude. See
#: the module docstring's category-error guard.
NAMEPLATE_KT_NM_PER_A = 9.5493 / 190.0

#: Flywheel geometry, independent of the compiled model -- mass on a kitchen
#: scale and radius with a ruler, per the model header, is what makes this
#: trustworthy where the rotor's own inertia is not. Kept as named constants
#: (mirroring bench_rig.xml exactly) rather than read back off the compiled
#: model, because reading it off the model would make `J_disc` circular with
#: the very quantity the two-run fit is solving for.
FLYWHEEL_RADIUS_M = 0.075
FLYWHEEL_THICKNESS_M = 0.012
FLYWHEEL_DENSITY_KG_M3 = 2700.0  # 6061 aluminium

#: Threshold for `current_tracking_ok`. Past this the drive has derated by
#: more than plausible measurement jitter -- ICD noise floors on reported
#: current are a few hundred mA -- and every other residual in a replay run
#: is meaningless until this is understood.
CURRENT_TRACKING_TOLERANCE_A = 0.5

_FLYWHEEL_GEOM_RE = re.compile(r'<geom name="flywheel"[^>]*/>')


def known_disc_inertia_kg_m2() -> float:
    """`J_disc` from geometry and material, NOT from the compiled model.

    `1/2*m*r^2` is exact here because every rotor geom in bench_rig.xml is
    centred ON the shaft axis (see the model header's note on the eccentric-
    offset defect this replaced) -- an off-axis disc would need a parallel-
    axis correction this does not have.
    """
    mass = FLYWHEEL_DENSITY_KG_M3 * math.pi * FLYWHEEL_RADIUS_M**2 * FLYWHEEL_THICKNESS_M
    return 0.5 * mass * FLYWHEEL_RADIUS_M**2


def load_model(path: Path = MODEL_PATH) -> mujoco.MjModel:
    return mujoco.MjModel.from_xml_path(str(path))


def _joint_inertia(model: mujoco.MjModel) -> float:
    """Joint-space inertia about the shaft hinge, kg*m^2.

    The rig has one hinge and no other DOF, so the (1x1) mass matrix IS this
    number -- exactly the `J` the `alpha ~= kt*i/J` approximation assumes.
    """
    data = mujoco.MjData(model)
    mujoco.mj_forward(model, data)
    full_m = np.zeros((model.nv, model.nv))
    mujoco.mj_fullM(model, data, full_m)
    return float(full_m[0, 0])


def strip_flywheel_geom(xml: str) -> str:
    """Remove the flywheel geom from the MJCF string.

    A regex, not `MjSpec` surgery: `MjsGeom` has no `.delete()`, and building
    the bare variant through spec editing is not worth the fragility for one
    geom. The caller MUST verify the strip actually worked -- see
    `build_bare_model` -- because a silent no-op here would make the bare and
    loaded runs identical and turn the two-run fit into a divide-by-zero.
    """
    stripped, n = _FLYWHEEL_GEOM_RE.subn("", xml)
    if n != 1:
        raise RuntimeError(
            f"expected exactly one <geom name=\"flywheel\".../> to strip, found {n}. "
            "The bench_rig.xml flywheel geom's markup may have changed; update the regex."
        )
    return stripped


def build_bare_model(xml_text: str | None = None) -> mujoco.MjModel:
    """The bare-rotor variant used as run 1 of the two-run identification.

    Verified, not trusted: asserts the flywheel geom is actually gone and that
    the resulting joint inertia is lower than a loaded rig's would be. Either
    assertion failing means the strip silently did nothing.
    """
    xml_text = xml_text if xml_text is not None else MODEL_PATH.read_text()
    bare = mujoco.MjModel.from_xml_string(strip_flywheel_geom(xml_text))
    assert mujoco.mj_name2id(bare, mujoco.mjtObj.mjOBJ_GEOM, "flywheel") < 0, (
        "flywheel geom is still present after stripping -- the regex did not match"
    )
    return bare


def _fit_line(t: np.ndarray, y: np.ndarray, window_s: float) -> tuple[float, float]:
    """Least-squares slope of `y` vs `t` over `t <= window_s`, plus R^2.

    A window fit, not a two-sample finite difference -- so a bad window (too
    long, catching friction or a current cutback) is visible in the R^2
    rather than silently baked into a slope from two noisy points.
    """
    mask = t <= window_s
    tt, yy = t[mask], y[mask]
    if len(tt) < 2:
        raise ValueError(f"fit window {window_s}s contains fewer than 2 samples")
    design = np.vstack([tt, np.ones_like(tt)]).T
    (slope, intercept), *_ = np.linalg.lstsq(design, yy, rcond=None)
    pred = slope * tt + intercept
    ss_res = float(np.sum((yy - pred) ** 2))
    ss_tot = float(np.sum((yy - yy.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0
    return float(slope), float(r2)


def _run_constant_current(
    model: mujoco.MjModel, current_a: float, seconds: float, profile: ImperfectionProfile,
) -> tuple[np.ndarray, np.ndarray]:
    """Drive `model` with a held commanded current; log time and SENSED shaft
    rate (i.e. run through the same quantisation/update-rate a real bench
    identification would see, not MuJoCo truth)."""
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    n_steps = int(round(seconds / dt))
    imp = ImperfectionState(profile=profile, dt_s=dt)
    mujoco.mj_forward(model, data)

    ts, ws = [], []
    for _ in range(n_steps):
        current = imp.apply_current(current_a)
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)
        ts.append(float(data.time))
        ws.append(imp.wheel_rate(float(data.qvel[0]), float(data.time)))
    return np.asarray(ts), np.asarray(ws)


# ---------------------------------------------------------------------------
# Mode 1 -- identify
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class IdentifyParams:
    """Everything that defines one identify run. Frozen: the fitted numbers
    are only meaningful alongside the exact parameters that produced them."""

    commanded_current_a: float = 20.0
    """Held constant across both runs. Large enough that the Coulomb bias in
    the kt fit (tau_c/i, see the module docstring) stays under ~0.5% of kt
    while remaining well inside the rig's 40 A / 2.0 N*m envelope."""

    fit_window_s: float = 0.005
    """Early window for the alpha fit. Short enough that viscous drag (which
    grows with w, unlike Coulomb) has not yet bent the ramp measurably."""

    sim_seconds: float = 0.02
    """Just past the fit window, so nothing here depends on a current cutback
    or on friction dominating -- both would show up as a fit window that
    silently ran past the data available."""


@dataclass
class IdentifyMetrics:
    kt_fit_nm_per_a: float = 0.0
    j_bare_fit_kg_m2: float = 0.0
    j_disc_known_kg_m2: float = 0.0
    alpha_bare_rad_s2: float = 0.0
    alpha_loaded_rad_s2: float = 0.0
    r2_bare: float = 0.0
    r2_loaded: float = 0.0

    #: Against the nameplate/geometry references -- informational, NOT the
    #: acceptance criterion for a bare-motor J (there is no independent
    #: reference for that; see the module docstring on why J_bare comes out
    #: unbiased rather than being validated against a "known" number).
    kt_error_vs_nameplate_pct: float = 0.0

    imperfection_profile_id: str = ""
    commanded_current_a: float = 0.0
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0


@dataclass
class IdentifyResult:
    params: IdentifyParams
    metrics: IdentifyMetrics
    t_bare: np.ndarray = field(default_factory=lambda: np.empty(0))
    w_bare: np.ndarray = field(default_factory=lambda: np.empty(0))
    t_loaded: np.ndarray = field(default_factory=lambda: np.empty(0))
    w_loaded: np.ndarray = field(default_factory=lambda: np.empty(0))

    def to_dict(self) -> dict:
        return {"mode": "identify", "params": asdict(self.params), "metrics": asdict(self.metrics)}


def identify(
    params: IdentifyParams | None = None,
    loaded_model: mujoco.MjModel | None = None,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
) -> IdentifyResult:
    """Bare-vs-loaded two-run kt / J_bare fit. See the module docstring."""
    params = params or IdentifyParams()
    loaded_model = loaded_model or load_model()
    bare_model = build_bare_model()
    assert _joint_inertia(bare_model) < _joint_inertia(loaded_model), (
        "bare rig has no less inertia than the loaded rig -- the flywheel strip "
        "did not actually remove any mass"
    )

    t_bare, w_bare = _run_constant_current(
        bare_model, params.commanded_current_a, params.sim_seconds, profile
    )
    t_loaded, w_loaded = _run_constant_current(
        loaded_model, params.commanded_current_a, params.sim_seconds, profile
    )

    alpha1, r2_1 = _fit_line(t_bare, w_bare, params.fit_window_s)
    alpha2, r2_2 = _fit_line(t_loaded, w_loaded, params.fit_window_s)

    j_disc = known_disc_inertia_kg_m2()
    i = params.commanded_current_a
    kt_fit = j_disc / (i * (1.0 / alpha2 - 1.0 / alpha1))
    j_bare_fit = kt_fit * i / alpha1

    m = IdentifyMetrics(
        kt_fit_nm_per_a=kt_fit,
        j_bare_fit_kg_m2=j_bare_fit,
        j_disc_known_kg_m2=j_disc,
        alpha_bare_rad_s2=alpha1,
        alpha_loaded_rad_s2=alpha2,
        r2_bare=r2_1,
        r2_loaded=r2_2,
        kt_error_vs_nameplate_pct=abs(kt_fit - NAMEPLATE_KT_NM_PER_A) / NAMEPLATE_KT_NM_PER_A * 100.0,
        imperfection_profile_id=profile.profile_id,
        commanded_current_a=i,
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=float(loaded_model.opt.timestep),
    )
    return IdentifyResult(
        params=params, metrics=m, t_bare=t_bare, w_bare=w_bare, t_loaded=t_loaded, w_loaded=w_loaded,
    )


# ---------------------------------------------------------------------------
# Mode 2 -- spindown
# ---------------------------------------------------------------------------

@dataclass(frozen=True)
class SpindownParams:
    spin_up_current_a: float = 20.0
    spin_up_s: float = 0.05
    """Just enough to get the flywheel moving before current is cut -- the
    spin-up itself is not measured."""

    decay_s: float = 2.0
    """Long enough to cover a wide speed range (this rig's damping time
    constant is ~18 s, so the decay is nowhere near complete, but the speed
    swept is still wide enough to separate the two friction terms)."""


@dataclass
class SpindownMetrics:
    w0_rad_s: float = 0.0
    w_end_rad_s: float = 0.0

    b_fit_nm_s_per_rad: float = 0.0
    tau_c_fit_nm: float = 0.0
    r2: float = 0.0

    #: What the model was actually built with -- read off the compiled model
    #: rather than hardcoded, so a change to bench_rig.xml's placeholders
    #: changes what this is checked against automatically.
    b_placeholder_nm_s_per_rad: float = 0.0
    tau_c_placeholder_nm: float = 0.0
    b_error_pct: float = 0.0
    tau_c_error_pct: float = 0.0

    #: The lumped-fit diagnostic (module docstring): a single viscous-only
    #: term absorbs part of the Coulomb term into a biased b, and the
    #: residual it leaves behind changes SIGN between low and high speed.
    #: A diagnostic that never changes sign is not doing its job.
    lumped_b_only_nm_s_per_rad: float = 0.0
    lumped_residual_low_speed_rad_s2: float = 0.0
    lumped_residual_high_speed_rad_s2: float = 0.0
    lumped_residual_changes_sign: bool = False

    imperfection_profile_id: str = ""
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0


@dataclass
class SpindownResult:
    params: SpindownParams
    metrics: SpindownMetrics
    t: np.ndarray = field(default_factory=lambda: np.empty(0))
    w: np.ndarray = field(default_factory=lambda: np.empty(0))
    qacc: np.ndarray = field(default_factory=lambda: np.empty(0))

    def to_dict(self) -> dict:
        return {"mode": "spindown", "params": asdict(self.params), "metrics": asdict(self.metrics)}


def spindown(
    params: SpindownParams | None = None,
    model: mujoco.MjModel | None = None,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
) -> SpindownResult:
    """Spin up, cut current to zero, log the decay, fit b and tau_c
    separately. See the module docstring for why they are not lumped."""
    params = params or SpindownParams()
    model = model or load_model()
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    imp = ImperfectionState(profile=profile, dt_s=dt)
    mujoco.mj_forward(model, data)

    n_spin = int(round(params.spin_up_s / dt))
    for _ in range(n_spin):
        current = imp.apply_current(params.spin_up_current_a)
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)

    n_decay = int(round(params.decay_s / dt))
    t0 = float(data.time)
    ts, ws, accs = [], [], []
    for _ in range(n_decay):
        current = imp.apply_current(0.0)
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)
        ts.append(float(data.time) - t0)
        ws.append(imp.wheel_rate(float(data.qvel[0]), float(data.time)))
        accs.append(float(data.qacc[0]))

    t = np.asarray(ts)
    w = np.asarray(ws)
    acc = np.asarray(accs)
    j = _joint_inertia(model)

    # Two-term fit: qacc = -(b/J)*w - (tau_c/J)*sign(w).
    sgn = np.sign(w)
    design = np.vstack([w, sgn]).T
    (c1, c2), *_ = np.linalg.lstsq(design, acc, rcond=None)
    b_fit = -c1 * j
    tau_c_fit = -c2 * j
    pred = design @ np.array([c1, c2])
    ss_res = float(np.sum((acc - pred) ** 2))
    ss_tot = float(np.sum((acc - acc.mean()) ** 2))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0.0 else 1.0

    # Lumped (viscous-only) fit -- the defect the two-term fit replaces.
    b_lump = -float(np.dot(w, acc) / np.dot(w, w))
    resid_lump = acc - (-b_lump * w)
    order = np.argsort(w)
    k = max(1, len(order) // 20)  # slowest / fastest 5% of the logged speeds
    resid_lo = float(resid_lump[order[:k]].mean())
    resid_hi = float(resid_lump[order[-k:]].mean())

    b_true = float(model.dof_damping[0])
    tau_c_true = float(model.dof_frictionloss[0])

    m = SpindownMetrics(
        w0_rad_s=float(w[0]),
        w_end_rad_s=float(w[-1]),
        b_fit_nm_s_per_rad=b_fit,
        tau_c_fit_nm=tau_c_fit,
        r2=r2,
        b_placeholder_nm_s_per_rad=b_true,
        tau_c_placeholder_nm=tau_c_true,
        b_error_pct=abs(b_fit - b_true) / b_true * 100.0 if b_true else 0.0,
        tau_c_error_pct=abs(tau_c_fit - tau_c_true) / tau_c_true * 100.0 if tau_c_true else 0.0,
        lumped_b_only_nm_s_per_rad=b_lump,
        lumped_residual_low_speed_rad_s2=resid_lo,
        lumped_residual_high_speed_rad_s2=resid_hi,
        lumped_residual_changes_sign=(resid_lo > 0.0) != (resid_hi > 0.0),
        imperfection_profile_id=profile.profile_id,
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=dt,
    )
    return SpindownResult(params=params, metrics=m, t=t, w=w, qacc=acc)


# ---------------------------------------------------------------------------
# Mode 3 -- replay
# ---------------------------------------------------------------------------

_REPLAY_COLUMNS = ("t_s", "commanded_current_a", "reported_current_a", "shaft_rate_rad_s")


@dataclass
class ReplayMetrics:
    """Residual of the sim against a measured-hardware CSV. Flat and
    JSON-serialisable, for the metrics artifact."""

    rms_residual_rad_s: float = 0.0
    max_residual_rad_s: float = 0.0

    #: The FIRST 20% of the record -- an inertia/kt error shows up early, a
    #: friction error late, and lumping them together hides which one it is.
    rms_residual_early_rad_s: float = 0.0
    max_residual_early_rad_s: float = 0.0

    #: The cutback detector. If the drive silently derated, the sim is being
    #: asked to reproduce a torque the hardware never applied, and every
    #: residual above is meaningless.
    current_rms_divergence_a: float = 0.0
    current_max_divergence_a: float = 0.0
    current_tracking_ok: bool = False

    n_samples: int = 0
    imperfection_profile_id: str = ""
    model_sha256: str = ""
    mujoco_version: str = ""
    timestep_s: float = 0.0

    def to_dict(self) -> dict:
        return asdict(self)


def _read_replay_csv(path: Path) -> dict[str, np.ndarray]:
    if not path.exists():
        raise FileNotFoundError(
            f"replay CSV not found: {path}. Expected columns: {', '.join(_REPLAY_COLUMNS)}"
        )
    with path.open(newline="") as f:
        reader = csv.DictReader(f)
        fieldnames = reader.fieldnames or []
        missing = [c for c in _REPLAY_COLUMNS if c not in fieldnames]
        if missing:
            raise ValueError(
                f"{path} is missing column(s) {missing}; expected {', '.join(_REPLAY_COLUMNS)}"
            )
        rows = list(reader)
    if not rows:
        raise ValueError(f"{path} has a header but no data rows")
    return {c: np.array([float(r[c]) for r in rows], dtype=float) for c in _REPLAY_COLUMNS}


def replay(
    csv_path: str | Path,
    profile: ImperfectionProfile = STAGE0_PLACEHOLDER,
    model: mujoco.MjModel | None = None,
) -> ReplayMetrics:
    """Drive the sim with the RECORDED commanded current (sample-and-hold
    between samples) and report the residual against the recorded shaft rate.

    `model` defaults to the committed bench_rig.xml; a caller with a fitted
    variant (different rotor density, say) can pass one in to check the fit
    against the hardware record it came from.
    """
    csv_path = Path(csv_path)
    cols = _read_replay_csv(csv_path)
    t_meas = cols["t_s"]
    cmd = cols["commanded_current_a"]
    reported = cols["reported_current_a"]
    measured_w = cols["shaft_rate_rad_s"]

    model = model or load_model()
    data = mujoco.MjData(model)
    dt = float(model.opt.timestep)
    imp = ImperfectionState(profile=profile, dt_s=dt)
    mujoco.mj_forward(model, data)

    n_steps = int(round(float(t_meas[-1]) / dt))
    sim_t = np.empty(n_steps + 1)
    sim_w = np.empty(n_steps + 1)
    sim_t[0] = 0.0
    sim_w[0] = float(data.qvel[0])
    for i in range(1, n_steps + 1):
        t = i * dt
        # Sample-and-hold: the most recent recorded command at or before `t`.
        idx = max(0, int(np.searchsorted(t_meas, t, side="right")) - 1)
        current = imp.apply_current(float(cmd[idx]))
        data.ctrl[0] = current * NAMEPLATE_KT_NM_PER_A
        mujoco.mj_step(model, data)
        sim_t[i] = t
        sim_w[i] = imp.wheel_rate(float(data.qvel[0]), t)

    sim_w_at_meas = np.interp(t_meas, sim_t, sim_w)
    residual = sim_w_at_meas - measured_w
    rms = float(np.sqrt((residual**2).mean()))
    mx = float(np.abs(residual).max())

    n_early = max(1, int(round(0.2 * len(residual))))
    early = residual[:n_early]
    rms_early = float(np.sqrt((early**2).mean()))
    mx_early = float(np.abs(early).max())

    current_div = reported - cmd
    cur_rms = float(np.sqrt((current_div**2).mean()))
    cur_max = float(np.abs(current_div).max())
    tracking_ok = cur_max <= CURRENT_TRACKING_TOLERANCE_A

    return ReplayMetrics(
        rms_residual_rad_s=rms,
        max_residual_rad_s=mx,
        rms_residual_early_rad_s=rms_early,
        max_residual_early_rad_s=mx_early,
        current_rms_divergence_a=cur_rms,
        current_max_divergence_a=cur_max,
        current_tracking_ok=tracking_ok,
        n_samples=len(t_meas),
        imperfection_profile_id=profile.profile_id,
        model_sha256=hashlib.sha256(MODEL_PATH.read_bytes()).hexdigest()[:16],
        mujoco_version=mujoco.__version__,
        timestep_s=dt,
    )


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", choices=("identify", "spindown", "replay"))
    ap.add_argument("--current-a", type=float, default=IdentifyParams().commanded_current_a,
                    help="identify: held commanded current, A")
    ap.add_argument("--spin-current-a", type=float, default=SpindownParams().spin_up_current_a,
                    help="spindown: current during the spin-up, A")
    ap.add_argument("--decay-s", type=float, default=SpindownParams().decay_s,
                    help="spindown: how long to log the decay, s")
    ap.add_argument("--csv", type=Path, help="replay: measured-hardware CSV path")
    ap.add_argument("--ideal", action="store_true",
                    help="use the IDEAL profile instead of STAGE0_PLACEHOLDER "
                    "(bring-up only -- see imperfections.py)")
    ap.add_argument("--out-dir", type=Path, default=OUT_DIR)
    args = ap.parse_args()

    args.out_dir.mkdir(parents=True, exist_ok=True)
    profile = IDEAL if args.ideal else STAGE0_PLACEHOLDER

    if args.mode == "identify":
        result = identify(IdentifyParams(commanded_current_a=args.current_a), profile=profile)
        m = result.metrics
        print(f"=== bench identify  [profile={m.imperfection_profile_id}]")
        print(f"  kt fit          {m.kt_fit_nm_per_a:.5f} N*m/A "
              f"(nameplate {NAMEPLATE_KT_NM_PER_A:.5f}, {m.kt_error_vs_nameplate_pct:.3f}% off)")
        print(f"  J_bare fit      {m.j_bare_fit_kg_m2:.6e} kg*m^2")
        print(f"  J_disc (known)  {m.j_disc_known_kg_m2:.6e} kg*m^2")
        print(f"  alpha bare/loaded  {m.alpha_bare_rad_s2:.2f} / {m.alpha_loaded_rad_s2:.2f} rad/s^2")
        print(f"  R^2 bare/loaded    {m.r2_bare:.6f} / {m.r2_loaded:.6f}")
        out = result.to_dict()
    elif args.mode == "spindown":
        params = SpindownParams(spin_up_current_a=args.spin_current_a, decay_s=args.decay_s)
        result = spindown(params, profile=profile)
        m = result.metrics
        print(f"=== bench spindown  [profile={m.imperfection_profile_id}]")
        print(f"  speed swept     {m.w0_rad_s:.2f} -> {m.w_end_rad_s:.2f} rad/s")
        print(f"  b fit           {m.b_fit_nm_s_per_rad:.6e} N*m*s/rad "
              f"(placeholder {m.b_placeholder_nm_s_per_rad:.2e}, {m.b_error_pct:.3f}% off)")
        print(f"  tau_c fit       {m.tau_c_fit_nm:.6e} N*m "
              f"(placeholder {m.tau_c_placeholder_nm:.2e}, {m.tau_c_error_pct:.3f}% off)")
        print(f"  R^2             {m.r2:.6f}")
        print(f"  lumped (viscous-only) fit  b={m.lumped_b_only_nm_s_per_rad:.4e}, "
              f"residual sign change: {m.lumped_residual_changes_sign} "
              f"(lo={m.lumped_residual_low_speed_rad_s2:+.4f}, "
              f"hi={m.lumped_residual_high_speed_rad_s2:+.4f})")
        out = result.to_dict()
    else:
        if args.csv is None:
            ap.error("--csv is required for replay mode")
        m = replay(args.csv, profile=profile)
        print(f"=== bench replay  [profile={m.imperfection_profile_id}]  {args.csv}")
        print(f"  shaft-rate RMS / max residual        {m.rms_residual_rad_s:.4f} / "
              f"{m.max_residual_rad_s:.4f} rad/s")
        print(f"  early-window RMS / max residual       {m.rms_residual_early_rad_s:.4f} / "
              f"{m.max_residual_early_rad_s:.4f} rad/s")
        print(f"  commanded-vs-reported current RMS/max {m.current_rms_divergence_a:.4f} / "
              f"{m.current_max_divergence_a:.4f} A")
        if not m.current_tracking_ok:
            print("  *** CURRENT TRACKING FAILED -- drive appears to have derated. "
                  "every residual above is meaningless until this is explained. ***")
        out = {"mode": "replay", "metrics": m.to_dict()}

    mpath = args.out_dir / f"bench_{args.mode}.json"
    mpath.write_text(json.dumps(out, indent=2) + "\n")
    print(f"wrote {mpath}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
