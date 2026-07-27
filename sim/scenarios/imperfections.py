"""The sim's imperfection profile — ICD §12.

> *The sim must be a **margin instrument, not an optimism amplifier**.*

Every result so far was measured on a perfect sensor chain: no noise, no
quantisation, no lag beyond one whole control cycle. SR-SIM-3 requires a
versioned imperfection profile and **no ideal-only mode in CI**, and until this
existed the requirement was not merely unmet — the gate actively violated it,
so every margin number was optimistic by an unknown amount.

WHICH ROWS OF ICD §12 ACTUALLY BITE TODAY
------------------------------------------
The ICD lists twelve. Only some have anything to act on while pitch comes from
MuJoCo truth rather than an estimator, and implementing the rest now would be
writing code whose effect is unobservable.

Implemented, because they change the loop's behaviour right now:

* **Actuation delay** — sub-cycle, interpolated, on top of the structural loop
  delay (§5.2: additive, never inclusive).
* **Current-loop lag** — the drive does not deliver a step. A backend that
  echoes the command back as measured current is explicitly non-conforming.
* **Gyro noise and bias** — this one is *not* deferred despite there being no
  estimator, because the inner loop consumes pitch RATE directly and the ICD
  says the gyro's y-axis IS the pitch rate (§10.1, "no flip"). Noise here lands
  straight on the D term, which is exactly where noise hurts a PD controller.
* **Wheel-rate quantisation and update rate** — the outer loop's only input,
  derived from ERPM, which arrives quantised at a finite rate.
* **A current cap that actually binds** — without it saturation is unreachable
  and the anti-windup path has never been exercised.

Deferred, with the reason:

* **Accelerometer noise, IMU misalignment, IMU clock drift, DRDY latency** —
  these corrupt an *attitude estimate*. Pitch is currently truth, so they have
  no path to the controller. They land with the estimator, and the estimator's
  acceptance criterion is already written in terms of this profile being on.
* **Cold-start `Invalid` window, CAN burstiness, dropped cycles** — fault
  injection, which is worth more against a working loop than against one being
  brought up. After the estimator.

DETERMINISM
-----------
Noise is drawn from a seeded generator owned by the profile, so a run is
reproducible bit-for-bit. A profile that reached for global RNG state would
make every gate flaky and every regression un-bisectable.
"""

from __future__ import annotations

from dataclasses import asdict, dataclass, field

import numpy as np


@dataclass(frozen=True)
class ImperfectionProfile:
    """A named, versioned set of sim imperfections.

    `profile_id` is stamped into every run's manifest (ICD §6.2), so a result
    can never be read without knowing what was modelled when it was produced.
    """

    profile_id: str

    #: Command → torque, seconds. **The Stage-0 go/no-go number** (ICD §5.4
    #: estimates 0.4–1.0 ms). Additive on top of the loop's own delay.
    actuation_delay_s: float = 0.0

    #: First-order lag of the current loop, seconds. The drive does not deliver
    #: a step, and pretending it does hides phase the controller must tolerate.
    current_loop_tau_s: float = 0.0

    #: Gyro white noise, rad/s (1σ) and a fixed bias, rad/s. Lands on the
    #: inner loop's D term.
    gyro_noise_rad_s: float = 0.0
    gyro_bias_rad_s: float = 0.0

    #: Accelerometer white noise, m/s² (1σ). **Live as of the estimator** — it
    #: was deferred while pitch came from truth, because it had no path to the
    #: controller. Now it lands directly on the attitude the balance law acts
    #: on, which is a far more consequential place than gyro noise on the D
    #: term.
    accel_noise_m_s2: float = 0.0

    #: Wheel-rate resolution and refresh. ERPM is an integer arriving at a
    #: finite rate, so the outer loop sees a staircase, not a smooth signal.
    wheel_rate_quantum_rad_s: float = 0.0
    wheel_rate_update_hz: float = 0.0

    #: Current cap, amps. Chosen so saturation is reachable.
    max_current_a: float = 40.0

    # -- proportional cutback ---------------------------------------------
    #
    # THE MECHANISM THE SIM COULD NOT SHOW. `control-core`'s feedforward
    # docstring states the gap in as many words: a VESC "derates commanded
    # torque through at least six cutback layers — battery voltage sag,
    # input-current limits, FET and motor temperature, ERPM limits… it would
    # appear under load, on a hill, at low state of charge, with a rider
    # aboard. The sim cannot show it, because the sim has no cutback."
    #
    # This is that row. It models the SPEED-dependent layer only (ERPM /
    # duty-cycle), because that is the one a descent actually exercises: the
    # faster the wheel spins, the less current the drive will pass, and a
    # balancer descending at speed needs that current precisely when it is
    # least available. The thermal and voltage-sag layers are real and are
    # NOT modelled here — they need a battery and a thermal state the sim
    # does not carry.
    #
    # **Every number is a placeholder awaiting Stage-0 bench measurement**,
    # like every other row in this class. The SHAPE is the claim; the
    # breakpoints are not.

    #: Wheel rate at which cutback begins, rad/s. Zero disables cutback
    #: entirely, which is why it is the default: adding this row must not
    #: silently move any existing scenario's baseline.
    derate_onset_rad_s: float = 0.0

    #: Wheel rate at and above which only `derate_floor_frac` of the cap
    #: remains. Linear in between.
    derate_full_rad_s: float = 0.0

    #: Fraction of `max_current_a` still available in full cutback. 1.0 is no
    #: cutback; 0.0 would be a drive that stops passing current altogether,
    #: which no real VESC does.
    derate_floor_frac: float = 1.0

    seed: int = 12345

    def models_cutback(self) -> bool:
        """True if this profile derates with speed.

        Callers use this to decide whether `apply_current` needs a wheel rate.
        """
        return self.derate_onset_rad_s > 0.0 and self.derate_floor_frac < 1.0

    def available_current_a(self, wheel_rate_rad_s: float) -> float:
        """The cap the drive will actually honour at this wheel speed.

        Symmetric in direction: cutback is about how fast the wheel is
        turning, not which way. Braking into a descent is exactly the case
        where that matters.
        """
        if not self.models_cutback():
            return self.max_current_a
        w = abs(float(wheel_rate_rad_s))
        if w <= self.derate_onset_rad_s:
            return self.max_current_a
        span = self.derate_full_rad_s - self.derate_onset_rad_s
        if span <= 0.0:
            frac = self.derate_floor_frac
        else:
            t = min(1.0, (w - self.derate_onset_rad_s) / span)
            frac = 1.0 + t * (self.derate_floor_frac - 1.0)
        return self.max_current_a * frac

    def is_ideal(self) -> bool:
        """True if this models nothing. CI must refuse to gate on such a run."""
        return (
            self.actuation_delay_s == 0.0
            and self.current_loop_tau_s == 0.0
            and self.gyro_noise_rad_s == 0.0
            and self.gyro_bias_rad_s == 0.0
            and self.accel_noise_m_s2 == 0.0
            and self.wheel_rate_quantum_rad_s == 0.0
            and not self.models_cutback()
        )

    def to_dict(self) -> dict:
        return asdict(self)


#: All zeros. **Not usable as a gate** — `is_ideal()` is true, and the tests
#: refuse it. Kept because during bring-up it is genuinely useful to separate a
#: sign error from noise, and forbidding it outright would just get an
#: equivalent added under another name.
IDEAL = ImperfectionProfile(profile_id="ideal-v1")

#: The working profile. **Every value is an ICD §12 placeholder awaiting
#: Stage-0 measurement**, not a fitted number:
#:
#:   actuation delay   1.0 ms   ICD §5.4 midpoint of the 0.4-1.0 ms estimate
#:   current loop      1.0 ms   ICD §12 "1st-order, ~1 ms"
#:   gyro noise        0.004 rad/s  ICM-42688-P datasheet noise density x2,
#:                                  per ICD §12's "datasheet x 2"
#:   gyro bias         0.002 rad/s  a small fixed offset, un-calibrated
#:   wheel quantum     0.007 rad/s  1 ERPM = 6.98e-3 rad/s (ICD §10.5)
#:   wheel update      500 Hz       the STATUS rate (ICD §11.2)
#:   current cap       40 A         the envelope limit
STAGE0_PLACEHOLDER = ImperfectionProfile(
    profile_id="stage0-placeholder-v1",
    actuation_delay_s=0.001,
    current_loop_tau_s=0.001,
    gyro_noise_rad_s=0.004,
    gyro_bias_rad_s=0.002,
    wheel_rate_quantum_rad_s=0.00698,
    wheel_rate_update_hz=500.0,
    max_current_a=40.0,
    #: ICM-42688-P accel noise density x2 per ICD §12, integrated over a
    #: 250 Hz band. Small in absolute terms, but it enters the attitude
    #: estimate through an atan2 and is not attenuated by the plant.
    accel_noise_m_s2=0.02,
)

#: STAGE-0 plus the speed-dependent cutback layer. **Use this for anything
#: that descends.**
#:
#: Kept as a separate profile rather than folded into
#: `STAGE0_PLACEHOLDER` on purpose: adding cutback to the shared profile
#: would move every existing CI baseline in one commit, and a baseline that
#: moves for two reasons at once cannot be reviewed. The `profile_id` is
#: stamped into every manifest (ICD §6.2), so a run can never be read without
#: knowing whether cutback was modelled.
#:
#: Breakpoints, all **placeholders awaiting Stage-0 bench measurement** — the
#: shape is the claim, not the numbers. At r_eff = 0.14605 m:
#:
#:   onset  27 rad/s  ≈ 4 m/s  (~14 km/h), where duty starts to bind
#:   full   55 rad/s  ≈ 8 m/s  (~29 km/h), a plausible top speed
#:   floor  0.35      → 14 A of the 40 A cap still available
#:
#: **Sr. Mechanical & Systems owns whether these match the real drive.** They
#: are shaped to be defensible, not measured.
STAGE0_CUTBACK = ImperfectionProfile(
    profile_id="stage0-cutback-v1",
    actuation_delay_s=0.001,
    current_loop_tau_s=0.001,
    gyro_noise_rad_s=0.004,
    gyro_bias_rad_s=0.002,
    wheel_rate_quantum_rad_s=0.00698,
    wheel_rate_update_hz=500.0,
    max_current_a=40.0,
    accel_noise_m_s2=0.02,
    derate_onset_rad_s=27.0,
    derate_full_rad_s=55.0,
    derate_floor_frac=0.35,
)


@dataclass
class ImperfectionState:
    """Per-run mutable state for a profile. One per scenario run.

    Separate from the profile so the profile stays a frozen, shareable
    description and two runs cannot contaminate each other's noise stream.
    """

    profile: ImperfectionProfile
    dt_s: float
    _rng: np.random.Generator = field(init=False)
    _cmd_history: list = field(init=False, default_factory=list)
    _applied_a: float = field(init=False, default=0.0)
    #: Cap the drive honoured on the last `apply_current` call, amps. Scenarios
    #: report it so a run that was cutback-limited can be told apart from one
    #: that simply did not ask for much current.
    _available_a: float = field(init=False, default=0.0)
    _held_wheel_rate: float = field(init=False, default=0.0)
    _last_wheel_update_s: float = field(init=False, default=-1e9)

    def __post_init__(self) -> None:
        self._rng = np.random.default_rng(self.profile.seed)

    # -- sensing -----------------------------------------------------------

    def gyro(self, true_rate_rad_s: float) -> float:
        """Pitch rate as the gyro reports it."""
        p = self.profile
        if p.gyro_noise_rad_s == 0.0 and p.gyro_bias_rad_s == 0.0:
            return true_rate_rad_s
        return float(
            true_rate_rad_s
            + p.gyro_bias_rad_s
            + self._rng.normal(0.0, p.gyro_noise_rad_s)
        )

    def gyro_vec(self, true_gyro: np.ndarray) -> np.ndarray:
        """Whole gyro vector. Only `[1]` reaches the controller today, but the
        estimator takes a vector and noising one component would quietly make
        the other two suspiciously perfect."""
        p = self.profile
        out = np.asarray(true_gyro, dtype=float).copy()
        if p.gyro_noise_rad_s > 0.0:
            out += self._rng.normal(0.0, p.gyro_noise_rad_s, size=3)
        out[1] += p.gyro_bias_rad_s
        return out

    def accel_vec(self, true_accel: np.ndarray) -> np.ndarray:
        p = self.profile
        out = np.asarray(true_accel, dtype=float).copy()
        if p.accel_noise_m_s2 > 0.0:
            out += self._rng.normal(0.0, p.accel_noise_m_s2, size=3)
        return out

    def wheel_rate(self, true_rate_rad_s: float, t_s: float) -> float:
        """Wheel rate as the drive reports it: quantised, and held between
        updates rather than interpolated -- a stale sample is what the loop
        actually gets, and smoothing it would hide the phase it costs."""
        p = self.profile
        if p.wheel_rate_update_hz <= 0.0:
            return self._quantise(true_rate_rad_s)
        if t_s - self._last_wheel_update_s >= 1.0 / p.wheel_rate_update_hz:
            self._held_wheel_rate = self._quantise(true_rate_rad_s)
            self._last_wheel_update_s = t_s
        return self._held_wheel_rate

    def _quantise(self, v: float) -> float:
        q = self.profile.wheel_rate_quantum_rad_s
        return float(v) if q <= 0.0 else float(np.round(v / q) * q)

    # -- actuation ---------------------------------------------------------

    def apply_current(
        self, commanded_a: float, wheel_rate_rad_s: float | None = None
    ) -> float:
        """Current the plant actually sees, after cutback, transport delay and
        the current loop's own dynamics.

        Cutback first — the drive decides what it is willing to pass before
        anything else happens to the signal — then delay, then the lag. Delay
        and lag are physically in series that way round, and swapping them
        changes the phase the controller sees.

        `wheel_rate_rad_s` is REQUIRED when the profile models cutback, and
        omitting it raises rather than defaulting. This codebase has twice
        shipped a result that was wrong because an optional argument was left
        at its default and the code degraded quietly instead of failing: the
        shuttle that never passed its IMU, and the analysis loop whose
        estimator was never in the loop at all. A cutback model that silently
        stops cutting back when a caller forgets an argument would be the
        third, and it would do it on the one scenario built to find nosedives.
        """
        p = self.profile
        if p.models_cutback():
            if wheel_rate_rad_s is None:
                raise ValueError(
                    f"profile {p.profile_id!r} models cutback, so apply_current() "
                    "needs wheel_rate_rad_s. Refusing to silently apply no cutback "
                    "-- pass the wheel rate, or use a profile without cutback."
                )
            cap = p.available_current_a(wheel_rate_rad_s)
        else:
            cap = p.max_current_a
        self._available_a = cap
        commanded_a = max(-cap, min(cap, commanded_a))

        # FRACTIONAL transport delay, interpolated between the two straddling
        # samples.
        #
        # Rounding to whole cycles looks harmless and is not: at the 2 ms
        # timestep the ICD's 1 ms estimate is exactly half a cycle, and
        # `round(0.5)` is 0 in Python, so the delay silently became ZERO --
        # quietly deleting the very imperfection this class exists to model,
        # and doing it for the ICD's own headline figure. Interpolating makes
        # any delay representable at any timestep, and removes a dependency on
        # the physics rate that has nothing to do with the physics.
        cycles = max(0.0, p.actuation_delay_s / self.dt_s)
        whole = int(cycles)
        frac = cycles - whole

        self._cmd_history.append(commanded_a)
        # Keep just enough history to straddle the delay.
        while len(self._cmd_history) > whole + 2:
            self._cmd_history.pop(0)

        def sample(back: int) -> float:
            """`back` cycles ago; 0 is this cycle. Before the run started the
            command was zero, which is the honest boundary condition."""
            i = len(self._cmd_history) - 1 - back
            return self._cmd_history[i] if i >= 0 else 0.0

        delayed = (1.0 - frac) * sample(whole) + frac * sample(whole + 1)

        if p.current_loop_tau_s <= 0.0:
            self._applied_a = delayed
        else:
            alpha = self.dt_s / (p.current_loop_tau_s + self.dt_s)
            self._applied_a += alpha * (delayed - self._applied_a)
        return self._applied_a
