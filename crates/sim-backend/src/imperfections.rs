//! The sim's imperfection profile — ICD §12, issue #129.
//!
//! Rust-side counterpart to `sim/scenarios/imperfections.py`, whose module
//! doc is the actual design rationale (which rows are modelled, which are
//! deferred, and why). That file also owns the profile *values* — this
//! module owns reproducing its *chain* exactly, checked bit-for-bit against
//! its generated conformance vectors by
//! `tests/imperfection_conformance.rs`. If a number here looks wrong, the
//! fix belongs in Python; this module's job is to match it, not to improve
//! on it.
//!
//! Deliberately NOT bit-identical to Python: the noise rows. Gyro/accel
//! noise here comes from this module's own seeded generator, not a
//! reimplementation of NumPy's PCG64 — conformance for those rows is
//! distributional (see `tests/imperfection_conformance.rs`'s sibling unit
//! tests below), exactly as `sim/scenarios/imperfections.py`'s own
//! conformance-contract section specifies.

/// A named, versioned set of sim imperfections. Mirrors
/// `sim.scenarios.imperfections.ImperfectionProfile` field-for-field; see
/// that class for what each row means and why it has the value it does.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct ImperfectionProfile {
    pub profile_id: &'static str,
    pub actuation_delay_s: f64,
    pub current_loop_tau_s: f64,
    pub gyro_noise_rad_s: f64,
    pub gyro_bias_rad_s: f64,
    pub accel_noise_m_s2: f64,
    pub wheel_rate_quantum_rad_s: f64,
    pub wheel_rate_update_hz: f64,
    pub max_current_a: f64,
    pub derate_onset_rad_s: f64,
    pub derate_full_rad_s: f64,
    pub derate_floor_frac: f64,
    pub seed: u64,
}

impl Default for ImperfectionProfile {
    fn default() -> Self {
        IDEAL
    }
}

impl ImperfectionProfile {
    /// True if this profile derates with speed. Mirrors
    /// `ImperfectionProfile.models_cutback` exactly.
    pub fn models_cutback(&self) -> bool {
        self.derate_onset_rad_s > 0.0 && self.derate_floor_frac < 1.0
    }

    /// The cap the drive will actually honour at this wheel speed. Mirrors
    /// `ImperfectionProfile.available_current_a` exactly, including its
    /// symmetric (direction-independent) treatment of wheel rate.
    pub fn available_current_a(&self, wheel_rate_rad_s: f64) -> f64 {
        if !self.models_cutback() {
            return self.max_current_a;
        }
        let w = wheel_rate_rad_s.abs();
        if w <= self.derate_onset_rad_s {
            return self.max_current_a;
        }
        let span = self.derate_full_rad_s - self.derate_onset_rad_s;
        let frac = if span <= 0.0 {
            self.derate_floor_frac
        } else {
            let t = ((w - self.derate_onset_rad_s) / span).min(1.0);
            1.0 + t * (self.derate_floor_frac - 1.0)
        };
        self.max_current_a * frac
    }

    /// True if this profile models nothing deterministic or stochastic.
    /// Mirrors `ImperfectionProfile.is_ideal` exactly — same fields, same
    /// omissions (`wheel_rate_update_hz` and `max_current_a` are
    /// deliberately not part of the check on the Python side either).
    pub fn is_ideal(&self) -> bool {
        self.actuation_delay_s == 0.0
            && self.current_loop_tau_s == 0.0
            && self.gyro_noise_rad_s == 0.0
            && self.gyro_bias_rad_s == 0.0
            && self.accel_noise_m_s2 == 0.0
            && self.wheel_rate_quantum_rad_s == 0.0
            && !self.models_cutback()
    }
}

/// All zeros. Mirrors `sim.scenarios.imperfections.IDEAL`. Not usable as a
/// gate on the Python side and not intended as one here either — kept for
/// the same bring-up reason: separating a sign error from noise.
pub const IDEAL: ImperfectionProfile = ImperfectionProfile {
    profile_id: "ideal-v1",
    actuation_delay_s: 0.0,
    current_loop_tau_s: 0.0,
    gyro_noise_rad_s: 0.0,
    gyro_bias_rad_s: 0.0,
    accel_noise_m_s2: 0.0,
    wheel_rate_quantum_rad_s: 0.0,
    wheel_rate_update_hz: 0.0,
    max_current_a: 60.0,
    derate_onset_rad_s: 0.0,
    derate_full_rad_s: 0.0,
    derate_floor_frac: 1.0,
    seed: 12345,
};

/// Mirrors `sim.scenarios.imperfections.STAGE0_PLACEHOLDER` field-for-field.
/// Every value there is a placeholder awaiting Stage-0 bench measurement —
/// see that module for the derivation of each one; this is a value copy,
/// not a re-derivation.
pub const STAGE0_PLACEHOLDER: ImperfectionProfile = ImperfectionProfile {
    profile_id: "stage0-placeholder-v1",
    actuation_delay_s: 0.001,
    current_loop_tau_s: 0.001,
    gyro_noise_rad_s: 0.004,
    gyro_bias_rad_s: 0.002,
    accel_noise_m_s2: 0.02,
    wheel_rate_quantum_rad_s: 0.00698,
    wheel_rate_update_hz: 500.0,
    max_current_a: 60.0,
    derate_onset_rad_s: 0.0,
    derate_full_rad_s: 0.0,
    derate_floor_frac: 1.0,
    seed: 12345,
};

/// Mirrors `sim.scenarios.imperfections.STAGE0_CUTBACK` field-for-field.
/// STAGE0_PLACEHOLDER plus the speed-dependent cutback layer — use this for
/// anything that descends.
pub const STAGE0_CUTBACK: ImperfectionProfile = ImperfectionProfile {
    profile_id: "stage0-cutback-v1",
    actuation_delay_s: 0.001,
    current_loop_tau_s: 0.001,
    gyro_noise_rad_s: 0.004,
    gyro_bias_rad_s: 0.002,
    accel_noise_m_s2: 0.02,
    wheel_rate_quantum_rad_s: 0.00698,
    wheel_rate_update_hz: 500.0,
    max_current_a: 60.0,
    derate_onset_rad_s: 27.0,
    derate_full_rad_s: 55.0,
    derate_floor_frac: 0.35,
    seed: 12345,
};

/// **Sensor noise only, derived (not chosen) from the ICM-42688-P datasheet
/// — the part `STAGE0_PLACEHOLDER`/`STAGE0_CUTBACK` already cite by name for
/// their own gyro/accel rows. Everything else is left at [`IDEAL`]'s zero:**
/// no actuation delay, no current-loop lag, no wheel-rate quantisation or
/// cutback. This exists to isolate ONE question -- what does the sensor
/// chain's own noise floor cost a margin claimed on a noiseless plant --
/// from the other, already-modelled imperfections above, which have their
/// own knobs and their own margin numbers.
///
/// **PROVENANCE CAVEAT, stated once and binding for every figure below:**
/// the ICM-42688-P is not sourced from a BoM line here -- it is the part
/// `sim/scenarios/imperfections.py`'s own `STAGE0_PLACEHOLDER` comment
/// already names, so this profile uses ITS datasheet, per the same
/// convention. The datasheet figures below are transcribed from the
/// commonly-published ICM-42688-P spec (the same "rate noise spectral
/// density" / "noise spectral density" numbers widely quoted for this part)
/// from this session's own knowledge, **not fetched from the live PDF** --
/// this sandbox has no network access. Cross-check every number below
/// against the actual TDK InvenSense datasheet before this profile is
/// treated as a bench-validated spec rather than a first-principles
/// estimate; that is a stronger caveat than "BoM pending" and is flagged
/// here for exactly that reason.
///
/// THE CONVERSION EVERYONE GETS WRONG: noise DENSITY (deg/s/sqrt(Hz),
/// ug/sqrt(Hz)) is a power-spectral-density amplitude, not a sigma. For a
/// white-noise process sampled at `f_s`, the discrete-sample standard
/// deviation is `sigma = density * sqrt(f_s / 2)` -- the sqrt(Nyquist
/// bandwidth) term is not optional, and using `f_s` instead of `f_s / 2`
/// (or skipping the bandwidth term altogether) is the single most common way
/// this conversion is silently wrong.
///
/// **Which `f_s`?** Not the ICD's headroom figure. `board_types::
/// MAX_IMU_SAMPLES`'s own doc comment reserves room for "a 4 kHz ODR at a
/// 500 Hz loop", but `SimBackend::wait_observe` only ever emits ONE
/// `ImuSample` per cycle (`obs.imu_count = 1`, above) -- this backend does
/// not sub-sample a faster stream, so the rate noise is actually being drawn
/// AT here is the 500 Hz cycle rate, not the IMU's own native ODR. Using the
/// higher, unexercised ODR figure would understate the per-sample sigma this
/// sim actually needs; this derivation uses the rate the backend truly runs
/// at. (If `wait_observe` ever grows to emit multiple samples per cycle at
/// the true ODR, this derivation is where to redo the arithmetic.)
///
/// ```text
/// f_s          = 500 Hz (CYCLE_NS, this crate)
/// Nyquist BW   = f_s / 2 = 250 Hz
/// sqrt(BW)     = 15.8114
/// ```
///
/// GYRO (pitch-rate channel -- lands directly on the inner loop's D term):
///
/// ```text
/// ND_gyro (ICM-42688-P, "Rate Noise Spectral Density", typ, all FSR)
///     = 2.8 mdps/sqrt(Hz) = 0.0028 deg/s/sqrt(Hz)
///     = 0.0028 * (pi/180) rad/s/sqrt(Hz) = 4.887e-5 rad/s/sqrt(Hz)
/// sigma_1x  = 4.887e-5 * 15.8114               = 7.729e-4 rad/s
/// sigma     = sigma_1x * 2 (ICD SS12 "datasheet x2" margin convention --
///             the same doubling STAGE0_PLACEHOLDER's own comment
///             cites for its gyro/accel rows, applied here rather than
///             invented fresh)
///           = 1.546e-3 rad/s  (~0.0885 deg/s, 1 sigma per sample)
/// ```
///
/// ACCEL (feeds the attitude estimate through atan2, per this crate's own
/// `accel_vec` comment):
///
/// ```text
/// ND_accel (ICM-42688-P, "Noise Spectral Density", typ)
///     = 70 ug/sqrt(Hz) = 70e-6 * 9.80665 m/s^2/sqrt(Hz)
///     = 6.865e-4 m/s^2/sqrt(Hz)
/// sigma_1x  = 6.865e-4 * 15.8114               = 1.0854e-2 m/s^2
/// sigma     = sigma_1x * 2 (same SS12 convention)
///           = 2.171e-2 m/s^2  (1 sigma per sample)
/// ```
///
/// Both land close to `STAGE0_PLACEHOLDER`'s own placeholder figures (0.02
/// m/s^2 accel, 8.5% away from 0.02171) -- EXCEPT gyro, where the
/// placeholder's 0.004 rad/s is ~2.6x this derivation's 1.546e-3 rad/s. That
/// gap is consistent with the placeholder's gyro row having been computed at
/// something closer to the IMU's native ~4 kHz ODR (Nyquist 2 kHz) rather
/// than the 500 Hz rate `wait_observe` actually samples at -- i.e. the two
/// existing placeholder rows appear to have used two DIFFERENT bandwidth
/// conventions. Noted here as a finding for whoever owns `STAGE0_PLACEHOLDER`
/// (Sr. Mechanical & Systems); **not corrected there** -- that profile backs
/// the existing ADR-0011 acceptance history and changing its numbers is a
/// re-baselining decision this derivation does not make unilaterally.
///
/// BIAS: this struct's `gyro_bias_rad_s` is a FIXED per-run offset (added
/// identically to every sample -- see `gyro_vec` above), which is the right
/// model for a CALIBRATION RESIDUAL, not for true Allan-variance "bias
/// instability" (a slow in-run WANDER of the bias, usually quoted deg/hr).
/// The ICM-42688-P's main datasheet table does not headline an Allan-variance
/// figure (typical of this consumer-MEMS class -- tactical-grade parts like
/// the ADIS16467 do, this part does not); the closest published number is the
/// gyro's initial, un-calibrated zero-rate offset, on the order of +/-0.5
/// deg/s turn-on-to-turn-on. Used here AS the fixed bias, uncalibrated
/// (conservative -- no startup bias trim assumed):
///
/// ```text
/// gyro_bias_rad_s = 0.5 * (pi/180) = 8.727e-3 rad/s
/// ```
///
/// **Bias instability proper is NOT modelled** -- there is no time-varying
/// bias term anywhere in this chain (Rust or Python); see the omissions list
/// below. Class-typical Allan-variance figures for this tier of MEMS gyro run
/// a few deg/hr, which is negligible against the fixed offset above at any
/// run length this sim exercises (tens of seconds), so its absence is a
/// stated gap, not a load-bearing one.
///
/// QUANTISATION (16-bit ADC, both channels -- present because the task asks
/// for it if the datasheet gives a resolution, NOT because it changes
/// anything here):
///
/// ```text
/// Gyro,  FSR +/-2000 deg/s: LSB = 4000/65536 = 0.06104 deg/s
///        sigma_q = LSB/sqrt(12) = 0.01762 deg/s = 3.075e-4 rad/s
/// Accel, FSR +/-16 g:       LSB = 32/65536 g = 4.883e-4 g
///        sigma_q = LSB/sqrt(12) * 9.80665 = 1.383e-3 m/s^2
/// ```
///
/// Both are sub-dominant to the noise-density sigma above by 5x (gyro) and
/// 16x (accel) at these FSRs, so quantisation is NOT separately wired into
/// `gyro_vec`/`accel_vec` -- folding a term this much smaller into the
/// existing Gaussian draw would not be a distinguishable behaviour change.
/// Revisit if a lower-FSR configuration or a different part ever makes it
/// non-negligible.
///
/// NOT MODELLED, stated rather than left silent: vibration (motor/wheel
/// mechanical coupling into the IMU -- `docs/sim-fidelity-session-prep.md`
/// names this explicitly), temperature drift (offset AND noise-density
/// temperature coefficients), misalignment (an unknown/uncalibrated mounting
/// axis error -- distinct from `RunMetadata::imu_mounting_rotation`, which is
/// a KNOWN, deliberately-applied rotation, not an error term), scale-factor
/// error, cross-axis sensitivity, and bias instability as a time-varying
/// process (see above).
///
/// `seed` below is a placeholder like every other profile's; the CLI knob
/// that wires this in (`sim-host --imu-noise-seed`, `HostConfig::
/// imu_noise_seed`) always overrides it, because a noise stream nobody else
/// can re-seed is not a measurement.
pub const IMU_NOISE_DATASHEET_V1: ImperfectionProfile = ImperfectionProfile {
    profile_id: "imu-noise-datasheet-v1",
    actuation_delay_s: 0.0,
    current_loop_tau_s: 0.0,
    gyro_noise_rad_s: 1.546e-3,
    gyro_bias_rad_s: 8.727e-3,
    accel_noise_m_s2: 2.171e-2,
    wheel_rate_quantum_rad_s: 0.0,
    wheel_rate_update_hz: 0.0,
    max_current_a: 60.0,
    derate_onset_rad_s: 0.0,
    derate_full_rad_s: 0.0,
    derate_floor_frac: 1.0,
    seed: 12345,
};

/// A tiny, dependency-free, seeded PRNG (SplitMix64) feeding a Box-Muller
/// transform for Gaussian noise.
///
/// Deliberately not NumPy's PCG64 -- `sim/scenarios/imperfections.py`'s own
/// conformance-contract section says as much: reproducing that stream
/// bitwise would mean reimplementing both generator and algorithm for the
/// project's *least* consequential row. What this must provide, and does,
/// is: reproducible under a fixed seed, and distributionally correct.
#[derive(Debug, Clone, Copy)]
struct SplitMix64 {
    state: u64,
}

impl SplitMix64 {
    fn new(seed: u64) -> Self {
        SplitMix64 { state: seed }
    }

    fn next_u64(&mut self) -> u64 {
        self.state = self.state.wrapping_add(0x9E3779B97F4A7C15);
        let mut z = self.state;
        z = (z ^ (z >> 30)).wrapping_mul(0xBF58476D1CE4E5B9);
        z = (z ^ (z >> 27)).wrapping_mul(0x94D049BB133111EB);
        z ^ (z >> 31)
    }

    /// Uniform in (0, 1] -- never exactly 0, so `ln()` below is always
    /// defined.
    fn next_open01(&mut self) -> f64 {
        // Top 53 bits -> a double with full mantissa precision, then shifted
        // off zero rather than clamped, so the distribution stays uniform.
        let bits = self.next_u64() >> 11;
        ((bits as f64) + 1.0) / ((1u64 << 53) as f64 + 1.0)
    }

    /// One standard-normal sample via Box-Muller. Not caching the paired
    /// sample: at most a handful of draws per control cycle, so the constant
    /// factor of two is not worth the extra state.
    fn next_gaussian(&mut self) -> f64 {
        let u1 = self.next_open01();
        let u2 = self.next_open01();
        (-2.0 * u1.ln()).sqrt() * (std::f64::consts::TAU * u2).cos()
    }
}

/// Per-run mutable state for a profile. One per backend/run — mirrors
/// `sim.scenarios.imperfections.ImperfectionState`.
#[derive(Debug)]
pub struct ImperfectionState {
    profile: ImperfectionProfile,
    dt_s: f64,
    rng: SplitMix64,
    cmd_history: Vec<f64>,
    applied_a: f64,
    held_wheel_rate: f64,
    last_wheel_update_s: f64,
}

impl ImperfectionState {
    pub fn new(profile: ImperfectionProfile, dt_s: f64) -> Self {
        ImperfectionState {
            profile,
            dt_s,
            rng: SplitMix64::new(profile.seed),
            cmd_history: Vec::new(),
            applied_a: 0.0,
            held_wheel_rate: 0.0,
            // Mirrors Python's `-1e9`: far enough in the past that the very
            // first `wheel_rate()` call always refreshes.
            last_wheel_update_s: -1.0e9,
        }
    }

    pub fn profile(&self) -> &ImperfectionProfile {
        &self.profile
    }

    // -- sensing -------------------------------------------------------

    /// Pitch-rate vector as the gyro reports it. Bias lands on `[1]`
    /// unconditionally (nose-up rate), mirroring `ImperfectionState.gyro_vec`
    /// -- noised even when the caller only reads one axis, so the other two
    /// are not suspiciously perfect next to it.
    pub fn gyro_vec(&mut self, true_gyro_rad_s: [f64; 3]) -> [f64; 3] {
        let p = self.profile;
        let mut out = true_gyro_rad_s;
        if p.gyro_noise_rad_s > 0.0 {
            for v in out.iter_mut() {
                *v += self.rng.next_gaussian() * p.gyro_noise_rad_s;
            }
        }
        out[1] += p.gyro_bias_rad_s;
        out
    }

    pub fn accel_vec(&mut self, true_accel_m_s2: [f64; 3]) -> [f64; 3] {
        let p = self.profile;
        let mut out = true_accel_m_s2;
        if p.accel_noise_m_s2 > 0.0 {
            for v in out.iter_mut() {
                *v += self.rng.next_gaussian() * p.accel_noise_m_s2;
            }
        }
        out
    }

    /// Wheel rate as the drive reports it: quantised, and held between
    /// updates rather than interpolated. Mirrors `ImperfectionState.wheel_rate`
    /// exactly, including the `>=` refresh test -- see that method's own
    /// docstring for why `>=` (not `>`) and assigning `t_s` (not
    /// accumulating `period`) are both load-bearing, not stylistic.
    pub fn wheel_rate(&mut self, true_rate_rad_s: f64, t_s: f64) -> f64 {
        let p = self.profile;
        if p.wheel_rate_update_hz <= 0.0 {
            return Self::quantise(p.wheel_rate_quantum_rad_s, true_rate_rad_s);
        }
        if t_s - self.last_wheel_update_s >= 1.0 / p.wheel_rate_update_hz {
            self.held_wheel_rate = Self::quantise(p.wheel_rate_quantum_rad_s, true_rate_rad_s);
            self.last_wheel_update_s = t_s;
        }
        self.held_wheel_rate
    }

    /// Round-half-to-even, matching NumPy's `np.round` -- NOT Rust's
    /// `f64::round`, which is round-half-away-from-zero and disagrees at
    /// exactly the half-quantum values a quantiser hits constantly. See this
    /// crate's conformance test and `sim/scenarios/imperfections.py`'s own
    /// header for why that divergence is the single most likely silent
    /// failure mode here.
    fn quantise(quantum: f64, v: f64) -> f64 {
        if quantum <= 0.0 {
            v
        } else {
            (v / quantum).round_ties_even() * quantum
        }
    }

    // -- actuation -------------------------------------------------------

    /// Current the plant actually sees, after cutback, transport delay and
    /// the current loop's own dynamics. Mirrors
    /// `ImperfectionState.apply_current` exactly, including its chain order
    /// (cutback -> saturate -> transport delay -> current-loop lag) and its
    /// fractional, interpolated delay -- see that method's docstring for why
    /// rounding to whole cycles is not an acceptable simplification.
    ///
    /// `wheel_rate_rad_s` is required when the profile models cutback;
    /// omitting it panics rather than silently applying no cutback, mirroring
    /// the Python method's own `ValueError` for the same reason it gives:
    /// this project has twice shipped a wrong result from an optional
    /// argument silently defaulting away real behaviour.
    pub fn apply_current(&mut self, commanded_a: f64, wheel_rate_rad_s: Option<f64>) -> f64 {
        let p = self.profile;
        let cap = if p.models_cutback() {
            let w = wheel_rate_rad_s.unwrap_or_else(|| {
                panic!(
                    "profile {:?} models cutback, so apply_current() needs wheel_rate_rad_s -- \
                     refusing to silently apply no cutback",
                    p.profile_id
                )
            });
            p.available_current_a(w)
        } else {
            p.max_current_a
        };
        let commanded_a = commanded_a.clamp(-cap, cap);

        let cycles = (p.actuation_delay_s / self.dt_s).max(0.0);
        let whole = cycles as usize;
        let frac = cycles - whole as f64;

        self.cmd_history.push(commanded_a);
        while self.cmd_history.len() > whole + 2 {
            self.cmd_history.remove(0);
        }

        let sample = |history: &[f64], back: usize| -> f64 {
            let len = history.len();
            if back < len {
                history[len - 1 - back]
            } else {
                0.0
            }
        };
        let delayed = (1.0 - frac) * sample(&self.cmd_history, whole)
            + frac * sample(&self.cmd_history, whole + 1);

        if p.current_loop_tau_s <= 0.0 {
            self.applied_a = delayed;
        } else {
            let alpha = self.dt_s / (p.current_loop_tau_s + self.dt_s);
            self.applied_a += alpha * (delayed - self.applied_a);
        }
        self.applied_a
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn ideal_is_ideal() {
        assert!(IDEAL.is_ideal());
    }

    #[test]
    fn stage0_profiles_are_not_ideal() {
        assert!(!STAGE0_PLACEHOLDER.is_ideal());
        assert!(!STAGE0_CUTBACK.is_ideal());
    }

    #[test]
    fn only_cutback_profile_models_cutback() {
        assert!(!STAGE0_PLACEHOLDER.models_cutback());
        assert!(STAGE0_CUTBACK.models_cutback());
    }

    #[test]
    fn imu_noise_datasheet_profile_is_not_ideal() {
        assert!(!IMU_NOISE_DATASHEET_V1.is_ideal());
    }

    #[test]
    fn imu_noise_datasheet_profile_models_nothing_but_the_sensor_chain() {
        // The whole point of this profile: isolate sensor noise from every
        // other imperfection row this module models. Any nonzero value here
        // would silently mix a delay/lag/cutback/quantisation cost into a
        // measurement meant to be about the IMU alone.
        let p = IMU_NOISE_DATASHEET_V1;
        assert_eq!(p.actuation_delay_s, 0.0);
        assert_eq!(p.current_loop_tau_s, 0.0);
        assert_eq!(p.wheel_rate_quantum_rad_s, 0.0);
        assert!(!p.models_cutback());
        assert!(p.gyro_noise_rad_s > 0.0);
        assert!(p.gyro_bias_rad_s > 0.0);
        assert!(p.accel_noise_m_s2 > 0.0);
    }

    #[test]
    fn zero_delay_zero_lag_is_a_same_cycle_passthrough() {
        // The property SimBackend's wiring depends on: with IDEAL, the
        // profile-level chain must be a no-op so the existing structural
        // one-cycle delay is the only delay a caller observes.
        let mut st = ImperfectionState::new(IDEAL, 0.002);
        assert_eq!(st.apply_current(5.0, Some(0.0)), 5.0);
        assert_eq!(st.apply_current(-3.0, Some(0.0)), -3.0);
    }

    #[test]
    fn cutback_without_wheel_rate_panics() {
        let mut st = ImperfectionState::new(STAGE0_CUTBACK, 0.002);
        let result =
            std::panic::catch_unwind(std::panic::AssertUnwindSafe(|| st.apply_current(5.0, None)));
        assert!(result.is_err());
    }

    #[test]
    fn quantise_is_round_half_to_even() {
        let q = 1.0;
        assert_eq!(ImperfectionState::quantise(q, 0.5), 0.0);
        assert_eq!(ImperfectionState::quantise(q, 1.5), 2.0);
        assert_eq!(ImperfectionState::quantise(q, 2.5), 2.0);
        assert_eq!(ImperfectionState::quantise(q, -0.5), 0.0);
    }

    #[test]
    fn same_seed_reproduces_the_same_noise_stream() {
        let mut a = ImperfectionState::new(STAGE0_PLACEHOLDER, 0.002);
        let mut b = ImperfectionState::new(STAGE0_PLACEHOLDER, 0.002);
        for _ in 0..20 {
            assert_eq!(
                a.gyro_vec([0.0, 0.0, 0.0]),
                b.gyro_vec([0.0, 0.0, 0.0]),
                "same seed must reproduce the same stream"
            );
        }
    }

    #[test]
    fn gyro_noise_is_distributionally_correct() {
        let mut st = ImperfectionState::new(STAGE0_PLACEHOLDER, 0.002);
        let n = 20_000;
        let mut sum = 0.0;
        let mut sum_sq = 0.0;
        for _ in 0..n {
            let v = st.gyro_vec([0.0, 0.0, 0.0])[1] - STAGE0_PLACEHOLDER.gyro_bias_rad_s;
            sum += v;
            sum_sq += v * v;
        }
        let mean = sum / n as f64;
        let var = sum_sq / n as f64 - mean * mean;
        let sigma = var.sqrt();
        let want = STAGE0_PLACEHOLDER.gyro_noise_rad_s;
        assert!(
            (sigma - want).abs() / want < 0.05,
            "measured sigma {sigma} not within 5% of profile sigma {want}"
        );
        assert!(mean.abs() < want * 0.05, "mean {mean} not close to zero");
    }
}
