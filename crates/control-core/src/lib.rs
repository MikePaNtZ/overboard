//! Pure, deterministic control logic. No I/O, no clock reads — `dt` is derived
//! from the timestamps carried on [`Observation`] (DR-CTRL-1, ICD §5.2).
//!
//! Depends on `board-types` only (not `hal`), so it can be unit-tested and
//! fuzzed with no backend in the loop.
#![no_std]

use board_types::{Command, ImuSample, Observation, Params};

/// A pitch estimate.
#[derive(Debug, Clone, Copy, PartialEq, Default)]
pub struct Attitude {
    /// Nose-up positive, radians (ICD §10.1).
    pub pitch_rad: f32,
    pub pitch_rate_rad_s: f32,
}

/// Turns raw inertial samples into an attitude.
///
/// Behind a trait so the ESKF can replace the complementary filter for the
/// free-board phase without touching the control law, and so the acceptance
/// criteria are stated on the **interface** — a pitch-error bound — rather than
/// on any particular algorithm. SR-CTRL-4 was amended for exactly this reason:
/// a requirement that names an algorithm cannot be met by a better one.
pub trait Estimator {
    /// Consume **every** sample since the last cycle, oldest first (DR-CTRL-2).
    ///
    /// Sub-sampling a ≥1 kHz IMU at a 500 Hz loop aliases the estimator's
    /// primary input, so taking only the newest is a defect rather than an
    /// optimisation.
    /// `forward_accel_m_s2` is the board's known longitudinal acceleration,
    /// from wheel odometry. Pass 0.0 for no aiding.
    ///
    /// This is what makes the accelerometer usable on a vehicle that
    /// accelerates: it measures **specific force**, so `f_x = a + g·sinθ`, and
    /// subtracting a known `a` recovers the gravity term. Without it the
    /// implied tilt is wrong by `atan(a/g)` exactly when the board is working
    /// hardest — about 5° at the outer loop's acceleration limit.
    fn update(&mut self, samples: &[ImuSample], forward_accel_m_s2: f32) -> Attitude;
    fn reset(&mut self);
}

/// Complementary filter: high-pass the gyro, low-pass the accelerometer.
///
/// ```text
/// θ̂ = α(θ̂ + ω·dt) + (1−α)·θ_accel,    α = τ/(τ + dt)
/// ```
///
/// # Why the accelerometer is only trusted slowly
///
/// It measures **specific force**, so under longitudinal acceleration `a` the
/// tilt it implies is wrong by `atan(a/g)`. The outer loop's 5° reference clamp
/// permits about 0.86 m/s², which is roughly **5° of apparent tilt error** —
/// the same size as the excursion being regulated, and *correlated with it*
/// rather than random. So the accelerometer is believed only about which way is
/// down on average over seconds; everything faster comes from the gyro.
///
/// # Choosing τ
///
/// Gyro bias `b` leaves a steady-state error of `b·τ`, so a long τ trades
/// bias error for acceleration rejection. At the ICD §12 placeholder bias of
/// 0.002 rad/s: τ = 1 s costs 0.11°, τ = 5 s costs 0.57° and would blow the
/// 0.5° RMS budget on bias alone.
///
/// **No bias estimator in v1** — at 0.11° against a 0.5° budget it would be
/// solving a problem that is not binding, while adding an integrator that
/// interacts with the accel corruption in ways worth measuring separately.
///
/// # Rejecting the accelerometer when it is lying
///
/// Aiding cancels the acceleration the board *commanded*. It cannot cancel
/// acceleration nobody commanded — an impulse, a kerb, a shove. On the nominal
/// 400 N·s disturbance that is 4.9 m/s², or **26° of apparent tilt**, and the
/// filter dutifully believes a quarter of it. Measured: peak pitch went from
/// 4.6° with truth to ~19.5° with the estimator, at every τ from 0.5 to 5 s,
/// which is the signature of a disturbance the filter cannot see rather than a
/// tuning problem.
///
/// So gate it. After aiding, the residual specific force should be gravity and
/// nothing else; if `‖a − a_fwd‖` is not within `accel_trust_band_m_s2` of g,
/// something unmodelled is happening and the correction is skipped for that
/// sample — the estimate coasts on the gyro until the world makes sense again.
/// This is the standard trick and it is cheap, but it is not free: while
/// coasting there is no attitude reference at all, so the band must be wide
/// enough that ordinary riding does not trip it. Zero disables the gate.
#[derive(Debug, Clone, Copy)]
pub struct ComplementaryFilter {
    tau_s: f32,
    accel_trust_band_m_s2: f32,
    pitch_rad: f32,
    pitch_rate_rad_s: f32,
    last_t_ns: Option<u64>,
    initialised: bool,
    /// Samples whose accelerometer update was skipped by the gate. Reported so
    /// a run that coasted most of the way is distinguishable from one that
    /// never tripped -- both look identical in the attitude trace alone.
    rejected: u32,
}

impl ComplementaryFilter {
    pub const fn new(tau_s: f32) -> Self {
        Self::with_trust_band(tau_s, 0.0)
    }

    pub const fn with_trust_band(tau_s: f32, accel_trust_band_m_s2: f32) -> Self {
        ComplementaryFilter {
            tau_s,
            accel_trust_band_m_s2,
            pitch_rad: 0.0,
            pitch_rate_rad_s: 0.0,
            last_t_ns: None,
            initialised: false,
            rejected: 0,
        }
    }

    /// How many accelerometer updates the trust gate has rejected.
    pub fn rejected_samples(&self) -> u32 {
        self.rejected
    }

    /// Is this sample's specific force consistent with gravity plus the
    /// acceleration we already know about?
    fn accel_trusted(&self, accel: [f32; 3], forward_accel_m_s2: f32) -> bool {
        if self.accel_trust_band_m_s2 <= 0.0 {
            return true;
        }
        let (x, y, z) = (accel[0] - forward_accel_m_s2, accel[1], accel[2]);
        let mag = libm::sqrtf(x * x + y * y + z * z);
        libm::fabsf(mag - 9.81) <= self.accel_trust_band_m_s2
    }

    /// Tilt implied by the accelerometer, with the known linear acceleration
    /// removed.
    ///
    /// Derivation, not assumption: with θ nose-up about +Y, gravity in the body
    /// frame is `(g·sinθ, 0, −g·cosθ)`. An accelerometer measures SPECIFIC
    /// FORCE, so under forward acceleration `a` the x channel reads
    /// `a + g·sinθ`. Subtracting `a` recovers the gravity term:
    ///
    /// ```text
    /// θ = atan2(f_x − a, −f_z)
    /// ```
    ///
    /// Checked against the sim rather than trusted — the model's body frame is
    /// z-up while the ICD's is z-down, and a derivation that looked right had
    /// the x sign backwards.
    fn accel_pitch(accel: [f32; 3], forward_accel_m_s2: f32) -> f32 {
        libm::atan2f(accel[0] - forward_accel_m_s2, -accel[2])
    }
}

impl Estimator for ComplementaryFilter {
    fn update(&mut self, samples: &[ImuSample], forward_accel_m_s2: f32) -> Attitude {
        for s in samples {
            let theta_accel = Self::accel_pitch(s.accel_m_s2, forward_accel_m_s2);

            // First sample: snap to the accelerometer rather than starting at
            // zero. Starting at zero and filtering in would take ~τ to become
            // correct, and the controller would be acting on a known-wrong
            // attitude for the whole of it.
            if !self.initialised {
                self.pitch_rad = theta_accel;
                self.initialised = true;
                self.last_t_ns = Some(s.t_sample_ns);
                self.pitch_rate_rad_s = s.gyro_rad_s[1];
                continue;
            }

            // dt from the sample's own timestamp, never assumed (ICD §5.2).
            let dt = match self.last_t_ns {
                Some(prev) => s.t_sample_ns.saturating_sub(prev) as f32 * 1e-9,
                None => 0.0,
            };
            self.last_t_ns = Some(s.t_sample_ns);

            // gyro[1] IS the nose-up pitch rate (ICD §10.1) — no flip.
            self.pitch_rate_rad_s = s.gyro_rad_s[1];

            if dt <= 0.0 {
                continue;
            }
            let predicted = self.pitch_rad + self.pitch_rate_rad_s * dt;
            if self.accel_trusted(s.accel_m_s2, forward_accel_m_s2) {
                let alpha = self.tau_s / (self.tau_s + dt);
                self.pitch_rad = alpha * predicted + (1.0 - alpha) * theta_accel;
            } else {
                // Dead reckoning on the gyro alone. Equivalent to alpha = 1 for
                // this sample, so the estimate keeps moving -- it just stops
                // being corrected towards a "down" it has no reason to believe.
                self.pitch_rad = predicted;
                self.rejected = self.rejected.saturating_add(1);
            }
        }

        Attitude {
            pitch_rad: self.pitch_rad,
            pitch_rate_rad_s: self.pitch_rate_rad_s,
        }
    }

    fn reset(&mut self) {
        self.pitch_rad = 0.0;
        self.pitch_rate_rad_s = 0.0;
        self.last_t_ns = None;
        self.initialised = false;
        self.rejected = 0;
    }
}

/// Inner-loop pitch regulator — the balance law itself.
///
/// `amps = −(kp·θ + kd·θ̇)` with θ **nose-up-positive in radians** (ICD §10.1).
/// The minus sign is the ICD's own `current ≈ −K·pitch`, K > 0, and it is the
/// stabilising sense: a nose-down excursion is negative pitch, correcting it
/// means driving the contact patch forward, which is positive current.
///
/// Deliberately takes an already-estimated pitch rather than an [`Observation`].
/// Fusing raw IMU into an attitude is a separate concern with its own interface
/// and its own error budget; keeping them apart means the regulator can be
/// tuned against truth first and the estimator's contribution measured
/// separately afterwards, instead of debugging both at once.
///
/// **No integrator yet.** The plant has a real restoring term, so steady-state
/// error may be small enough that an integrator only adds windup risk. That is
/// an open question to settle with data, not on a whiteboard — and adding one
/// requires the anti-windup path (ICD §7.6) to be wired to `Saturation` first.
#[derive(Debug, Clone, Copy, Default)]
pub struct PitchRegulator {
    kp_a_per_rad: f32,
    kd_a_per_rad_s: f32,
}

impl PitchRegulator {
    pub const fn new(kp_a_per_rad: f32, kd_a_per_rad_s: f32) -> Self {
        PitchRegulator {
            kp_a_per_rad,
            kd_a_per_rad_s,
        }
    }

    /// Requested current in amps, before any clamping.
    ///
    /// `pitch_ref_rad` is the attitude to hold — zero for pure disturbance
    /// rejection, or the outer loop's output when a [`VelocityLoop`] is
    /// cascaded on top.
    ///
    /// Unclamped on purpose: bounding the command is the safety envelope's job
    /// (stage 2, ICD §7.6), and a controller that silently clamps its own
    /// output hides saturation from the anti-windup that needs to see it.
    pub fn update(&self, pitch_rad: f32, pitch_rate_rad_s: f32, pitch_ref_rad: f32) -> f32 {
        -(self.kp_a_per_rad * (pitch_rad - pitch_ref_rad) + self.kd_a_per_rad_s * pitch_rate_rad_s)
    }
}

/// Forward acceleration from wheel speed, for aiding the estimator.
///
/// A plain difference of the reported speed is unusable: ERPM arrives
/// quantised (1 ERPM ≈ 7 mrad/s) and held at a finite rate, so differencing it
/// produces a staircase of spikes rather than an acceleration. This
/// differentiates through a first-order low pass, which is the standard
/// dirty-derivative and costs phase in exchange for being finite.
///
/// **That phase cost was measured, and it is not affordable.** An earlier
/// version of this comment argued the opposite — that the aiding term only has
/// to be right on the timescale the accelerometer is believed on, around 1 s,
/// and not at the control loop's bandwidth. The reasoning was that the filter
/// low-passes the accelerometer branch anyway, so errors above the filter's
/// crossover get attenuated before they reach the control law.
///
/// It is wrong because the low-pass is `1/(τs+1)`, not a brick wall: it leaves
/// roughly `1/(ωτ)` of the accelerometer's error at every frequency above
/// crossover, and the accelerometer's error is not small. Under a commanded
/// velocity change the residual acceleration this fails to cancel dominates
/// the gravity signal, and what reaches the P term at the loop's ~12 rad/s
/// crossover is an amplified, phase-shifted version of pitch — measured at
/// **+6.4 dB and −73°**, which is enough to destabilise a loop that tolerates
/// 40 ms of pure delay. See `scripts/analyse_estimator_phase.py` and
/// `notebooks/03-attitude-estimation.ipynb`.
///
/// [`CommandFeedforward`] exists because of that measurement.
#[derive(Debug, Clone, Copy)]
pub struct WheelAccelEstimator {
    tau_s: f32,
    last_v: Option<f32>,
    accel: f32,
}

impl WheelAccelEstimator {
    pub const fn new(tau_s: f32) -> Self {
        WheelAccelEstimator {
            tau_s,
            last_v: None,
            accel: 0.0,
        }
    }

    /// Filtered `dv/dt`, m/s². `dt_s <= 0` holds the previous value.
    pub fn update(&mut self, v_m_s: f32, dt_s: f32) -> f32 {
        let prev = match self.last_v {
            Some(p) => p,
            None => {
                self.last_v = Some(v_m_s);
                return 0.0;
            }
        };
        self.last_v = Some(v_m_s);
        if dt_s <= 0.0 {
            return self.accel;
        }
        let raw = (v_m_s - prev) / dt_s;
        let alpha = dt_s / (self.tau_s + dt_s);
        self.accel += alpha * (raw - self.accel);
        self.accel
    }

    pub fn reset(&mut self) {
        self.last_v = None;
        self.accel = 0.0;
    }
}

/// Forward acceleration predicted from the current we just commanded.
///
/// The wheel-odometry route above has to *measure* an acceleration by
/// differentiating a quantised, slowly-updated speed. This one does not
/// measure anything: it asserts that the current we asked for produced the
/// acceleration Newton says it should,
///
/// ```text
///     a ≈ (k_t · I) / (r_eff · m)
/// ```
///
/// collapsed into a single fitted constant, `gain_m_s2_per_a`. That is one
/// number to measure on the bench — command a current on the flat, read the
/// accelerometer, divide — rather than three to estimate separately.
///
/// **The trade is measurement noise for model error.** It has no lag and no
/// quantisation, which is the whole point: the wheel-derived estimate is
/// wrong exactly where the control loop lives, and that error lands straight
/// on the accelerometer branch of the attitude filter. What it cannot see is
/// anything that breaks the assumed relationship — a slope, a kerb, wheel
/// slip, a rider shifting their weight. Those all appear as real acceleration
/// the feedforward denies is happening.
///
/// So it is not strictly better, and the choice is left to the caller. On the
/// bench, where the ground is flat and the mass is bolted down, the model
/// error is small and this wins comfortably. Outdoors on a hill it would not,
/// and blending the two against a slope estimate is the obvious next step.
#[derive(Debug, Clone, Copy)]
pub struct CommandFeedforward {
    gain_m_s2_per_a: f32,
}

impl CommandFeedforward {
    pub const fn new(gain_m_s2_per_a: f32) -> Self {
        CommandFeedforward { gain_m_s2_per_a }
    }

    /// Predicted forward acceleration, m/s², from the last commanded current.
    ///
    /// Takes the *commanded* current rather than a measured one so it stays
    /// usable on hardware that cannot report phase current, and so it carries
    /// no sensing delay. Where measured current is available it is the better
    /// input and can be passed here instead — the arithmetic is the same.
    pub fn predict(&self, commanded_a: f32) -> f32 {
        self.gain_m_s2_per_a * commanded_a
    }
}

/// Which way the plant's pitch-to-velocity coupling runs.
///
/// **This is not a preference, it is a property of the vehicle**, and it
/// inverts depending on whether the centre of mass sits above or below the
/// axle. Measured on both plants rather than assumed:
///
/// | plant | CoM vs axle | commanded nose-down | result |
/// |---|---|---|---|
/// | driverless board | 19 mm below | −3° | travelled **backward** |
/// | with a 70 kg rider | 633 mm above | −1° | travelled **forward** |
///
/// With the CoM above the axle you tilt forward and gravity drives you
/// forward — ordinary onewheel behaviour. With it below, holding a nose-up
/// attitude requires continuously accelerating the wheel forward, so the
/// correlation flips.
///
/// An outer loop tuned on the driverless board and moved to a ridden one
/// without flipping this is **positive feedback in velocity**, and the
/// driverless tests would have passed the whole way.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PlantCoupling {
    /// Centre of mass above the axle — the ridden vehicle. Nose-down ⇒ forward.
    ComAboveAxle,
    /// Centre of mass below the axle — the driverless board. Nose-up ⇒ forward.
    ComBelowAxle,
}

impl PlantCoupling {
    fn sign(self) -> f32 {
        match self {
            PlantCoupling::ComAboveAxle => 1.0,
            PlantCoupling::ComBelowAxle => -1.0,
        }
    }
}

/// Outer loop: turns a ground-speed error into a pitch reference for the
/// inner [`PitchRegulator`].
///
/// A pure inner loop holds attitude and lets the board ride away under any
/// sustained disturbance — correct behaviour, and the reason this exists.
/// Position and speed are regulated here, by *asking the inner loop to lean*,
/// which is the only actuator a onewheel has.
///
/// Deliberately slow relative to the inner loop. The two are not independent:
/// the outer loop's actuator is the inner loop's setpoint, so if their
/// bandwidths approach each other the inner loop is still chasing a reference
/// that has already moved, and the pair rings.
#[derive(Debug, Clone, Copy)]
pub struct VelocityLoop {
    kp_rad_per_m_s: f32,
    ki_rad_per_m: f32,
    max_pitch_ref_rad: f32,
    coupling: PlantCoupling,
    integral: f32,
}

impl VelocityLoop {
    pub const fn new(
        kp_rad_per_m_s: f32,
        ki_rad_per_m: f32,
        max_pitch_ref_rad: f32,
        coupling: PlantCoupling,
    ) -> Self {
        VelocityLoop {
            kp_rad_per_m_s,
            ki_rad_per_m,
            max_pitch_ref_rad,
            coupling,
            integral: 0.0,
        }
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
    }

    pub fn integral(&self) -> f32 {
        self.integral
    }

    /// Pitch reference in radians, nose-up-positive, clamped to
    /// `max_pitch_ref_rad`.
    ///
    /// `inner_saturated` is the inner loop's current clamp, forwarded from
    /// `Applied.saturated` (ICD §7.6). When the inner loop is already at its
    /// limit, leaning further asks for authority that does not exist, so the
    /// integrator must not keep winding — that is how a bounded disturbance
    /// turns into an unbounded reference.
    pub fn update(&mut self, v_m_s: f32, v_ref_m_s: f32, dt_s: f32, inner_saturated: bool) -> f32 {
        let err = v_m_s - v_ref_m_s;
        let limit = self.max_pitch_ref_rad.abs();
        let sign = self.coupling.sign();

        let proportional = sign * self.kp_rad_per_m_s * err;
        let candidate = proportional + sign * self.ki_rad_per_m * self.integral;

        // Conditional integration. Wind only when the output is genuinely free
        // to move, or when the error is pushing back off the limit -- so a
        // clamp cannot silently accumulate a reference it will never honour.
        let clamped = candidate.abs() > limit;
        let pushing_further = clamped && (candidate.is_sign_positive() == err.is_sign_positive());
        if dt_s > 0.0 && !inner_saturated && !pushing_further {
            self.integral += err * dt_s;
        }

        let out = proportional + sign * self.ki_rad_per_m * self.integral;
        out.clamp(-limit, limit)
    }
}

/// Cascaded balance controller.
///
/// **Stub.** Returns [`Command::ZERO`] regardless of input; the sim checkpoint
/// therefore still shows an *uncontrolled* board. The real law lands with the
/// stand phase — an inner pitch regulator at zero setpoint, `current ≈ −K·θ`
/// with θ nose-up-positive (ICD §10.2), plus anti-windup driven by the
/// `Saturation` the envelope reports.
///
/// It deliberately does **not** guess at a pitch estimate yet: the estimator is
/// a separate increment behind its own interface, and wiring a throwaway one in
/// here would put an unvalidated fusion step on the control path.
#[derive(Debug, Default)]
pub struct Controller {
    params: Params,
    last_t_sample_ns: Option<u64>,
}

impl Controller {
    pub fn new(params: Params) -> Self {
        Controller {
            params,
            last_t_sample_ns: None,
        }
    }

    pub fn params(&self) -> &Params {
        &self.params
    }

    /// Seconds since the previous observation, from the **newest IMU sample's**
    /// timestamp.
    ///
    /// `None` on the first cycle, when there is no previous sample to
    /// difference against, and on any cycle carrying no IMU data at all. Never
    /// assumed to be `1/rate`: a missed instant makes the real `dt` a multiple
    /// of the nominal period, and on a balancer a `dt` that lies is a phase
    /// error — indistinguishable from negative damping.
    fn dt_s(&mut self, obs: &Observation) -> Option<f32> {
        let newest = obs.newest_imu()?.t_sample_ns;
        // Saturating: a backend handing back a non-monotonic timestamp is
        // buggy, but it must not be able to produce a negative or wrapped dt
        // that silently inverts a derivative term.
        let dt = self
            .last_t_sample_ns
            .map(|prev| newest.saturating_sub(prev) as f32 * 1e-9);
        self.last_t_sample_ns = Some(newest);
        dt
    }

    /// Compute the next command from the latest observation.
    pub fn update(&mut self, obs: &Observation) -> Command {
        let _dt = self.dt_s(obs);
        Command::ZERO
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use board_types::{ImuSample, ValidityFlags};

    fn obs_at(t_sample_ns: u64) -> Observation {
        let mut obs = Observation::COLD_START;
        obs.imu[0] = ImuSample {
            gyro_rad_s: [0.0, 0.1, 0.0],
            accel_m_s2: [0.0, 0.0, -9.81],
            t_sample_ns,
        };
        obs.imu_count = 1;
        obs.validity = ValidityFlags::ALL_FRESH;
        obs
    }

    #[test]
    fn stub_controller_always_returns_zero() {
        let mut ctrl = Controller::new(Params::default());
        assert_eq!(ctrl.update(&obs_at(1_000)), Command::ZERO);
    }

    #[test]
    fn dt_is_none_on_the_first_cycle() {
        let mut ctrl = Controller::new(Params::default());
        assert_eq!(ctrl.dt_s(&obs_at(1_000_000)), None);
    }

    #[test]
    fn dt_is_the_difference_of_newest_imu_timestamps() {
        let mut ctrl = Controller::new(Params::default());
        ctrl.dt_s(&obs_at(1_000_000));
        let dt = ctrl.dt_s(&obs_at(3_000_000)).expect("second cycle has dt");
        assert!((dt - 0.002).abs() < 1e-9, "expected 2 ms, got {dt}");
    }

    #[test]
    fn a_missed_cycle_widens_dt_rather_than_reporting_the_nominal_period() {
        // The whole reason dt is derived instead of assumed. A controller using
        // 1/rate here would under-integrate by exactly the gap.
        let mut ctrl = Controller::new(Params::default());
        ctrl.dt_s(&obs_at(1_000_000));
        let dt = ctrl.dt_s(&obs_at(7_000_000)).expect("dt");
        assert!((dt - 0.006).abs() < 1e-9, "expected 6 ms, got {dt}");
    }

    #[test]
    fn dt_uses_the_newest_sample_of_a_batch_not_the_oldest() {
        let mut ctrl = Controller::new(Params::default());
        ctrl.dt_s(&obs_at(1_000_000));

        let mut batch = Observation::COLD_START;
        for (i, s) in batch.imu.iter_mut().enumerate().take(4) {
            s.t_sample_ns = 1_500_000 + (i as u64) * 500_000; // 1.5 .. 3.0 ms
        }
        batch.imu_count = 4;

        let dt = ctrl.dt_s(&batch).expect("dt");
        assert!(
            (dt - 0.002).abs() < 1e-9,
            "expected 2 ms to newest, got {dt}"
        );
    }

    #[test]
    fn a_non_monotonic_timestamp_cannot_produce_a_negative_dt() {
        let mut ctrl = Controller::new(Params::default());
        ctrl.dt_s(&obs_at(5_000_000));
        let dt = ctrl.dt_s(&obs_at(1_000_000)).expect("dt");
        assert_eq!(dt, 0.0);
    }

    #[test]
    fn an_observation_with_no_imu_samples_yields_no_dt() {
        let mut ctrl = Controller::new(Params::default());
        assert_eq!(ctrl.dt_s(&Observation::COLD_START), None);
    }

    // ---- PitchRegulator -------------------------------------------------
    //
    // These assert the SIGN and the shape, not tuned values. The gains that
    // matter are validated against the plant in the sim scenario; what must be
    // true here, independent of any plant, is that the law opposes the error.

    const KP: f32 = 80.0;
    const KD: f32 = 11.0;

    #[test]
    fn nose_down_commands_positive_current() {
        // Nose-down is NEGATIVE pitch (ICD 10.1). Correcting it means driving
        // the contact patch forward, i.e. POSITIVE current. If this ever
        // inverts, the board accelerates into its own nosedive.
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(-0.1, 0.0, 0.0) > 0.0);
    }

    #[test]
    fn nose_up_commands_negative_current() {
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(0.1, 0.0, 0.0) < 0.0);
    }

    #[test]
    fn the_rate_term_opposes_the_rate() {
        // At zero pitch error, a nose-up RATE must still be resisted -- that is
        // the damping, and without it the proportional term alone rings.
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(0.0, 0.5, 0.0) < 0.0);
        assert!(r.update(0.0, -0.5, 0.0) > 0.0);
    }

    #[test]
    fn level_and_still_commands_nothing() {
        let r = PitchRegulator::new(KP, KD);
        assert_eq!(r.update(0.0, 0.0, 0.0), 0.0);
    }

    #[test]
    fn the_law_is_odd_symmetric() {
        // Equal and opposite errors must produce equal and opposite commands.
        // An asymmetry here would mean the board recovers better one way than
        // the other, which is the kind of thing that only shows up in a hard
        // save in the wrong direction.
        let r = PitchRegulator::new(KP, KD);
        assert_eq!(r.update(0.07, 0.3, 0.0), -r.update(-0.07, -0.3, 0.0));
    }

    // ---- ComplementaryFilter --------------------------------------------

    const G: f32 = 9.81;

    fn sample(t_ns: u64, pitch: f32, rate: f32) -> ImuSample {
        // Gravity in the body frame at pitch theta: (g sin, 0, -g cos).
        ImuSample {
            gyro_rad_s: [0.0, rate, 0.0],
            accel_m_s2: [G * libm::sinf(pitch), 0.0, -G * libm::cosf(pitch)],
            t_sample_ns: t_ns,
        }
    }

    #[test]
    fn a_level_board_reads_level() {
        let mut f = ComplementaryFilter::new(1.0);
        let a = f.update(&[sample(0, 0.0, 0.0)], 0.0);
        assert!(a.pitch_rad.abs() < 1e-5);
    }

    #[test]
    fn the_accelerometer_sign_matches_the_icd_convention() {
        // Nose UP must read POSITIVE. Getting this backwards inverts the whole
        // balance law, which is the defect class the ICD calls most dangerous.
        let mut f = ComplementaryFilter::new(1.0);
        let up = f.update(&[sample(0, 0.15, 0.0)], 0.0);
        assert!(up.pitch_rad > 0.1, "nose-up read {}", up.pitch_rad);

        let mut g = ComplementaryFilter::new(1.0);
        let down = g.update(&[sample(0, -0.15, 0.0)], 0.0);
        assert!(down.pitch_rad < -0.1, "nose-down read {}", down.pitch_rad);
    }

    #[test]
    fn it_initialises_from_the_accelerometer_rather_than_from_zero() {
        // Starting at zero and filtering in would leave the controller acting
        // on a known-wrong attitude for about tau.
        let mut f = ComplementaryFilter::new(5.0);
        let a = f.update(&[sample(0, 0.20, 0.0)], 0.0);
        assert!(
            (a.pitch_rad - 0.20).abs() < 1e-3,
            "snapped to {}",
            a.pitch_rad
        );
    }

    #[test]
    fn the_gyro_is_passed_through_as_the_rate() {
        let mut f = ComplementaryFilter::new(1.0);
        f.update(&[sample(0, 0.0, 0.0)], 0.0);
        let a = f.update(&[sample(2_000_000, 0.0, 0.37)], 0.0);
        assert_eq!(a.pitch_rate_rad_s, 0.37);
    }

    #[test]
    fn it_tracks_a_steady_rotation_using_the_gyro() {
        // 0.5 rad/s for 1 s, with a consistent accelerometer. Should end near
        // 0.5 rad -- the gyro carries it, the accel confirms it.
        let mut f = ComplementaryFilter::new(1.0);
        let (dt_ns, dt) = (2_000_000u64, 0.002f32);
        let mut theta = 0.0f32;
        f.update(&[sample(0, theta, 0.5)], 0.0);
        for k in 1..=500 {
            theta += 0.5 * dt;
            f.update(&[sample(k * dt_ns, theta, 0.5)], 0.0);
        }
        let a = f.update(&[sample(501 * dt_ns, theta, 0.5)], 0.0);
        assert!(
            (a.pitch_rad - theta).abs() < 0.02,
            "got {} want {theta}",
            a.pitch_rad
        );
    }

    #[test]
    fn gyro_only_drift_is_pulled_out_by_the_accelerometer() {
        // A biased gyro on a stationary, level board. Integrating alone would
        // ramp without bound; the accelerometer must hold it near zero.
        let mut f = ComplementaryFilter::new(1.0);
        let bias = 0.05f32; // rad/s, deliberately large
        let dt_ns = 2_000_000u64;
        let mut last = 0.0;
        for k in 0..5_000 {
            let mut s = sample(k * dt_ns, 0.0, bias);
            s.gyro_rad_s[1] = bias;
            last = f.update(&[s], 0.0).pitch_rad;
        }
        // Steady state is bias*tau, NOT unbounded: 0.05 * 1.0 = 0.05 rad.
        assert!(last < 0.08, "drifted to {last}");
        assert!(
            last > 0.02,
            "expected the predicted bias*tau offset, got {last}"
        );
    }

    #[test]
    fn a_longer_tau_leaves_a_proportionally_larger_bias_error() {
        // The trade the design doc states, asserted rather than described.
        let bias = 0.05f32;
        let dt_ns = 2_000_000u64;
        let settle = |tau: f32| {
            let mut f = ComplementaryFilter::new(tau);
            let mut last = 0.0;
            for k in 0..20_000 {
                let mut s = sample(k * dt_ns, 0.0, bias);
                s.gyro_rad_s[1] = bias;
                last = f.update(&[s], 0.0).pitch_rad;
            }
            last
        };
        let (short, long) = (settle(0.5), settle(2.0));
        assert!(
            long > short * 2.0,
            "tau=2 gave {long}, tau=0.5 gave {short}"
        );
    }

    #[test]
    fn it_consumes_every_sample_in_a_batch() {
        // DR-CTRL-2, stated as the aliasing case it actually is.
        //
        // A CONSTANT rate would not catch this: dt comes from each sample's own
        // timestamp, so integrating one 2 ms step or four 0.5 ms steps gives
        // the same total, and a filter that dropped samples would still pass.
        // The batch matters when the rate VARIES inside the cycle -- here all
        // the motion happens in the first sub-sample and the board is still by
        // the last, so taking only the newest sees no rotation at all.
        let dt_ns = 500_000u64;
        let rates = [4.0f32, 0.0, 0.0, 0.0];
        let batch: [ImuSample; 4] = core::array::from_fn(|i| {
            let mut s = sample((i as u64 + 1) * dt_ns, 0.0, rates[i]);
            s.gyro_rad_s[1] = rates[i];
            s
        });

        // tau large so the accelerometer does not immediately pull it back --
        // this is a test about integration, not about fusion.
        let mut all = ComplementaryFilter::new(1000.0);
        all.update(&[sample(0, 0.0, 0.0)], 0.0);
        let a = all.update(&batch, 0.0);

        let mut newest = ComplementaryFilter::new(1000.0);
        newest.update(&[sample(0, 0.0, 0.0)], 0.0);
        let b = newest.update(&batch[3..], 0.0);

        // Truth: 4 rad/s for 0.5 ms = 2 mrad. Newest-only sees rate 0.
        assert!(
            (a.pitch_rad - 0.002).abs() < 2e-4,
            "batch integrated {}",
            a.pitch_rad
        );
        assert!(
            b.pitch_rad.abs() < 1e-4,
            "newest-only should see no rotation, got {}",
            b.pitch_rad
        );
    }

    #[test]
    fn an_empty_batch_holds_the_last_estimate() {
        let mut f = ComplementaryFilter::new(1.0);
        let a = f.update(&[sample(0, 0.1, 0.0)], 0.0);
        let b = f.update(&[], 0.0);
        assert_eq!(a.pitch_rad, b.pitch_rad);
    }

    #[test]
    fn reset_clears_the_estimate() {
        let mut f = ComplementaryFilter::new(1.0);
        f.update(&[sample(0, 0.2, 0.1)], 0.0);
        f.reset();
        let a = f.update(&[sample(0, 0.0, 0.0)], 0.0);
        assert!(a.pitch_rad.abs() < 1e-5);
    }

    // ---- VelocityLoop ---------------------------------------------------

    const LIM: f32 = 0.087; // 5 degrees

    fn vloop(coupling: PlantCoupling) -> VelocityLoop {
        VelocityLoop::new(0.05, 0.02, LIM, coupling)
    }

    #[test]
    fn too_fast_on_a_ridden_board_commands_nose_up() {
        // CoM above the axle: nose-down accelerates, so shedding speed means
        // pitching UP.
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        assert!(v.update(1.0, 0.0, 0.002, false) > 0.0);
    }

    #[test]
    fn too_slow_on_a_ridden_board_commands_nose_down() {
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        assert!(v.update(-1.0, 0.0, 0.002, false) < 0.0);
    }

    #[test]
    fn the_coupling_inverts_the_whole_loop() {
        // The finding this enum exists for. Same error, opposite reference --
        // and getting it wrong is positive feedback in velocity.
        let (mut above, mut below) = (
            vloop(PlantCoupling::ComAboveAxle),
            vloop(PlantCoupling::ComBelowAxle),
        );
        let a = above.update(1.0, 0.0, 0.002, false);
        let b = below.update(1.0, 0.0, 0.002, false);
        assert!(a > 0.0 && b < 0.0, "a={a} b={b}");
        assert!((a + b).abs() < 1e-6, "should be exact negations");
    }

    #[test]
    fn at_the_setpoint_it_asks_for_nothing() {
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        assert_eq!(v.update(0.0, 0.0, 0.002, false), 0.0);
    }

    #[test]
    fn tracking_a_nonzero_speed_setpoint_uses_the_error_not_the_speed() {
        // Cruising at exactly the setpoint must command level, however fast
        // that is. A loop keyed off raw speed would lean forever.
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        assert_eq!(v.update(3.0, 3.0, 0.002, false), 0.0);
    }

    #[test]
    fn the_reference_is_clamped() {
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        for _ in 0..500 {
            let out = v.update(20.0, 0.0, 0.002, false);
            assert!(out.abs() <= LIM + 1e-6, "{out} exceeded the clamp");
        }
    }

    #[test]
    fn the_integrator_does_not_wind_while_the_inner_loop_is_saturated() {
        // Leaning further when the wheel is already at its current limit asks
        // for authority that does not exist (ICD 7.6).
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        for _ in 0..200 {
            v.update(2.0, 0.0, 0.002, true);
        }
        assert_eq!(v.integral(), 0.0);
    }

    #[test]
    fn the_integrator_does_not_wind_past_a_clamped_reference() {
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        for _ in 0..2000 {
            v.update(20.0, 0.0, 0.002, false);
        }
        // Bounded, not runaway: a sustained error must not leave an integral
        // that takes just as long to unwind once the error reverses.
        assert!(v.integral() < 1.0, "integral ran away to {}", v.integral());
    }

    #[test]
    fn the_integral_is_position_error_so_it_pulls_back_to_where_it_started() {
        // Integrating velocity error IS position error, which is what makes a
        // velocity loop hold station rather than merely stop.
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        for _ in 0..100 {
            v.update(0.5, 0.0, 0.002, false); // drift forward 0.1 m
        }
        // Now stationary but displaced: it must still ask for a lean back.
        let out = v.update(0.0, 0.0, 0.002, false);
        assert!(out > 0.0, "expected a corrective lean, got {out}");
    }

    #[test]
    fn a_zero_dt_does_not_advance_the_integrator() {
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        v.update(1.0, 0.0, 0.0, false);
        assert_eq!(v.integral(), 0.0);
    }

    #[test]
    fn reset_clears_accumulated_position_error() {
        let mut v = vloop(PlantCoupling::ComAboveAxle);
        for _ in 0..100 {
            v.update(1.0, 0.0, 0.002, false);
        }
        assert!(v.integral() > 0.0);
        v.reset();
        assert_eq!(v.integral(), 0.0);
    }

    #[test]
    fn a_pitch_reference_offsets_the_inner_loop() {
        // At exactly the commanded attitude the inner loop must ask for
        // nothing, or the cascade fights itself in steady state.
        let r = PitchRegulator::new(KP, KD);
        assert_eq!(r.update(0.05, 0.0, 0.05), 0.0);
        assert!(r.update(0.0, 0.0, 0.05) > 0.0);
    }

    #[test]
    fn output_is_unclamped_so_the_envelope_can_see_saturation() {
        // A controller that clamps itself hides saturation from the anti-windup
        // that needs to observe it (ICD 7.6).
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(-1.0, 0.0, 0.0) > 40.0);
    }
}
