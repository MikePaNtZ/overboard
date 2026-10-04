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
///
/// # A near-zero specific force has no direction at all (issue #250)
///
/// The band above is opt-in and centred on deviation *from g* — a caller can
/// legitimately leave it at zero. This next check is neither: it is
/// unconditional, and it is not about deviation from g, it is about the raw
/// magnitude approaching zero.
///
/// `atan2` never refuses. Fed a vector of (near) zero length it still returns
/// a confident angle — `atan2(0, -0) = 180°` exactly, "upside down" — but
/// that angle carries no information, because a vanishing vector has no
/// defined direction. Trusting it is not a tuning problem, it is a category
/// error: there is nothing to fuse in. A real board reads this on any brief
/// unloading — cresting a rise, a kerb, a drop, a jump — all ordinary events
/// on the terrain this board is meant for, not exotic edge cases.
///
/// [`MIN_TRUSTED_ACCEL_MAG_M_S2`] is derived, not picked, from the same
/// envelope [`ComplementaryFilter::with_trust_band`]'s doc reasons about:
/// swept over the outer loop's own ±0.86 m/s² commanded-acceleration limit
/// and every pitch up to the ±20° fallen threshold, specific-force magnitude
/// never moves more than ~0.33 m/s² away from g in either direction — and
/// every disturbance this repo has characterised (the 400 N·s nominal kick,
/// the 260–320 N·s envelope sweep) is a horizontal shove, which by
/// `‖(g·sinθ+a, g·cosθ)‖ ≥ g` for any horizontal `a` can only ever raise the
/// magnitude, never lower it. So nothing this codebase has measured, and
/// nothing the controller itself can command, drives magnitude down at all —
/// only genuine unloading does that, running the magnitude toward zero as
/// support vanishes. Half of g leaves wide separation from both sides: far
/// above zero (so it never mistakes a real low-g moment for noise) and far
/// below the ~9.5 m/s² floor of grounded, in-envelope operation.
pub const MIN_TRUSTED_ACCEL_MAG_M_S2: f32 = 9.81 / 2.0;

/// [`TiltFilter`] ignores an accelerometer sample whose magnitude is further
/// than this from g, m/s^2.
pub const MAX_ACCEL_DEVIATION_M_S2: f32 = 2.5;

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
        let (x, y, z) = (accel[0] - forward_accel_m_s2, accel[1], accel[2]);
        let mag = libm::sqrtf(x * x + y * y + z * z);

        // Unconditional (issue #250): a vector this small has no direction to
        // trust, regardless of whether the caller configured a trust band.
        if mag < MIN_TRUSTED_ACCEL_MAG_M_S2 {
            return false;
        }

        if self.accel_trust_band_m_s2 <= 0.0 {
            return true;
        }
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
            //
            // Issue #250: this is exactly where the un-aided-by-time atan2(0,
            // -0) degeneracy bit hardest -- a board spawning (or resuming)
            // out of ground contact snaps straight to a confident 180° on its
            // very first cycle, before anything else has a chance to
            // disagree. If THIS sample's accelerometer isn't trusted, hold
            // level (the struct's own zeroed default) and stay
            // uninitialised rather than commit to a degenerate angle -- the
            // gyro rate is still tracked below, so nothing is lost by
            // waiting for a trustworthy sample to actually initialise on.
            if !self.initialised {
                if !self.accel_trusted(s.accel_m_s2, forward_accel_m_s2) {
                    self.pitch_rate_rad_s = s.gyro_rad_s[1];
                    self.rejected = self.rejected.saturating_add(1);
                    continue;
                }
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
/// `τ = −(kp·θ + kd·θ̇)`, in **N·m**, with θ **nose-up-positive in radians**
/// (ICD §10.1). The minus sign is the ICD's own `current ≈ −K·pitch`, K > 0,
/// carried through to torque, and it is the stabilising sense: a nose-down
/// excursion is negative pitch, correcting it means driving the contact patch
/// forward, which is positive torque.
///
/// # Why torque, not amps (issue #137)
///
/// Loop gain is `kp·kt/J`. Denominating the law in amps means `kt` never
/// appears anywhere the law is defined, tuned or tested — the controller's
/// stability then depends on a motor constant it structurally cannot see.
/// Denominating it in torque removes `kt` from the law entirely: gains here
/// are a property of the plant (its inertia and restoring stiffness) and
/// nothing else. **`kt` appears exactly once in this stack, as the single
/// division `amps = τ / kt` applied at the actuation boundary** (currently
/// `control-ffi::ob_controller_update` and
/// `board-app-driverless`'s `impulse-response-rust` binary), never inside
/// this law. A wrong `kt` then changes the amps a given torque command costs
/// — a headroom error — instead of changing the torque itself — a gain error.
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
/// Longitudinal-acceleration aiding that stays unbiased on a grade.
///
/// [`CommandFeedforward`] predicts `a = K i`. That is the flat-ground model:
/// on a grade the motor also holds the board against gravity, so
/// `K i = a - g sin(alpha)` and an estimator aided by it is biased by about
/// the grade angle (measured: +4.3 deg on a 12 % descent). Wheel odometry
/// measures `a` without that bias, but `dv/dt` of the wheel spikes when the
/// wheel unloads on rough ground under a saturated brake.
///
/// So: the command supplies the fast part and wheel odometry the slow part.
///
/// ```text
/// load  <- LPF_tau_b(K i - dv/dt)       only while the sample is trusted
/// a_aid  = K i - load
/// ```
///
/// `load` is the grade load (`-g sin(alpha)` + rolling loss + model error),
/// in m/s^2: negative going downhill, positive climbing. It is also the
/// current needed to hold the grade (`load / K`), which the regulator can use
/// as a feedforward so the board does not droop nose-up on a descent.
#[derive(Debug, Clone, Copy)]
pub struct GradeAwareAiding {
    k_m_s2_per_a: f32,
    tau_b_s: f32,
    load_m_s2: f32,
    last_v: Option<f32>,
}

impl GradeAwareAiding {
    /// Largest |load| believed, m/s^2 (about g sin 20 deg plus rolling loss).
    pub const LOAD_LIMIT_M_S2: f32 = 3.5;
    /// A wheel acceleration beyond this is a slip or bounce, not the board.
    pub const MAX_ODOMETRY_ACCEL_M_S2: f32 = 6.0;

    pub const fn new(k_m_s2_per_a: f32, tau_b_s: f32) -> Self {
        GradeAwareAiding { k_m_s2_per_a, tau_b_s, load_m_s2: 0.0, last_v: None }
    }

    /// Grade load, m/s^2 (negative downhill).
    pub fn load_m_s2(&self) -> f32 {
        self.load_m_s2
    }

    /// Grade estimate, rad (positive = downhill ahead).
    pub fn grade_rad(&self) -> f32 {
        libm::asinf((-self.load_m_s2 / 9.81).clamp(-1.0, 1.0))
    }

    /// One step. `amps` is the current applied last cycle, `v_m_s` the wheel
    /// speed, `accel_mag_m_s2` the IMU specific-force magnitude (for the
    /// impact gate). Returns the aiding acceleration, m/s^2.
    pub fn update(&mut self, amps: f32, v_m_s: f32, accel_mag_m_s2: f32, dt_s: f32) -> f32 {
        let a_cmd = self.k_m_s2_per_a * amps;
        let prev = self.last_v.replace(v_m_s);
        if let Some(prev) = prev {
            if dt_s > 0.0 {
                let a_odo = (v_m_s - prev) / dt_s;
                let trusted = libm::fabsf(accel_mag_m_s2 - 9.81) < MAX_ACCEL_DEVIATION_M_S2
                    && libm::fabsf(a_odo) < Self::MAX_ODOMETRY_ACCEL_M_S2;
                if trusted {
                    let alpha = dt_s / (self.tau_b_s + dt_s);
                    self.load_m_s2 += alpha * ((a_cmd - a_odo) - self.load_m_s2);
                    self.load_m_s2 = self
                        .load_m_s2
                        .clamp(-Self::LOAD_LIMIT_M_S2, Self::LOAD_LIMIT_M_S2);
                }
            }
        }
        a_cmd - self.load_m_s2
    }

    pub fn reset(&mut self) {
        self.load_m_s2 = 0.0;
        self.last_v = None;
    }
}

/// Pitch AND roll, for a board that leans to steer.
///
/// [`ComplementaryFilter`] integrates the pitch-axis gyro alone. That is right
/// while the board stays level in roll, and wrong in a banked turn: the body
/// pitch gyro then reads `q = theta_dot * cos(phi) + psi_dot * sin(phi) *
/// cos(theta)`, so the yaw rate of the turn leaks into the pitch estimate.
/// Measured in sim: with lean-to-steer the leak drives the board nose-down
/// until the motor saturates and it falls in the first turn.
///
/// This filter runs the same complementary structure on both tilt angles with
/// ZYX Euler kinematics, in the ICD's forward-right-down body frame
/// (`f = [g sin(theta), -g sin(phi) cos(theta), -g cos(phi) cos(theta)]` at rest):
///
/// ```text
/// phi_dot   = p + (q sin(phi) + r cos(phi)) tan(theta)
/// theta_dot = q cos(phi) - r sin(phi)
/// ```
///
/// The accelerometer is aided for the two accelerations the board knows from
/// its wheel: longitudinal (`forward_accel_m_s2`, as before) and centripetal,
/// `v * psi_dot` towards the turn, with `v` from [`TiltFilter::set_speed`].
/// Without the centripetal term a balanced (coordinated) turn reads as zero
/// roll.
///
/// With `phi = 0` and no yaw rate this reduces to [`ComplementaryFilter`].
#[derive(Debug, Clone, Copy)]
pub struct TiltFilter {
    tau_s: f32,
    pitch_rad: f32,
    roll_rad: f32,
    pitch_rate_rad_s: f32,
    speed_m_s: f32,
    last_t_ns: Option<u64>,
    initialised: bool,
}

impl TiltFilter {
    pub const fn new(tau_s: f32) -> Self {
        TiltFilter {
            tau_s,
            pitch_rad: 0.0,
            roll_rad: 0.0,
            pitch_rate_rad_s: 0.0,
            speed_m_s: 0.0,
            last_t_ns: None,
            initialised: false,
        }
    }

    /// Forward ground speed from wheel odometry, m/s. Call before `update`.
    pub fn set_speed(&mut self, speed_m_s: f32) {
        self.speed_m_s = speed_m_s;
    }

    /// Roll estimate, rad, positive = right side down (ICD body frame).
    pub fn roll_rad(&self) -> f32 {
        self.roll_rad
    }

    /// Tilt implied by one accelerometer sample after removing the known
    /// accelerations. `None` if what is left is too small to have a direction.
    fn accel_tilt(&self, s: &ImuSample, forward_accel_m_s2: f32, yaw_rate: f32) -> Option<(f32, f32)> {
        let (sp, cp) = (libm::sinf(self.roll_rad), libm::cosf(self.roll_rad));
        let a_c = self.speed_m_s * yaw_rate; // centripetal, + towards the right
        let fx = s.accel_m_s2[0] - forward_accel_m_s2;
        let fy = s.accel_m_s2[1] - a_c * cp;
        let fz = s.accel_m_s2[2] - a_c * sp;
        let mag = libm::sqrtf(fx * fx + fy * fy + fz * fz);
        if mag < MIN_TRUSTED_ACCEL_MAG_M_S2 {
            return None;
        }
        // Impacts (a wheel bouncing on rough ground) are not "which way is
        // down": take the gyro alone through them. Measured on the authored
        // courses: f_z spikes to -16..-20 m/s^2.
        if libm::fabsf(mag - 9.81) > MAX_ACCEL_DEVIATION_M_S2 {
            return None;
        }
        let roll = libm::atan2f(-fy, -fz);
        let pitch = libm::atan2f(fx, libm::sqrtf(fy * fy + fz * fz));
        Some((pitch, roll))
    }
}

impl Estimator for TiltFilter {
    fn update(&mut self, samples: &[ImuSample], forward_accel_m_s2: f32) -> Attitude {
        for s in samples {
            let [p, q, r] = s.gyro_rad_s;
            let (sp, cp) = (libm::sinf(self.roll_rad), libm::cosf(self.roll_rad));
            let ct = libm::cosf(self.pitch_rad).max(0.2);
            let tt = libm::tanf(self.pitch_rad).clamp(-5.0, 5.0);
            let theta_dot = q * cp - r * sp;
            let phi_dot = p + (q * sp + r * cp) * tt;
            let yaw_rate = (q * sp + r * cp) / ct;
            self.pitch_rate_rad_s = theta_dot;

            let acc = self.accel_tilt(s, forward_accel_m_s2, yaw_rate);
            if !self.initialised {
                if let Some((pitch, roll)) = acc {
                    self.pitch_rad = pitch;
                    self.roll_rad = roll;
                    self.initialised = true;
                    self.last_t_ns = Some(s.t_sample_ns);
                }
                continue;
            }
            let dt = match self.last_t_ns {
                Some(prev) => s.t_sample_ns.saturating_sub(prev) as f32 * 1e-9,
                None => 0.0,
            };
            self.last_t_ns = Some(s.t_sample_ns);
            if dt <= 0.0 {
                continue;
            }
            let pitch_pred = self.pitch_rad + theta_dot * dt;
            let roll_pred = self.roll_rad + phi_dot * dt;
            match acc {
                Some((pitch_acc, roll_acc)) => {
                    let alpha = self.tau_s / (self.tau_s + dt);
                    self.pitch_rad = alpha * pitch_pred + (1.0 - alpha) * pitch_acc;
                    self.roll_rad = alpha * roll_pred + (1.0 - alpha) * roll_acc;
                }
                None => {
                    self.pitch_rad = pitch_pred;
                    self.roll_rad = roll_pred;
                }
            }
        }
        Attitude {
            pitch_rad: self.pitch_rad,
            pitch_rate_rad_s: self.pitch_rate_rad_s,
        }
    }

    fn reset(&mut self) {
        let tau = self.tau_s;
        *self = TiltFilter::new(tau);
    }
}

#[derive(Debug, Clone, Copy, Default)]
pub struct PitchRegulator {
    kp_nm_per_rad: f32,
    kd_nm_per_rad_s: f32,
}

impl PitchRegulator {
    pub const fn new(kp_nm_per_rad: f32, kd_nm_per_rad_s: f32) -> Self {
        PitchRegulator {
            kp_nm_per_rad,
            kd_nm_per_rad_s,
        }
    }

    /// Requested torque in N·m, before any clamping.
    ///
    /// `pitch_ref_rad` is the attitude to hold — zero for pure disturbance
    /// rejection, or the outer loop's output when a [`VelocityLoop`] is
    /// cascaded on top.
    ///
    /// Unclamped on purpose: bounding the command is the safety envelope's job
    /// (stage 2, ICD §7.6), and a controller that silently clamps its own
    /// output hides saturation from the anti-windup that needs to see it.
    /// Note the clamp itself is expressed in amps (the drive's real limit) —
    /// converting this torque to amps via `kt` happens at the caller's
    /// actuation boundary, not here.
    pub fn update(&self, pitch_rad: f32, pitch_rate_rad_s: f32, pitch_ref_rad: f32) -> f32 {
        -(self.kp_nm_per_rad * (pitch_rad - pitch_ref_rad)
            + self.kd_nm_per_rad_s * pitch_rate_rad_s)
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

    /// Predicted forward acceleration, m/s², from motor current.
    ///
    /// **Pass the MEASURED current wherever the drive reports one.** An earlier
    /// version of this comment argued for the commanded value, on the grounds
    /// that it carries no sensing delay and works on drives that cannot report
    /// phase current. That reasoning does not survive contact with the drive
    /// this project actually selected.
    ///
    /// A VESC silently derates commanded torque through at least six cutback
    /// layers — battery voltage sag, input-current limits, FET and motor
    /// temperature, ERPM limits. Fed the *commanded* value during a cutback,
    /// this function subtracts an acceleration that is not happening, and the
    /// error lands directly on the accelerometer's gravity reference. That is
    /// the same corruption the estimator work removed, re-entering through the
    /// actuator instead of the sensor — and it would appear under load, on a
    /// hill, at low state of charge, with a rider aboard. The sim cannot show
    /// it, because the sim has no cutback.
    ///
    /// The commanded value remains the fallback for a drive with no current
    /// telemetry, and the *difference* between the two is worth reporting as a
    /// fault in its own right: it is exactly the cutback detector such a drive
    /// needs anyway.
    pub fn predict(&self, current_a: f32) -> f32 {
        self.gain_m_s2_per_a * current_a
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

/// Full-state speed hold: one law for balance and speed, for a board whose
/// rider stays passive (the sim's authority sweep).
///
/// `amps = -k_pitch·θ - k_rate·θ̇ + k_speed·e + k_int·∫e`, with θ
/// nose-up-positive, e = v − v_ref and forward current positive.
///
/// This replaces a slow [`VelocityLoop`] cascaded on a fixed
/// [`PitchRegulator`]. That pair overshot by 2–3 m/s at grade changes,
/// because the outer loop could not see the inner loop's state. The gains
/// come from a discrete LQR on the planar wheel-and-body model
/// (`sim/carve/lqr_design.py`). That script also checks the closed loop with
/// the rider's fore/aft spring mode, which this law cannot measure.
///
/// The positive speed gain is the non-minimum-phase part of a balancing
/// vehicle: to slow down it first drives forward, so the body tips back.
///
/// The reference slews at `accel_limit_m_s2`, so a step in target speed does
/// not ask for a lean the motor cannot recover from. While it slews, the law
/// adds the steady lean and current of that acceleration (feedforward). If
/// it does not, the integral winds up during every ramp and the board
/// overshoots by about 1.4 m/s when the ramp ends -- measured on a 20 %
/// climb, where the overshoot saturated the motor.
/// Jerk limit of the [`SpeedHoldLqr`] reference, m/s³.
pub const SPEED_HOLD_JERK_M_S3: f32 = 1.0;
/// Reference approach gain, 1/s: the reference acceleration is this times
/// the remaining speed error, before the acceleration limit.
pub const SPEED_HOLD_APPROACH_PER_S: f32 = 1.0;

#[derive(Debug, Clone, Copy)]
pub struct SpeedHoldLqr {
    k_pitch: f32,
    k_rate: f32,
    k_speed: f32,
    k_int: f32,
    accel_limit_m_s2: f32,
    integral_limit_m: f32,
    ff_pitch_rad_per_m_s2: f32,
    ff_amps_per_m_s2: f32,
    integral: f32,
    v_ref: Option<f32>,
    a_ref: f32,
}

impl SpeedHoldLqr {
    /// Gains in A/rad, A/(rad/s), A/(m/s), A/m — all magnitudes.
    pub const fn new(k_pitch: f32, k_rate: f32, k_speed: f32, k_int: f32, accel_limit_m_s2: f32) -> Self {
        SpeedHoldLqr {
            k_pitch,
            k_rate,
            k_speed,
            k_int,
            accel_limit_m_s2,
            integral_limit_m: 10.0,
            ff_pitch_rad_per_m_s2: 0.0,
            ff_amps_per_m_s2: 0.0,
            integral: 0.0,
            v_ref: None,
            a_ref: 0.0,
        }
    }

    /// Steady lean (rad, nose-up positive) and current (A) per m/s² of
    /// reference acceleration, from the same linear model as the gains.
    pub const fn with_feedforward(mut self, pitch_rad_per_m_s2: f32, amps_per_m_s2: f32) -> Self {
        self.ff_pitch_rad_per_m_s2 = pitch_rad_per_m_s2;
        self.ff_amps_per_m_s2 = amps_per_m_s2;
        self
    }

    pub fn reset(&mut self) {
        self.integral = 0.0;
        self.v_ref = None;
        self.a_ref = 0.0;
    }

    #[cfg(test)]
    fn reset_integral_for_test(&mut self) {
        self.integral = 0.0;
    }

    /// The slewed speed reference of the last update.
    pub fn v_ref(&self) -> Option<f32> {
        self.v_ref
    }

    /// Requested current in amps, before the envelope clamp, on level
    /// ground: [`SpeedHoldLqr::update_with_grade`] with no grade load and no
    /// current budget.
    pub fn update(
        &mut self,
        pitch_rad: f32,
        pitch_rate_rad_s: f32,
        v_m_s: f32,
        v_target_m_s: f32,
        dt_s: f32,
        saturated: bool,
    ) -> f32 {
        self.update_with_grade(
            pitch_rad,
            pitch_rate_rad_s,
            v_m_s,
            v_target_m_s,
            dt_s,
            saturated,
            0.0,
            f32::INFINITY,
        )
    }

    /// Requested current in amps, before the envelope clamp.
    ///
    /// The reference starts at the measured speed and slews toward
    /// `v_target_m_s`. `saturated` is the envelope's clamp bit from the last
    /// cycle: while it is set, the integral does not wind in the direction
    /// that pushes the demand further past the limit, the reference stops
    /// accelerating, and it leaks toward the measured speed.
    ///
    /// `load_m_s2` is [`GradeAwareAiding::load_m_s2`] (positive on a climb).
    /// It adds the steady current AND the steady lean of that grade. Without
    /// them the speed integral alone must find 20-40 A on a vertical curve;
    /// the board then runs behind the new equilibrium, the pitch loop pays
    /// the deficit as a burst, and the burst saturates the motor (Monte
    /// Carlo, 2026-10-04: 30 of 167 runs inside the static envelope fell
    /// this way). Current alone, without the lean, runs away (`--grade-ff`).
    ///
    /// `load / k` is the current the real plant needed at steady speed, so
    /// it needs no rider mass or Kt. `i_max_a` is the envelope limit: the
    /// reference acceleration is held inside a current budget and a lean
    /// budget (see [`SPEED_HOLD_LEAN_BUDGET_RAD`]).
    #[allow(clippy::too_many_arguments)]
    pub fn update_with_grade(
        &mut self,
        pitch_rad: f32,
        pitch_rate_rad_s: f32,
        v_m_s: f32,
        v_target_m_s: f32,
        dt_s: f32,
        saturated: bool,
        load_m_s2: f32,
        i_max_a: f32,
    ) -> f32 {
        let i_grade = load_m_s2 / SPEED_HOLD_LOAD_M_S2_PER_A;
        // Reference trajectory: acceleration limited, and jerk limited so
        // the feedforward lean never steps. A lean step makes the board
        // first drive backward (non-minimum phase), and the accelerometer
        // then misreads the pitch: that combination fell at start-up.
        let v_ref = match self.v_ref {
            None => v_m_s,
            Some(r) => {
                // Governor: the acceleration the grade leaves room for.
                let grade_rad = libm::asinf((load_m_s2 / 9.81).clamp(-1.0, 1.0));
                let a_lim_i = if self.ff_amps_per_m_s2 > 0.0 {
                    ((SPEED_HOLD_CURRENT_BUDGET * i_max_a - libm::fabsf(i_grade))
                        / self.ff_amps_per_m_s2)
                        .max(0.0)
                } else {
                    f32::INFINITY
                };
                let a_lim_th = if self.ff_pitch_rad_per_m_s2 != 0.0 {
                    ((SPEED_HOLD_LEAN_BUDGET_RAD
                        - libm::fabsf(grade_rad)
                        - libm::fabsf(SPEED_HOLD_GRADE_PITCH_RAD_PER_M_S2 * load_m_s2))
                        / libm::fabsf(self.ff_pitch_rad_per_m_s2))
                    .max(0.0)
                } else {
                    f32::INFINITY
                };
                let a_lim = self.accel_limit_m_s2.min(a_lim_i).min(a_lim_th);
                let mut r = r;
                let a_des = if saturated {
                    // A board that cannot follow must not be left behind by
                    // its own reference.
                    r += (v_m_s - r) * (dt_s / SPEED_HOLD_SATURATED_LEAK_S).min(1.0);
                    0.0
                } else {
                    (SPEED_HOLD_APPROACH_PER_S * (v_target_m_s - r)).clamp(-a_lim, a_lim)
                };
                let da = SPEED_HOLD_JERK_M_S3 * dt_s;
                self.a_ref += (a_des - self.a_ref).clamp(-da, da);
                r + self.a_ref * dt_s
            }
        };
        let a_ref = self.a_ref;
        self.v_ref = Some(v_ref);
        let err = v_m_s - v_ref;
        let pitch_ff =
            self.ff_pitch_rad_per_m_s2 * a_ref + SPEED_HOLD_GRADE_PITCH_RAD_PER_M_S2 * load_m_s2;
        let balance = self.ff_amps_per_m_s2 * a_ref + i_grade
            - self.k_pitch * (pitch_rad - pitch_ff)
            - self.k_rate * pitch_rate_rad_s;
        let demand = balance + self.k_speed * err + self.k_int * self.integral;
        let pushing_further = saturated && demand.is_sign_positive() == err.is_sign_positive();
        if dt_s > 0.0 && !pushing_further {
            self.integral =
                (self.integral + err * dt_s).clamp(-self.integral_limit_m, self.integral_limit_m);
        }
        balance + self.k_speed * err + self.k_int * self.integral
    }
}

/// Grade load per amp, m/s² per A: the same scale [`GradeAwareAiding`] learns
/// its load in (Kt / (R * 83 kg)), so `load / this` is amps.
pub const SPEED_HOLD_LOAD_M_S2_PER_A: f32 = 0.0584;
/// Steady lean per m/s² of grade load, rad: -R / (g L) at 70 kg. It equals
/// the acceleration lean (-0.1261) less its inertial part (1/g), so the two
/// feedforwards agree. Small (1 deg at 8 %), so a mass error costs < 0.5 deg.
pub const SPEED_HOLD_GRADE_PITCH_RAD_PER_M_S2: f32 = -0.0234;
/// Fraction of the current limit the reference may plan to use.
pub const SPEED_HOLD_CURRENT_BUDGET: f32 = 0.6;
/// Lean the reference may plan to use against the road, rad: the 18.6 deg
/// deck strike less 5 deg for the transient on a vertical curve.
pub const SPEED_HOLD_LEAN_BUDGET_RAD: f32 = 0.237;
/// Time constant of the reference leak toward the measured speed while the
/// motor is saturated, s.
pub const SPEED_HOLD_SATURATED_LEAK_S: f32 = 0.5;

#[cfg(test)]
mod speed_hold_tests {
    use super::*;

    fn lqr() -> SpeedHoldLqr {
        SpeedHoldLqr::new(376.8, 88.4, 28.35, 8.06, 1.0)
    }

    #[test]
    fn nose_down_drives_forward() {
        let mut c = lqr();
        assert!(c.update(-0.05, 0.0, 0.0, 0.0, 0.002, false) > 0.0);
    }

    #[test]
    fn too_fast_first_drives_forward_to_tip_back() {
        let mut c = lqr();
        c.update(0.0, 0.0, 3.0, 3.0, 0.002, false);
        // Reference held at 3 m/s; board now at 3.5 m/s, level.
        assert!(c.update(0.0, 0.0, 3.5, 3.0, 0.002, false) > 0.0);
    }

    #[test]
    fn reference_starts_at_measured_speed_and_never_steps_acceleration() {
        let mut c = lqr();
        c.update(0.0, 0.0, 1.0, 5.0, 0.002, false);
        assert_eq!(c.v_ref(), Some(1.0));
        let mut prev = 1.0;
        let mut prev_a = 0.0f32;
        for _ in 0..5000 {
            c.update(0.0, 0.0, 1.0, 5.0, 0.002, false);
            let r = c.v_ref().unwrap();
            let a = (r - prev) / 0.002;
            assert!((a - prev_a).abs() <= SPEED_HOLD_JERK_M_S3 * 0.002 + 1e-3);
            assert!(a <= 1.0 + 1e-3 && r <= 5.0 + 0.05, "a {a} r {r}");
            prev = r;
            prev_a = a;
        }
        assert!((prev - 5.0).abs() < 0.1, "reference did not arrive: {prev}");
    }

    #[test]
    fn integral_does_not_wind_further_while_saturated() {
        let mut c = lqr();
        c.update(0.0, 0.0, 3.0, 3.0, 0.002, false);
        // The reference also leaks toward the measured speed while saturated,
        // so the output moves; the integral must not.
        c.update(0.0, 0.0, 4.0, 3.0, 0.1, true);
        let a = c.integral;
        c.update(0.0, 0.0, 4.0, 3.0, 0.1, true);
        assert!((a - c.integral).abs() < 1e-6, "integral wound while saturated: {a} -> {}", c.integral);
    }

    #[test]
    fn grade_load_adds_its_current_and_its_lean() {
        // On the reference, at the grade lean, the demand is the grade current.
        let mut c = lqr().with_feedforward(-0.1261, 17.82);
        let load = 0.8;
        let lean = SPEED_HOLD_GRADE_PITCH_RAD_PER_M_S2 * load;
        c.update_with_grade(lean, 0.0, 3.0, 3.0, 0.002, false, load, 40.0);
        let i = c.update_with_grade(lean, 0.0, 3.0, 3.0, 0.002, false, load, 40.0);
        assert!((i - load / SPEED_HOLD_LOAD_M_S2_PER_A).abs() < 0.05, "{i}");
    }

    #[test]
    fn governor_stops_acceleration_when_the_grade_uses_the_current_budget() {
        // 0.6 * 30 A = 18 A budget; a 1.2 m/s² load needs 20.5 A already.
        let mut c = lqr().with_feedforward(-0.1261, 17.82);
        c.update_with_grade(0.0, 0.0, 2.0, 5.0, 0.002, false, 1.2, 30.0);
        for _ in 0..1000 {
            c.update_with_grade(0.0, 0.0, 2.0, 5.0, 0.002, false, 1.2, 30.0);
        }
        assert!(c.a_ref.abs() < 1e-3, "a_ref {}", c.a_ref);
    }

    #[test]
    fn feedforward_holds_a_level_board_at_the_ramp_lean() {
        // While the reference ramps, a board already at the feedforward lean
        // and on the reference gets exactly the feedforward current.
        let mut c = lqr().with_feedforward(-0.1261, 17.82);
        c.update(0.0, 0.0, 1.0, 9.0, 0.002, false);
        // Let the reference reach its 1 m/s^2 limit (1 s at 1 m/s^3).
        for _ in 0..600 {
            let v = c.v_ref().unwrap();
            c.update(-0.1261, 0.0, v, 9.0, 0.002, false);
        }
        c.reset_integral_for_test();
        let r = c.v_ref().unwrap();
        let i = c.update(-0.1261, 0.0, r + 0.002, 9.0, 0.002, false);
        assert!((i - 17.82).abs() < 0.1, "{i}");
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
    fn nose_down_commands_positive_torque() {
        // Nose-down is NEGATIVE pitch (ICD 10.1). Correcting it means driving
        // the contact patch forward, i.e. POSITIVE torque. If this ever
        // inverts, the board accelerates into its own nosedive.
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(-0.1, 0.0, 0.0) > 0.0);
    }

    #[test]
    fn nose_up_commands_negative_torque() {
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

    fn freefall_sample(t_ns: u64) -> ImuSample {
        // True specific force in free-fall is (near) zero -- no accelerometer
        // channel carries a "down" to resolve.
        ImuSample {
            gyro_rad_s: [0.0, 0.0, 0.0],
            accel_m_s2: [0.0, 0.0, 0.0],
            t_sample_ns: t_ns,
        }
    }

    #[test]
    fn a_first_sample_of_near_zero_specific_force_does_not_snap_to_a_degenerate_angle() {
        // Regression for issue #250: a board that spawns (or resumes) out of
        // ground contact must not initialise on atan2(0, -0) = 180 deg.
        let mut f = ComplementaryFilter::new(1.0);
        let a = f.update(&[freefall_sample(0)], 0.0);
        assert!(
            a.pitch_rad.abs() < 1e-3,
            "snapped to a degenerate {} deg on an untrusted first sample",
            a.pitch_rad.to_degrees()
        );
        assert_eq!(f.rejected_samples(), 1);

        // A real (trusted) reading right after should initialise cleanly --
        // waiting one sample must not have broken the mechanism.
        let b = f.update(&[sample(2_000_000, 0.3, 0.0)], 0.0);
        assert!(
            (b.pitch_rad - 0.3).abs() < 1e-3,
            "did not initialise cleanly on the first trusted sample, got {}",
            b.pitch_rad
        );
    }

    #[test]
    fn sustained_near_zero_specific_force_is_rejected_and_never_inverts() {
        // Regression for issue #250: sustained free-fall must not let the
        // estimate converge on the atan2(0, -0) degeneracy, even though the
        // trust band above (an opt-in, deviation-from-g gate) is left off
        // here (`ComplementaryFilter::new` defaults `accel_trust_band_m_s2`
        // to 0.0) -- the near-zero-magnitude floor is unconditional.
        let mut f = ComplementaryFilter::new(1.0);
        // Initialise on a real, level reading first.
        f.update(&[sample(0, 0.0, 0.0)], 0.0);

        let dt_ns = 2_000_000u64; // 500 Hz
        let mut last = Attitude::default();
        for k in 1..=2_500u64 {
            // 5 s -- several times tau, long enough that an ungated filter
            // would have converged onto theta_accel.
            last = f.update(&[freefall_sample(k * dt_ns)], 0.0);
        }

        assert!(
            last.pitch_rad.abs() < 0.05,
            "estimate drifted to {} deg under sustained free-fall",
            last.pitch_rad.to_degrees()
        );
        assert_eq!(f.rejected_samples(), 2_500);
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
        // that needs to observe it (ICD 7.6). Magnitude here is arbitrary --
        // just large enough to be clearly outside any plausible torque ceiling.
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(-1.0, 0.0, 0.0) > 40.0);
    }
}

#[cfg(test)]
mod tilt_filter_tests {
    use super::*;
    const G: f32 = 9.81;

    fn sample(t_ns: u64, gyro: [f32; 3], accel: [f32; 3]) -> ImuSample {
        ImuSample { gyro_rad_s: gyro, accel_m_s2: accel, t_sample_ns: t_ns }
    }

    /// Specific force and body rates of a board in a steady, balanced turn at
    /// speed `v`, yaw rate `psi_dot`, bank `phi`, pitch `theta` (FRD).
    fn banked(v: f32, psi_dot: f32, phi: f32, theta: f32) -> ([f32; 3], [f32; 3]) {
        let (sp, cp, st, ct) = (libm::sinf(phi), libm::cosf(phi), libm::sinf(theta), libm::cosf(theta));
        // Body rates of a constant yaw rate about the world down axis.
        let gyro = [-psi_dot * st, psi_dot * sp * ct, psi_dot * cp * ct];
        // Gravity part, plus the centripetal acceleration v*psi_dot rotated in.
        let a_c = v * psi_dot;
        let accel = [
            G * st,
            -G * sp * ct + a_c * cp,
            -G * cp * ct + a_c * sp,
        ];
        (gyro, accel)
    }

    #[test]
    fn level_and_still_reads_zero() {
        let mut f = TiltFilter::new(0.5);
        let mut a = Attitude::default();
        for k in 0..1000 {
            a = f.update(&[sample(k * 2_000_000, [0.0; 3], [0.0, 0.0, -G])], 0.0);
        }
        assert!(a.pitch_rad.abs() < 1e-5 && f.roll_rad().abs() < 1e-5);
    }

    #[test]
    fn a_banked_turn_does_not_leak_into_pitch() {
        let (v, psi_dot, phi) = (5.0, 0.5, 0.25);
        let (gyro, accel) = banked(v, psi_dot, phi, 0.0);
        let mut f = TiltFilter::new(0.5);
        f.set_speed(v);
        let mut a = Attitude::default();
        for k in 0..5000 {
            a = f.update(&[sample(k * 2_000_000, gyro, accel)], 0.0);
        }
        assert!(a.pitch_rad.abs() < 0.01, "pitch {}", a.pitch_rad);
        assert!(a.pitch_rate_rad_s.abs() < 1e-3, "rate {}", a.pitch_rate_rad_s);
        assert!((f.roll_rad() - phi).abs() < 0.01, "roll {}", f.roll_rad());

        // The single-axis filter, given the same turn, drifts nose-up: q > 0.
        let mut c = ComplementaryFilter::new(0.5);
        let mut b = Attitude::default();
        for k in 0..5000 {
            b = c.update(&[sample(k * 2_000_000, gyro, accel)], 0.0);
        }
        assert!(b.pitch_rad > 0.05, "single-axis pitch {}", b.pitch_rad);
    }

    #[test]
    fn it_matches_the_complementary_filter_with_no_roll() {
        let mut f = TiltFilter::new(0.5);
        let mut c = ComplementaryFilter::new(0.5);
        let (mut a, mut b) = (Attitude::default(), Attitude::default());
        for k in 0..2000u64 {
            let th = 0.05 * libm::sinf(k as f32 * 0.01);
            let s = sample(k * 2_000_000, [0.0, 0.05 * 0.01 / 0.002 * libm::cosf(k as f32 * 0.01), 0.0],
                [G * libm::sinf(th), 0.0, -G * libm::cosf(th)]);
            a = f.update(&[s], 0.0);
            b = c.update(&[s], 0.0);
        }
        assert!((a.pitch_rad - b.pitch_rad).abs() < 1e-3, "{} vs {}", a.pitch_rad, b.pitch_rad);
    }
}

#[cfg(test)]
mod grade_aware_tests {
    use super::*;
    const K: f32 = 0.0584;

    #[test]
    fn holding_speed_on_a_descent_learns_the_grade_and_reports_zero_acceleration() {
        let mut g = GradeAwareAiding::new(K, 0.3);
        // 12 % descent: holding speed takes i = -g sin(6.84 deg) / K of brake.
        let i = -9.81 * 6.84_f32.to_radians().sin() / K;
        let mut a = 1.0;
        for _ in 0..2000 {
            a = g.update(i, 3.0, 9.81, 0.002);
        }
        assert!(a.abs() < 0.02, "aid {a}");
        assert!((g.grade_rad().to_degrees() - 6.84).abs() < 0.1, "grade {}", g.grade_rad().to_degrees());
    }

    #[test]
    fn an_impact_sample_does_not_move_the_load() {
        let mut g = GradeAwareAiding::new(K, 0.3);
        g.update(0.0, 3.0, 9.81, 0.002);
        // a 20 m/s^2 impact with a wheel spike: gated
        g.update(0.0, 2.9, 20.0, 0.002);
        assert_eq!(g.load_m_s2(), 0.0);
    }

    #[test]
    fn on_the_flat_the_aid_is_the_command_prediction() {
        let mut g = GradeAwareAiding::new(K, 0.3);
        let mut v = 0.0;
        let mut a = 0.0;
        for _ in 0..1000 {
            let acc = K * 10.0;
            v += acc * 0.002;
            a = g.update(10.0, v, 9.81, 0.002);
        }
        assert!((a - K * 10.0).abs() < 0.02, "aid {a}");
    }
}
