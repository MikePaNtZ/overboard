//! Pure, deterministic control logic. No I/O, no clock reads — `dt` is derived
//! from the timestamps carried on [`Observation`] (DR-CTRL-1, ICD §5.2).
//!
//! Depends on `board-types` only (not `hal`), so it can be unit-tested and
//! fuzzed with no backend in the loop.
#![no_std]

use board_types::{Command, Observation, Params};

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
