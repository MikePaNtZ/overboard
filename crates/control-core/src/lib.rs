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
    /// Unclamped on purpose: bounding the command is the safety envelope's job
    /// (stage 2, ICD §7.6), and a controller that silently clamps its own
    /// output hides saturation from the anti-windup that needs to see it.
    pub fn update(&self, pitch_rad: f32, pitch_rate_rad_s: f32) -> f32 {
        -(self.kp_a_per_rad * pitch_rad + self.kd_a_per_rad_s * pitch_rate_rad_s)
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
        assert!(r.update(-0.1, 0.0) > 0.0);
    }

    #[test]
    fn nose_up_commands_negative_current() {
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(0.1, 0.0) < 0.0);
    }

    #[test]
    fn the_rate_term_opposes_the_rate() {
        // At zero pitch error, a nose-up RATE must still be resisted -- that is
        // the damping, and without it the proportional term alone rings.
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(0.0, 0.5) < 0.0);
        assert!(r.update(0.0, -0.5) > 0.0);
    }

    #[test]
    fn level_and_still_commands_nothing() {
        let r = PitchRegulator::new(KP, KD);
        assert_eq!(r.update(0.0, 0.0), 0.0);
    }

    #[test]
    fn the_law_is_odd_symmetric() {
        // Equal and opposite errors must produce equal and opposite commands.
        // An asymmetry here would mean the board recovers better one way than
        // the other, which is the kind of thing that only shows up in a hard
        // save in the wrong direction.
        let r = PitchRegulator::new(KP, KD);
        assert_eq!(r.update(0.07, 0.3), -r.update(-0.07, -0.3));
    }

    #[test]
    fn output_is_unclamped_so_the_envelope_can_see_saturation() {
        // A controller that clamps itself hides saturation from the anti-windup
        // that needs to observe it (ICD 7.6).
        let r = PitchRegulator::new(KP, KD);
        assert!(r.update(-1.0, 0.0) > 40.0);
    }
}
