//! Lean-to-steer: the board turns because it ROLLS, as a real onewheel does.
//!
//! Enabled by `sim-host --lean-steer`. Without the flag nothing here runs and
//! the commanded-yaw law in [`crate::host`] is unchanged.
//!
//! # Plant side (the tire)
//!
//! A tilted wheel with a rounded crown rolls like a cone. A rigid cone of
//! rolling radius `r_c` tilted by `phi` rolls on a circle of curvature
//! `tan(phi) / r_c`. A real tire scrubs and slips laterally, so it turns less
//! than the rigid cone. `eta` ([`LeanSteerParams::camber_efficiency`]) carries
//! that loss:
//!
//! ```text
//! kappa_tire(phi) = eta * tan(phi) / r_c
//! ```
//!
//! The tire does not reach that curvature at once. Its turn slip relaxes over
//! a distance `sigma`, so the target yaw rate `-v * kappa_tire` is filtered
//! with time constant `sigma / |v|`. The yaw rate is imposed by a stiff yaw
//! torque on the frame (a model of the contact patch's turn-slip moment), not
//! by rotating the pose. MuJoCo's contact friction then supplies the
//! centripetal force at the contact patch, below the centre of mass, so a turn
//! tries to roll the board OUT of the turn and the rider must lean IN.
//! `v* = sqrt(g r_c / eta)` (3.4 m/s) is the speed at which camber steer and
//! balance agree for a board and rider leaning together.
//!
//! # Rider side: an idealised skilled rider
//!
//! The steer stick is the rider's INTENT, a target curvature
//! `kappa = steer * kappa_max`, eased with a lag and clamped to a turn the
//! rider can hold. From it:
//!
//! ```text
//! phi_ref = atan(kappa * r_c / eta)                    deck camber that steers kappa
//! d_ref   = (h v^2 kappa / g - (h - rho) sin phi_ref) / cos phi_ref
//!                                                      hip offset that balances the turn
//! tau     = -kp (phi - phi_ref) - kd phi_dot           balance skill, about the board axis
//! ```
//!
//! **What is idealised.** `tau` is a roll torque applied to the board about
//! its forward axis. It stands for the balance a real rider gets from foot
//! pressure and the ground reaction under the tire, which this model does not
//! resolve. It is the one non-physical force in the model; its size is
//! bounded ([`LeanSteerParams::balance_torque_limit_nm`]) and logged. Camber
//! steer, the cornering force from tire friction, gravity, the hip mass and
//! the pitch controller are all physical.
//!
//! **Why idealised.** Two resolved riders were built and measured first, and
//! both are kept in git history (`b64b841`, `3f1e287`):
//! - hips only (70 kg on a +-0.25 m slide, speed-scheduled LQR): balances
//!   standing and carves at low speed, but cannot reverse a lean near or above
//!   `v*` -- too little roll authority there;
//! - ankles and hips (a roll hinge at deck level + the slide, two-input LQR):
//!   the ankle servo cannot hold its angle against the body, and a stiffer
//!   one designed by the same linear method is unstable in the full sim.
//!
//! # Signs
//!
//! MuJoCo frame: forward is `-X`, up is `+Z`, so RIGHT is `+Y`.
//! - deck roll `phi > 0`: the deck leans RIGHT (its top moves to `+Y`);
//! - ankle `a > 0`: the body leans right of the deck normal;
//! - yaw rate `> 0`: a LEFT turn (counter-clockwise from above);
//! - `steer > 0`: the rider wants to turn RIGHT;
//! - hip offset `d > 0`: the mass moves RIGHT.

/// Physical and rider-model parameters. See the module doc for the laws.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeanSteerParams {
    /// Rolling radius at the contact, metres (the tire radius).
    pub rolling_radius_m: f32,
    /// Fraction of the rigid-cone curvature the tire achieves (0..1].
    pub camber_efficiency: f32,
    /// Turn-slip relaxation length, metres.
    pub relaxation_m: f32,
    /// Yaw-rate servo stiffness, N*m per rad/s.
    pub yaw_servo_nms: f32,
    /// Largest curvature the rider asks for at full stick, 1/m.
    pub kappa_max_per_m: f32,
    /// 1 = camber thrust at speed (fable-oracle, 2026-10-05): above the
    /// self-steer speed v* the tyre's camber steer is force-limited, so the
    /// bank that steers a turn is the bank that balances it. 0 = the old
    /// kinematic camber steer at every speed.
    pub camber_thrust: f32,
    /// Largest side acceleration the rider asks for with camber thrust, m/s^2.
    pub max_side_accel_m_s2: f32,
    /// Height of the centre of mass above the ground, metres.
    pub com_height_m: f32,
    /// Crown radius of the tire profile, metres.
    pub crown_radius_m: f32,
    /// Rider hip reach, metres (symmetric; matches the model joint).
    pub hip_reach_m: f32,
    /// Share of the hip reach the steady-turn offset may use.
    pub feasible_reach_fraction: f32,
    /// Balance skill: roll stiffness, N*m/rad, damping, N*m*s/rad, limit, N*m.
    pub balance_kp_nm_per_rad: f32,
    pub balance_kd_nms_per_rad: f32,
    pub balance_torque_limit_nm: f32,
    /// How fast the rider eases into a new curvature intent, s.
    pub intent_lag_s: f32,
}

impl Default for LeanSteerParams {
    fn default() -> Self {
        Self {
            rolling_radius_m: 0.1454,
            camber_efficiency: 0.12,
            relaxation_m: 0.15,
            yaw_servo_nms: 150.0,
            kappa_max_per_m: 0.25,
            camber_thrust: 1.0,
            max_side_accel_m_s2: 0.6 * G,
            com_height_m: 0.82,
            crown_radius_m: 0.099,
            hip_reach_m: 0.25,
            feasible_reach_fraction: 0.6,
            // Roll inertia about the contact ~ m h^2 = 52 kg m^2 and gravity
            // destiffens by m g h ~ 630 N*m/rad: kp 2000 leaves ~1400 net,
            // ~5 rad/s, and kd 400 damps it near zeta 0.8.
            balance_kp_nm_per_rad: 2000.0,
            balance_kd_nms_per_rad: 400.0,
            balance_torque_limit_nm: 400.0,
            intent_lag_s: 0.4,
        }
    }
}

impl LeanSteerParams {
    /// Defaults, with overrides from `OVERBOARD_LEAN` (tuning only), for
    /// example `OVERBOARD_LEAN="eta=0.1,intent_lag=0.6"`. Unknown keys and bad
    /// numbers are reported and ignored.
    pub fn from_env() -> Self {
        let mut p = Self::default();
        if let Ok(spec) = std::env::var("OVERBOARD_LEAN") {
            for kv in spec.split(',').filter(|s| !s.trim().is_empty()) {
                let Some((k, v)) = kv.split_once('=') else {
                    continue;
                };
                let Ok(v) = v.trim().parse::<f32>() else {
                    eprintln!("sim-host: OVERBOARD_LEAN: '{kv}' is not key=number");
                    continue;
                };
                match k.trim() {
                    "eta" => p.camber_efficiency = v,
                    "sigma" => p.relaxation_m = v,
                    "yaw_servo" => p.yaw_servo_nms = v,
                    "kappa_max" => p.kappa_max_per_m = v,
                    "camber_thrust" => p.camber_thrust = v,
                    "max_side_accel" => p.max_side_accel_m_s2 = v,
                    "balance_kp" => p.balance_kp_nm_per_rad = v,
                    "balance_kd" => p.balance_kd_nms_per_rad = v,
                    "balance_limit" => p.balance_torque_limit_nm = v,
                    "intent_lag" => p.intent_lag_s = v,
                    other => eprintln!("sim-host: OVERBOARD_LEAN: unknown key '{other}'"),
                }
            }
            eprintln!("sim-host: lean-steer params {p:?}");
        }
        p
    }
}

/// Speed below which the tire relaxation is held at this floor, m/s. Keeps
/// `sigma / |v|` finite at a standstill.
const RELAX_SPEED_FLOOR_M_S: f32 = 0.3;
const G: f32 = 9.81;
/// Below this speed the rider asks for no turn, m/s: a board that is not
/// moving cannot turn, and a lean target held at a standstill only parks the
/// board at that lean (measured with the hip-only rider).
const NO_TURN_BELOW_M_S: f32 = 0.8;
/// Speed from which the full curvature intent is allowed, m/s.
const FULL_TURN_SPEED_M_S: f32 = 3.0;

/// Curvature the tire produces at deck roll `phi`, 1/m, positive = RIGHT turn.
pub fn tire_curvature(p: &LeanSteerParams, roll_rad: f32) -> f32 {
    p.camber_efficiency * roll_rad.tan() / p.rolling_radius_m
}

/// Deck camber that gives curvature `kappa` (1/m, +right).
pub fn roll_reference(p: &LeanSteerParams, kappa: f32) -> f32 {
    (kappa * p.rolling_radius_m / p.camber_efficiency).atan()
}

/// Effective camber efficiency at speed `v`. Below the self-steer speed
/// v* = sqrt(g r_c / eta) a cambered tyre rolls like a cone (eta). Above it
/// the tyre cannot supply the cone's side force: the contact sideslips until
/// the side force is m g tan(phi), so kappa = g tan(phi) / v^2, which is
/// eta * (v*/v)^2. That makes the bank that steers a turn the bank that
/// balances it, as on a motorcycle; without it the hips had to carry about
/// 11 deg of a 13.5 deg lean at 8 m/s, and carving at speed fell at 0.24 g.
pub fn eta_eff(p: &LeanSteerParams, v: f32) -> f32 {
    if p.camber_thrust <= 0.0 {
        return p.camber_efficiency;
    }
    let v_star_sq = G * p.rolling_radius_m / p.camber_efficiency;
    p.camber_efficiency * (v_star_sq / (v * v).max(1e-6)).min(1.0)
}

/// [`tire_curvature`] at speed `v` (camber thrust above v*).
pub fn tire_curvature_at(p: &LeanSteerParams, roll_rad: f32, v: f32) -> f32 {
    eta_eff(p, v) * roll_rad.tan() / p.rolling_radius_m
}

/// [`roll_reference`] at speed `v` (camber thrust above v*).
pub fn roll_reference_at(p: &LeanSteerParams, kappa: f32, v: f32) -> f32 {
    (kappa * p.rolling_radius_m / eta_eff(p, v)).atan()
}

/// What the rider senses each cycle.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RiderObs {
    /// Board roll, rad, + = leaning right, and its rate.
    pub roll_rad: f32,
    pub roll_rate_rad_s: f32,
}

/// What the rider commands each cycle.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RiderCmd {
    /// Hip offset target, m (+ = right).
    pub offset_m: f32,
    /// Balance torque about the board's forward axis, N*m (+ rolls it right).
    pub roll_torque_nm: f32,
}

/// The rider model. Holds the rider's eased intent.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Rider {
    /// The rider's eased curvature intent, 1/m, + = right.
    pub kappa_intent_per_m: f32,
}

/// Hip offset, metres (+right), that balances a steady turn of curvature
/// `kappa` (+right) at speed `v` and board lean `phi`.
pub fn steady_turn_offset(p: &LeanSteerParams, v: f32, kappa: f32, phi: f32) -> f32 {
    let h = p.com_height_m;
    (h * v * v * kappa / G - (h - p.crown_radius_m) * phi.sin()) / phi.cos()
}

impl Rider {
    /// One rider decision.
    pub fn command(
        &mut self,
        p: &LeanSteerParams,
        steer: f32,
        speed_m_s: f32,
        o: &RiderObs,
        dt_s: f32,
    ) -> RiderCmd {
        let speed_frac = ((speed_m_s.abs() - NO_TURN_BELOW_M_S)
            / (FULL_TURN_SPEED_M_S - NO_TURN_BELOW_M_S))
            .clamp(0.0, 1.0);
        let mut kappa_cap = p.kappa_max_per_m;
        if p.camber_thrust > 0.0 {
            // The rider asks for no more than max_side_accel of turn.
            kappa_cap = kappa_cap.min(p.max_side_accel_m_s2 / (speed_m_s * speed_m_s).max(1e-6));
        }
        let goal = steer.clamp(-1.0, 1.0) * kappa_cap * speed_frac;
        let b = (dt_s / p.intent_lag_s.max(dt_s)).min(1.0);
        self.kappa_intent_per_m += b * (goal - self.kappa_intent_per_m);
        // Only ask for a turn the hips can balance with headroom to spare.
        let reference = |k: f32| {
            let phi = roll_reference_at(p, k, speed_m_s);
            (phi, steady_turn_offset(p, speed_m_s, k, phi))
        };
        let budget = p.feasible_reach_fraction * p.hip_reach_m;
        if reference(self.kappa_intent_per_m).1.abs() > budget {
            let (mut lo, mut hi) = (0.0_f32, 1.0_f32);
            for _ in 0..12 {
                let mid = 0.5 * (lo + hi);
                if reference(self.kappa_intent_per_m * mid).1.abs() > budget {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            self.kappa_intent_per_m *= lo;
        }
        let (phi_ref, d_ref) = reference(self.kappa_intent_per_m);
        let tau = (-p.balance_kp_nm_per_rad * (o.roll_rad - phi_ref)
            - p.balance_kd_nms_per_rad * o.roll_rate_rad_s)
            .clamp(-p.balance_torque_limit_nm, p.balance_torque_limit_nm);
        RiderCmd {
            offset_m: d_ref.clamp(-p.hip_reach_m, p.hip_reach_m),
            roll_torque_nm: tau,
        }
    }
}

/// Tire yaw state: the relaxed yaw-rate target.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct TireYaw {
    /// Filtered yaw-rate target, rad/s, + = LEFT (MuJoCo +Z).
    pub yaw_rate_target_rad_s: f32,
}

impl TireYaw {
    /// Advances the relaxation by `dt_s` and returns the yaw torque about
    /// world `+Z`, N*m, that drives the frame's yaw rate to the target.
    ///
    /// `speed_m_s` is signed forward speed: rolling backwards reverses the
    /// turn, as it does for any rolling cone.
    pub fn step(
        &mut self,
        p: &LeanSteerParams,
        speed_m_s: f32,
        roll_rad: f32,
        yaw_rate_meas_rad_s: f32,
        dt_s: f32,
    ) -> f32 {
        // + roll (lean right) at + speed turns right, which is - yaw rate.
        let target = -speed_m_s * tire_curvature_at(p, roll_rad, speed_m_s.abs());
        let tau = p.relaxation_m / speed_m_s.abs().max(RELAX_SPEED_FLOOR_M_S);
        let a = (dt_s / tau).min(1.0);
        self.yaw_rate_target_rad_s += a * (target - self.yaw_rate_target_rad_s);
        -p.yaw_servo_nms * (yaw_rate_meas_rad_s - self.yaw_rate_target_rad_s)
    }
}

/// Heading of the board, rad, + = LEFT, from the frame's rotation matrix.
/// Zero when the body `X` axis lies along world `X`.
pub fn heading_from_xmat(xmat: &[f64; 9]) -> f32 {
    (xmat[3] as f32).atan2(xmat[0] as f32)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn leaning_right_turns_right_and_the_law_inverts() {
        let p = LeanSteerParams::default();
        let phi = 10.0_f32.to_radians();
        let k = tire_curvature(&p, phi);
        assert!(k > 0.0);
        assert!((roll_reference(&p, k) - phi).abs() < 1e-5);
        let radius = 1.0 / k;
        assert!((4.0..10.0).contains(&radius), "radius {radius}");
    }

    #[test]
    fn yaw_relaxes_towards_the_tire_target_with_the_right_sign() {
        let p = LeanSteerParams::default();
        let mut t = TireYaw::default();
        let phi = 8.0_f32.to_radians();
        for _ in 0..2000 {
            t.step(&p, 3.0, phi, 0.0, 0.002);
        }
        let want = -3.0 * tire_curvature(&p, phi);
        assert!((t.yaw_rate_target_rad_s - want).abs() < 1e-3);
        assert!(want < 0.0, "lean right at +speed must turn right (-yaw)");
        assert!(t.step(&p, 3.0, phi, want, 0.002).abs() < 0.2);
    }

    #[test]
    fn upright_and_still_with_no_intent_commands_nothing() {
        let p = LeanSteerParams::default();
        let mut r = Rider::default();
        assert_eq!(
            r.command(&p, 0.0, 3.0, &RiderObs::default(), 0.002),
            RiderCmd::default()
        );
    }

    #[test]
    fn the_intent_is_clamped_to_what_the_hips_can_balance() {
        let p = LeanSteerParams::default();
        for v10 in 0..=95 {
            let v = v10 as f32 / 10.0;
            let mut r = Rider::default();
            for _ in 0..3000 {
                r.command(&p, 1.0, v, &RiderObs::default(), 0.002);
            }
            let k = r.kappa_intent_per_m;
            let d = steady_turn_offset(&p, v, k, roll_reference_at(&p, k, v));
            assert!(
                d.abs() <= p.feasible_reach_fraction * p.hip_reach_m + 1e-3,
                "v {v}: {d}"
            );
        }
    }

    #[test]
    fn camber_thrust_makes_the_steering_bank_the_balancing_bank_at_speed() {
        let p = LeanSteerParams::default();
        // At 8 m/s a 0.35 g turn: the bank is atan(a/g) and the hips trim only.
        let v = 8.0_f32;
        let k = 0.35 * G / (v * v);
        let phi = roll_reference_at(&p, k, v);
        assert!((phi - 0.35_f32.atan()).abs() < 1e-3, "phi {phi}");
        assert!(steady_turn_offset(&p, v, k, phi).abs() < 0.08);
        // Below v* nothing changes.
        assert!((eta_eff(&p, 3.0) - p.camber_efficiency).abs() < 1e-6);
        assert!((roll_reference_at(&p, 0.1, 3.0) - roll_reference(&p, 0.1)).abs() < 1e-6);
    }

    #[test]
    fn the_balance_torque_rolls_the_board_towards_the_reference() {
        let p = LeanSteerParams::default();
        let mut r = Rider::default();
        let o = RiderObs {
            roll_rad: 0.1,
            ..Default::default()
        };
        assert!(r.command(&p, 0.0, 3.0, &o, 0.002).roll_torque_nm < 0.0);
    }

    #[test]
    fn heading_is_zero_at_identity_and_positive_for_a_left_rotation() {
        let id = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0];
        assert_eq!(heading_from_xmat(&id), 0.0);
        let a = 0.3_f64;
        let rz = [a.cos(), -a.sin(), 0.0, a.sin(), a.cos(), 0.0, 0.0, 0.0, 1.0];
        assert!((heading_from_xmat(&rz) - 0.3).abs() < 1e-6);
    }
}
