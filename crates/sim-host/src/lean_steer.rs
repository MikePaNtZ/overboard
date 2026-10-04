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
//! than the rigid cone. [`LeanSteerParams::camber_efficiency`] (`eta`)
//! carries that loss:
//!
//! ```text
//! kappa_tire(phi) = eta * tan(phi) / r_c
//! ```
//!
//! The tire does not reach that curvature at once. Its turn slip relaxes over
//! a distance `sigma`, so the target yaw rate is filtered with time constant
//! `sigma / |v|`. The resulting yaw rate is imposed through a stiff yaw
//! torque on the frame (a model of the contact patch's turn-slip moment), not
//! by rotating the pose. MuJoCo's contact friction then supplies the
//! centripetal force, and that force acts at the contact patch, below the
//! centre of mass. So a turn tries to roll the board OUT of the turn, and the
//! rider must lean IN. That is the coupling that makes the board bank.
//!
//! # Rider side (the human)
//!
//! The steer stick is the rider's INTENT, a target curvature
//! `kappa_cmd = steer * kappa_max`. The rider inverts the tire law for the
//! roll that gives it, and moves the 70 kg rider mass sideways to hold that
//! roll:
//!
//! ```text
//! phi_ref = atan(kappa_cmd * r_c / eta)
//! d_ref   = (h * v^2 * kappa_cmd / g - (h - rho) * sin(phi_ref)) / cos(phi_ref)
//! d_cmd   = d_ref - K . (x - x_ref)                          (metres, +right)
//! x       = [phi, phi_dot, d, d_dot, v_lat, a]
//! ```
//!
//! `d_ref` is the steady-turn balance: the mass offset that puts the centre
//! of mass where gravity balances the centripetal load at lean `phi_ref`.
//! `a` is the rider's own servo state (the lagged command).
//!
//! **Why state feedback, not PD on roll.** Moving the mass right first pushes
//! the deck LEFT (reaction at the slide), and only then does gravity act on
//! the shifted centre of mass. PD on roll alone, with the servo lag, rings up
//! and falls (measured).
//!
//! **Why scheduled on speed.** `K(v)` comes from an LQR on the roll and yaw
//! subsystem of the MuJoCo model linearised at forward speed `v`, with this
//! module's tire yaw law in the loop (`Q = diag(400 roll, 5 d, 10 roll_rate,
//! 0.1 d_dot, 0.5 yaw_rate)`, `R = 1000`). Standing, the roll mode diverges
//! at 2.9 1/s. The camber coupling slows it as speed rises; above
//! `v* = sqrt(g r_c / eta)` (3.4 m/s) an oscillatory "weave" mode grows
//! instead, and the roll gain changes sign: the rider stops catching a lean with the mass and lets
//! the camber turn's centripetal load right the board, which is how
//! counter-steer works on a bicycle. `v_lat` is the board's sideways velocity
//! in its own frame (the slip). See [`RIDER_GAIN_SCHEDULE`].
//!
//! # Signs
//!
//! MuJoCo frame: forward is `-X`, up is `+Z`, so RIGHT is `+Y`.
//! - roll `phi > 0`: the board leans RIGHT (its top moves to `+Y`);
//! - yaw rate `> 0`: a LEFT turn (counter-clockwise from above);
//! - `steer > 0`: the rider wants to turn RIGHT;
//! - rider offset `d > 0`: the mass moves RIGHT.

/// Physical and rider-model parameters. See the module doc for the laws.
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct LeanSteerParams {
    /// Rolling radius at the contact, metres (the tire radius).
    pub rolling_radius_m: f32,
    /// Crown radius of the tire profile, metres.
    pub crown_radius_m: f32,
    /// Fraction of the rigid-cone curvature the tire achieves (0..1].
    pub camber_efficiency: f32,
    /// Turn-slip relaxation length, metres.
    pub relaxation_m: f32,
    /// Yaw-rate servo stiffness, N*m per rad/s.
    pub yaw_servo_nms: f32,
    /// Largest curvature the rider asks for at full stick, 1/m.
    pub kappa_max_per_m: f32,
    /// Height of the centre of mass above the ground, metres.
    pub com_height_m: f32,
    /// Scale on the scheduled rider gains (1.0 = as designed; tuning only).
    pub rider_gain_scale: f32,
    /// Lag of the rider's lateral servo, s (matches the model actuator).
    pub rider_servo_lag_s: f32,
    /// Rider's slow roll trim, metres per (rad*s). Off (0) by default: both
    /// signs were tried and both wind up and park the board at a lean; the
    /// standstill lean it was meant to fix is avoided by asking for no lean
    /// below `NO_LEAN_BELOW_M_S` instead.
    pub rider_roll_ki: f32,
    /// How fast the rider eases into a new curvature intent, s. A step in
    /// lean target overshoots and the reach saturates (measured: a half-stick
    /// step at 1.8 m/s rolled the board past 20 deg and it fell).
    pub intent_lag_s: f32,
    /// Rider lateral reach, metres (symmetric).
    pub rider_reach_m: f32,
    /// Largest roll the rider will ask for, rad.
    pub max_roll_ref_rad: f32,
}

impl Default for LeanSteerParams {
    fn default() -> Self {
        Self {
            rolling_radius_m: 0.1454,
            crown_radius_m: 0.099,
            camber_efficiency: 0.12,
            relaxation_m: 0.15,
            yaw_servo_nms: 150.0,
            kappa_max_per_m: 0.25,
            com_height_m: 0.82,
            rider_gain_scale: 1.0,
            rider_servo_lag_s: 0.05,
            rider_roll_ki: 0.0,
            intent_lag_s: 0.4,
            rider_reach_m: 0.25,
            max_roll_ref_rad: 25.0_f32.to_radians(),
        }
    }
}

impl LeanSteerParams {
    /// Defaults, with overrides from `OVERBOARD_LEAN` (tuning only), for
    /// example `OVERBOARD_LEAN="rider_kp=0.6,rider_kd=0.3"`. Unknown keys and
    /// bad numbers are reported and ignored.
    pub fn from_env() -> Self {
        let mut p = Self::default();
        if let Ok(spec) = std::env::var("OVERBOARD_LEAN") {
            for kv in spec.split(',').filter(|s| !s.trim().is_empty()) {
                let Some((k, v)) = kv.split_once('=') else { continue };
                let Ok(v) = v.trim().parse::<f32>() else {
                    eprintln!("sim-host: OVERBOARD_LEAN: '{kv}' is not key=number");
                    continue;
                };
                match k.trim() {
                    "eta" => p.camber_efficiency = v,
                    "sigma" => p.relaxation_m = v,
                    "yaw_servo" => p.yaw_servo_nms = v,
                    "kappa_max" => p.kappa_max_per_m = v,
                    "com_h" => p.com_height_m = v,
                    "gain_scale" => p.rider_gain_scale = v,
                    "intent_lag" => p.intent_lag_s = v,
                    "roll_ki" => p.rider_roll_ki = v,
                    "reach" => p.rider_reach_m = v,
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
/// Anti-windup bound on the rider's roll trim integral, rad*s.
const ROLL_TRIM_LIMIT_RAD_S: f32 = 0.05;

fn e_roll_for_trim(roll_rad: f32, phi_ref: f32) -> f32 {
    roll_rad - phi_ref
}

/// Speed from which the rider will commit to the full lean, m/s.
const FULL_LEAN_SPEED_M_S: f32 = 3.0;
/// Share of the rider's reach the steady-turn offset may use; the rest is
/// kept for the balance feedback.
pub const FEASIBLE_REACH_FRACTION: f32 = 0.6;

/// Mass offset, metres (+right), that balances a steady turn of curvature
/// `kappa` (+right) at speed `v` and board lean `phi`: it puts the centre of
/// mass where gravity balances the centripetal load.
pub fn steady_turn_offset(p: &LeanSteerParams, v: f32, kappa: f32, phi: f32) -> f32 {
    let h = p.com_height_m;
    (h * v * v * kappa / G - (h - p.crown_radius_m) * phi.sin()) / phi.cos()
}

/// Below this speed the rider asks for no lean at all, m/s. A lean target
/// held at a standstill parks the board at the wrong lean: there, the roll and
/// offset gains nearly cancel (measured: 11.7 deg held against 3.75 deg).
const NO_LEAN_BELOW_M_S: f32 = 0.8;
const G: f32 = 9.81;

/// Curvature the tire produces at roll `phi`, 1/m, positive = RIGHT turn.
pub fn tire_curvature(p: &LeanSteerParams, roll_rad: f32) -> f32 {
    p.camber_efficiency * roll_rad.tan() / p.rolling_radius_m
}

/// Roll the rider aims for to get `kappa_cmd` (1/m, +right), clamped.
pub fn roll_reference(p: &LeanSteerParams, kappa_cmd: f32) -> f32 {
    (kappa_cmd * p.rolling_radius_m / p.camber_efficiency)
        .atan()
        .clamp(-p.max_roll_ref_rad, p.max_roll_ref_rad)
}

/// Rider state-feedback gains `K(v)` on `[phi, phi_dot, d, d_dot, v_lat, a,
/// r, r_t]` (rad, rad/s, m, m/s, m/s, m, rad/s, rad/s; `r` = yaw rate and
/// `r_t` = the tire's relaxed yaw-rate target, both + = left), output metres,
/// against forward speed `|v|`
/// (m/s). Linear interpolation between rows; held at the ends. Generated by
/// the speed-scheduled LQR described in the module doc.
pub const RIDER_GAIN_SCHEDULE: &[(f32, [f32; 8])] = &[
    (0.0, [15.7260, 4.6570, 13.7311, 5.1887, 5.2585, 3.2123, -0.0000, -0.0000]),
    (0.5, [15.0122, 4.4700, 13.0593, 4.9872, 5.0565, 3.2117, 0.0956, 0.3802]),
    (1.0, [12.4681, 3.8035, 10.6653, 4.2688, 4.3364, 3.2093, 0.1646, 0.4370]),
    (1.5, [9.1517, 2.9323, 7.5391, 3.3298, 3.3950, 3.2052, 0.1949, 0.3896]),
    (2.0, [6.1314, 2.1344, 4.6819, 2.4696, 2.5326, 3.1998, 0.1962, 0.3156]),
    (2.5, [3.8011, 1.5138, 2.4679, 1.8005, 1.8616, 3.1942, 0.1834, 0.2476]),
    (3.0, [2.1453, 1.0703, 0.8943, 1.3222, 1.3820, 3.1901, 0.1670, 0.1948]),
    (3.5, [1.0121, 0.7697, -0.1652, 0.9983, 1.0571, 3.1899, 0.1529, 0.1573]),
    (4.0, [0.2296, 0.5709, -0.8617, 0.7845, 0.8427, 3.1951, 0.1431, 0.1319]),
    (5.0, [-0.7929, 0.3425, -1.6590, 0.5405, 0.5981, 3.2188, 0.1338, 0.1023]),
    (6.0, [-1.5036, 0.2165, -2.1009, 0.4076, 0.4652, 3.2520, 0.1313, 0.0860]),
    (7.0, [-2.0827, 0.1339, -2.3935, 0.3220, 0.3798, 3.2888, 0.1313, 0.0754]),
    (8.0, [-2.5914, 0.0740, -2.6092, 0.2608, 0.3189, 3.3269, 0.1324, 0.0678]),
    (9.5, [-3.2772, 0.0077, -2.8523, 0.1947, 0.2533, 3.3845, 0.1347, 0.0596]),
];

/// `K(|v|)` from [`RIDER_GAIN_SCHEDULE`].
pub fn rider_gains(speed_m_s: f32) -> [f32; 8] {
    let v = speed_m_s.abs();
    let t = RIDER_GAIN_SCHEDULE;
    if v <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        let ((v0, k0), (v1, k1)) = (w[0], w[1]);
        if v <= v1 {
            let a = (v - v0) / (v1 - v0);
            let mut k = [0.0; 8];
            for i in 0..8 {
                k[i] = k0[i] + a * (k1[i] - k0[i]);
            }
            return k;
        }
    }
    t[t.len() - 1].1
}

/// What the rider senses each cycle.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RiderObs {
    /// Board roll, rad, + = leaning right.
    pub roll_rad: f32,
    /// Board roll rate, rad/s.
    pub roll_rate_rad_s: f32,
    /// Rider mass offset on its slide, m, + = right.
    pub offset_m: f32,
    /// Rate of that offset, m/s.
    pub offset_rate_m_s: f32,
    /// Board sideways velocity in its own heading frame, m/s, + = right.
    pub lateral_velocity_m_s: f32,
    /// Yaw rate, rad/s, + = left.
    pub yaw_rate_rad_s: f32,
    /// The tire's relaxed yaw-rate target ([`TireYaw`]), rad/s, + = left.
    pub tire_yaw_target_rad_s: f32,
}

/// The rider model. Holds the rider's own servo state.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Rider {
    /// Lagged command, m: what the slide servo is currently aiming at.
    pub servo_state_m: f32,
    /// The rider's eased curvature intent, 1/m, + = right.
    pub kappa_intent_per_m: f32,
    /// Integrated roll error, rad*s (the rider's slow trim).
    pub roll_error_integral: f32,
}

impl Rider {
    /// Lateral mass offset command, metres, +right.
    pub fn command(
        &mut self,
        p: &LeanSteerParams,
        steer: f32,
        speed_m_s: f32,
        o: &RiderObs,
        dt_s: f32,
    ) -> f32 {
        let kappa_goal = steer.clamp(-1.0, 1.0) * p.kappa_max_per_m;
        let b = (dt_s / p.intent_lag_s.max(dt_s)).min(1.0);
        self.kappa_intent_per_m += b * (kappa_goal - self.kappa_intent_per_m);
        // Nobody carves at walking pace: the lean the rider will commit to
        // grows with speed, full from `FULL_LEAN_SPEED_M_S`. Measured: a
        // half-stick intent at 0.4 m/s asked for a lean no reach can hold.
        let lean_limit = p.max_roll_ref_rad
            * ((speed_m_s.abs() - NO_LEAN_BELOW_M_S) / (FULL_LEAN_SPEED_M_S - NO_LEAN_BELOW_M_S))
                .clamp(0.0, 1.0);
        // Feasibility: only ask for a turn the mass can hold with headroom
        // left for the feedback. Above v* the steady-turn offset grows with
        // v^2; a full-stick intent at 4.8 m/s needs d_ref = 0.28 m, past the
        // stop. With the mass pinned on a stop the loop loses its damping and
        // a lean reversal swings until the board falls (measured). Scale the
        // intent itself down so the intent lag stops pushing an infeasible goal.
        let reference = |kappa: f32| -> (f32, f32, f32) {
            let phi = roll_reference(p, kappa).clamp(-lean_limit, lean_limit);
            let k = tire_curvature(p, phi);
            (phi, k, steady_turn_offset(p, speed_m_s, k, phi))
        };
        let budget = FEASIBLE_REACH_FRACTION * p.rider_reach_m;
        if reference(self.kappa_intent_per_m).2.abs() > budget {
            let (mut lo, mut hi) = (0.0_f32, 1.0_f32);
            for _ in 0..12 {
                let mid = 0.5 * (lo + hi);
                if reference(self.kappa_intent_per_m * mid).2.abs() > budget {
                    hi = mid;
                } else {
                    lo = mid;
                }
            }
            self.kappa_intent_per_m *= lo;
        }
        let (phi_ref, kappa_cmd, d_ref) = reference(self.kappa_intent_per_m);
        let e = [
            o.roll_rad - phi_ref,
            o.roll_rate_rad_s,
            o.offset_m - d_ref,
            o.offset_rate_m_s,
            o.lateral_velocity_m_s,
            self.servo_state_m - d_ref,
            // In a steady turn of curvature kappa (+right) at speed v the yaw
            // rate is -v*kappa (+left); both yaw states are measured from it.
            o.yaw_rate_rad_s + speed_m_s * kappa_cmd,
            o.tire_yaw_target_rad_s + speed_m_s * kappa_cmd,
        ];
        let k = rider_gains(speed_m_s).map(|g| g * p.rider_gain_scale);
        // Slow trim on the roll error. At a standstill the roll and offset
        // gains nearly cancel, so a small model error in the centre-of-mass
        // height parks the board at the wrong lean (measured: 11.7 deg held
        // against a 3.75 deg target). A rider fixes that by feel, slowly.
        self.roll_error_integral = (self.roll_error_integral + e_roll_for_trim(o.roll_rad, phi_ref) * dt_s)
            .clamp(-ROLL_TRIM_LIMIT_RAD_S, ROLL_TRIM_LIMIT_RAD_S);
        let u: f32 = d_ref
            - k.iter().zip(e.iter()).map(|(k, e)| k * e).sum::<f32>()
            // `+`: in balance a mass offset `d` holds a lean of about
            // `-d / (h - rho)`, so the slow trim moves the mass TOWARDS the
            // lean to stand the board up -- the opposite of the fast reaction.
            + p.rider_roll_ki * self.roll_error_integral;
        let u = u.clamp(-p.rider_reach_m, p.rider_reach_m);
        let a = (dt_s / p.rider_servo_lag_s).min(1.0);
        self.servo_state_m += a * (u - self.servo_state_m);
        u
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
        let target = -speed_m_s * tire_curvature(p, roll_rad);
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
        // 10 deg of lean gives a turn of a few metres, not a pivot.
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
        // At the target, the servo applies no torque.
        assert!(t.step(&p, 3.0, phi, want, 0.002).abs() < 0.2);
    }

    #[test]
    fn rider_corrects_towards_the_high_side_and_respects_reach() {
        let p = LeanSteerParams::default();
        let mut r = Rider::default();
        // Leaning right with no intent: the mass goes LEFT (the high side).
        let o = RiderObs { roll_rad: 0.02, ..Default::default() };
        assert!(r.command(&p, 0.0, 3.0, &o, 0.002) < 0.0);
        // Upright and still with no intent: no offset.
        let mut r = Rider::default();
        assert_eq!(r.command(&p, 0.0, 3.0, &RiderObs::default(), 0.002), 0.0);
        // Anything large saturates at the reach.
        let o = RiderObs { roll_rad: -0.5, roll_rate_rad_s: -3.0, ..Default::default() };
        assert_eq!(r.command(&p, 0.0, 3.0, &o, 0.002), p.rider_reach_m);
    }

    #[test]
    fn a_right_turn_needs_the_mass_inside_at_speed() {
        let p = LeanSteerParams::default();
        let mut r = Rider::default();
        // Settled at the reference roll for a half-stick right turn, the
        // command is the steady-turn offset: inside (right) at 4 m/s.
        let phi = roll_reference(&p, 0.5 * p.kappa_max_per_m);
        let o = RiderObs { roll_rad: phi, ..Default::default() };
        let mut u = 0.0;
        for _ in 0..500 {
            u = r.command(&p, 0.5, 4.0, &o, 0.002);
        }
        assert!(u > 0.0, "u {u}");
    }

    #[test]
    fn gain_schedule_interpolates_and_holds_at_the_ends() {
        assert_eq!(rider_gains(0.0), RIDER_GAIN_SCHEDULE[0].1);
        assert_eq!(rider_gains(-20.0), RIDER_GAIN_SCHEDULE[RIDER_GAIN_SCHEDULE.len() - 1].1);
        let k = rider_gains(0.25);
        let (a, b) = (RIDER_GAIN_SCHEDULE[0].1[0], RIDER_GAIN_SCHEDULE[1].1[0]);
        assert!((k[0] - (a + b) / 2.0).abs() < 1e-3);
    }

    #[test]
    fn the_turn_intent_is_clamped_to_what_the_reach_can_hold() {
        let p = LeanSteerParams::default();
        for v10 in 0..=60 {
            let v = v10 as f32 / 10.0;
            let mut r = Rider::default();
            let o = RiderObs::default();
            for _ in 0..2000 {
                r.command(&p, 1.0, v, &o, 0.002);
            }
            let k = r.kappa_intent_per_m;
            let lean_limit = p.max_roll_ref_rad
                * ((v - NO_LEAN_BELOW_M_S) / (FULL_LEAN_SPEED_M_S - NO_LEAN_BELOW_M_S)).clamp(0.0, 1.0);
            let phi = roll_reference(&p, k).clamp(-lean_limit, lean_limit);
            let d = steady_turn_offset(&p, v, tire_curvature(&p, phi), phi);
            assert!(d.abs() <= FEASIBLE_REACH_FRACTION * p.rider_reach_m + 1e-3, "v {v}: d_ref {d}");
        }
        // At 2.3 m/s full stick is feasible and is not reduced.
        let mut r = Rider::default();
        for _ in 0..2000 {
            r.command(&p, 1.0, 2.3, &RiderObs::default(), 0.002);
        }
        assert!((r.kappa_intent_per_m - p.kappa_max_per_m).abs() < 1e-3, "{}", r.kappa_intent_per_m);
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
