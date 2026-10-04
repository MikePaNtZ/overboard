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
//! # Rider side (the human): feet and hips
//!
//! The rider has two inputs, as a real one does:
//! - **ankles** (`a`, a roll hinge at deck level, torque-limited to what heel
//!   and toe pressure can apply): tilt the DECK relative to the body. Deck
//!   camber is what steers, so this is the fast steering input;
//! - **hips** (`d`, the lateral slide of the 70 kg mass): move the centre of
//!   mass for balance.
//!
//! The steer stick is the rider's INTENT, a target curvature
//! `kappa = steer * kappa_max`, eased with a lag. From it:
//!
//! ```text
//! phi_ref   = atan(kappa * r_c / eta)          deck camber that steers kappa
//! theta_ref = atan(v^2 * kappa / g)            body lean that balances the turn
//! a_ref     = theta_ref - phi_ref,  d_ref = 0
//! [a_cmd, d_cmd] = [a_ref, 0] - K(v) . (x - x_ref)
//! x = [phi, a, d, phi_dot, a_dot, d_dot, v_lat, r, r_t, a_servo, d_servo]
//! ```
//!
//! `K(v)` is a speed-scheduled discrete LQR on the MuJoCo model linearised at
//! forward speed `v` (finite differences), with this module's tire law and
//! relaxation in the loop ([`RIDER_GAIN_SCHEDULE`]). Closed-loop damping is
//! >= 0.61 at every scheduled speed from 0 to 9.5 m/s.
//!
//! **Why two inputs.** A first rider had only the hip slide. It balanced at
//! a standstill and carved at low speed, but could not reverse a lean near or
//! above `v*`: the mass shift has too little roll authority there, and the
//! lean it asked for needed more reach than a body has (measured, and
//! confirmed by an independent review). Heel/toe pressure on the deck is the
//! input a real rider steers with.
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
    /// Largest body lean the rider will commit to, rad.
    pub max_body_lean_rad: f32,
    /// Rider ankle range, rad (symmetric; matches the model joint).
    pub ankle_range_rad: f32,
    /// Rider hip reach, metres (symmetric; matches the model joint).
    pub hip_reach_m: f32,
    /// Scale on the scheduled rider gains (1.0 = as designed; tuning only).
    pub rider_gain_scale: f32,
    /// How fast the rider eases into a new curvature intent, s.
    pub intent_lag_s: f32,
    /// Lags of the rider's ankle and hip servos, s (match the actuators).
    pub ankle_servo_lag_s: f32,
    pub hip_servo_lag_s: f32,
}

impl Default for LeanSteerParams {
    fn default() -> Self {
        Self {
            rolling_radius_m: 0.1454,
            camber_efficiency: 0.12,
            relaxation_m: 0.15,
            yaw_servo_nms: 150.0,
            kappa_max_per_m: 0.25,
            max_body_lean_rad: 35.0_f32.to_radians(),
            ankle_range_rad: 0.6,
            hip_reach_m: 0.25,
            rider_gain_scale: 1.0,
            intent_lag_s: 0.4,
            ankle_servo_lag_s: 0.03,
            hip_servo_lag_s: 0.05,
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
                    "gain_scale" => p.rider_gain_scale = v,
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

/// Number of rider feedback states. See the module doc for the order.
pub const RIDER_STATES: usize = 11;

/// Rider state-feedback gains `K(v)`, two rows (ankle rad, hip m) on the
/// state vector in the module doc, against forward speed `|v|` (m/s). Linear
/// interpolation between rows; held at the ends. Generated by the
/// speed-scheduled LQR described in the module doc (`Q` on deck camber 400,
/// body lean 400, hip 5, their rates 40/40/0.1, yaw rate 0.5; `R` = 50 on
/// the ankle, 1000 on the hip). The first row is linearised at 0.05 m/s: at
/// exactly zero the heading is uncontrollable and the Riccati solve fails.
pub const RIDER_GAIN_SCHEDULE: &[(f32, [[f32; RIDER_STATES]; 2])] = &[
    (0.1, [[18.3266, 14.6296, 15.9508, 5.4309, 5.0649, 5.9173, 5.9274, 0.0117, 0.0623, 2.0576, 3.4535], [14.2103, 11.8756, 12.6926, 4.2060, 3.8995, 4.6178, 4.6896, 0.0088, 0.0433, 0.1027, 2.3608]]),
    (0.5, [[19.8593, 15.9969, 17.6251, 5.9202, 5.5149, 6.4453, 6.4610, 0.1249, 0.5130, 2.0627, 3.5004], [13.2587, 11.1454, 11.7784, 3.9453, 3.6604, 4.3375, 4.4064, 0.0834, 0.3324, 0.1042, 2.3525]]),
    (1.0, [[22.4424, 18.6147, 20.7797, 6.8581, 6.3792, 7.4600, 7.4867, 0.2872, 0.7747, 2.0832, 3.6343], [9.9199, 8.5595, 8.5458, 3.0216, 2.8132, 3.3442, 3.4029, 0.1296, 0.3439, 0.1082, 2.3190]]),
    (1.5, [[20.3961, 17.7061, 19.5200, 6.5363, 6.0883, 7.1208, 7.1446, 0.4107, 0.8286, 2.1114, 3.7331], [6.1315, 5.5806, 4.8320, 1.9573, 1.8367, 2.1992, 2.2461, 0.1300, 0.2596, 0.1111, 2.2733]]),
    (2.0, [[15.3889, 14.2946, 15.1803, 5.3189, 4.9746, 5.8163, 5.8271, 0.4488, 0.7259, 2.1350, 3.7677], [3.4698, 3.4586, 2.1938, 1.1988, 1.1407, 1.3830, 1.4214, 0.1122, 0.1804, 0.1119, 2.2365]]),
    (2.5, [[10.5925, 10.8699, 10.8669, 4.0951, 3.8538, 4.5035, 4.5009, 0.4372, 0.5918, 2.1515, 3.7760], [1.9026, 2.2000, 0.6330, 0.7487, 0.7275, 0.8986, 0.9319, 0.0950, 0.1284, 0.1120, 2.2137]]),
    (3.0, [[6.9404, 8.2487, 7.5818, 3.1573, 2.9948, 3.4975, 3.4845, 0.4108, 0.4792, 2.1633, 3.7835], [0.9914, 1.4719, -0.2680, 0.4881, 0.4883, 0.6182, 0.6486, 0.0825, 0.0967, 0.1121, 2.2017]]),
    (3.5, [[4.3059, 6.3917, 5.2634, 2.4920, 2.3855, 2.7844, 2.7639, 0.3852, 0.3954, 2.1728, 3.7986], [0.4316, 1.0349, -0.8080, 0.3316, 0.3447, 0.4499, 0.4785, 0.0742, 0.0770, 0.1125, 2.1966]]),
    (4.0, [[2.3833, 5.0889, 3.6422, 2.0244, 1.9576, 2.2840, 2.2582, 0.3649, 0.3348, 2.1815, 3.8226], [0.0595, 0.7578, -1.1505, 0.2322, 0.2535, 0.3432, 0.3707, 0.0689, 0.0643, 0.1133, 2.1959]]),
    (5.0, [[-0.2281, 3.4767, 1.6447, 1.4441, 1.4273, 1.6655, 1.6330, 0.3396, 0.2577, 2.1986, 3.8912], [-0.4164, 0.4387, -1.5459, 0.1176, 0.1486, 0.2206, 0.2468, 0.0633, 0.0495, 0.1154, 2.2021]]),
    (6.0, [[-2.0019, 2.5602, 0.5156, 1.1129, 1.1258, 1.3154, 1.2789, 0.3284, 0.2130, 2.2162, 3.9772], [-0.7343, 0.2629, -1.7658, 0.0543, 0.0910, 0.1534, 0.1789, 0.0611, 0.0412, 0.1181, 2.2134]]),
    (7.0, [[-3.3825, 1.9799, -0.1962, 0.9024, 0.9352, 1.0954, 1.0563, 0.3252, 0.1845, 2.2344, 4.0719], [-0.9843, 0.1499, -1.9091, 0.0136, 0.0541, 0.1107, 0.1356, 0.0603, 0.0358, 0.1211, 2.2268]]),
    (8.0, [[-4.5563, 1.5795, -0.6861, 0.7566, 0.8041, 0.9452, 0.9043, 0.3266, 0.1649, 2.2529, 4.1709], [-1.1991, 0.0693, -2.0133, -0.0155, 0.0279, 0.0804, 0.1051, 0.0601, 0.0320, 0.1243, 2.2412]]),
    (9.5, [[-6.1071, 1.1639, -1.1954, 0.6046, 0.6689, 0.7920, 0.7491, 0.3335, 0.1449, 2.2810, 4.3224], [-1.4837, -0.0184, -2.1296, -0.0471, -0.0004, 0.0479, 0.0722, 0.0602, 0.0279, 0.1291, 2.2634]]),
];

/// `K(|v|)` from [`RIDER_GAIN_SCHEDULE`].
pub fn rider_gains(speed_m_s: f32) -> [[f32; RIDER_STATES]; 2] {
    let v = speed_m_s.abs();
    let t = RIDER_GAIN_SCHEDULE;
    if v <= t[0].0 {
        return t[0].1;
    }
    for w in t.windows(2) {
        let ((v0, k0), (v1, k1)) = (w[0], w[1]);
        if v <= v1 {
            let a = (v - v0) / (v1 - v0);
            let mut k = [[0.0; RIDER_STATES]; 2];
            for r in 0..2 {
                for i in 0..RIDER_STATES {
                    k[r][i] = k0[r][i] + a * (k1[r][i] - k0[r][i]);
                }
            }
            return k;
        }
    }
    t[t.len() - 1].1
}

/// What the rider senses each cycle.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RiderObs {
    /// Deck roll, rad, + = leaning right.
    pub roll_rad: f32,
    pub roll_rate_rad_s: f32,
    /// Ankle angle (body relative to deck), rad, + = body right, and rate.
    pub ankle_rad: f32,
    pub ankle_rate_rad_s: f32,
    /// Hip offset on its slide, m, + = right, and rate.
    pub offset_m: f32,
    pub offset_rate_m_s: f32,
    /// Board sideways velocity in its own heading frame, m/s, + = right.
    pub lateral_velocity_m_s: f32,
    /// Yaw rate, rad/s, + = left.
    pub yaw_rate_rad_s: f32,
    /// The tire's relaxed yaw-rate target ([`TireYaw`]), rad/s, + = left.
    pub tire_yaw_target_rad_s: f32,
}

/// What the rider commands each cycle.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct RiderCmd {
    /// Ankle target, rad (body relative to deck, + = body right).
    pub ankle_rad: f32,
    /// Hip offset target, m (+ = right).
    pub offset_m: f32,
}

/// The rider model. Holds the rider's intent and servo states.
#[derive(Debug, Clone, Copy, Default, PartialEq)]
pub struct Rider {
    /// The rider's eased curvature intent, 1/m, + = right.
    pub kappa_intent_per_m: f32,
    /// Lagged commands: what each servo is currently aiming at.
    pub ankle_servo_rad: f32,
    pub hip_servo_m: f32,
}

/// Steady-turn references for curvature `kappa` at speed `v`:
/// `(phi_ref, theta_ref)`, deck camber and body lean, rad.
pub fn turn_reference(p: &LeanSteerParams, v: f32, kappa: f32) -> (f32, f32) {
    (roll_reference(p, kappa), (v * v * kappa / G).atan())
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
        // Intent: eased, zero at a standstill, and clamped to a turn the
        // body can lean into (theta_ref <= max_body_lean) and the ankles can
        // reach (|a_ref| within 60 % of the range, the rest is for feedback).
        let speed_frac = ((speed_m_s.abs() - NO_TURN_BELOW_M_S)
            / (FULL_TURN_SPEED_M_S - NO_TURN_BELOW_M_S))
            .clamp(0.0, 1.0);
        let goal = steer.clamp(-1.0, 1.0) * p.kappa_max_per_m * speed_frac;
        let b = (dt_s / p.intent_lag_s.max(dt_s)).min(1.0);
        self.kappa_intent_per_m += b * (goal - self.kappa_intent_per_m);
        let feasible = |k: f32| {
            let (phi, theta) = turn_reference(p, speed_m_s, k);
            theta.abs() <= p.max_body_lean_rad && (theta - phi).abs() <= 0.6 * p.ankle_range_rad
        };
        if !feasible(self.kappa_intent_per_m) {
            let (mut lo, mut hi) = (0.0_f32, 1.0_f32);
            for _ in 0..12 {
                let mid = 0.5 * (lo + hi);
                if feasible(self.kappa_intent_per_m * mid) {
                    lo = mid;
                } else {
                    hi = mid;
                }
            }
            self.kappa_intent_per_m *= lo;
        }
        let kappa = self.kappa_intent_per_m;
        let (phi_ref, theta_ref) = turn_reference(p, speed_m_s, kappa);
        let a_ref = theta_ref - phi_ref;
        let r_ref = -speed_m_s * kappa;

        let e = [
            o.roll_rad - phi_ref,
            o.ankle_rad - a_ref,
            o.offset_m,
            o.roll_rate_rad_s,
            o.ankle_rate_rad_s,
            o.offset_rate_m_s,
            o.lateral_velocity_m_s,
            o.yaw_rate_rad_s - r_ref,
            o.tire_yaw_target_rad_s - r_ref,
            self.ankle_servo_rad - a_ref,
            self.hip_servo_m,
        ];
        let k = rider_gains(speed_m_s);
        let dot = |row: &[f32; RIDER_STATES]| -> f32 {
            row.iter().zip(e.iter()).map(|(k, e)| k * e).sum::<f32>() * p.rider_gain_scale
        };
        let ankle = (a_ref - dot(&k[0])).clamp(-p.ankle_range_rad, p.ankle_range_rad);
        let hip = (-dot(&k[1])).clamp(-p.hip_reach_m, p.hip_reach_m);
        self.ankle_servo_rad += (dt_s / p.ankle_servo_lag_s).min(1.0) * (ankle - self.ankle_servo_rad);
        self.hip_servo_m += (dt_s / p.hip_servo_lag_s).min(1.0) * (hip - self.hip_servo_m);
        RiderCmd { ankle_rad: ankle, offset_m: hip }
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
        assert_eq!(r.command(&p, 0.0, 3.0, &RiderObs::default(), 0.002), RiderCmd::default());
    }

    #[test]
    fn a_right_turn_leans_the_body_further_than_the_deck_at_speed() {
        let p = LeanSteerParams::default();
        let (phi, theta) = turn_reference(&p, 6.0, 0.1);
        assert!(phi > 0.0 && theta > phi, "phi {phi} theta {theta}");
        let (phi, theta) = turn_reference(&p, 1.5, 0.1);
        assert!(theta < phi, "below v* the body leans less than the deck");
    }

    #[test]
    fn the_intent_is_clamped_to_a_lean_the_body_and_ankles_can_hold() {
        let p = LeanSteerParams::default();
        for v10 in 0..=95 {
            let v = v10 as f32 / 10.0;
            let mut r = Rider::default();
            for _ in 0..3000 {
                r.command(&p, 1.0, v, &RiderObs::default(), 0.002);
            }
            let (phi, theta) = turn_reference(&p, v, r.kappa_intent_per_m);
            assert!(theta.abs() <= p.max_body_lean_rad + 1e-3, "v {v} theta {theta}");
            assert!((theta - phi).abs() <= 0.6 * p.ankle_range_rad + 1e-3, "v {v}");
        }
    }

    #[test]
    fn gain_schedule_interpolates_and_holds_at_the_ends() {
        let t = RIDER_GAIN_SCHEDULE;
        assert_eq!(rider_gains(0.0), t[0].1);
        assert_eq!(rider_gains(-20.0), t[t.len() - 1].1);
        let v = 0.5 * (t[0].0 + t[1].0);
        let k = rider_gains(v);
        assert!((k[0][0] - 0.5 * (t[0].1[0][0] + t[1].1[0][0])).abs() < 1e-3);
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
