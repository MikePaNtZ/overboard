//! C ABI over the real control law, so the MuJoCo scenario can drive it.
//!
//! # Why this exists
//!
//! Python owns MuJoCo and the scenario harness — the collision-hull geometry,
//! the determinism, the metrics, the CI gate. Rust owns the control law, which
//! is the part that eventually runs on the Pi. Re-implementing either in the
//! other language would mean maintaining two copies of something whose whole
//! value is being singular.
//!
//! So the physics calls the controller, not the other way round. That is the
//! opposite of the eventual hardware arrangement, where `board-app-driverless`
//! owns the loop and calls `BoardActuate` — and that is fine, because **the portable
//! part is [`control_core::PitchRegulator`] and [`safety::Envelope`]**, both of
//! which are exercised here exactly as they will be on hardware. Only the
//! direction of the call differs, and switching it later means deleting this
//! crate and the Python glue, nothing more.
//!
//! # ABI stability
//!
//! Every struct leads with `size`, set by the caller to its own `sizeof`. The
//! callee compares it and refuses a mismatch rather than reading past the end
//! of a struct compiled against an older header. This is what lets the
//! observation grow — a real IMU batch, validity flags, ERPM — without an
//! ABI break or a version bump on every field.
//!
//! # What this is NOT
//!
//! Not the [`hal`] seam. There is no `wait_observe`/`apply` here and no notion
//! of who advances time: Python steps the physics. The ICD §5.2 contract is
//! honoured on the *scenario* side, which applies a command one step after the
//! state that produced it.

use board_types::ImuSample;
use board_types::{Command, Faults, Params, Saturation, DEFAULT_R_EFF_M};
use control_core::{
    Attitude, CommandFeedforward, ComplementaryFilter, Estimator, PitchRegulator, PlantCoupling,
    VelocityLoop, WheelAccelEstimator,
};
use safety::Envelope;

/// Bumped for an incompatible change to the *function* signatures, OR to a
/// struct's field layout (renamed/reordered/retyped fields) -- anything the
/// `size` guard cannot catch on its own. `size` only proves the byte count
/// matches; it says nothing about what those bytes mean. Growing a struct
/// with a purely additive new field at the end, with existing fields
/// untouched, is still handled by `size` alone and does not need a bump.
///
/// Bumped to 2 by issue #137: `ObParamsV1` renamed two fields
/// (`kp_a_per_rad`/`kd_a_per_rad_s` -> `kp_nm_per_rad`/`kd_nm_per_rad_s`) and
/// is now `ObParamsV2`. A caller still holding the old ctypes struct has the
/// same `size` in the lucky case and a different one otherwise, but even a
/// same-`size` collision must not be told "compatible" -- that is silent
/// misreading of memory, exactly what the version field exists to prevent.
pub const ABI_VERSION: u32 = 2;

/// Motor torque constant fallback, N·m per amp — mirrors
/// `sim/scenarios/plant.py::KT_NM_PER_A`. **Unfitted placeholder**, same
/// caveat as that constant: nothing has been measured on the bench yet.
/// Used only when a caller passes `kt_nm_per_a <= 0.0`, the same
/// zero-means-use-the-default convention `r_eff_m` already follows below.
pub const DEFAULT_KT_NM_PER_A: f32 = 0.7;

// --- status codes ----------------------------------------------------------

pub const OB_OK: i32 = 0;
/// A pointer argument was null.
pub const OB_ERR_NULL: i32 = -1;
/// A struct's `size` did not match what this build expects.
pub const OB_ERR_SIZE: i32 = -2;
/// A non-finite value crossed the boundary.
pub const OB_ERR_NOT_FINITE: i32 = -3;

/// Controller gains and envelope limits.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ObParamsV2 {
    pub size: u32,
    /// Proportional gain, **N·m per radian** of nose-up-positive pitch
    /// (issue #137 — a property of the plant, not the motor).
    pub kp_nm_per_rad: f32,
    /// Derivative gain, N·m per rad/s.
    pub kd_nm_per_rad_s: f32,
    /// Motor torque constant, N·m per amp. **The one place `kt` crosses this
    /// ABI.** Converts the regulator's torque output to the current the drive
    /// accepts, and turns `max_current_a` into a torque ceiling
    /// (`board_types::Params::tau_max_nm`) — never used inside the law
    /// itself. Zero or negative falls back to [`DEFAULT_KT_NM_PER_A`].
    pub kt_nm_per_a: f32,
    /// Envelope clamp on commanded current, amps (ICD §7.6 stage 2).
    pub max_current_a: f32,

    // --- outer velocity loop --------------------------------------------
    /// Radians of pitch reference per m/s of speed error. Zero disables the
    /// outer loop entirely, leaving a pure inner-loop regulator.
    pub kp_v_rad_per_m_s: f32,
    pub ki_v_rad_per_m: f32,
    /// Clamp on the pitch the outer loop may ask for, radians.
    pub max_pitch_ref_rad: f32,
    /// Wheel radius used to turn wheel rate into ground speed, metres.
    pub r_eff_m: f32,
    /// **1 = centre of mass ABOVE the axle** (a ridden board), 0 = below (the
    /// driverless board). Not a preference: the pitch-to-velocity coupling
    /// genuinely inverts between them, and getting it wrong makes the outer
    /// loop positive feedback. See `control_core::PlantCoupling`.
    pub com_above_axle: u32,

    // --- estimator -------------------------------------------------------
    /// 0 = off (regulate on the observation's `pitch_rad`), 1 = active
    /// (regulate on the fused estimate), **2 = shadow** (fuse and report, but
    /// still regulate on `pitch_rad`).
    ///
    /// Shadow mirrors ICD §6.6's shadow mode for the ridden board, and exists
    /// for the same reason: it measures what the estimator WOULD have done
    /// without letting it affect the outcome. Without it, estimator accuracy
    /// and closed-loop stability can only be measured together, and a filter
    /// that is accurate but destabilising looks identical to one that is
    /// simply inaccurate.
    ///
    /// The truth path is kept deliberately. The regulator was tuned against a
    /// perfect attitude, and keeping that path is what makes the estimator's
    /// contribution to the error measurable **on its own** rather than tangled
    /// with the gains.
    pub use_estimator: u32,
    /// Complementary-filter crossover, seconds.
    pub estimator_tau_s: f32,
    /// Where the forward-acceleration correction comes from. Subtracting it
    /// from the accelerometer before deriving tilt is not optional: without
    /// any correction the implied tilt is wrong by `atan(a/g)` whenever the
    /// board accelerates — around 5° at the outer loop's limit, the same size
    /// as the excursion being regulated and correlated with it.
    ///
    /// - **0 — off.** Diagnostic only; the loop does not survive the shuttle.
    /// - **1 — wheel odometry.** Differentiates the reported wheel speed.
    ///   Sees real acceleration whatever causes it, but is quantised and
    ///   lagged, and the residual it leaves is what destabilised the loop at
    ///   τ = 1 s. Needs τ ≥ 10 s to be flyable, which costs return accuracy.
    /// - **2 — command feedforward.** Predicts acceleration from the current
    ///   just commanded, via `accel_ff_gain_m_s2_per_a`. No lag, no
    ///   quantisation; blind to slopes and slip. Best on the bench.
    ///
    /// See [`control_core::CommandFeedforward`] for the trade in full.
    pub estimator_accel_aiding: u32,
    /// Low-pass on the wheel-speed derivative, seconds. Mode 1 only.
    pub wheel_accel_tau_s: f32,
    /// Amps → m/s² for mode 2: one fitted constant standing in for
    /// `k_t / (r_eff · m)`. Zero falls back to the ridden-board value.
    ///
    /// **This is a number to measure on the bench, not to trust from a
    /// datasheet** — it absorbs `k_t`, the loaded rolling radius and the total
    /// mass including the rider, none of which are known to better than about
    /// 10% before hardware arrives.
    pub accel_ff_gain_m_s2_per_a: f32,
    /// Skip the accelerometer correction when the aiding-corrected specific
    /// force is further than this from g, m/s². Zero disables the gate.
    ///
    /// Buys disturbance immunity at the cost of running open-loop on the gyro
    /// while tripped, so too narrow a band is worse than none.
    pub accel_trust_band_m_s2: f32,
    /// Which current the mode-2 feedforward believes. **0 = commanded**
    /// (`cmd.amps` from the previous cycle), **1 = measured**
    /// (`obs.motor_current_a`).
    ///
    /// **Hardware should use 1.** The VESC derates commanded torque silently
    /// through half a dozen cutback layers, and a feedforward fed the command
    /// during a cutback subtracts acceleration that is not happening. Defaults
    /// to 0 only because a backend that does not populate `motor_current_a`
    /// would otherwise feed the estimator a constant zero — which is precisely
    /// the "optional field quietly left at its default" failure this codebase
    /// has already been bitten by once.
    pub accel_ff_current_source: u32,
}

/// One cycle of plant state.
///
/// v1 carries an **already-estimated** pitch, which in sim is MuJoCo truth.
/// That is a deliberate stub: it lets the regulator be tuned against a perfect
/// attitude first, so that when a real estimator lands its contribution to the
/// error is measurable on its own rather than tangled with the gains.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ObObsV1 {
    pub size: u32,
    pub t_ns: u64,
    /// Nose-up-positive, radians (ICD §10.1).
    pub pitch_rad: f32,
    /// Nose-up-positive rate, rad/s.
    pub pitch_rate_rad_s: f32,
    /// Positive = forward translation.
    pub wheel_rate_rad_s: f32,
    /// Current actually flowing, amps — not an echo of what was commanded.
    pub motor_current_a: f32,
    /// Raw body-frame gyro, rad/s. `[1]` IS the nose-up pitch rate (ICD §10.1).
    pub gyro_rad_s: [f32; 3],
    /// Raw body-frame specific force, m/s².
    pub accel_m_s2: [f32; 3],
    /// Ground-speed setpoint for THIS cycle, m/s. Zero is station keeping.
    ///
    /// Per-cycle rather than fixed at construction, so a command sequence can
    /// change it without rebuilding the controller — which would discard the
    /// integrator, and the integrator is the thing that remembers where home
    /// is. Its running integral tracks position against `∫v_ref`, so a profile
    /// that integrates to zero net displacement returns to the start for free.
    pub v_ref_m_s: f32,
}

/// The resulting command.
#[repr(C)]
#[derive(Debug, Clone, Copy)]
pub struct ObCmdV1 {
    pub size: u32,
    /// Post-envelope current, amps.
    pub amps: f32,
    /// 0 = not saturated, 1 = saturated, 2 = unknown. Mirrors
    /// [`board_types::Saturation`], which anti-windup will read.
    pub saturated: u32,
    /// What the outer loop asked the inner loop to hold, radians. Reported so
    /// a run can be diagnosed without instrumenting the library.
    pub pitch_ref_rad: f32,
    /// The attitude the controller actually acted on. Equals the observation's
    /// `pitch_rad` when the estimator is off; reported always, so the harness
    /// can score estimator error against truth without a second code path.
    pub pitch_used_rad: f32,
}

/// Opaque controller handle.
pub struct ObController {
    regulator: PitchRegulator,
    estimator: Option<ComplementaryFilter>,
    /// True only in mode 1. In shadow mode the estimate is computed and
    /// reported but never reaches the regulator.
    estimate_active: bool,
    wheel_accel: Option<WheelAccelEstimator>,
    /// Mode 2's aiding source. Mutually exclusive with `wheel_accel`.
    accel_ff: Option<CommandFeedforward>,
    /// True when the feedforward should believe `obs.motor_current_a` rather
    /// than the command we issued last cycle.
    accel_ff_measured: bool,
    /// Last current commanded, amps — the feedforward's input. One cycle old
    /// by construction: it is what we asked for on the previous tick, which is
    /// what the plant has been acting on since.
    last_amps: f32,
    outer: Option<VelocityLoop>,
    envelope: Envelope,
    /// The single point `kt` enters this stack (issue #137): divides the
    /// regulator's torque output down to amps, immediately before the
    /// envelope's amp clamp.
    kt_nm_per_a: f32,
    r_eff_m: f32,
    last_t_ns: Option<u64>,
    /// Carried between cycles so the outer loop can hold off integrating while
    /// the inner loop is against its clamp (ICD §7.6).
    last_saturated: bool,
}

fn saturation_code(s: Saturation) -> u32 {
    match s {
        Saturation::No => 0,
        Saturation::Yes => 1,
        Saturation::Unknown => 2,
    }
}

/// ABI version of this library.
#[no_mangle]
pub extern "C" fn ob_abi_version() -> u32 {
    ABI_VERSION
}

/// Construct a controller. Returns null on a null or mis-sized `params`.
///
/// # Safety
/// `params` must point to a valid `ObParamsV2`.
#[no_mangle]
pub unsafe extern "C" fn ob_controller_new(params: *const ObParamsV2) -> *mut ObController {
    if params.is_null() {
        return core::ptr::null_mut();
    }
    let p = unsafe { &*params };
    if p.size as usize != core::mem::size_of::<ObParamsV2>() {
        return core::ptr::null_mut();
    }

    let outer = if p.kp_v_rad_per_m_s != 0.0 || p.ki_v_rad_per_m != 0.0 {
        Some(VelocityLoop::new(
            p.kp_v_rad_per_m_s,
            p.ki_v_rad_per_m,
            p.max_pitch_ref_rad,
            if p.com_above_axle != 0 {
                PlantCoupling::ComAboveAxle
            } else {
                PlantCoupling::ComBelowAxle
            },
        ))
    } else {
        None
    };

    let kt_nm_per_a = if p.kt_nm_per_a > 0.0 {
        p.kt_nm_per_a
    } else {
        DEFAULT_KT_NM_PER_A
    };

    let boxed = Box::new(ObController {
        regulator: PitchRegulator::new(p.kp_nm_per_rad, p.kd_nm_per_rad_s),
        estimate_active: p.use_estimator == 1,
        wheel_accel: if p.estimator_accel_aiding == 1 {
            Some(WheelAccelEstimator::new(if p.wheel_accel_tau_s > 0.0 {
                p.wheel_accel_tau_s
            } else {
                0.05
            }))
        } else {
            None
        },
        accel_ff: if p.estimator_accel_aiding == 2 {
            Some(CommandFeedforward::new(
                if p.accel_ff_gain_m_s2_per_a > 0.0 {
                    p.accel_ff_gain_m_s2_per_a
                } else {
                    // kt 0.7 N·m/A / (r_eff 0.1454 m × 82.5 kg ridden).
                    // Re-derived for #89: was 0.0581 against the stale
                    // 0.14605 m r_eff, a 0.45% shift that does not change
                    // which regime this feedforward gain lands in.
                    0.0584
                },
            ))
        } else {
            None
        },
        accel_ff_measured: p.accel_ff_current_source == 1,
        last_amps: 0.0,
        estimator: if p.use_estimator != 0 {
            Some(ComplementaryFilter::with_trust_band(
                if p.estimator_tau_s > 0.0 {
                    p.estimator_tau_s
                } else {
                    1.0
                },
                p.accel_trust_band_m_s2.max(0.0),
            ))
        } else {
            None
        },
        outer,
        kt_nm_per_a,
        r_eff_m: if p.r_eff_m > 0.0 {
            p.r_eff_m
        } else {
            DEFAULT_R_EFF_M
        },
        last_t_ns: None,
        last_saturated: false,
        envelope: Envelope::new(Params {
            kp_nm_per_rad: p.kp_nm_per_rad,
            kd_nm_per_rad_s: p.kd_nm_per_rad_s,
            kt_nm_per_a,
            max_current_a: p.max_current_a,
            ..Params::default()
        }),
    });
    Box::into_raw(boxed)
}

/// Enable motion. Until this is called the envelope zeroes every command.
///
/// Explicit rather than implicit in `new`, so that "armed" is always something
/// the caller did on purpose — the same property the hardware path needs.
///
/// # Safety
/// `h` must be a live handle from [`ob_controller_new`].
#[no_mangle]
pub unsafe extern "C" fn ob_controller_arm(h: *mut ObController) -> i32 {
    if h.is_null() {
        return OB_ERR_NULL;
    }
    unsafe { &mut *h }.envelope.arm();
    OB_OK
}

/// Free a controller. Null is a no-op.
///
/// # Safety
/// `h` must have come from [`ob_controller_new`] and must not be used after.
#[no_mangle]
pub unsafe extern "C" fn ob_controller_free(h: *mut ObController) {
    if !h.is_null() {
        drop(unsafe { Box::from_raw(h) });
    }
}

/// Compute one command.
///
/// # Safety
/// All pointers must be valid and correctly sized.
#[no_mangle]
pub unsafe extern "C" fn ob_controller_update(
    h: *mut ObController,
    obs: *const ObObsV1,
    out: *mut ObCmdV1,
) -> i32 {
    if h.is_null() || obs.is_null() || out.is_null() {
        return OB_ERR_NULL;
    }
    let ctl = unsafe { &mut *h };
    let o = unsafe { &*obs };
    let cmd = unsafe { &mut *out };

    if o.size as usize != core::mem::size_of::<ObObsV1>()
        || cmd.size as usize != core::mem::size_of::<ObCmdV1>()
    {
        return OB_ERR_SIZE;
    }

    // A NaN reaching the regulator would propagate silently into the plant and
    // show up as an unexplained divergence. Refuse it at the boundary, where it
    // is still attributable, and command nothing.
    if !o.pitch_rad.is_finite()
        || !o.pitch_rate_rad_s.is_finite()
        || !o.wheel_rate_rad_s.is_finite()
    {
        cmd.amps = 0.0;
        cmd.saturated = saturation_code(Saturation::No);
        cmd.pitch_ref_rad = 0.0;
        cmd.pitch_used_rad = 0.0;
        return OB_ERR_NOT_FINITE;
    }

    // dt from the caller's clock, never assumed. Zero on the first cycle, and
    // a zero dt simply means the integrator does not advance -- which is the
    // right thing when there is no interval to integrate over.
    let dt_s = match ctl.last_t_ns {
        Some(prev) => (o.t_ns.saturating_sub(prev)) as f32 * 1e-9,
        None => 0.0,
    };
    ctl.last_t_ns = Some(o.t_ns);

    // Forward acceleration, for aiding the accelerometer. Measured from wheel
    // odometry, or predicted from the last command -- see the mode notes on
    // `estimator_accel_aiding`. At most one is Some.
    let aiding = match (ctl.wheel_accel.as_mut(), ctl.accel_ff.as_ref()) {
        (Some(w), _) => w.update(o.wheel_rate_rad_s * ctl.r_eff_m, dt_s),
        (None, Some(ff)) => ff.predict(if ctl.accel_ff_measured {
            o.motor_current_a
        } else {
            ctl.last_amps
        }),
        (None, None) => 0.0,
    };

    // Attitude: fused from the raw IMU, or truth straight off the observation.
    let attitude = match ctl.estimator.as_mut() {
        Some(est) => est.update(
            &[ImuSample {
                gyro_rad_s: o.gyro_rad_s,
                accel_m_s2: o.accel_m_s2,
                t_sample_ns: o.t_ns,
            }],
            aiding,
        ),
        None => Attitude {
            pitch_rad: o.pitch_rad,
            pitch_rate_rad_s: o.pitch_rate_rad_s,
        },
    };

    let pitch_ref = match ctl.outer.as_mut() {
        Some(outer) => {
            let v = o.wheel_rate_rad_s * ctl.r_eff_m;
            outer.update(v, o.v_ref_m_s, dt_s, ctl.last_saturated)
        }
        None => 0.0,
    };

    // NOTE: `attitude`, not `o.pitch_rad`. An earlier revision computed the
    // estimate, reported it in `pitch_used_rad`, and then regulated on truth
    // anyway -- so the estimator was fully observable and entirely inert, and
    // the error metric looked healthy the whole time.
    // `test_the_estimator_is_actually_in_the_control_path` exists to catch it.
    let (pitch_in, rate_in) = if ctl.estimate_active {
        (attitude.pitch_rad, attitude.pitch_rate_rad_s)
    } else {
        (o.pitch_rad, o.pitch_rate_rad_s)
    };
    // The law lives entirely in torque; this division is the ONLY place `kt`
    // enters the control path (issue #137). A wrong `kt` here changes how
    // much current a given torque command costs -- headroom -- not the
    // torque itself, which is fixed by `pitch_in`/`rate_in` alone.
    let proposed_torque_nm = ctl.regulator.update(pitch_in, rate_in, pitch_ref);
    let proposed_amps = proposed_torque_nm / ctl.kt_nm_per_a;
    let (bounded, sat) = ctl.envelope.apply(
        Command::MotorCurrent {
            amps: proposed_amps,
        },
        Faults::NONE,
    );

    cmd.amps = match bounded {
        Command::MotorCurrent { amps } => amps,
        // The regulator only ever emits MotorCurrent; a RemoteSpeed here would
        // mean the envelope substituted a different profile's command.
        Command::RemoteSpeed { .. } => 0.0,
    };
    cmd.saturated = saturation_code(sat);
    cmd.pitch_ref_rad = pitch_ref;
    cmd.pitch_used_rad = attitude.pitch_rad;
    ctl.last_saturated = sat == Saturation::Yes;
    // The POST-envelope current, not the regulator's proposal: the plant only
    // ever sees the clamped value, so feeding the proposal to the feedforward
    // would over-predict acceleration for exactly as long as the loop is
    // saturated -- which is when the attitude estimate matters most.
    ctl.last_amps = cmd.amps;
    OB_OK
}

#[cfg(test)]
mod tests {
    use super::*;

    fn params() -> ObParamsV2 {
        ObParamsV2 {
            size: core::mem::size_of::<ObParamsV2>() as u32,
            // 80 A/rad, 11 A/(rad/s) at kt = 0.7 N*m/A -- the same numbers
            // this crate shipped before issue #137, re-denominated.
            kp_nm_per_rad: 56.0,
            kd_nm_per_rad_s: 7.7,
            kt_nm_per_a: 0.7,
            max_current_a: 60.0,
            kp_v_rad_per_m_s: 0.0,
            ki_v_rad_per_m: 0.0,
            max_pitch_ref_rad: 0.087,
            r_eff_m: DEFAULT_R_EFF_M,
            com_above_axle: 1,
            use_estimator: 0,
            estimator_tau_s: 1.0,
            estimator_accel_aiding: 0,
            wheel_accel_tau_s: 0.05,
            accel_ff_gain_m_s2_per_a: 0.0,
            accel_trust_band_m_s2: 0.0,
            accel_ff_current_source: 0,
        }
    }

    fn obs(pitch_rad: f32, rate: f32) -> ObObsV1 {
        ObObsV1 {
            size: core::mem::size_of::<ObObsV1>() as u32,
            t_ns: 0,
            pitch_rad,
            pitch_rate_rad_s: rate,
            wheel_rate_rad_s: 0.0,
            motor_current_a: 0.0,
            gyro_rad_s: [0.0, rate, 0.0],
            accel_m_s2: [0.0, 0.0, -9.81],
            v_ref_m_s: 0.0,
        }
    }

    fn out() -> ObCmdV1 {
        ObCmdV1 {
            size: core::mem::size_of::<ObCmdV1>() as u32,
            amps: f32::NAN,
            saturated: 99,
            pitch_ref_rad: f32::NAN,
            pitch_used_rad: f32::NAN,
        }
    }

    struct Handle(*mut ObController);
    impl Drop for Handle {
        fn drop(&mut self) {
            unsafe { ob_controller_free(self.0) }
        }
    }

    fn armed() -> Handle {
        let p = params();
        let h = unsafe { ob_controller_new(&p) };
        assert!(!h.is_null());
        assert_eq!(unsafe { ob_controller_arm(h) }, OB_OK);
        Handle(h)
    }

    #[test]
    fn nose_down_commands_positive_current_through_the_whole_chain() {
        let h = armed();
        let (o, mut c) = (obs(-0.05, 0.0), out());
        assert_eq!(unsafe { ob_controller_update(h.0, &o, &mut c) }, OB_OK);
        assert!(c.amps > 0.0, "got {}", c.amps);
        assert_eq!(c.saturated, 0);
    }

    #[test]
    fn disarmed_commands_nothing_however_large_the_error() {
        // The envelope is in the path, not bypassed. Without arming, a big
        // error must still produce zero -- not a clamped-but-nonzero command.
        let p = params();
        let h = Handle(unsafe { ob_controller_new(&p) });
        let (o, mut c) = (obs(-0.5, -2.0), out());
        assert_eq!(unsafe { ob_controller_update(h.0, &o, &mut c) }, OB_OK);
        assert_eq!(c.amps, 0.0);
    }

    #[test]
    fn a_large_error_saturates_and_says_so() {
        let h = armed();
        let (o, mut c) = (obs(-1.0, 0.0), out());
        assert_eq!(unsafe { ob_controller_update(h.0, &o, &mut c) }, OB_OK);
        assert_eq!(c.amps, 40.0);
        assert_eq!(c.saturated, 1, "anti-windup needs to see this");
    }

    #[test]
    fn a_stale_header_size_is_refused_rather_than_misread() {
        // The entire point of the size field: an older caller must get an
        // error, not a struct read past its end.
        let h = armed();
        let mut o = obs(-0.05, 0.0);
        o.size = 8;
        let mut c = out();
        assert_eq!(
            unsafe { ob_controller_update(h.0, &o, &mut c) },
            OB_ERR_SIZE
        );
    }

    #[test]
    fn null_pointers_are_refused_not_dereferenced() {
        let h = armed();
        let mut c = out();
        assert_eq!(
            unsafe { ob_controller_update(h.0, core::ptr::null(), &mut c) },
            OB_ERR_NULL
        );
        assert_eq!(
            unsafe { ob_controller_arm(core::ptr::null_mut()) },
            OB_ERR_NULL
        );
        unsafe { ob_controller_free(core::ptr::null_mut()) };
    }

    #[test]
    fn a_nan_is_stopped_at_the_boundary_and_commands_zero() {
        let h = armed();
        let (o, mut c) = (obs(f32::NAN, 0.0), out());
        assert_eq!(
            unsafe { ob_controller_update(h.0, &o, &mut c) },
            OB_ERR_NOT_FINITE
        );
        assert_eq!(c.amps, 0.0);
    }

    #[test]
    fn new_refuses_a_mis_sized_params() {
        let mut p = params();
        p.size = 4;
        assert!(unsafe { ob_controller_new(&p) }.is_null());
    }

    #[test]
    fn abi_version_is_reported() {
        assert_eq!(ob_abi_version(), ABI_VERSION);
    }

    #[test]
    fn zero_kt_falls_back_to_the_default_placeholder() {
        // Same zero-means-default convention `r_eff_m` already follows --
        // asserted here because it is new with issue #137.
        let mut p = params();
        p.kt_nm_per_a = 0.0;
        let h = unsafe { ob_controller_new(&p) };
        assert!(!h.is_null());
        let ctl = unsafe { &*h };
        assert_eq!(ctl.kt_nm_per_a, DEFAULT_KT_NM_PER_A);
        unsafe { ob_controller_free(h) };
    }

    // ---- issue #137 AC4: kt moves headroom, not loop gain -----------------
    //
    // "Demonstrate it: run with kt at 0.5 and 0.9 and show the loop gain is
    // unchanged while headroom moves." Both controllers below share the SAME
    // kp_nm_per_rad/kd_nm_per_rad_s -- only kt_nm_per_a differs.

    fn controller_with_kt(kt_nm_per_a: f32) -> Handle {
        let mut p = params();
        p.kt_nm_per_a = kt_nm_per_a;
        let h = unsafe { ob_controller_new(&p) };
        assert!(!h.is_null());
        assert_eq!(unsafe { ob_controller_arm(h) }, OB_OK);
        Handle(h)
    }

    #[test]
    fn loop_gain_the_torque_actually_commanded_is_unchanged_by_kt() {
        // Small error, well inside either kt's headroom: neither saturates.
        // If loop gain depended on kt this would fail -- it does not, because
        // kt appears nowhere in the law.
        let (lo, hi) = (controller_with_kt(0.5), controller_with_kt(0.9));
        let (o, mut c_lo) = (obs(-0.05, 0.0), out());
        let mut c_hi = out();
        assert_eq!(unsafe { ob_controller_update(lo.0, &o, &mut c_lo) }, OB_OK);
        assert_eq!(unsafe { ob_controller_update(hi.0, &o, &mut c_hi) }, OB_OK);
        assert_eq!(c_lo.saturated, 0);
        assert_eq!(c_hi.saturated, 0);

        // amps differ (headroom cost of the SAME torque differs)...
        assert!(
            (c_lo.amps - c_hi.amps).abs() > 1e-3,
            "expected different amps for different kt: lo={} hi={}",
            c_lo.amps,
            c_hi.amps
        );
        // ...but amps * kt -- the torque actually realised -- is identical.
        let (torque_lo, torque_hi) = (c_lo.amps * 0.5, c_hi.amps * 0.9);
        assert!(
            (torque_lo - torque_hi).abs() < 1e-3,
            "loop gain moved with kt: lo={torque_lo} N*m hi={torque_hi} N*m"
        );
    }

    #[test]
    fn headroom_moves_with_kt_at_a_fixed_current_limit() {
        // pitch = -0.5 rad asks the law (kp_nm_per_rad = 56.0) for 28 N*m,
        // exactly halfway between kt=0.5's ceiling (0.5*40 = 20 N*m) and
        // kt=0.9's ceiling (0.9*40 = 36 N*m) -- so the SAME command saturates
        // at one kt and not the other. Same max_current_a (40 A) throughout;
        // only the belief about kt changes what that 40 A is worth in torque.
        let (lo, hi) = (controller_with_kt(0.5), controller_with_kt(0.9));
        let (o, mut c_lo) = (obs(-0.5, 0.0), out());
        let mut c_hi = out();
        assert_eq!(unsafe { ob_controller_update(lo.0, &o, &mut c_lo) }, OB_OK);
        assert_eq!(unsafe { ob_controller_update(hi.0, &o, &mut c_hi) }, OB_OK);

        assert_eq!(c_lo.saturated, 1, "kt=0.5: 28 N*m exceeds a 20 N*m ceiling");
        assert_eq!(
            c_hi.saturated, 0,
            "kt=0.9: 28 N*m is within a 36 N*m ceiling"
        );
        assert_eq!(c_lo.amps, 40.0);
        assert!(c_hi.amps < 40.0, "got {}", c_hi.amps);
    }
}
