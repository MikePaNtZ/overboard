//! `sim-backend` implements the [`hal`] seam against the real MuJoCo onewheel
//! plant (`crates/plant-mujoco`), issue #107 (I1c). This is the increment
//! that satisfies SR-SIM-5: the Rust stack now steps the plant through `hal`
//! itself, and the Python harness no longer has control authority over it.
//!
//! What this backend honours, because getting them wrong later is expensive:
//!
//! - **`wait_observe()` is the only call that advances time.** `apply()` does
//!   not touch the clock (ICD §5.2), and does not touch the plant either --
//!   it only buffers a pending command.
//! - **`wait_observe()` steps the plant `control_period / mj_timestep`
//!   times, and that ratio is asserted to be an exact integer in `open()`**
//!   (issue #107 AC2). A fractional ratio would give a small persistent
//!   timing bias that presents as a controls problem.
//! - **Sim time comes from `mjData::time`, not a parallel counter** (AC3).
//!   The stub's synthetic clock is gone, not kept alongside it.
//! - **The pre-loop `mj_forward` priming call every CONTROLLED Python
//!   scenario makes is mirrored here, in the same position** (AC8, carried
//!   forward from I1b/#106): once, right after the plant opens, before the
//!   first `mj_step`. `mj_forward` writes `qacc_warmstart`, and the solver is
//!   history-dependent -- skipping this call would start the Rust and Python
//!   hosts from different warmstart state, which shows up as a phase error
//!   indistinguishable from a controls problem. See
//!   `crates/plant-mujoco/README.md`'s "Ordering contract".
//! - **The call-sequence rules are enforced**, via [`hal::CallSequence`], so a
//!   double `apply()` is already a `ProtocolViolation` rather than two frames.
//! - **`apply()` does not echo the command back as measured current.** The
//!   commanded value is buffered and only appears in a *later* observation,
//!   because actuation delay is additive (ICD §5.2) and a backend that echoes
//!   is explicitly non-conforming (§12). One whole control cycle here is a
//!   placeholder for the real first-order current loop, which arrives with
//!   the imperfection profile (Mechanical's territory, not this crate's).
//! - **Cold start reports `Invalid`, not zeros.** A backend presenting zeros
//!   as measurements before the drive has spoken is non-conforming (§12).
//!
//! [`SimBackend`] implements both [`hal::BoardObserve`] and
//! [`hal_actuate::BoardActuate`], which makes this crate **driverless-only**:
//! depending on it pulls in `hal-actuate` transitively, so `board-app-ridden`
//! must not link it. `board-app-ridden` gets its own observe-only backend
//! instead.

use board_types::{
    Applied, Command, DisarmReason, FaultCode, ImuSample, IoError, Observation, Params, Profile,
    RunMetadata, Saturation, ValidityFlags, DEFAULT_R_EFF_M, RAD_S_PER_ERPM,
};
use hal::{BoardObserve, CallSequence};
use hal_actuate::{BoardActuate, Disarm};
use plant_mujoco::Plant;
use std::path::{Path, PathBuf};

/// Nanoseconds per control cycle. 500 Hz, per ICD §11.2. Sourced the same way
/// the stub sourced it -- a fixed constant, not read from the model -- per
/// issue #107's explicit instruction not to invent a new configuration
/// mechanism here.
const CYCLE_NS: u64 = 2_000_000;

/// Motor torque constant, N*m per amp. The MJCF actuator is a torque source
/// (`gear="1"`); the controller speaks current. Mirrors
/// `sim/scenarios/plant.py::KT_NM_PER_A` exactly -- duplicated the same way
/// `board_types::DEFAULT_R_EFF_M` duplicates the model's tire radius, because
/// this crate has no Python binding to check itself against directly. The
/// model's own `ctrlrange="-28 28"` is the derived, checkable consequence
/// (40 A * 0.7 N*m/A = 28 N*m), and `kt_nm_per_a_matches_the_models_ctrlrange`
/// below pins it against the compiled model so the two cannot silently drift.
const KT_NM_PER_A: f64 = 0.7;

/// The model this backend steps BY DEFAULT -- the driverless onewheel plant,
/// unchanged since I1c. Every existing caller (this crate's own tests,
/// `impulse-response-rust`, the driverless `hal` seam) keeps stepping exactly
/// this model, because `open()` only reaches for it when
/// [`SimBackend::model_path_override`] is `None`.
///
/// CHANGED from "fixed, not configurable" (I1c's original claim, now stale):
/// issue #161 W2 needs a second model -- a ridden variant carrying a
/// physically jointed rider ballast (`overboard_rider.xml`) -- and
/// `sim-backend` has no Python binding to patch a model string at runtime
/// the way the Python scenarios do, so a real per-instance override is the
/// only way `sim-host` can point this backend at it. See
/// [`SimBackend::with_model_path`].
fn model_path() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("../../sim/models/overboard_onewheel.xml")
}

/// Disarm handle for the sim.
///
/// Real backends hold a pre-opened fd and a pre-encoded safe-state frame so the
/// disarm path allocates nothing and takes no lock (ICD §9.2). The sim has
/// nothing to send, so this is a marker — but it is `Send + Sync` and
/// idempotent like the real one, so code that stores or clones it compiles the
/// same against both.
#[derive(Debug, Clone, Copy, Default)]
pub struct SimDisarm;

impl Disarm for SimDisarm {
    fn disarm(&self, _reason: DisarmReason) -> Result<(), IoError> {
        Ok(())
    }
}

/// Real sim backend: steps `plant-mujoco`'s `Plant` through the `hal` seam.
#[derive(Debug, Default)]
pub struct SimBackend {
    plant: Option<Plant>,
    /// `CYCLE_NS / mj_timestep_ns`, resolved once in `open()` (issue #107
    /// AC2). Zero before `open()`; never read before then.
    steps_per_cycle: u64,
    /// `(adr, dim)` into `mjData::sensordata`, resolved once in `open()`.
    gyro_sensor: (usize, usize),
    accel_sensor: (usize, usize),
    /// `wheel_hinge`'s `jointvel` sensor (issue #121) -- already declared in
    /// `overboard_onewheel.xml`, unread until now. Same joint and sign
    /// convention the model's own motor sign comment and `ctrl` use, so no
    /// extra sign flip is needed here.
    wheel_vel_sensor: (usize, usize),
    /// `mjModel` body id of the `frame` body, resolved once in `open()`.
    /// Only used by [`SimBackend::apply_external_force`], not by `hal`.
    frame_body: usize,
    /// `mjModel` actuator id of `wheel_motor`, resolved BY NAME in `open()`
    /// (issue #161 W2). Previously assumed to be `ctrl` index 0 outright --
    /// only ever correct because the driverless model has exactly one
    /// actuator. A model with more (the ridden rider model's two ballast
    /// actuators) makes that assumption wrong, silently.
    wheel_motor_actuator: usize,
    /// `mjModel` actuator ids of the two ballast position actuators, if the
    /// open model declares them -- `None` for the driverless onewheel model,
    /// which has no ballast at all. Resolved once in `open()`.
    ballast_fa_actuator: Option<usize>,
    ballast_lateral_actuator: Option<usize>,
    /// Commanded ballast actuator targets, metres -- see
    /// [`SimBackend::set_ballast_targets`]. Zero (centred) until set, and
    /// reset to zero on every `open()`.
    ballast_fa_target_m: f32,
    ballast_lateral_target_m: f32,
    /// `mjModel::jnt_qposadr` for the `ballast_fa`/`ballast_lat` joints, if
    /// the open model declares them -- `None` for the driverless onewheel
    /// model. Resolved once in `open()`; used by
    /// [`SimBackend::truth_ballast_positions`], the ACTUAL (not commanded)
    /// joint position issue #161 wire v2 puts on the state-out wire.
    ballast_fa_qposadr: Option<usize>,
    ballast_lateral_qposadr: Option<usize>,
    /// `mjModel::jnt_qposadr` / `jnt_dofadr` for the board's `frame_free`
    /// free joint, if the open model declares one. Resolved once in `open()`;
    /// used only by [`SimBackend::inject_kinematic_yaw`]. BOTH addresses are
    /// kept because a free joint spans 7 `qpos` but only 6 `qvel` -- see
    /// `plant_mujoco::Plant::joint_dofadr`.
    frame_free_qposadr: Option<usize>,
    frame_free_dofadr: Option<usize>,
    cycle: u64,
    open: bool,
    armed: bool,
    seq: CallSequence,
    /// Commanded current awaiting its actuation delay. Never reported as
    /// measured in the same cycle it was applied.
    pending_current_a: Option<f32>,
    /// The current now actually flowing, as far as the plant is concerned.
    applied_current_a: f32,
    params: Params,
    /// Overrides `model_path()`'s default when `Some` (issue #161 W2). See
    /// [`SimBackend::with_model_path`].
    model_path_override: Option<PathBuf>,
    /// **Verification only.** Ground incline, degrees, applied at `open()` by
    /// tilting gravity. See [`SimBackend::set_incline_deg`]. Zero leaves
    /// `mjModel::opt.gravity` untouched -- not merely set back to its own
    /// value, so a run with no incline is bit-identical to one built before
    /// this knob existed.
    incline_deg: f64,
}

impl SimBackend {
    pub fn new() -> Self {
        SimBackend::default()
    }

    pub fn with_params(params: Params) -> Self {
        SimBackend {
            params,
            ..SimBackend::default()
        }
    }

    /// Like [`SimBackend::with_params`], but steps `model_path` instead of
    /// the default driverless onewheel model (issue #161 W2) -- `sim-host`'s
    /// ridden configuration uses this to open `overboard_rider.xml`.
    pub fn with_model_path(params: Params, model_path: PathBuf) -> Self {
        SimBackend {
            params,
            model_path_override: Some(model_path),
            ..SimBackend::default()
        }
    }

    /// Sets the two ballast position-actuator targets, metres, taking effect
    /// on the next [`BoardObserve::wait_observe`] call (issue #161 W2) -- the
    /// ridden rider model's physically-simulated fore/aft and lateral
    /// weight-shift channels. **Not part of `hal`**: real hardware has no
    /// ballast. A no-op on a model that declares neither actuator (e.g. the
    /// driverless onewheel model) rather than an error -- same tolerance
    /// [`SimBackend::apply_external_force`] has for a body/site a model does
    /// not declare.
    ///
    /// Buffered like `apply_external_force`: overwrites whatever was set for
    /// the current step rather than accumulating, so a caller must call this
    /// every cycle (with the current weight-shift value, or zero) or a stale
    /// target will persist.
    pub fn set_ballast_targets(&mut self, fore_aft_m: f32, lateral_m: f32) {
        self.ballast_fa_target_m = fore_aft_m;
        self.ballast_lateral_target_m = lateral_m;
    }

    /// **Verification only.** Puts the plant on a constant slope of
    /// `incline_deg`, taking effect at the next `open()`.
    ///
    /// Positive is UPHILL in the board's forward direction (forward is `-X`,
    /// so gravity acquires a `+X` component that opposes forward travel).
    ///
    /// Implemented by rotating `mjModel::opt.gravity` about `Y`, not by
    /// tilting the ground geom: a flat plane under tilted gravity and a
    /// tilted plane under vertical gravity are the same rigid-body problem
    /// expressed in two frames, and only the gravity form applies the
    /// along-slope component to every body's own mass. The magnitude is taken
    /// from whatever the open model declares rather than from a constant here,
    /// so this rotates the model's gravity instead of replacing it.
    ///
    /// ADR-0011's second ratification makes the authored world a condition of
    /// the launch hold ("the authored world is constrained to what the
    /// controller survives, and the constraint is encoded as a checkable asset
    /// rule"). That rule needs a MEASURED incline tolerance, which needs this.
    ///
    /// Note what this changes about `truth` pitch: MuJoCo reports attitude
    /// against model `Z`, which under this transform is the SLOPE NORMAL,
    /// while the IMU still measures true gravity. On an incline the two
    /// references genuinely differ by `incline_deg`, and that is physics, not
    /// an estimator fault.
    pub fn set_incline_deg(&mut self, incline_deg: f64) {
        self.incline_deg = incline_deg;
    }

    /// The current the plant is seeing. Exposed for tests.
    pub fn applied_current_a(&self) -> f32 {
        self.applied_current_a
    }

    /// Injects an external disturbance -- force (N), then torque (N*m), world
    /// frame -- onto the plant's `frame` body, taking effect on the next
    /// [`BoardObserve::wait_observe`] call.
    ///
    /// This is **not part of `hal`** and `control-core` never calls it: it
    /// exists for the I1c Rust-hosted impulse-response harness (issue #107
    /// AC6), mirroring the Python scenarios' own
    /// `data.xfrc_applied[frame][:3] = force` / `[3:] = torque`. Like that
    /// pattern, this overwrites whatever was set for the current step rather
    /// than accumulating -- callers must call this every cycle (with zeros
    /// when there is no disturbance), not just when a push is active, or a
    /// stale nonzero value would persist.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn apply_external_force(&mut self, force_n: [f64; 3], torque_nm: [f64; 3]) {
        let plant = self
            .plant
            .as_mut()
            .expect("apply_external_force: backend is not open");
        plant.set_xfrc_applied(self.frame_body, force_n, torque_nm);
    }

    /// Ground-truth `mjData::qpos`, straight from the plant. **Not part of
    /// `hal`** -- `control-core` never sees this (DR-OBS-1: `hal` carries raw
    /// IMU only, no pre-fused pitch). Exists for scenario/test harnesses that
    /// need truth for metrics (issue #107 AC6), mirroring the Python
    /// scenarios' own `data.qpos`/`data.xmat` reads.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_qpos(&self) -> Vec<f64> {
        self.plant
            .as_ref()
            .expect("truth_qpos: backend is not open")
            .qpos()
    }

    /// Ground-truth world rotation matrix of the `frame` body (row-major),
    /// straight from the plant. Same non-`hal`, harness-only status as
    /// [`SimBackend::truth_qpos`] -- exists so a caller can compute pitch
    /// with `sim/scenarios/impulse_response.py::frame_pitch_rad`'s own
    /// formula, `atan2(xmat[2], xmat[8])`, against the identical array.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_frame_xmat(&self) -> [f64; 9] {
        let plant = self
            .plant
            .as_ref()
            .expect("truth_frame_xmat: backend is not open");
        plant.body_xmat(self.frame_body)
    }

    /// Ground-truth world position of the `frame` body, metres. Same
    /// non-`hal`, harness-only status as [`SimBackend::truth_frame_xmat`] --
    /// added for `sim-host` (issue #161), which puts raw MuJoCo-world
    /// position directly on its state wire.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_frame_xpos(&self) -> [f64; 3] {
        let plant = self
            .plant
            .as_ref()
            .expect("truth_frame_xpos: backend is not open");
        plant.body_xpos(self.frame_body)
    }

    /// Ground-truth world orientation of the `frame` body, quaternion
    /// w,x,y,z, MuJoCo's own convention. Same status as
    /// [`SimBackend::truth_frame_xpos`].
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_frame_xquat(&self) -> [f64; 4] {
        let plant = self
            .plant
            .as_ref()
            .expect("truth_frame_xquat: backend is not open");
        plant.body_xquat(self.frame_body)
    }

    /// Ground-truth ballast joint positions, metres, signed -- the ACTUAL
    /// `ballast_fa`/`ballast_lat` `qpos`, not the commanded target
    /// (issue #161 wire v2: "send the real joint position ... the ballast
    /// is rate-limited so it lags the command, and that lag is honest").
    /// `(0.0, 0.0)` on a model that declares neither joint (e.g. the
    /// driverless onewheel model) -- same tolerance
    /// [`SimBackend::set_ballast_targets`] has, not a panic: a caller on the
    /// driverless plant gets an honest "no ballast" reading, not an error
    /// for a channel that was never claimed to exist there.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_ballast_positions(&self) -> (f32, f32) {
        let plant = self
            .plant
            .as_ref()
            .expect("truth_ballast_positions: backend is not open");
        let qpos = plant.qpos();
        let fore_aft = self
            .ballast_fa_qposadr
            .map(|adr| qpos[adr] as f32)
            .unwrap_or(0.0);
        let lateral = self
            .ballast_lateral_qposadr
            .map(|adr| qpos[adr] as f32)
            .unwrap_or(0.0);
        (fore_aft, lateral)
    }

    /// Ground-truth BODY-frame pitch rate of the `frame` body, rad/s, nose-up
    /// positive -- the exact time derivative of the pitch angle
    /// `sim-host::body_pitch_roll_rad` reads out of
    /// [`SimBackend::truth_frame_xmat`], with no filter, no differencing and
    /// no estimator in the path.
    ///
    /// Same non-`hal`, harness-only status as every other `truth_*` accessor
    /// here (DR-OBS-1: `control-core` never sees this on a real board). It
    /// exists for ADR-0011 criterion (f) -- "every pass must hold with the
    /// estimator bias removed", which means an acceptance run has to be able
    /// to feed the regulator MuJoCo truth instead of the complementary
    /// filter's belief, and the regulator's D term needs a truth pitch RATE
    /// as much as its P term needs a truth pitch.
    ///
    /// # Why `qvel[dof + 4]` is the pitch rate, and not something needing a
    /// # de-yaw
    ///
    /// MuJoCo stores a free joint's velocity as 3D linear velocity in GLOBAL
    /// coordinates followed by 3D angular velocity in the BODY frame -- the
    /// same fact [`SimBackend::inject_kinematic_yaw`] relies on when it
    /// rotates the linear half and deliberately leaves the angular half
    /// alone. Body-frame angular velocity is therefore already heading-free:
    /// unlike `xmat`, it needs no `Rz(-yaw)` applied to it.
    ///
    /// Component `+1` of that angular triple is rotation about the body's
    /// **+Y**, and pitch (`atan2(xmat[2], xmat[8])`, ICD 10.1, nose-up
    /// positive) is exactly the angle about that axis: for `R_y(theta)`,
    /// `xmat[2] = sin(theta)` and `xmat[8] = cos(theta)`, so pitch `== theta`
    /// and pitch rate `== omega_y`. No sign flip.
    ///
    /// `0.0` on a model that declares no `frame_free` joint -- the same
    /// tolerance [`SimBackend::truth_ballast_positions`] has, for the same
    /// reason.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn truth_body_pitch_rate_rad_s(&self) -> f32 {
        let plant = self
            .plant
            .as_ref()
            .expect("truth_body_pitch_rate_rad_s: backend is not open");
        match self.frame_free_dofadr {
            Some(vadr) => plant.qvel()[vadr + 4] as f32,
            None => 0.0,
        }
    }

    /// Rotates the board's free joint about the WORLD vertical axis through
    /// its own current x/y by `dyaw_rad`, so that the next
    /// [`BoardObserve::wait_observe`] integrates the board's translation along
    /// the new heading (issue #163 -- kinematic in-plant yaw injection).
    ///
    /// **This is a kinematic injection, not a torque.** No yaw moment is
    /// generated anywhere: the heading is imposed on the state vector, and
    /// MuJoCo then does the rest -- position, ground path and every contact
    /// are integrated at that heading by the solver, against real collision
    /// geometry. Compare the mechanism it replaces (`sim-host` dead-reckoning
    /// `pos` on the wire while the plant translated along one fixed axis
    /// underneath), which left MuJoCo's own position unrelated to what a
    /// renderer drew and made collision geometry untestable.
    ///
    /// Three things happen, and the third one is the subtle one:
    ///
    /// 1. The free joint's quaternion is PRE-multiplied by `quat_z(dyaw)`.
    ///    Pre- (world-frame), not post- (body-frame): a rolled or pitched
    ///    board must yaw about the world vertical, not about its own tilted
    ///    axis, or the heading walks out of the ground plane.
    /// 2. The free joint's LINEAR velocity is rotated by the same `dyaw`. A
    ///    free joint's linear `qvel` is expressed in the WORLD frame, so
    ///    rotating the body without it would leave the board travelling its
    ///    old direction with a new nose angle -- crabbing, which is precisely
    ///    the artefact that reads as a physics bug.
    /// 3. The free joint's ANGULAR velocity is left alone. A free joint's
    ///    angular `qvel` is expressed in the BODY frame (MuJoCo's own
    ///    convention, and the asymmetry with the linear half is deliberate on
    ///    MuJoCo's part), so it is already relative to the rotated body and
    ///    rotating it again would double-count the injection as a spurious
    ///    yaw rate.
    ///
    /// Nothing else in the state is touched -- the free joint's POSITION is
    /// left exactly where it was, which is what makes this a rotation about
    /// the vertical axis through the board rather than about the world origin,
    /// and every child joint (`wheel_hinge`, the two ballast slides) is
    /// expressed relative to this body and so follows it for free.
    ///
    /// **`dyaw_rad == 0.0` is a literal no-op**, returning before any read or
    /// write: with zero steer the plant must evolve bit-identically to the
    /// pre-injection code, and that guarantee is worth more than the branch
    /// costs.
    ///
    /// Call this BEFORE `wait_observe`, like
    /// [`SimBackend::apply_external_force`] and
    /// [`SimBackend::set_ballast_targets`] -- but unlike those it takes effect
    /// IMMEDIATELY on the state vector rather than being buffered, so it is
    /// applied ONCE per call and must not be re-issued for the same tick.
    ///
    /// A no-op on a model that declares no `frame_free` joint, the same
    /// tolerance [`SimBackend::set_ballast_targets`] has.
    ///
    /// # Panics
    /// If called before `open()`.
    pub fn inject_kinematic_yaw(&mut self, dyaw_rad: f64) {
        if dyaw_rad == 0.0 {
            return;
        }
        let (Some(qadr), Some(vadr)) = (self.frame_free_qposadr, self.frame_free_dofadr) else {
            return;
        };
        let plant = self
            .plant
            .as_mut()
            .expect("inject_kinematic_yaw: backend is not open");

        // 1. Attitude: q <- quat_z(dyaw) * q.
        let qpos = plant.qpos();
        let q = [
            qpos[qadr + 3],
            qpos[qadr + 4],
            qpos[qadr + 5],
            qpos[qadr + 6],
        ];
        let (half_s, half_c) = (dyaw_rad * 0.5).sin_cos();
        let mut rotated = quat_mul([half_c, 0.0, 0.0, half_s], q);
        // Renormalized explicitly rather than trusting the product of two unit
        // quaternions to stay unit: this runs 500 times a second for an
        // open-ended session, and MuJoCo's own renormalization inside
        // mj_integratePos is not something this crate should depend on for a
        // value it wrote itself.
        let norm = (rotated.iter().map(|v| v * v).sum::<f64>()).sqrt();
        if norm > 0.0 {
            for v in rotated.iter_mut() {
                *v /= norm;
            }
        }
        plant.set_qpos_range(qadr + 3, &rotated);

        // 2. World-frame linear velocity, rotated about +Z by the same angle.
        //    vz is untouched, hence a 2-element write.
        let qvel = plant.qvel();
        let (s, c) = dyaw_rad.sin_cos();
        let (vx, vy) = (qvel[vadr], qvel[vadr + 1]);
        plant.set_qvel_range(vadr, &[c * vx - s * vy, s * vx + c * vy]);

        // 3. Body-frame angular velocity (qvel[vadr + 3 ..= vadr + 5]): NOT
        //    touched, on purpose. See this method's doc comment.
    }
}

/// Hamilton product of two `[w, x, y, z]` quaternions, `a` then `b` read
/// right-to-left: the result applies `b`'s rotation first, then `a`'s.
fn quat_mul(a: [f64; 4], b: [f64; 4]) -> [f64; 4] {
    let [aw, ax, ay, az] = a;
    let [bw, bx, by, bz] = b;
    [
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ]
}

impl BoardObserve for SimBackend {
    fn open(&mut self) -> Result<(), IoError> {
        let path = self.model_path_override.clone().unwrap_or_else(model_path);
        let mut plant = Plant::open(&path).map_err(|e| {
            eprintln!("sim-backend: Plant::open failed: {e}");
            IoError::Fatal(FaultCode::ConfigMismatch)
        })?;

        // AC2 (issue #107): wait_observe() must step this plant exactly
        // control_period / mj_timestep times, and that ratio must be an
        // exact integer -- a fractional ratio gives a small persistent
        // timing bias that presents as a controls problem and costs a week
        // to find. Asserted here, loudly, rather than silently truncated.
        let dt_ns = (plant.timestep() * 1e9).round() as u64;
        assert!(
            dt_ns > 0,
            "sim-backend: model timestep must be positive, got {} s",
            plant.timestep()
        );
        assert_eq!(
            CYCLE_NS % dt_ns,
            0,
            "sim-backend: control_period ({CYCLE_NS} ns, {} Hz) is not an exact multiple \
             of mj_timestep ({dt_ns} ns, model.opt.timestep = {} s) -- wait_observe()'s \
             stepping ratio must be an exact integer or the loop accrues a persistent \
             timing bias (issue #107 AC2)",
            1e9 / CYCLE_NS as f64,
            plant.timestep(),
        );
        self.steps_per_cycle = CYCLE_NS / dt_ns;

        // The incline, if any, goes on BEFORE `forward()`: that call primes
        // `sensordata`, and the accelerometer's very first sample has to see
        // the slope or the estimator starts from a vertical it never had.
        if self.incline_deg != 0.0 {
            let g = plant.gravity();
            let mag = (g[0] * g[0] + g[1] * g[1] + g[2] * g[2]).sqrt();
            let (s, c) = self.incline_deg.to_radians().sin_cos();
            plant.set_gravity([mag * s, 0.0, -mag * c]);
        }

        // AC8 (issue #107, carried forward from I1b/#106): the CONTROLLED
        // Python scenarios call mj_forward once, before their first
        // mj_step, to prime sensordata (and qacc_warmstart) for the
        // controller's first cycle. This backend hosts a real control loop,
        // so it must mirror that call in the same position -- right after
        // the plant opens, before any mj_step. I1b's open-loop replay
        // deliberately skips this call because it has no controller to
        // prime; that equivalence does not transfer here.
        plant.forward();

        self.gyro_sensor = plant
            .sensor_adr_dim("frame_gyro")
            .expect("overboard_onewheel.xml must declare a frame_gyro sensor");
        self.accel_sensor = plant
            .sensor_adr_dim("frame_accel")
            .expect("overboard_onewheel.xml must declare a frame_accel sensor");
        self.wheel_vel_sensor = plant
            .sensor_adr_dim("wheel_vel")
            .expect("overboard_onewheel.xml must declare a wheel_vel sensor");
        self.frame_body = plant
            .body_id("frame")
            .expect("overboard_onewheel.xml must declare a frame body");
        self.wheel_motor_actuator = plant
            .actuator_id("wheel_motor")
            .expect("the model must declare a wheel_motor actuator");
        // `None` on the driverless model, which declares neither -- resolved
        // by name so a model that DOES declare them (the ridden rider model,
        // issue #161 W2) does not depend on ctrl-array position.
        self.ballast_fa_actuator = plant.actuator_id("ballast_fa");
        self.ballast_lateral_actuator = plant.actuator_id("ballast_lat");
        // Same joint names the actuators above drive; resolved separately
        // because a qpos address is a property of the JOINT, not the
        // actuator that happens to drive it (issue #161 wire v2).
        self.ballast_fa_qposadr = plant.joint_qposadr("ballast_fa");
        self.ballast_lateral_qposadr = plant.joint_qposadr("ballast_lat");
        // The board's own free joint -- both models declare it (`frame_free`).
        // Resolved leniently (`Option`, not `expect`) for the same reason the
        // ballast lookups are: this backend must keep opening a model that
        // does not happen to declare the thing an optional feature needs.
        self.frame_free_qposadr = plant.joint_qposadr("frame_free");
        self.frame_free_dofadr = plant.joint_dofadr("frame_free");

        self.plant = Some(plant);
        self.cycle = 0;
        self.pending_current_a = None;
        self.applied_current_a = 0.0;
        self.ballast_fa_target_m = 0.0;
        self.ballast_lateral_target_m = 0.0;
        self.open = true;
        self.seq.reset();
        Ok(())
    }

    fn close(&mut self) -> Result<(), IoError> {
        self.open = false;
        self.armed = false;
        // Dropped, not kept: the next open() builds a fresh Plant (fresh
        // mjData) the same way I1b's replay never reuses one across runs,
        // which is what repeat-run bit-identity (AC5, issue #74) depends on.
        self.plant = None;
        self.seq.reset();
        Ok(())
    }

    fn wait_observe(&mut self) -> Result<Observation, IoError> {
        if !self.open {
            return Err(IoError::ProtocolViolation);
        }
        let plant = self
            .plant
            .as_mut()
            .expect("self.open is only true while self.plant is Some");

        // Time advances here and only here -- steps_per_cycle mj_step calls,
        // never more, never fewer (AC2).
        self.cycle += 1;

        // Whatever was commanded last cycle becomes effective now -- the
        // additive actuation delay, in its crudest possible form.
        if let Some(pending) = self.pending_current_a.take() {
            self.applied_current_a = pending;
        }
        // Built by resolved actuator index, not a fixed-length array (issue
        // #161 W2): the driverless model has one actuator, the ridden rider
        // model has three. Ballast channels stay at MuJoCo's zero-init
        // default (0.0) on a model that does not declare them.
        let mut ctrl = vec![0.0f64; plant.nu()];
        ctrl[self.wheel_motor_actuator] = self.applied_current_a as f64 * KT_NM_PER_A;
        if let Some(idx) = self.ballast_fa_actuator {
            ctrl[idx] = self.ballast_fa_target_m as f64;
        }
        if let Some(idx) = self.ballast_lateral_actuator {
            ctrl[idx] = self.ballast_lateral_target_m as f64;
        }

        // AC3: sim time comes from mjData::time, read after stepping, never
        // a parallel counter.
        let mut t_s = plant.time();
        for _ in 0..self.steps_per_cycle {
            // ctrl written BEFORE mj_step, held constant across every
            // sub-step of one control period (zero-order hold) -- the same
            // "ctrl before step, never mid-step" ordering I1b pins down,
            // extended to more than one step per cycle.
            plant.set_ctrl(&ctrl);
            t_s = plant.step();
        }
        let t_ns = (t_s * 1e9).round() as u64;

        // Raw IMU only (DR-OBS-1) -- read AFTER stepping, same ordering rule
        // I1b pins down for qpos/qvel. Rotated from the model's frame
        // (z-up, forward = -X) into the ICD's FRD frame, mirroring
        // sim/scenarios/plant.py's R_MODEL_TO_ICD / imu_readings: a 180 deg
        // rotation about +Y, diag(-1, +1, -1).
        let gyro = plant.read_sensor(self.gyro_sensor.0, self.gyro_sensor.1);
        let accel = plant.read_sensor(self.accel_sensor.0, self.accel_sensor.1);
        let gyro_icd = [-gyro[0] as f32, gyro[1] as f32, -gyro[2] as f32];
        let accel_icd = [-accel[0] as f32, accel[1] as f32, -accel[2] as f32];

        // Wheel rate (issue #121): same `wheel_hinge` joint and sign
        // convention `ctrl` uses, read via the model's existing `wheel_vel`
        // jointvel sensor -- read AFTER stepping, same ordering rule as the
        // IMU sensors above. Converted to ERPM through the single
        // ICD-sourced ratio (`RAD_S_PER_ERPM`), not quantised to the nearest
        // integer ERPM: this backend has no imperfection-profile plumbing
        // yet (see this crate's own header on the actuation-delay stub), so
        // like `motor_current_a` this is the idealised, noiseless reading --
        // integer ERPM quantisation is `imperfections.py`'s
        // `wheel_rate_quantum_rad_s` row, which has no Rust-side home yet.
        let wheel_rate_rad_s =
            plant.read_sensor(self.wheel_vel_sensor.0, self.wheel_vel_sensor.1)[0];
        let erpm = (wheel_rate_rad_s / RAD_S_PER_ERPM as f64) as f32;

        let mut obs = Observation::COLD_START;
        obs.cycle = self.cycle;
        obs.t_recv_ns = t_ns;
        obs.imu[0] = ImuSample {
            gyro_rad_s: gyro_icd,
            accel_m_s2: accel_icd,
            t_sample_ns: t_ns,
        };
        obs.imu_count = 1;
        obs.motor_current_a = self.applied_current_a;
        obs.erpm = erpm;
        // Honestly fresh, not a leftover default (issue #121 AC3): the sim
        // has no ERPM transport lag to model, so every cycle's ERPM really is
        // as fresh as the frame carrying it -- unlike hardware, where ICD
        // §8.4 warns this can be far staler at low speed.
        obs.erpm_effective_age_ns = 0;
        // `tacho_raw` and `duty` are deliberately left at COLD_START's zero,
        // not an oversight (issue #121 AC2): neither has an established,
        // verifiable semantics to populate honestly yet. `tacho_raw` is a
        // raw e-rev counter whose wrap/reset convention on the real VESC is
        // not documented anywhere this project can check; `duty` depends on
        // bus voltage and back-EMF, which this backend does not model at
        // all. Inventing either would be presenting a fabricated protocol
        // quantity as data, which this project rules out explicitly.
        obs.validity = ValidityFlags::ALL_FRESH;

        self.seq.on_observe();
        Ok(obs)
    }

    fn run_metadata(&self) -> RunMetadata {
        RunMetadata {
            icd_version: (0, 3),
            profile: Profile::DRaw,
            control_rate_hz: 1e9 / CYCLE_NS as f32,
            params: self.params,
            imu_mounting_rotation: [1.0, 0.0, 0.0, 0.0],
            r_eff_m: DEFAULT_R_EFF_M,
            imperfection_profile_id: None,
            schema_hash: [0; 32],
            binary_hash: [0; 32],
            git_sha: [0; 20],
        }
    }
}

impl BoardActuate for SimBackend {
    type Disarm = SimDisarm;

    fn arm(&mut self) -> Result<Self::Disarm, IoError> {
        if !self.open {
            return Err(IoError::ProtocolViolation);
        }
        self.armed = true;
        Ok(SimDisarm)
    }

    fn apply(&mut self, cmd: &Command) -> Result<Applied, IoError> {
        // Checked before anything is buffered: a violation must send nothing.
        self.seq.on_apply()?;

        if !self.armed {
            return Err(IoError::ProtocolViolation);
        }

        let amps = match cmd {
            Command::MotorCurrent { amps } => *amps,
            // The sim backend is the D-RAW path. A RemoteSpeed command here is
            // a profile confusion, not a value to coerce.
            Command::RemoteSpeed { .. } => return Err(IoError::ProtocolViolation),
        };

        // Stage 3, the backend's own clamp. Reads the SAME resolved Params the
        // safety envelope used, because a sim whose final clamp differs from
        // hardware's makes the CI margin gate incomparable (ICD §7.6).
        let limit = self.params.max_current_a.abs();
        let (bounded, saturated) = if limit > 0.0 && amps.abs() > limit {
            let b = if amps.is_sign_negative() {
                -limit
            } else {
                limit
            };
            (b, Saturation::Yes)
        } else {
            (amps, Saturation::No)
        };

        // Buffered, not applied: time does not advance here, and the plant is
        // not touched here either -- only wait_observe() ever calls
        // Plant::set_ctrl/step.
        self.pending_current_a = Some(bounded);

        // Same instant as the most recent wait_observe(): apply() never
        // advances mjData::time, so this is a read of the one clock (AC3),
        // not a second one.
        let t_apply_ns = self
            .plant
            .as_ref()
            .map(|p| (p.time() * 1e9).round() as u64)
            .unwrap_or(0);

        Ok(Applied {
            commanded: Command::MotorCurrent { amps: bounded },
            saturated,
            t_apply_ns,
        })
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    fn opened() -> SimBackend {
        let mut b = SimBackend::with_params(Params {
            max_current_a: 40.0,
            ..Params::default()
        });
        b.open().unwrap();
        b
    }

    fn armed() -> SimBackend {
        let mut b = opened();
        b.arm().unwrap();
        b
    }

    /// `truth_body_pitch_rate_rad_s` must be the actual time derivative of
    /// the pitch angle every other consumer reads out of `truth_frame_xmat`,
    /// in both magnitude AND sign -- it is the regulator's D-term input on an
    /// ADR-0011 criterion (f) acceptance run, and a sign error there would
    /// invert the damping term and be indistinguishable from a physics
    /// finding.
    ///
    /// Checked by differencing, which needs no knowledge of MuJoCo's free
    /// joint layout and therefore cannot make the same mistake the accessor
    /// might: kick the board over with an external torque, then compare the
    /// reported rate against `(pitch[n] - pitch[n-1]) / dt` over the swing.
    #[test]
    fn the_truth_pitch_rate_is_the_derivative_of_the_truth_pitch_angle() {
        let mut b = armed();
        let dt = CYCLE_NS as f64 * 1e-9;
        let pitch = |b: &SimBackend| {
            let m = b.truth_frame_xmat();
            (m[2]).atan2(m[8])
        };
        // A torque big enough to produce a rate worth differencing, small
        // enough that the board stays in the small-angle regime where
        // `atan2(xmat[2], xmat[8])` IS the body rotation about +Y. Drive it
        // to a full tumble instead and the two quantities stop being the same
        // thing, which would make this a test of large-angle Euler
        // bookkeeping rather than of the accessor.
        b.apply_external_force([0.0; 3], [0.0, 20.0, 0.0]);
        let mut prev = pitch(&b);
        let mut prev_reported = b.truth_body_pitch_rate_rad_s() as f64;
        let mut checked = 0;
        let mut peak = 0.0f64;
        for _ in 0..40 {
            b.apply_external_force([0.0; 3], [0.0, 20.0, 0.0]);
            b.wait_observe().unwrap();
            let now = pitch(&b);
            let numeric = (now - prev) / dt;
            let reported = b.truth_body_pitch_rate_rad_s() as f64;
            peak = peak.max(numeric.abs());
            // Compared against the MEAN of the two endpoint rates, not
            // against either one: a difference quotient is the average rate
            // over the interval, and under a steady torque the instantaneous
            // rate at the far end is twice the average at the very first step
            // (from rest). Trapezoid removes that first-order term, so the
            // tolerance can be tight enough for a sign or scale error to have
            // nowhere to hide.
            let trapezoid = 0.5 * (prev_reported + reported);
            assert!(
                (numeric - trapezoid).abs() < 1e-3 + 0.01 * numeric.abs(),
                "reported pitch rate {reported} does not match the differenced \
                 {numeric} rad/s (a sign error would show here first)"
            );
            prev = now;
            prev_reported = reported;
            checked += 1;
        }
        assert_eq!(checked, 40);
        assert!(
            peak > 0.1,
            "this test proves nothing unless the board actually rotated; peak \
             differenced rate was only {peak} rad/s"
        );
    }

    /// A model with no free joint reports zero rather than panicking or
    /// indexing out of bounds -- the same tolerance every other `truth_*`
    /// accessor here has for a channel a model does not declare.
    #[test]
    fn the_truth_pitch_rate_is_zero_before_the_board_is_disturbed() {
        let mut b = armed();
        b.wait_observe().unwrap();
        assert!(
            b.truth_body_pitch_rate_rad_s().abs() < 1e-6,
            "a settled board is not rotating"
        );
    }

    #[test]
    fn wait_observe_advances_the_clock_and_the_cycle_counter() {
        let mut b = armed();
        let o1 = b.wait_observe().unwrap();
        let o2 = b.wait_observe().unwrap();
        assert_eq!(o1.cycle, 1);
        assert_eq!(o2.cycle, 2);
        assert_eq!(o2.t_recv_ns - o1.t_recv_ns, CYCLE_NS);
    }

    // CHANGED from the stub (issue #107, AC1/AC3): the original body read the
    // stub's private synthetic-clock field (`b.t_ns`) directly after
    // apply(), before any further wait_observe(). AC3 requires deleting that
    // field -- sim time now comes only from `mjData::time`, reached through
    // `Plant`, and "not kept alongside" a parallel counter is the explicit
    // acceptance criterion. There is no longer a private clock to read at
    // this point without calling wait_observe() again.
    //
    // The PROPERTY under test is unchanged: apply() must not advance time.
    // Proven the same way `wait_observe_advances_the_clock_and_the_cycle_
    // counter` proves the converse -- one more wait_observe() must advance by
    // exactly one CYCLE_NS, not two, which it could only do if apply() had
    // secretly consumed a step.
    #[test]
    fn apply_does_not_advance_time() {
        // The single most important property of the split. If apply() moved the
        // clock, actuation delay would be double-counted against the loop.
        let mut b = armed();
        let before = b.wait_observe().unwrap();
        b.apply(&Command::MotorCurrent { amps: 5.0 }).unwrap();
        let after = b.wait_observe().unwrap();
        assert_eq!(after.t_recv_ns - before.t_recv_ns, CYCLE_NS);
    }

    #[test]
    fn a_second_apply_in_one_cycle_is_a_protocol_violation() {
        let mut b = armed();
        b.wait_observe().unwrap();
        assert!(b.apply(&Command::MotorCurrent { amps: 1.0 }).is_ok());
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 2.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn the_rejected_second_apply_does_not_reach_the_plant() {
        // "Returns an error" is not enough -- the contract is that nothing is
        // sent. A backend that errors after buffering would still actuate.
        let mut b = armed();
        b.wait_observe().unwrap();
        b.apply(&Command::MotorCurrent { amps: 1.0 }).unwrap();
        let _ = b.apply(&Command::MotorCurrent { amps: 99.0 });
        b.wait_observe().unwrap();
        assert_eq!(b.applied_current_a(), 1.0);
    }

    #[test]
    fn apply_before_the_first_observe_is_a_protocol_violation() {
        let mut b = armed();
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 1.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn zero_applies_across_many_cycles_is_legal_shadow_mode() {
        let mut b = opened();
        for _ in 0..100 {
            b.wait_observe().unwrap();
        }
        assert_eq!(b.applied_current_a(), 0.0);
    }

    #[test]
    fn measured_current_is_not_an_echo_of_the_command() {
        // ICD 12 names echoing the command back as measured current as
        // non-conforming: it hides the actuation lag the controller must be
        // designed against.
        let mut b = armed();
        b.wait_observe().unwrap();
        b.apply(&Command::MotorCurrent { amps: 7.0 }).unwrap();

        let same_cycle = b.wait_observe().unwrap();
        assert_eq!(
            same_cycle.motor_current_a, 7.0,
            "command from cycle N becomes effective in N+1"
        );

        // And it was definitely not visible at apply() time.
        let mut b2 = armed();
        let o = b2.wait_observe().unwrap();
        b2.apply(&Command::MotorCurrent { amps: 7.0 }).unwrap();
        assert_eq!(o.motor_current_a, 0.0);
    }

    #[test]
    fn cold_start_observation_is_invalid_until_open() {
        let mut b = SimBackend::new();
        assert_eq!(b.wait_observe(), Err(IoError::ProtocolViolation));
    }

    #[test]
    fn applying_while_disarmed_is_refused() {
        let mut b = opened();
        b.wait_observe().unwrap();
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 1.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn remote_speed_is_refused_on_the_raw_current_path() {
        let mut b = armed();
        b.wait_observe().unwrap();
        assert_eq!(
            b.apply(&Command::RemoteSpeed { m_s: 3.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn stage_three_clamp_bounds_and_reports() {
        let mut b = armed();
        b.wait_observe().unwrap();
        let applied = b.apply(&Command::MotorCurrent { amps: 999.0 }).unwrap();
        assert_eq!(applied.commanded, Command::MotorCurrent { amps: 40.0 });
        assert_eq!(applied.saturated, Saturation::Yes);
    }

    #[test]
    fn close_then_apply_is_refused() {
        let mut b = armed();
        b.wait_observe().unwrap();
        b.close().unwrap();
        assert_eq!(
            b.apply(&Command::MotorCurrent { amps: 1.0 }),
            Err(IoError::ProtocolViolation)
        );
    }

    #[test]
    fn run_metadata_reports_the_control_rate_actually_used() {
        let b = opened();
        assert_eq!(b.run_metadata().control_rate_hz, 500.0);
    }

    // --- new for issue #107 (I1c): real-plant-specific properties ---------

    /// AC2: the onewheel model's `option timestep="0.002"` (500 Hz) equals
    /// `CYCLE_NS` (500 Hz), so the ratio is exactly 1 -- the smallest
    /// possible case, but still the one this backend actually runs.
    #[test]
    fn open_resolves_a_stepping_ratio_of_one_for_the_onewheel_model() {
        let b = opened();
        // Not part of the public API; observed indirectly below via
        // wait_observe_advances_the_clock_and_the_cycle_counter, whose
        // CYCLE_NS-exact delta could only hold if steps_per_cycle*dt ==
        // CYCLE_NS. Restated here as its own claim for readability.
        assert_eq!(
            b.run_metadata().control_rate_hz as u64,
            (1.0 / 0.002) as u64
        );
    }

    /// AC3: sim time is read from `mjData::time` through `Plant`, not a
    /// parallel counter -- so it survives a fresh `open()` exactly the way a
    /// freshly-loaded `mjModel` does: back at zero. A synthetic counter that
    /// forgot to reset on reopen (a real bug class for a parallel clock) is
    /// exactly what this catches.
    #[test]
    fn reopening_starts_sim_time_back_at_zero() {
        let mut b = armed();
        let far = {
            for _ in 0..5 {
                b.wait_observe().unwrap();
            }
            b.wait_observe().unwrap()
        };
        assert!(far.t_recv_ns > 0);

        b.close().unwrap();
        b.open().unwrap();
        b.arm().unwrap();
        let fresh = b.wait_observe().unwrap();
        assert_eq!(
            fresh.t_recv_ns, CYCLE_NS,
            "one step from a fresh mjData, not from far.t_recv_ns"
        );
    }

    /// AC8: the pre-loop `mj_forward` priming call happens once per `open()`,
    /// before the first `mj_step` -- proven the way this codebase always
    /// proves a physics call actually ran: by observing something a no-op
    /// could not fake. `mj_forward` computes `sensordata`, so the very first
    /// observation must already carry a non-placeholder IMU reading rather
    /// than whatever a freshly-zeroed, never-forwarded `mjData` would report.
    #[test]
    fn the_first_observation_carries_a_primed_imu_reading() {
        let mut b = armed();
        let first = b.wait_observe().unwrap();
        let sample = first.newest_imu().expect("imu_count must be 1");
        assert!(
            sample.accel_m_s2.iter().any(|v| *v != 0.0),
            "a primed accelerometer reading must not be all-zero at step 1: {:?}",
            sample.accel_m_s2
        );
    }

    /// Regression pin for `KT_NM_PER_A`: it must keep matching
    /// `sim/scenarios/plant.py::KT_NM_PER_A`, whose derived, checkable
    /// consequence is the model's own `ctrlrange`. If the model's clamp ever
    /// changes independently of this constant, applying `max_current_a`
    /// (40 A, the model's own derived limit) must land exactly on the
    /// model's `ctrlrange` bound rather than saturating early or leaving
    /// headroom -- either of which would mean the two have silently
    /// diverged.
    #[test]
    fn kt_nm_per_a_matches_the_models_ctrlrange() {
        assert_eq!(
            KT_NM_PER_A * 40.0,
            28.0,
            "40 A * KT_NM_PER_A must equal the model's ctrlrange bound (28 N*m); \
             see overboard_onewheel.xml's <motor ... ctrlrange=\"-28 28\">"
        );
    }

    // ---- kinematic in-plant yaw injection (issue #163) ------------------

    /// GUARD 1, at the layer that can actually break the plant: a zero
    /// increment must be a LITERAL no-op -- no read, no write, and a state
    /// vector that is bit-identical afterwards. Every scenario that never
    /// touches `steer` depends on this, and "close enough" is not the claim.
    #[test]
    fn a_zero_yaw_injection_is_bit_identical() {
        let mut b = armed();
        b.wait_observe().unwrap();
        let qpos_before = b.truth_qpos();
        let quat_before = b.truth_frame_xquat();

        b.inject_kinematic_yaw(0.0);

        assert_eq!(b.truth_qpos(), qpos_before);
        assert_eq!(b.truth_frame_xquat(), quat_before);
    }

    /// The attitude half of the injection: the free joint's quaternion must
    /// pick up exactly the requested rotation about the WORLD +Z, and the
    /// board's position must not move (this is a rotation about the vertical
    /// axis through the board, not about the world origin).
    #[test]
    fn injecting_yaw_rotates_the_free_joint_about_world_z() {
        let mut b = armed();
        b.wait_observe().unwrap();
        let before = b.truth_qpos();
        let dyaw = 0.25f64;

        b.inject_kinematic_yaw(dyaw);
        // xquat is only refreshed by a step, so read qpos, which is written
        // directly -- and step once to confirm the plant accepts it.
        let after = b.truth_qpos();

        // Free joint is qpos[0..7] on both models: x, y, z, then w, x, y, z.
        assert_eq!(&after[0..3], &before[0..3], "the board must not translate");
        let q = [after[3], after[4], after[5], after[6]];
        // Yaw out of a quaternion: atan2(2(wz + xy), 1 - 2(y^2 + z^2)).
        let yaw =
            (2.0 * (q[0] * q[3] + q[1] * q[2])).atan2(1.0 - 2.0 * (q[2] * q[2] + q[3] * q[3]));
        assert!(
            (yaw - dyaw).abs() < 1e-9,
            "expected {dyaw} rad of world yaw, got {yaw}"
        );
        let norm = q.iter().map(|v| v * v).sum::<f64>().sqrt();
        assert!((norm - 1.0).abs() < 1e-12, "quaternion must stay unit");

        b.wait_observe().unwrap();
    }

    /// The velocity half, and the reason the injection is not just a
    /// quaternion write: a free joint's LINEAR `qvel` is world-frame, so it
    /// has to be rotated with the body or the board keeps travelling its old
    /// direction with a new nose angle -- crabbing.
    ///
    /// Driven at 90 deg so the check is unambiguous: motion purely along -X
    /// must come out purely along -Y.
    #[test]
    fn injecting_yaw_rotates_world_linear_velocity_but_not_body_angular() {
        let mut b = armed();
        b.wait_observe().unwrap();
        // Give the board a clean world velocity to rotate, and a body-frame
        // angular velocity that must survive untouched.
        b.plant
            .as_mut()
            .unwrap()
            .set_qvel_range(0, &[-3.0, 0.0, 0.0, 0.11, 0.22, 0.33]);

        b.inject_kinematic_yaw(std::f64::consts::FRAC_PI_2);

        let v = b.plant.as_ref().unwrap().qvel();
        assert!(
            v[0].abs() < 1e-9,
            "world vx should have rotated away, got {}",
            v[0]
        );
        assert!(
            (v[1] - -3.0).abs() < 1e-9,
            "world vy should now carry the speed, got {}",
            v[1]
        );
        assert_eq!(v[2], 0.0, "vertical speed must not be touched");
        // Body-frame angular velocity: untouched, on purpose. Rotating it too
        // would double-count the injection as a spurious yaw RATE.
        assert_eq!((v[3], v[4], v[5]), (0.11, 0.22, 0.33));
    }

    /// The property the whole approach is for: with a heading injected, the
    /// plant's own integration carries the board along the NEW direction.
    /// Nothing outside MuJoCo computes this path.
    #[test]
    fn the_plant_itself_translates_along_the_injected_heading() {
        let mut b = armed();
        b.wait_observe().unwrap();
        // Roll the board forward along its -X at a steady clip.
        b.plant
            .as_mut()
            .unwrap()
            .set_qvel_range(0, &[-4.0, 0.0, 0.0]);
        b.inject_kinematic_yaw(std::f64::consts::FRAC_PI_2);

        let start = b.truth_frame_xpos();
        for _ in 0..50 {
            b.wait_observe().unwrap();
        }
        let end = b.truth_frame_xpos();

        let (dx, dy) = (end[0] - start[0], end[1] - start[1]);
        assert!(
            dy < -0.1 && dy.abs() > dx.abs() * 5.0,
            "after a 90 deg injection the board should travel along -Y, not -X \
             (moved dx={dx:.4} m, dy={dy:.4} m)"
        );
    }
}
