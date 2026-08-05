//! I1a (issue #91): proves Rust can call `mj_step` at all. Extended for I1b
//! (issue #106) with the `ctrl`-in / `qpos`+`qvel`-out surface a bit-for-bit
//! open-loop replay against the Python-hosted plant needs (see
//! `crates/plant-mujoco/README.md`'s "Ordering contract" section, and
//! `tests/test_rust_python_plant_replay_equivalence.py`).
//!
//! [`Plant`] is a thin owning wrapper around one `mjModel` + one `mjData`,
//! reached only through `src/shim.c`'s small, opaque-handle C surface
//! (`build.rs` compiles it against MuJoCo's own headers, so field offsets are
//! resolved by the C compiler rather than hand-mirrored in Rust). Nothing
//! here implements the `hal` seam, runs a controller, or changes the control
//! law -- that is I1c (#107).
//!
//! `build.rs` links `libmujoco.so.3.10.0` (or the macOS `.dylib` of the same
//! version) straight out of the pip-installed `mujoco` wheel, so
//! [`REQUIRED_VERSION`] is asserted at [`Plant::open`] time as the STRING
//! `mj_versionString()` returns -- not `mj_version()`'s packed integer, which
//! is ambiguous between e.g. `3.1.0` and `3.10.0`.

use std::ffi::{c_char, c_int, c_void, CStr, CString};
use std::fmt;
use std::path::Path;

/// The exact `mj_versionString()` this crate is built and tested against
/// (`requirements-sim.txt`'s `mujoco==` pin). [`Plant::open`] refuses to
/// proceed against any other linked version.
pub const REQUIRED_VERSION: &str = "3.10.0";

extern "C" {
    fn plant_mujoco_version_string() -> *const c_char;
    fn plant_mujoco_load_model(
        path: *const c_char,
        error: *mut c_char,
        error_sz: c_int,
    ) -> *mut c_void;
    fn plant_mujoco_free_model(model: *mut c_void);
    fn plant_mujoco_make_data(model: *mut c_void) -> *mut c_void;
    fn plant_mujoco_free_data(data: *mut c_void);
    fn plant_mujoco_step(model: *mut c_void, data: *mut c_void);
    fn plant_mujoco_reset_data(model: *mut c_void, data: *mut c_void);
    fn plant_mujoco_data_time(data: *mut c_void) -> f64;
    fn plant_mujoco_nq(model: *mut c_void) -> c_int;
    fn plant_mujoco_nv(model: *mut c_void) -> c_int;
    fn plant_mujoco_nu(model: *mut c_void) -> c_int;
    fn plant_mujoco_set_ctrl(data: *mut c_void, ctrl: *const f64, n: c_int);
    fn plant_mujoco_get_qpos(data: *mut c_void, out: *mut f64, n: c_int);
    fn plant_mujoco_get_qvel(data: *mut c_void, out: *mut f64, n: c_int);
    fn plant_mujoco_timestep(model: *mut c_void) -> f64;
    fn plant_mujoco_get_gravity(model: *mut c_void, out: *mut f64);
    fn plant_mujoco_set_gravity(model: *mut c_void, g: *const f64);
    fn plant_mujoco_get_dof_damping(model: *mut c_void, dofadr: c_int) -> f64;
    fn plant_mujoco_set_dof_damping(model: *mut c_void, dofadr: c_int, value: f64);
    fn plant_mujoco_forward(model: *mut c_void, data: *mut c_void);
    fn plant_mujoco_sensor_id(model: *mut c_void, name: *const c_char) -> c_int;
    fn plant_mujoco_sensor_adr(model: *mut c_void, sensor_id: c_int) -> c_int;
    fn plant_mujoco_sensor_dim(model: *mut c_void, sensor_id: c_int) -> c_int;
    fn plant_mujoco_get_sensordata(data: *mut c_void, adr: c_int, out: *mut f64, n: c_int);
    fn plant_mujoco_body_id(model: *mut c_void, name: *const c_char) -> c_int;
    fn plant_mujoco_set_xfrc_applied(data: *mut c_void, body_id: c_int, frc6: *const f64);
    fn plant_mujoco_get_body_xmat(data: *mut c_void, body_id: c_int, out: *mut f64);
    fn plant_mujoco_get_body_xpos(data: *mut c_void, body_id: c_int, out: *mut f64);
    fn plant_mujoco_get_body_xquat(data: *mut c_void, body_id: c_int, out: *mut f64);
    fn plant_mujoco_actuator_id(model: *mut c_void, name: *const c_char) -> c_int;
    fn plant_mujoco_joint_id(model: *mut c_void, name: *const c_char) -> c_int;
    fn plant_mujoco_joint_qposadr(model: *mut c_void, joint_id: c_int) -> c_int;
    fn plant_mujoco_joint_dofadr(model: *mut c_void, joint_id: c_int) -> c_int;
    fn plant_mujoco_set_qpos_range(data: *mut c_void, adr: c_int, src: *const f64, n: c_int);
    fn plant_mujoco_set_qvel_range(data: *mut c_void, adr: c_int, src: *const f64, n: c_int);
}

/// The linked libmujoco's own `mj_versionString()`.
pub fn mujoco_version_string() -> String {
    // SAFETY: `mj_versionString` returns a pointer to a static,
    // NUL-terminated string compiled into libmujoco -- valid for the whole
    // program lifetime, never null.
    unsafe { CStr::from_ptr(plant_mujoco_version_string()) }
        .to_string_lossy()
        .into_owned()
}

/// Why [`Plant::open`] failed.
#[derive(Debug, Clone, PartialEq, Eq)]
pub enum OpenError {
    /// The linked libmujoco reports a version other than [`REQUIRED_VERSION`].
    VersionMismatch { found: String },
    /// `mj_loadXML` rejected the model; the string is MuJoCo's own message.
    LoadFailed(String),
}

impl fmt::Display for OpenError {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            OpenError::VersionMismatch { found } => write!(
                f,
                "linked libmujoco is {found}, this crate requires exactly {REQUIRED_VERSION}"
            ),
            OpenError::LoadFailed(msg) => write!(f, "mj_loadXML failed: {msg}"),
        }
    }
}

impl std::error::Error for OpenError {}

/// A live MuJoCo plant: one owned `mjModel` + one owned `mjData`.
///
/// `mjData` is not thread-safe -- `mj_step` mutates it in place with no
/// internal locking -- and `Plant` holds it only behind raw pointers, which
/// already makes this type neither `Send` nor `Sync` automatically (raw
/// pointers implement neither). Stated here so the property is documented
/// rather than merely incidental to the field types.
#[derive(Debug)]
pub struct Plant {
    model: *mut c_void,
    data: *mut c_void,
}

impl Plant {
    /// Asserts the linked libmujoco is exactly [`REQUIRED_VERSION`], then
    /// loads `model_path` (`mj_loadXML`) and allocates its `mjData`
    /// (`mj_makeData`).
    pub fn open(model_path: &Path) -> Result<Plant, OpenError> {
        let found = mujoco_version_string();
        if found != REQUIRED_VERSION {
            return Err(OpenError::VersionMismatch { found });
        }

        let path_c = CString::new(model_path.to_string_lossy().into_owned())
            .expect("model path must not contain a NUL byte");
        let mut error_buf: [c_char; 1024] = [0; 1024];

        // SAFETY: `path_c` is a valid NUL-terminated C string kept alive for
        // the call; `error_buf` is a valid, writable buffer of the given size
        // that `mj_loadXML` NUL-terminates on failure.
        let model = unsafe {
            plant_mujoco_load_model(
                path_c.as_ptr(),
                error_buf.as_mut_ptr(),
                error_buf.len() as c_int,
            )
        };
        if model.is_null() {
            // SAFETY: `error_buf` was NUL-terminated by the failed call above.
            let msg = unsafe { CStr::from_ptr(error_buf.as_ptr()) }
                .to_string_lossy()
                .into_owned();
            return Err(OpenError::LoadFailed(msg));
        }

        // SAFETY: `model` was just checked non-null and owns a valid mjModel
        // that outlives the `mjData` built from it, for as long as `Plant`
        // (which owns both) exists.
        let data = unsafe { plant_mujoco_make_data(model) };
        assert!(
            !data.is_null(),
            "mj_makeData returned null for a model mj_loadXML just accepted"
        );

        Ok(Plant { model, data })
    }

    /// Advances the plant by exactly one `mj_step` and returns the new
    /// simulation time (`mjData::time`, seconds) -- the one call this whole
    /// crate exists to prove Rust can make.
    pub fn step(&mut self) -> f64 {
        // SAFETY: `self.model`/`self.data` are non-null and owned for the
        // life of `self`, and `model` outlives `data` per `Plant::open`.
        unsafe {
            plant_mujoco_step(self.model, self.data);
            plant_mujoco_data_time(self.data)
        }
    }

    /// Resets state to the model's defaults (`mj_resetData`), including
    /// simulation time back to zero.
    pub fn reset(&mut self) {
        // SAFETY: see `step`.
        unsafe { plant_mujoco_reset_data(self.model, self.data) };
    }

    /// The current simulation time (`mjData::time`, seconds).
    pub fn time(&self) -> f64 {
        // SAFETY: `self.data` is non-null and owned for the life of `self`.
        unsafe { plant_mujoco_data_time(self.data) }
    }

    /// `mjModel::nq` -- the number of generalized position coordinates.
    pub fn nq(&self) -> usize {
        // SAFETY: `self.model` is non-null and owned for the life of `self`.
        unsafe { plant_mujoco_nq(self.model) as usize }
    }

    /// `mjModel::nv` -- the number of generalized velocity coordinates
    /// (degrees of freedom; not always equal to `nq`, e.g. a free joint's
    /// quaternion orientation is 4 `qpos` values but 3 `qvel` values).
    pub fn nv(&self) -> usize {
        // SAFETY: see `nq`.
        unsafe { plant_mujoco_nv(self.model) as usize }
    }

    /// `mjModel::nu` -- the number of actuators, i.e. the length `set_ctrl`
    /// expects.
    pub fn nu(&self) -> usize {
        // SAFETY: see `nq`.
        unsafe { plant_mujoco_nu(self.model) as usize }
    }

    /// Writes `ctrl` into `mjData::ctrl` (issue #106 / I1b). Callers must call
    /// this **before** [`Plant::step`], never after -- this is the "ctrl
    /// written before vs within the step" seam I1b exists to pin down, and it
    /// mirrors the Python scenarios' `data.ctrl[...] = ...` line immediately
    /// preceding `mujoco.mj_step(model, data)` (see the crate README's
    /// "Ordering contract" section).
    ///
    /// # Panics
    /// If `ctrl.len() != self.nu()`.
    pub fn set_ctrl(&mut self, ctrl: &[f64]) {
        assert_eq!(
            ctrl.len(),
            self.nu(),
            "set_ctrl: expected {} values (mjModel::nu), got {}",
            self.nu(),
            ctrl.len()
        );
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // `ctrl` is a valid slice of exactly `nu` `f64`s, matching the `n`
        // passed and what `mjData::ctrl` is sized for.
        unsafe { plant_mujoco_set_ctrl(self.data, ctrl.as_ptr(), ctrl.len() as c_int) };
    }

    /// Reads `mjData::qpos` (issue #106 / I1b). Callers must call this
    /// **after** [`Plant::step`], mirroring the Python scenarios reading
    /// state only once `mj_step` has returned -- never before, which is the
    /// "sensor/state read before vs after the step" seam I1b exists to pin
    /// down.
    pub fn qpos(&self) -> Vec<f64> {
        let mut out = vec![0.0f64; self.nq()];
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // `out` has exactly `nq` elements, matching the `n` passed.
        unsafe { plant_mujoco_get_qpos(self.data, out.as_mut_ptr(), out.len() as c_int) };
        out
    }

    /// Reads `mjData::qvel` (issue #106 / I1b). Same call-after-`step` rule as
    /// [`Plant::qpos`].
    pub fn qvel(&self) -> Vec<f64> {
        let mut out = vec![0.0f64; self.nv()];
        // SAFETY: see `qpos`.
        unsafe { plant_mujoco_get_qvel(self.data, out.as_mut_ptr(), out.len() as c_int) };
        out
    }

    /// Writes `values` into `mjData::qpos` starting at `adr` -- the ONLY write
    /// path into the state vector this crate exposes, added for the kinematic
    /// in-plant yaw injection (issue #163). See
    /// [`Plant::set_qvel_range`] for the ordering rule both share.
    ///
    /// A RANGE write rather than a whole-array one on purpose: the caller
    /// names exactly which joint's slots it means, so a mistake corrupts one
    /// joint instead of the entire state.
    ///
    /// # Panics
    /// If `adr + values.len()` exceeds `nq`.
    pub fn set_qpos_range(&mut self, adr: usize, values: &[f64]) {
        assert!(
            adr + values.len() <= self.nq(),
            "set_qpos_range: [{adr}, {}) is out of bounds for nq={}",
            adr + values.len(),
            self.nq()
        );
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // the assertion above proves the write stays inside `mjData::qpos`.
        unsafe {
            plant_mujoco_set_qpos_range(
                self.data,
                adr as c_int,
                values.as_ptr(),
                values.len() as c_int,
            )
        };
    }

    /// Writes `values` into `mjData::qvel` starting at `adr` (issue #163).
    ///
    /// **Call this BEFORE [`Plant::step`], never after.** `mj_step` runs the
    /// forward kinematics that turn a written `qpos`/`qvel` into a consistent
    /// `xpos`/`xquat`/`xmat`, so anything read between the write and the next
    /// step is stale -- the same before-vs-after-the-step seam
    /// [`Plant::set_ctrl`] already documents.
    ///
    /// # Panics
    /// If `adr + values.len()` exceeds `nv`.
    pub fn set_qvel_range(&mut self, adr: usize, values: &[f64]) {
        assert!(
            adr + values.len() <= self.nv(),
            "set_qvel_range: [{adr}, {}) is out of bounds for nv={}",
            adr + values.len(),
            self.nv()
        );
        // SAFETY: see `set_qpos_range`.
        unsafe {
            plant_mujoco_set_qvel_range(
                self.data,
                adr as c_int,
                values.as_ptr(),
                values.len() as c_int,
            )
        };
    }

    /// `mjModel::opt.timestep`, seconds -- issue #107 (I1c) AC2: `hal`'s
    /// `wait_observe()` must step this plant `control_period / mj_timestep`
    /// times, and that ratio must be an exact integer. Read from the model
    /// rather than assumed, so a model whose timestep changes trips the
    /// caller's assertion instead of silently mis-stepping.
    pub fn timestep(&self) -> f64 {
        // SAFETY: `self.model` is non-null and owned for the life of `self`.
        unsafe { plant_mujoco_timestep(self.model) }
    }

    /// `mjModel::opt.gravity`, m/s^2, world frame.
    pub fn gravity(&self) -> [f64; 3] {
        let mut out = [0.0f64; 3];
        // SAFETY: `self.model` is non-null and owned for the life of `self`;
        // `out` is exactly the 3 doubles the shim copies.
        unsafe { plant_mujoco_get_gravity(self.model, out.as_mut_ptr()) };
        out
    }

    /// Overwrites `mjModel::opt.gravity` -- **verification only**, and the
    /// only supported way to put this plant on a slope.
    ///
    /// A ground plane inclined by `phi` under vertical gravity and a flat
    /// ground plane under gravity tilted by `phi` are the same rigid-body
    /// problem in two frames, so this measures a real incline rather than
    /// approximating one. It is done here rather than by editing the model's
    /// `<geom type="plane">` because ground geometry is the fidelity
    /// contract's (`sim/models/` is Sr. Mechanical & Systems' turf) and
    /// because a *model* edit would change every scenario, not one run.
    ///
    /// Must be called before the first [`Plant::step`]: `opt.gravity` is read
    /// every step, so changing it mid-run is a step disturbance rather than an
    /// incline, and the caller almost never means that.
    ///
    /// # Panics
    /// If called after any [`Plant::step`].
    pub fn set_gravity(&mut self, g: [f64; 3]) {
        assert!(
            self.time() == 0.0,
            "Plant::set_gravity must be called before the first Plant::step (mjData::time is \
             {:.6}, not 0) -- a mid-run change is a disturbance, not an incline",
            self.time()
        );
        // SAFETY: see `gravity`; `g` is 3 doubles the shim only reads.
        unsafe { plant_mujoco_set_gravity(self.model, g.as_ptr()) };
    }

    /// `mj_forward` -- issue #107 (I1c) AC8, carried forward from I1b. Every
    /// CONTROLLED Python scenario calls this exactly once, right after
    /// building its `mjData` and before its first `mj_step`, to populate
    /// `sensordata` (and `qacc_warmstart`) for the controller's first cycle.
    /// I1b's open-loop replay deliberately skips this call -- it has no
    /// controller to prime -- but any Rust host driving a real controller
    /// must make it, in the same position, or the two hosts' first `mj_step`
    /// starts from different warmstart state (see the crate README's
    /// "Ordering contract").
    ///
    /// # Panics
    /// If called after any [`Plant::step`] -- the position relative to the
    /// first step is exactly what this call exists to pin down, so calling
    /// it "eventually" rather than "first" would defeat the point.
    pub fn forward(&mut self) {
        assert!(
            self.time() == 0.0,
            "Plant::forward must be called before the first Plant::step (mjData::time is \
             {:.6}, not 0) -- see AC8, issue #107",
            self.time()
        );
        // SAFETY: see `step`.
        unsafe { plant_mujoco_forward(self.model, self.data) };
    }

    /// `mjData::sensordata`'s address and length for the sensor named `name`,
    /// or `None` if the model has no such sensor. Looked up by NAME rather
    /// than a hardcoded offset -- an index quietly pointing at the wrong
    /// sensor is exactly the defect class `sim/scenarios/plant.py::imu_readings`
    /// documents having shipped once already.
    pub fn sensor_adr_dim(&self, name: &str) -> Option<(usize, usize)> {
        let name_c = CString::new(name).expect("sensor name must not contain a NUL byte");
        // SAFETY: `self.model` is non-null and owned for the life of `self`;
        // `name_c` is a valid NUL-terminated string kept alive for the call.
        let id = unsafe { plant_mujoco_sensor_id(self.model, name_c.as_ptr()) };
        if id < 0 {
            return None;
        }
        // SAFETY: `id` was just resolved against this same model.
        let (adr, dim) = unsafe {
            (
                plant_mujoco_sensor_adr(self.model, id),
                plant_mujoco_sensor_dim(self.model, id),
            )
        };
        Some((adr as usize, dim as usize))
    }

    /// Reads `dim` `f64`s of `mjData::sensordata` starting at `adr` (from
    /// [`Plant::sensor_adr_dim`]). Callers must call this after
    /// [`Plant::step`] or [`Plant::forward`] -- sensordata is computed by
    /// both, never populated eagerly.
    pub fn read_sensor(&self, adr: usize, dim: usize) -> Vec<f64> {
        let mut out = vec![0.0f64; dim];
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // `adr`/`dim` come from `sensor_adr_dim` against this same model, so
        // `adr + dim` is within `sensordata`'s bounds.
        unsafe {
            plant_mujoco_get_sensordata(
                self.data,
                adr as c_int,
                out.as_mut_ptr(),
                out.len() as c_int,
            )
        };
        out
    }

    /// `mjModel`'s body id for `name`, or `None` if there is no such body.
    /// Exists for the I1c Rust-hosted impulse-response harness's disturbance
    /// injection (AC6) -- `hal`'s own implementation never calls this.
    pub fn body_id(&self, name: &str) -> Option<usize> {
        let name_c = CString::new(name).expect("body name must not contain a NUL byte");
        // SAFETY: see `sensor_adr_dim`.
        let id = unsafe { plant_mujoco_body_id(self.model, name_c.as_ptr()) };
        if id < 0 {
            None
        } else {
            Some(id as usize)
        }
    }

    /// `mjModel`'s actuator id for `name`, or `None` if there is no such
    /// actuator. Exists for issue #161 W2: `sim-host` now drives models with
    /// a variable actuator count (the driverless model's single
    /// `wheel_motor` vs. the rider model's `wheel_motor` plus two ballast
    /// position actuators), so `hal`'s `wait_observe()` no longer assumes a
    /// fixed `ctrl` layout -- see `sim-backend`'s own resolve-by-name.
    pub fn actuator_id(&self, name: &str) -> Option<usize> {
        let name_c = CString::new(name).expect("actuator name must not contain a NUL byte");
        // SAFETY: see `sensor_adr_dim`.
        let id = unsafe { plant_mujoco_actuator_id(self.model, name_c.as_ptr()) };
        if id < 0 {
            None
        } else {
            Some(id as usize)
        }
    }

    /// `mjModel::jnt_qposadr` for the joint named `name`, or `None` if the
    /// model has no such joint -- the index into `Plant::qpos()`'s vector
    /// where that joint's own generalized coordinate(s) start. Exists for
    /// issue #161 wire v2: the state-out wire needs the ballast joints'
    /// ACTUAL position, and there is no sensor declared for them (no model
    /// change this pass). Correct for any 1-DOF joint (e.g. the `slide`
    /// joints `overboard_rider.xml`'s ballast uses); a multi-DOF joint (a
    /// `free` or `ball` joint) would need more than one address, which this
    /// method does not attempt to return.
    pub fn joint_qposadr(&self, name: &str) -> Option<usize> {
        let name_c = CString::new(name).expect("joint name must not contain a NUL byte");
        // SAFETY: `self.model` is non-null and owned for the life of `self`;
        // `name_c` is a valid NUL-terminated string kept alive for the call.
        let id = unsafe { plant_mujoco_joint_id(self.model, name_c.as_ptr()) };
        if id < 0 {
            return None;
        }
        // SAFETY: `id` was just resolved against this same model.
        let adr = unsafe { plant_mujoco_joint_qposadr(self.model, id) };
        Some(adr as usize)
    }

    /// `mjModel::jnt_dofadr` for the joint named `name`, or `None` if the model
    /// has no such joint -- the index into `Plant::qvel()`'s vector where that
    /// joint's own degrees of freedom start.
    ///
    /// **Not interchangeable with [`Plant::joint_qposadr`].** A free joint is 7
    /// `qpos` values but only 6 `qvel` ones, so the two addresses diverge for
    /// every joint declared after one -- reaching `qvel` through a `qpos`
    /// address silently writes the wrong slots. Added with the kinematic
    /// in-plant yaw injection (issue #163), which writes both.
    pub fn joint_dofadr(&self, name: &str) -> Option<usize> {
        let name_c = CString::new(name).expect("joint name must not contain a NUL byte");
        // SAFETY: see `joint_qposadr`.
        let id = unsafe { plant_mujoco_joint_id(self.model, name_c.as_ptr()) };
        if id < 0 {
            return None;
        }
        // SAFETY: `id` was just resolved against this same model.
        let adr = unsafe { plant_mujoco_joint_dofadr(self.model, id) };
        Some(adr as usize)
    }

    /// `mjModel::dof_damping[dofadr]` -- the passive damping coefficient
    /// MuJoCo applies to the degree of freedom at `dofadr` (from
    /// [`Plant::joint_dofadr`]). Read AND write, for the same reason
    /// [`Plant::gravity`]/[`Plant::set_gravity`] are: ADR-0011 criterion (g)
    /// needs `wheel_hinge`'s `damping="0.08"` sweepable at runtime rather than
    /// edited in the MJCF, and `dof_damping` is exactly the array that
    /// attribute compiles into.
    ///
    /// # Panics
    /// If `dofadr` is not less than [`Plant::nv`].
    pub fn dof_damping(&self, dofadr: usize) -> f64 {
        assert!(
            dofadr < self.nv(),
            "dof_damping: dofadr {dofadr} out of range (nv={})",
            self.nv()
        );
        // SAFETY: `self.model` is non-null and owned for the life of `self`;
        // `dofadr` was just checked against `nv()`.
        unsafe { plant_mujoco_get_dof_damping(self.model, dofadr as c_int) }
    }

    /// Overwrites `mjModel::dof_damping[dofadr]` -- **verification only**, the
    /// runtime hook ADR-0011 criterion (g)'s damping sweep needs. Like
    /// [`Plant::set_gravity`], scales a resolved value rather than replacing
    /// it with a literal, so it stays generic across whichever model is open.
    ///
    /// # Panics
    /// If `dofadr` is not less than [`Plant::nv`].
    pub fn set_dof_damping(&mut self, dofadr: usize, value: f64) {
        assert!(
            dofadr < self.nv(),
            "set_dof_damping: dofadr {dofadr} out of range (nv={})",
            self.nv()
        );
        // SAFETY: see `dof_damping`.
        unsafe { plant_mujoco_set_dof_damping(self.model, dofadr as c_int, value) };
    }

    /// `mjData::xmat[body_id]`, row-major, exactly `data.xmat[body].reshape(3,
    /// 3)` on the Python side. Exists so the I1c Rust-hosted impulse harness
    /// (AC6) can compute ground-truth pitch with
    /// `sim/scenarios/impulse_response.py::frame_pitch_rad`'s own formula
    /// against the same underlying array. Not part of `hal`.
    pub fn body_xmat(&self, body_id: usize) -> [f64; 9] {
        let mut out = [0.0f64; 9];
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // `out` has exactly the 9 elements `xmat`'s per-body stride expects.
        unsafe { plant_mujoco_get_body_xmat(self.data, body_id as c_int, out.as_mut_ptr()) };
        out
    }

    /// `mjData::xpos[body_id]`, exactly `data.xpos[body]` on the Python side
    /// -- world position, metres. Exists for `sim-host` (issue #161), which
    /// puts the board body's position on the wire directly rather than
    /// re-deriving it from qpos indices that depend on joint declaration
    /// order. Not part of `hal`.
    pub fn body_xpos(&self, body_id: usize) -> [f64; 3] {
        let mut out = [0.0f64; 3];
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // `out` has exactly the 3 elements `xpos`'s per-body stride expects.
        unsafe { plant_mujoco_get_body_xpos(self.data, body_id as c_int, out.as_mut_ptr()) };
        out
    }

    /// `mjData::xquat[body_id]`, w,x,y,z -- MuJoCo's own quaternion
    /// convention, exactly `data.xquat[body]` on the Python side. Same
    /// rationale and caller as [`Plant::body_xpos`].
    pub fn body_xquat(&self, body_id: usize) -> [f64; 4] {
        let mut out = [0.0f64; 4];
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // `out` has exactly the 4 elements `xquat`'s per-body stride expects.
        unsafe { plant_mujoco_get_body_xquat(self.data, body_id as c_int, out.as_mut_ptr()) };
        out
    }

    /// Sets `mjData::xfrc_applied` for `body_id` to `force` (N) then `torque`
    /// (N*m), mirroring the Python scenarios' `data.xfrc_applied[body][:3] =
    /// force; data.xfrc_applied[body][3:] = torque`. Exists for the I1c
    /// Rust-hosted impulse-response harness (AC6); not part of `hal`.
    pub fn set_xfrc_applied(&mut self, body_id: usize, force: [f64; 3], torque: [f64; 3]) {
        let frc6 = [
            force[0], force[1], force[2], torque[0], torque[1], torque[2],
        ];
        // SAFETY: `self.data` is non-null and owned for the life of `self`;
        // `body_id` is caller-provided (from `body_id()` against this same
        // model) and `frc6` has exactly the 6 values `xfrc_applied`'s
        // per-body stride expects.
        unsafe { plant_mujoco_set_xfrc_applied(self.data, body_id as c_int, frc6.as_ptr()) };
    }
}

impl Drop for Plant {
    fn drop(&mut self) {
        // SAFETY: `self.data` and `self.model` are the still-owned handles
        // `Plant::open` allocated; freed in this order because `data` was
        // built from `model` and must not outlive it.
        unsafe {
            plant_mujoco_free_data(self.data);
            plant_mujoco_free_model(self.model);
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::path::PathBuf;

    fn model_path() -> PathBuf {
        Path::new(env!("CARGO_MANIFEST_DIR")).join("../../sim/models/overboard_onewheel.xml")
    }

    /// The acceptance test for this whole crate (issue #91, AC1): load the
    /// real onewheel model and prove `mj_step` actually ran, by observing the
    /// one thing a no-op shim could not fake -- simulation time advancing.
    #[test]
    fn mj_step_advances_simulation_time() {
        let mut plant = Plant::open(&model_path()).expect("the onewheel model should load");
        assert_eq!(plant.time(), 0.0, "a freshly loaded model starts at t=0");

        let t1 = plant.step();
        assert!(t1 > 0.0, "mj_step must advance mjData::time");

        for _ in 0..9 {
            plant.step();
        }
        assert!(
            plant.time() > t1,
            "stepping repeatedly keeps advancing time"
        );
    }

    #[test]
    fn reset_returns_simulation_time_to_zero() {
        let mut plant = Plant::open(&model_path()).expect("the onewheel model should load");
        plant.step();
        assert!(plant.time() > 0.0);

        plant.reset();
        assert_eq!(plant.time(), 0.0);
    }

    #[test]
    fn linked_libmujoco_reports_the_required_version_string() {
        // The trap this exists to catch: mj_version()'s packed integer
        // encoding is ambiguous between e.g. 3.1.0 and 3.10.0. This checks
        // the string.
        assert_eq!(mujoco_version_string(), REQUIRED_VERSION);
    }

    #[test]
    fn opening_a_nonexistent_model_fails_loudly_rather_than_panicking() {
        let bogus = Path::new(env!("CARGO_MANIFEST_DIR")).join("no-such-model.xml");
        match Plant::open(&bogus) {
            Err(OpenError::LoadFailed(_)) => {}
            Err(other) => panic!("expected OpenError::LoadFailed, got {other:?}"),
            Ok(_) => panic!("expected OpenError::LoadFailed, but the nonexistent model opened"),
        }
    }

    /// The onewheel model's own dimensions, checked against known values so a
    /// silent mismatch (e.g. `nq`/`nv` swapped) fails here rather than only
    /// showing up as a confusing panic in the I1b replay binary.
    #[test]
    fn dimensions_match_the_onewheel_model() {
        let plant = Plant::open(&model_path()).expect("the onewheel model should load");
        assert_eq!(plant.nu(), 1, "one motor actuator (wheel_motor)");
        assert_eq!(plant.qpos().len(), plant.nq());
        assert_eq!(plant.qvel().len(), plant.nv());
    }

    /// `set_ctrl` actually reaches `mjData::ctrl` -- proven the same way I1a
    /// proved `mj_step` ran: by observing something a no-op shim could not
    /// fake. A held nonzero torque on an otherwise-resting board changes
    /// `qvel` measurably more than leaving `ctrl` at its default zero.
    #[test]
    fn set_ctrl_changes_the_trajectory() {
        let mut driven = Plant::open(&model_path()).expect("the onewheel model should load");
        let mut coasting = Plant::open(&model_path()).expect("the onewheel model should load");

        for _ in 0..20 {
            driven.set_ctrl(&[10.0]);
            driven.step();
            coasting.step();
        }

        assert_ne!(
            driven.qvel(),
            coasting.qvel(),
            "20 steps of nonzero wheel torque must diverge from ctrl=0 coasting"
        );
    }

    #[test]
    #[should_panic(expected = "set_ctrl: expected 1 values")]
    fn set_ctrl_panics_on_the_wrong_length() {
        let mut plant = Plant::open(&model_path()).expect("the onewheel model should load");
        plant.set_ctrl(&[1.0, 2.0]);
    }

    /// Issue #107 (I1c) AC2 reads this to compute `wait_observe()`'s stepping
    /// ratio -- pinned against the model's documented `option timestep`.
    #[test]
    fn timestep_matches_the_onewheel_models_documented_option() {
        let plant = Plant::open(&model_path()).expect("the onewheel model should load");
        assert_eq!(plant.timestep(), 0.002);
    }

    #[test]
    fn forward_populates_sensordata_before_any_step() {
        let mut plant = Plant::open(&model_path()).expect("the onewheel model should load");
        let (adr, dim) = plant
            .sensor_adr_dim("frame_accel")
            .expect("model has a frame_accel sensor");
        // Freshly made mjData has zeroed sensordata; forward() must compute
        // it without advancing time or requiring a step.
        assert_eq!(plant.read_sensor(adr, dim), vec![0.0; dim]);
        plant.forward();
        assert_eq!(
            plant.time(),
            0.0,
            "forward() must not advance simulation time"
        );
        // A single-point wheel contact has no roll restoring moment (only
        // pitch does, per the model header), so at qpos's default the frame
        // is NOT in static equilibrium and qacc -- hence specific force at an
        // offset site -- is genuinely nonzero. Assert it moved off the
        // freshly-zeroed baseline rather than an analytic "at rest" value,
        // which would assume an equilibrium this model does not have.
        let accel = plant.read_sensor(adr, dim);
        assert_ne!(
            accel,
            vec![0.0; dim],
            "forward() should have computed sensordata"
        );
        assert!(accel.iter().all(|v| v.is_finite()));
    }

    #[test]
    #[should_panic(expected = "must be called before the first Plant::step")]
    fn forward_after_a_step_panics() {
        let mut plant = Plant::open(&model_path()).expect("the onewheel model should load");
        plant.step();
        plant.forward();
    }

    #[test]
    fn unknown_sensor_name_resolves_to_none() {
        let plant = Plant::open(&model_path()).expect("the onewheel model should load");
        assert_eq!(plant.sensor_adr_dim("no_such_sensor"), None);
    }

    #[test]
    fn sensor_adr_dim_resolves_every_declared_sensor() {
        let plant = Plant::open(&model_path()).expect("the onewheel model should load");
        for (name, dim) in [
            ("frame_quat", 4),
            ("frame_gyro", 3),
            ("frame_accel", 3),
            ("wheel_pos", 1),
            ("wheel_vel", 1),
            ("motor_torque", 1),
        ] {
            let (_, got_dim) = plant
                .sensor_adr_dim(name)
                .unwrap_or_else(|| panic!("expected a sensor named {name}"));
            assert_eq!(got_dim, dim, "sensor {name} has an unexpected dimension");
        }
    }

    #[test]
    fn body_id_resolves_the_frame_body_and_rejects_an_unknown_name() {
        let plant = Plant::open(&model_path()).expect("the onewheel model should load");
        assert!(plant.body_id("frame").is_some());
        assert_eq!(plant.body_id("no_such_body"), None);
    }

    /// ADR-0011 criterion (g)'s instrument (issue #229): `wheel_hinge`'s
    /// declared `damping="0.08"` must round-trip through `dof_damping`/
    /// `set_dof_damping` exactly, or `sim-backend`'s damping sweep would be
    /// scaling a value that was never the MJCF's own.
    #[test]
    fn dof_damping_reads_the_wheel_hinges_declared_value_and_set_dof_damping_scales_it() {
        let mut plant = Plant::open(&model_path()).expect("the onewheel model should load");
        let dofadr = plant
            .joint_dofadr("wheel_hinge")
            .expect("model must declare a wheel_hinge joint");
        let base = plant.dof_damping(dofadr);
        assert!(
            (base - 0.08).abs() < 1e-9,
            "wheel_hinge's declared damping should be 0.08, got {base}"
        );

        plant.set_dof_damping(dofadr, base * 2.0);
        assert!(
            (plant.dof_damping(dofadr) - 0.16).abs() < 1e-9,
            "set_dof_damping did not persist the scaled value"
        );
    }

    #[test]
    fn body_xmat_is_the_identity_for_an_unrotated_frame_at_rest() {
        let mut plant = Plant::open(&model_path()).expect("the onewheel model should load");
        let frame = plant.body_id("frame").expect("model has a frame body");
        // xmat is a computed (kinematic) quantity, zeroed until forward() or
        // step() runs it -- same rule as sensordata.
        plant.forward();
        // The frame body has no <body ... euler=".."/quat=".."> offset, so at
        // the model's default qpos (identity free-joint orientation) its
        // world rotation is the identity matrix.
        assert_eq!(
            plant.body_xmat(frame),
            [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
        );
    }

    /// `set_xfrc_applied` actually reaches `mjData::xfrc_applied` -- proven
    /// the same way `set_ctrl_changes_the_trajectory` proves `set_ctrl`
    /// reaches `mjData::ctrl`: a pushed body diverges from an unpushed one.
    #[test]
    fn set_xfrc_applied_changes_the_trajectory() {
        let mut pushed = Plant::open(&model_path()).expect("the onewheel model should load");
        let mut coasting = Plant::open(&model_path()).expect("the onewheel model should load");
        let frame = pushed.body_id("frame").expect("model has a frame body");

        for _ in 0..20 {
            pushed.set_xfrc_applied(frame, [500.0, 0.0, 0.0], [0.0, 0.0, 0.0]);
            pushed.step();
            coasting.step();
        }

        assert_ne!(
            pushed.qvel(),
            coasting.qvel(),
            "20 steps of a held external force must diverge from an unpushed coast"
        );
    }
}
