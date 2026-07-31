//! I1a (issue #91): proves Rust can call `mj_step` at all. Extended for I1b
//! (issue #106) with the `ctrl`-in / `qpos`+`qvel`-out surface a bit-for-bit
//! open-loop replay against the Python-hosted plant needs (see
//! `crates/plant-mujoco/README.md`'s "Ordering contract" section, and
//! `tests/test_plant_equivalence.py`).
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
}
