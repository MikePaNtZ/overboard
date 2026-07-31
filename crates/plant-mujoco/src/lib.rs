//! I1a (issue #91): proves Rust can call `mj_step` at all.
//!
//! [`Plant`] is a thin owning wrapper around one `mjModel` + one `mjData`,
//! reached only through `src/shim.c`'s ~60-line, 8-function C surface
//! (`build.rs` compiles it against MuJoCo's own headers, so field offsets are
//! resolved by the C compiler rather than hand-mirrored in Rust). Nothing
//! here implements the `hal` seam, is compared against the Python-hosted
//! plant, or changes the control law -- that is I1b (#106) and I1c (#107).
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
}
