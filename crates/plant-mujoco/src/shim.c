// ~60-line C shim over MuJoCo's C API (issue #91 / I1a).
//
// Rust's `extern "C"` surface (src/lib.rs) binds ONLY to the functions below,
// every one of which passes mjModel*/mjData* around as an opaque `void*`.
// Rust never mirrors either struct's layout -- the C compiler here resolves
// every field offset from MuJoCo's own headers at build time, so a
// version-skew field mismatch is a build error in this file, not silent
// memory corruption on the Rust side.

#include <string.h>

#include <mujoco/mujoco.h>

const char* plant_mujoco_version_string(void) {
  return mj_versionString();
}

// Ownership: the returned pointer, if non-NULL, is an mjModel the caller
// must eventually pass to plant_mujoco_free_model. NULL means load failed;
// `error` (size `error_sz`) then holds MuJoCo's own NUL-terminated message.
void* plant_mujoco_load_model(const char* path, char* error, int error_sz) {
  return (void*)mj_loadXML(path, NULL, error, error_sz);
}

void plant_mujoco_free_model(void* model) {
  mj_deleteModel((mjModel*)model);
}

// Ownership: the returned mjData must outlive no call against `model` made
// after it is freed, and must itself be freed with plant_mujoco_free_data
// before `model` is freed with plant_mujoco_free_model.
void* plant_mujoco_make_data(void* model) {
  return (void*)mj_makeData((const mjModel*)model);
}

void plant_mujoco_free_data(void* data) {
  mj_deleteData((mjData*)data);
}

// The one call this whole crate exists to prove Rust can make.
void plant_mujoco_step(void* model, void* data) {
  mj_step((const mjModel*)model, (mjData*)data);
}

void plant_mujoco_reset_data(void* model, void* data) {
  mj_resetData((const mjModel*)model, (mjData*)data);
}

double plant_mujoco_data_time(void* data) {
  return ((mjData*)data)->time;
}

// The four functions below exist for I1b (issue #106): replaying a recorded
// `ctrl` sequence open-loop and comparing qpos/qvel/time against the
// Python-hosted plant bit-for-bit. `plant_mujoco_step` above already IS the
// full `mj_step` (never the mj_step1/mj_step2 split), so there is nothing to
// add there -- only a way to write ctrl in and read qpos/qvel out.

int plant_mujoco_nq(void* model) {
  return ((mjModel*)model)->nq;
}

int plant_mujoco_nv(void* model) {
  return ((mjModel*)model)->nv;
}

int plant_mujoco_nu(void* model) {
  return ((mjModel*)model)->nu;
}

// Ownership: `ctrl` must point to at least `n` doubles. Copies verbatim into
// data->ctrl. Callers must call this BEFORE plant_mujoco_step, never after --
// that is the "ctrl written before vs within the step" seam I1b exists to
// pin, and it mirrors the Python scenarios' `data.ctrl[...] = ...` line
// immediately preceding `mujoco.mj_step(model, data)`.
void plant_mujoco_set_ctrl(void* data, const double* ctrl, int n) {
  memcpy(((mjData*)data)->ctrl, ctrl, (size_t)n * sizeof(double));
}

// Ownership: `out` must point to at least `n` writable doubles. Callers must
// call this AFTER plant_mujoco_step, mirroring the Python scenarios reading
// state only once mj_step has returned.
void plant_mujoco_get_qpos(void* data, double* out, int n) {
  memcpy(out, ((mjData*)data)->qpos, (size_t)n * sizeof(double));
}

void plant_mujoco_get_qvel(void* data, double* out, int n) {
  memcpy(out, ((mjData*)data)->qvel, (size_t)n * sizeof(double));
}

// Everything below exists for I1c (issue #107): sim-backend's `hal`
// implementation against the real plant, plus the closed-loop Rust-hosted
// impulse-response harness that AC6 asks for.

// `mjModel::opt.timestep`, seconds. AC2's `wait_observe()` stepping-ratio
// assertion (control_period / mj_timestep) reads this rather than assuming
// it matches CYCLE_NS.
double plant_mujoco_timestep(void* model) {
  return ((mjModel*)model)->opt.timestep;
}

// The pre-loop priming call the CONTROLLED scenarios make (AC8 / issue #107's
// carried-forward criterion): populates sensordata and qacc_warmstart for the
// controller's first cycle, in the same position relative to the first
// mj_step every Python scenario uses -- once, right after mj_makeData,
// before any mj_step.
void plant_mujoco_forward(void* model, void* data) {
  mj_forward((const mjModel*)model, (mjData*)data);
}

// Sensor lookup by NAME, mirroring Python's `mujoco.mj_name2id(...,
// mjOBJ_SENSOR, name)` -- an index into `sensordata` would silently point at
// the wrong sensor if the block is ever reordered (see plant.py's
// imu_readings() docstring for exactly that history).
int plant_mujoco_sensor_id(void* model, const char* name) {
  return mj_name2id((const mjModel*)model, mjOBJ_SENSOR, name);
}

int plant_mujoco_sensor_adr(void* model, int sensor_id) {
  return ((mjModel*)model)->sensor_adr[sensor_id];
}

int plant_mujoco_sensor_dim(void* model, int sensor_id) {
  return ((mjModel*)model)->sensor_dim[sensor_id];
}

// Ownership: `out` must point to at least `n` writable doubles. Callers must
// call this AFTER plant_mujoco_step, same rule as plant_mujoco_get_qpos --
// sensordata is computed during mj_step (or by plant_mujoco_forward before
// the first one).
void plant_mujoco_get_sensordata(void* data, int adr, double* out, int n) {
  memcpy(out, ((mjData*)data)->sensordata + adr, (size_t)n * sizeof(double));
}

// Body lookup by name, for the Rust-hosted impulse harness's disturbance
// injection (AC6) -- not used by sim-backend's `hal` implementation itself.
int plant_mujoco_body_id(void* model, const char* name) {
  return mj_name2id((const mjModel*)model, mjOBJ_BODY, name);
}

// Ownership: `frc6` must point to 6 doubles (force xyz, torque xyz), mirroring
// `mjData::xfrc_applied`'s own per-body layout. Mirrors the Python scenarios'
// `data.xfrc_applied[body][:3] = force; data.xfrc_applied[body][3:] = torque`.
void plant_mujoco_set_xfrc_applied(void* data, int body_id, const double* frc6) {
  memcpy(((mjData*)data)->xfrc_applied + 6 * body_id, frc6, 6 * sizeof(double));
}

// Ownership: `out` must point to 9 writable doubles. Row-major 3x3 rotation
// matrix, exactly `data.xmat[body_id].reshape(3, 3)` on the Python side --
// this exists so the I1c Rust-hosted impulse harness (AC6) can compute
// ground-truth pitch with `sim/scenarios/impulse_response.py::frame_pitch_rad`'s
// own formula (atan2(R[0,2], R[2,2])) against the SAME underlying array,
// rather than re-deriving it from the free joint's quaternion and risking a
// second, independently-wrong convention.
void plant_mujoco_get_body_xmat(void* data, int body_id, double* out) {
  memcpy(out, ((mjData*)data)->xmat + 9 * body_id, 9 * sizeof(double));
}
