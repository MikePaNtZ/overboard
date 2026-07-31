// ~60-line C shim over MuJoCo's C API (issue #91 / I1a).
//
// Rust's `extern "C"` surface (src/lib.rs) binds ONLY to the functions below,
// every one of which passes mjModel*/mjData* around as an opaque `void*`.
// Rust never mirrors either struct's layout -- the C compiler here resolves
// every field offset from MuJoCo's own headers at build time, so a
// version-skew field mismatch is a build error in this file, not silent
// memory corruption on the Rust side.

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
