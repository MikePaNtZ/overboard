/* Overboard control law, C ABI. Hand-written -- keep in sync with
 * crates/control-ffi/src/lib.rs. Small enough that a generator would cost more
 * than it saves, and the size fields make a mismatch loud rather than silent.
 *
 * Every struct leads with `size`, set by the caller to its own sizeof. The
 * callee refuses a mismatch instead of reading past the end of a struct
 * compiled against an older copy of this header.
 *
 * Angles are RADIANS, nose-up-positive (BoardIo ICD 10.1). Current is AMPS.
 */
#ifndef OVERBOARD_CONTROL_H
#define OVERBOARD_CONTROL_H

#include <stdint.h>

#define OB_OK             0
#define OB_ERR_NULL      -1
#define OB_ERR_SIZE      -2
#define OB_ERR_NOT_FINITE -3

typedef struct {
    uint32_t size;
    float    kp_a_per_rad;
    float    kd_a_per_rad_s;
    float    max_current_a;

    /* Outer velocity loop. Zero both gains to disable it. */
    float    kp_v_rad_per_m_s;
    float    ki_v_rad_per_m;
    float    max_pitch_ref_rad;
    float    v_ref_m_s;
    float    r_eff_m;
    /* 1 = centre of mass ABOVE the axle (ridden), 0 = below (driverless).
     * The pitch-to-velocity coupling genuinely inverts between the two; this
     * is a property of the vehicle, not a tuning knob. */
    uint32_t com_above_axle;
} ob_params_v1;

typedef struct {
    uint32_t size;
    uint64_t t_ns;
    float    pitch_rad;         /* nose-up positive */
    float    pitch_rate_rad_s;  /* nose-up positive */
    float    wheel_rate_rad_s;  /* positive = forward */
    float    motor_current_a;   /* measured, not commanded */
} ob_obs_v1;

typedef struct {
    uint32_t size;
    float    amps;
    uint32_t saturated;         /* 0 no, 1 yes, 2 unknown */
    float    pitch_ref_rad;      /* what the outer loop asked for */
} ob_cmd_v1;

typedef struct ob_controller ob_controller;

uint32_t       ob_abi_version(void);
ob_controller *ob_controller_new(const ob_params_v1 *params);
int32_t        ob_controller_arm(ob_controller *h);
void           ob_controller_free(ob_controller *h);
int32_t        ob_controller_update(ob_controller *h,
                                    const ob_obs_v1 *obs,
                                    ob_cmd_v1 *out);

#endif /* OVERBOARD_CONTROL_H */
