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
    float    r_eff_m;
    /* 1 = centre of mass ABOVE the axle (ridden), 0 = below (driverless).
     * The pitch-to-velocity coupling genuinely inverts between the two; this
     * is a property of the vehicle, not a tuning knob. */
    uint32_t com_above_axle;

    /* Attitude estimator. 1 = fuse from the raw IMU, 0 = use obs.pitch_rad
     * (truth in sim). The truth path is kept so the estimator's contribution
     * to the error stays measurable on its own. */
    uint32_t use_estimator;
    float    estimator_tau_s;
    /* Source of the forward-acceleration correction subtracted from the
     * accelerometer before deriving tilt. Some correction is mandatory:
     * without one the implied tilt is wrong by atan(a/g) -- about 5 deg at the
     * outer loop's acceleration limit, correlated with what is being
     * regulated.
     *   0 = off             diagnostic only; does not survive the shuttle
     *   1 = wheel odometry  sees any acceleration, but quantised and lagged;
     *                       needs estimator_tau_s >= 10 s to be flyable
     *   2 = command feedforward   predicts a from the commanded current via
     *                       accel_ff_gain_m_s2_per_a. No lag; blind to slope
     *                       and slip. Best on the bench. */
    uint32_t estimator_accel_aiding;
    float    wheel_accel_tau_s;          /* mode 1 only */
    /* Mode 2 only: amps -> m/s^2, standing in for k_t / (r_eff * m).
     * MEASURE THIS ON THE BENCH. Zero falls back to the ridden-board value. */
    float    accel_ff_gain_m_s2_per_a;
    /* Skip the accelerometer correction when the aiding-corrected specific
     * force is further than this from g, m/s^2. Zero disables the gate.
     * Buys immunity to shoves the aiding cannot see, at the cost of running
     * open-loop on the gyro while tripped. */
    float    accel_trust_band_m_s2;
} ob_params_v1;

typedef struct {
    uint32_t size;
    uint64_t t_ns;
    float    pitch_rad;         /* nose-up positive */
    float    pitch_rate_rad_s;  /* nose-up positive */
    float    wheel_rate_rad_s;  /* positive = forward */
    float    motor_current_a;   /* measured, not commanded */
    float    gyro_rad_s[3];     /* body frame; [1] IS the nose-up pitch rate */
    float    accel_m_s2[3];    /* body frame specific force */
    float    v_ref_m_s;         /* speed setpoint for THIS cycle */
} ob_obs_v1;

typedef struct {
    uint32_t size;
    float    amps;
    uint32_t saturated;         /* 0 no, 1 yes, 2 unknown */
    float    pitch_ref_rad;     /* what the outer loop asked for */
    float    pitch_used_rad;     /* the attitude actually acted on */
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
