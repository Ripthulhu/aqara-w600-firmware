#ifndef MOTOR_HOMING_H
#define MOTOR_HOMING_H
#include "motor_guard.h"

/* Loaded mounted captures stay below1.1V; this qualifies feedback, not pin travel. */
enum { MOTOR_HOMING_MOTION_MIN_MV=600 };

enum motor_homing_state {
    MOTOR_HOMING_IDLE,
    MOTOR_HOMING_SEEK_MOTION,
    MOTOR_HOMING_SEEK_ENDPOINT,
    MOTOR_HOMING_ENDPOINT_OBSERVED,
    MOTOR_HOMING_FAILED
};
enum motor_homing_error {
    MOTOR_HOMING_OK,
    MOTOR_HOMING_BAD_POLICY,
    MOTOR_HOMING_INVALID_FEEDBACK,
    MOTOR_HOMING_STALE_FEEDBACK,
    MOTOR_HOMING_GUARD_FAILURE,
    MOTOR_HOMING_ABORTED
};
struct motor_homing_policy {
    uint32_t max_move_ticks, foreground_lease_ticks, feedback_lease_ticks;
    uint32_t max_sample_age_ticks;
    uint32_t motion_min, endpoint_max, signal_max;
    uint32_t motion_dwell_ticks, endpoint_dwell_ticks;
    uint16_t motion_samples, endpoint_samples;
};
struct motor_homing {
    struct motor_guard guard;
    struct motor_homing_policy policy;
    enum motor_homing_state state;
    enum motor_homing_error error;
    uint32_t started_at, last_sample_at, motion_started_at, endpoint_started_at;
    uint16_t motion_count, endpoint_count;
    bool have_sample;
};

/* Valid objects; serialize all calls. now is one monotonic uint32_t clock.
 * tick must run independently. start only arms: it never turns a motor on.
 * ENDPOINT_OBSERVED means caller-policy evidence, NOT physical calibration. */
bool motor_homing_init(struct motor_homing *, void (*stop)(void *), void *context);
/* Pure validation: no mutation, stop callback or guard arm. */
bool motor_homing_policy_valid(const struct motor_homing_policy *);
bool motor_homing_start(struct motor_homing *, uint32_t now,
                        const struct motor_homing_policy *);
void motor_homing_tick(struct motor_homing *, uint32_t now);
bool motor_homing_foreground_progress(struct motor_homing *, uint32_t now);
bool motor_homing_feedback(struct motor_homing *, uint32_t now,
                          uint32_t measured_at, uint32_t value, bool valid);
void motor_homing_abort(struct motor_homing *);
bool motor_homing_reset(struct motor_homing *, bool motor_is_stopped);
#endif
