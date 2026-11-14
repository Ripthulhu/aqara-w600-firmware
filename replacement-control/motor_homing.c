#include "motor_homing.h"

#define HALF_RANGE UINT32_C(0x80000000)

static bool active(const struct motor_homing *h)
{
    return h->state==MOTOR_HOMING_SEEK_MOTION || h->state==MOTOR_HOMING_SEEK_ENDPOINT;
}
static void guard_result(struct motor_homing *h)
{
    if (h->guard.error!=MOTOR_GUARD_OK) {
        if (h->error==MOTOR_HOMING_OK) h->error=MOTOR_HOMING_GUARD_FAILURE;
        h->state=MOTOR_HOMING_FAILED;
    }
}
static void fail(struct motor_homing *h,enum motor_homing_error error)
{
    if (h->error==MOTOR_HOMING_OK) h->error=error;
    motor_guard_trip(&h->guard);
    h->state=MOTOR_HOMING_FAILED;
}
static void new_motion_streak(struct motor_homing *h)
{
    h->motion_count=0; h->endpoint_count=0;
    h->motion_started_at=0; h->endpoint_started_at=0;
    h->state=MOTOR_HOMING_SEEK_MOTION;
}
bool motor_homing_policy_valid(const struct motor_homing_policy *p)
{
    return p && p->max_move_ticks && p->max_move_ticks<HALF_RANGE &&
        p->foreground_lease_ticks && p->foreground_lease_ticks<=p->max_move_ticks &&
        p->feedback_lease_ticks && p->feedback_lease_ticks<=p->max_move_ticks &&
        p->max_sample_age_ticks && p->max_sample_age_ticks<=p->feedback_lease_ticks &&
        p->motion_samples && p->endpoint_samples && p->motion_dwell_ticks &&
        p->motion_dwell_ticks<p->max_move_ticks && p->endpoint_dwell_ticks &&
        p->endpoint_dwell_ticks<p->max_move_ticks-p->motion_dwell_ticks &&
        p->endpoint_max<p->motion_min && p->motion_min<=p->signal_max;
}
bool motor_homing_init(struct motor_homing *h,void (*stop)(void *),void *context)
{
    *h=(struct motor_homing){0};
    if (!motor_guard_init(&h->guard,stop,context)) {
        guard_result(h);
        return false;
    }
    return true;
}
bool motor_homing_start(struct motor_homing *h,uint32_t now,
                        const struct motor_homing_policy *policy)
{
    if (h->state!=MOTOR_HOMING_IDLE) {
        if (h->guard.armed)
            (void)motor_guard_arm(&h->guard,now,0,0,0);
        else
            motor_guard_disarm(&h->guard);
        guard_result(h);
        return false;
    }
    if (!motor_homing_policy_valid(policy)) {
        fail(h,MOTOR_HOMING_BAD_POLICY);
        return false;
    }
    if (!motor_guard_arm(&h->guard,now,policy->max_move_ticks,
                        policy->foreground_lease_ticks,policy->feedback_lease_ticks)) {
        guard_result(h);
        return false;
    }
    h->policy=*policy;
    h->started_at=now;
    h->have_sample=false;
    new_motion_streak(h);
    return true;
}
void motor_homing_tick(struct motor_homing *h,uint32_t now)
{
    motor_guard_tick(&h->guard,now);
    guard_result(h);
    /* Receipt progress cannot hide a stale acquisition; guard calls use now only. */
    if (active(h) && h->have_sample &&
        (uint32_t)(now-(h->last_sample_at+h->policy.feedback_lease_ticks))<HALF_RANGE)
        fail(h,MOTOR_HOMING_STALE_FEEDBACK);
}
bool motor_homing_foreground_progress(struct motor_homing *h,uint32_t now)
{
    motor_homing_tick(h,now);
    if (!active(h)) return false;
    bool ok=motor_guard_foreground_progress(&h->guard,now);
    guard_result(h);
    return ok;
}
bool motor_homing_feedback(struct motor_homing *h,uint32_t now,
                          uint32_t measured_at,uint32_t value,bool valid)
{
    motor_homing_tick(h,now);
    if (!active(h)) return false;
    if (!valid || value>h->policy.signal_max) {
        fail(h,MOTOR_HOMING_INVALID_FEEDBACK);
        return false;
    }
    if ((uint32_t)(now-measured_at)>=h->policy.max_sample_age_ticks ||
        (uint32_t)(measured_at-h->started_at)>=HALF_RANGE ||
        (h->have_sample && ((uint32_t)(measured_at-h->last_sample_at)==0 ||
                           (uint32_t)(measured_at-h->last_sample_at)>=HALF_RANGE))) {
        fail(h,MOTOR_HOMING_STALE_FEEDBACK);
        return false;
    }
    if (!motor_guard_adc_progress(&h->guard,now)) {
        guard_result(h);
        return false;
    }
    h->last_sample_at=measured_at;
    h->have_sample=true;
    motor_homing_tick(h,now);
    if (!active(h)) return false;
    if (value>=h->policy.motion_min) {
        if (h->state==MOTOR_HOMING_SEEK_ENDPOINT) {
            h->endpoint_count=0; h->endpoint_started_at=0;
            return true;
        }
        if (!h->motion_count) h->motion_started_at=measured_at;
        if (h->motion_count<h->policy.motion_samples) ++h->motion_count;
        if (h->motion_count>=h->policy.motion_samples &&
            (uint32_t)(measured_at-h->motion_started_at)>=h->policy.motion_dwell_ticks)
            h->state=MOTOR_HOMING_SEEK_ENDPOINT;
    } else if (value<=h->policy.endpoint_max && h->state==MOTOR_HOMING_SEEK_ENDPOINT) {
        if (!h->endpoint_count) h->endpoint_started_at=measured_at;
        if (h->endpoint_count<h->policy.endpoint_samples) ++h->endpoint_count;
        if (h->endpoint_count>=h->policy.endpoint_samples &&
            (uint32_t)(measured_at-h->endpoint_started_at)>=h->policy.endpoint_dwell_ticks) {
            motor_guard_disarm(&h->guard);
            h->state=MOTOR_HOMING_ENDPOINT_OBSERVED;
        }
    } else {
        /* End-stop slowdown crosses the ambiguous band after established motion. */
        if (h->state==MOTOR_HOMING_SEEK_ENDPOINT) {
            h->endpoint_count=0; h->endpoint_started_at=0;
        } else new_motion_streak(h);
    }
    return true;
}
void motor_homing_abort(struct motor_homing *h)
{
    if (active(h)) fail(h,MOTOR_HOMING_ABORTED);
    else motor_guard_disarm(&h->guard);
}
bool motor_homing_reset(struct motor_homing *h,bool motor_is_stopped)
{
    if (!motor_guard_reset(&h->guard,motor_is_stopped)) {
        guard_result(h);
        return false;
    }
    h->policy=(struct motor_homing_policy){0};
    h->error=MOTOR_HOMING_OK; h->have_sample=false;
    h->started_at=0; h->last_sample_at=0;
    new_motion_streak(h);
    h->state=MOTOR_HOMING_IDLE;
    return true;
}
