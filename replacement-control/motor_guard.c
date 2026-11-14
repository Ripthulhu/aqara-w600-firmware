#include "motor_guard.h"

#define HALF_RANGE UINT32_C(0x80000000)

static void stop(struct motor_guard *guard)
{
    guard->armed = false;
    if (guard->stop)
        guard->stop(guard->context);
}

static void fail(struct motor_guard *guard, enum motor_guard_error error)
{
    if (guard->error == MOTOR_GUARD_OK)
        guard->error = error;
    stop(guard);
}

static bool reached(uint32_t now, uint32_t deadline)
{
    return (uint32_t)(now - deadline) < HALF_RANGE;
}

bool motor_guard_init(struct motor_guard *guard, void (*stop_callback)(void *), void *context)
{
    *guard = (struct motor_guard){ .stop = stop_callback, .context = context };
    if (!stop_callback) {
        guard->error = MOTOR_GUARD_BAD_STATE;
        return false;
    }
    stop(guard);
    return true;
}

bool motor_guard_arm(struct motor_guard *guard, uint32_t now, uint32_t move_limit,
                     uint32_t foreground_lease, uint32_t adc_lease)
{
    if (guard->armed) {
        fail(guard, MOTOR_GUARD_REARM);
        return false;
    }
    if (guard->error != MOTOR_GUARD_OK || !guard->stop) {
        fail(guard, MOTOR_GUARD_BAD_STATE);
        return false;
    }
    if (!move_limit || move_limit >= HALF_RANGE || !foreground_lease ||
        foreground_lease > move_limit || !adc_lease || adc_lease > move_limit) {
        fail(guard, MOTOR_GUARD_BAD_LIMITS);
        return false;
    }
    guard->move_deadline = now + move_limit;
    guard->foreground_deadline = now + foreground_lease;
    guard->adc_deadline = now + adc_lease;
    guard->foreground_lease = foreground_lease;
    guard->adc_lease = adc_lease;
    guard->armed = true;
    return true;
}

enum motor_guard_error motor_guard_due(const struct motor_guard *guard,uint32_t now)
{
    if (!guard->armed)
        return MOTOR_GUARD_OK;
    if (reached(now, guard->move_deadline))
        return MOTOR_GUARD_MOVE_TIMEOUT;
    else if (reached(now, guard->foreground_deadline))
        return MOTOR_GUARD_FOREGROUND_TIMEOUT;
    else if (reached(now, guard->adc_deadline))
        return MOTOR_GUARD_ADC_TIMEOUT;
    return MOTOR_GUARD_OK;
}

void motor_guard_tick(struct motor_guard *guard, uint32_t now)
{
    enum motor_guard_error error=motor_guard_due(guard,now);
    if (error!=MOTOR_GUARD_OK) fail(guard,error);
}

static bool progress(struct motor_guard *guard, uint32_t now, bool adc)
{
    motor_guard_tick(guard, now);
    if (!guard->armed)
        return false;
    uint32_t remaining = guard->move_deadline - now;
    uint32_t lease = adc ? guard->adc_lease : guard->foreground_lease;
    uint32_t deadline = now + (lease < remaining ? lease : remaining);
    if (adc)
        guard->adc_deadline = deadline;
    else
        guard->foreground_deadline = deadline;
    return true;
}

bool motor_guard_foreground_progress(struct motor_guard *guard, uint32_t now)
{
    return progress(guard, now, false);
}

bool motor_guard_adc_progress(struct motor_guard *guard, uint32_t now)
{
    return progress(guard, now, true);
}

void motor_guard_trip(struct motor_guard *guard)
{
    fail(guard, MOTOR_GUARD_EXTERNAL_FAULT);
}

void motor_guard_disarm(struct motor_guard *guard)
{
    stop(guard);
}

bool motor_guard_reset(struct motor_guard *guard, bool motor_is_stopped)
{
    if (guard->armed || !motor_is_stopped || !guard->stop) {
        fail(guard, MOTOR_GUARD_BAD_STATE);
        return false;
    }
    guard->error = MOTOR_GUARD_OK;
    return true;
}
