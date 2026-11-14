#ifndef MOTOR_GUARD_H
#define MOTOR_GUARD_H

#include <stdbool.h>
#include <stdint.h>

enum motor_guard_error {
    MOTOR_GUARD_OK,
    MOTOR_GUARD_BAD_LIMITS,
    MOTOR_GUARD_BAD_STATE,
    MOTOR_GUARD_REARM,
    MOTOR_GUARD_MOVE_TIMEOUT,
    MOTOR_GUARD_FOREGROUND_TIMEOUT,
    MOTOR_GUARD_ADC_TIMEOUT,
    MOTOR_GUARD_EXTERNAL_FAULT
};

struct motor_guard {
    void (*stop)(void *context);
    void *context;
    uint32_t move_deadline, foreground_deadline, adc_deadline;
    uint32_t foreground_lease, adc_lease;
    enum motor_guard_error error;
    bool armed;
};

/* Serialize all calls. The periodic tick must run independently of foreground/ADC work. */
bool motor_guard_init(struct motor_guard *guard, void (*stop)(void *), void *context);
bool motor_guard_arm(struct motor_guard *guard, uint32_t now, uint32_t move_limit,
                     uint32_t foreground_lease, uint32_t adc_lease);
void motor_guard_tick(struct motor_guard *guard, uint32_t now);
/* Pure deadline query: does not stop, mutate, disarm or renew a lease. */
enum motor_guard_error motor_guard_due(const struct motor_guard *guard,uint32_t now);
bool motor_guard_foreground_progress(struct motor_guard *guard, uint32_t now);
bool motor_guard_adc_progress(struct motor_guard *guard, uint32_t now);
void motor_guard_trip(struct motor_guard *guard);
void motor_guard_disarm(struct motor_guard *guard);
bool motor_guard_reset(struct motor_guard *guard, bool motor_is_stopped);

#endif
