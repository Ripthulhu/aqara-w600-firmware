#include "motor_guard.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

struct fixture { struct motor_guard guard; unsigned stops; bool motor_on; };

static void stopped(void *context)
{
    struct fixture *f = context;
    f->motor_on = false;
    ++f->stops;
}

static void init(struct fixture *f)
{
    *f = (struct fixture){ .motor_on = true };
    assert(motor_guard_init(&f->guard, stopped, f));
    assert(!f->motor_on && !f->guard.armed && f->stops == 1);
}

static void arm(struct fixture *f, uint32_t now, uint32_t move, uint32_t fg, uint32_t adc)
{
    assert(motor_guard_arm(&f->guard, now, move, fg, adc));
    f->motor_on = true;
}

static void deadline_and_rollover(void)
{
    const uint32_t starts[] = {0, 1, UINT32_MAX - 49, UINT32_C(0x80000000)};
    for (unsigned i = 0; i < sizeof starts / sizeof *starts; ++i) {
        struct fixture f;
        init(&f);
        arm(&f, starts[i], 100, 100, 100);
        motor_guard_tick(&f.guard, starts[i] + 99);
        assert(f.motor_on && f.guard.armed);
        motor_guard_tick(&f.guard, starts[i] + 100);
        assert(!f.motor_on && !f.guard.armed && f.stops == 2);
        assert(f.guard.error == MOTOR_GUARD_MOVE_TIMEOUT);
        motor_guard_tick(&f.guard, starts[i] + 101);
        assert(f.stops == 2);
    }
}

static void foreground_and_adc_timeouts(void)
{
    struct fixture f;
    init(&f);
    arm(&f, 0, 100, 10, 20);
    assert(motor_guard_adc_progress(&f.guard, 5));
    motor_guard_tick(&f.guard, 10);
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_FOREGROUND_TIMEOUT);
    assert(!motor_guard_foreground_progress(&f.guard, 11));

    init(&f);
    arm(&f, UINT32_MAX - 5, 100, 40, 10);
    assert(motor_guard_foreground_progress(&f.guard, UINT32_MAX));
    motor_guard_tick(&f.guard, 4);
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_ADC_TIMEOUT);

    init(&f);
    arm(&f, 0, 100, 10, 20);
    assert(!motor_guard_foreground_progress(&f.guard, 10));
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_FOREGROUND_TIMEOUT);

    init(&f);
    arm(&f, 0, 100, 20, 10);
    assert(!motor_guard_adc_progress(&f.guard, 10));
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_ADC_TIMEOUT);
}

static void leases_cannot_extend_move(void)
{
    struct fixture f;
    init(&f);
    arm(&f, 0, 100, 60, 80);
    const uint32_t deadline = f.guard.move_deadline;
    assert(motor_guard_foreground_progress(&f.guard, 50));
    assert(motor_guard_adc_progress(&f.guard, 50));
    assert(f.guard.foreground_deadline == 100 && f.guard.adc_deadline == 100);
    assert(motor_guard_foreground_progress(&f.guard, 95));
    assert(motor_guard_adc_progress(&f.guard, 95));
    assert(f.guard.move_deadline == deadline);
    assert(f.guard.foreground_deadline == 100 && f.guard.adc_deadline == 100);
    motor_guard_tick(&f.guard, 100);
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_MOVE_TIMEOUT);
}

static void invalid_limits_and_rearm_stop(void)
{
    const uint32_t invalid[][3] = {
        {0, 1, 1}, {UINT32_C(0x80000000), 1, 1}, {UINT32_MAX, 1, 1},
        {100, 0, 1}, {100, 101, 1}, {100, 1, 0}, {100, 1, 101}
    };
    for (unsigned i = 0; i < sizeof invalid / sizeof *invalid; ++i) {
        struct fixture f;
        init(&f);
        assert(!motor_guard_arm(&f.guard, 0, invalid[i][0], invalid[i][1], invalid[i][2]));
        assert(!f.motor_on && !f.guard.armed && f.guard.error == MOTOR_GUARD_BAD_LIMITS);
    }
    for (unsigned invalid_request = 0; invalid_request < 2; ++invalid_request) {
        struct fixture f;
        init(&f);
        arm(&f, 0, 100, 100, 100);
        assert(!motor_guard_arm(&f.guard, 50, invalid_request ? 0 : 100, 100, 100));
        assert(!f.motor_on && f.stops == 2 && f.guard.error == MOTOR_GUARD_REARM);
        assert(f.guard.move_deadline == 100);
    }
    struct fixture f;
    init(&f);
    arm(&f, 0, INT32_MAX, INT32_MAX, INT32_MAX);
    motor_guard_tick(&f.guard, INT32_MAX);
    assert(f.guard.error == MOTOR_GUARD_MOVE_TIMEOUT && !f.motor_on);
}

static void latched_fault_and_explicit_reset(void)
{
    struct fixture f;
    init(&f);
    arm(&f, 0, 100, 100, 100);
    motor_guard_trip(&f.guard);
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_EXTERNAL_FAULT);
    assert(!motor_guard_arm(&f.guard, 10, 100, 100, 100));
    motor_guard_disarm(&f.guard);
    assert(f.guard.error == MOTOR_GUARD_EXTERNAL_FAULT);
    assert(!motor_guard_reset(&f.guard, false));
    assert(f.guard.error == MOTOR_GUARD_EXTERNAL_FAULT);
    assert(motor_guard_reset(&f.guard, true));
    arm(&f, 20, 100, 100, 100);
    assert(!motor_guard_reset(&f.guard, true));
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_BAD_STATE);
    assert(motor_guard_reset(&f.guard, true));
    arm(&f, 30, 100, 100, 100);
    motor_guard_disarm(&f.guard);
    assert(!f.motor_on && f.guard.error == MOTOR_GUARD_OK);
    assert(!motor_guard_adc_progress(&f.guard, 31));

    struct motor_guard no_callback;
    assert(!motor_guard_init(&no_callback, NULL, NULL));
    assert(!motor_guard_arm(&no_callback, 0, 100, 100, 100));
    assert(!motor_guard_reset(&no_callback, true));
}

int main(void)
{
    deadline_and_rollover();
    foreground_and_adc_timeouts();
    leases_cannot_extend_move();
    invalid_limits_and_rearm_stop();
    latched_fault_and_explicit_reset();
    puts("motor guard tests passed");
    return 0;
}
