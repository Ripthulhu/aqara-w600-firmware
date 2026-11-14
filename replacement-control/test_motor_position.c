#include "motor_position.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static struct motor_position_plan plan(struct motor_position *p, uint32_t token,
                                      uint16_t target)
{
    struct motor_position_plan out = {UINT32_MAX, UINT32_MAX, UINT8_MAX};
    assert(motor_position_plan(p, token, target, &out));
    assert(p->requested_known && p->requested_permille == target);
    assert(out.token == token && out.required_ticks <=
        (out.direction == MOTOR_POSITION_CLOSING ? p->closing_ticks : p->opening_ticks));
    return out;
}

static void complete(struct motor_position *p, uint32_t token, uint16_t target)
{
    struct motor_position_plan out = plan(p, token, target);
    if (out.direction != MOTOR_POSITION_NONE)
        assert(motor_position_observe(p, token, out.required_ticks, true));
    assert(p->valid && !p->active && p->requested_permille == target);
}

static void unknown_and_explicit_reference(void)
{
    struct motor_position p;
    struct motor_position_plan out = {9, 9, 9};
    motor_position_init(&p);
    assert(!p.valid && !p.active && !p.requested_known);
    assert(!motor_position_plan(&p, 1, 400, &out));
    assert(p.requested_known && p.requested_permille == 400 && !p.valid && !out.token &&
           !out.required_ticks && out.direction == MOTOR_POSITION_NONE);
    assert(!motor_position_observe(&p, 1, 20000, true) && !p.valid);
    assert(motor_position_reference(&p, 20000, 30000, 0));
    assert(p.valid && !p.active && p.position_ppm == 0 && p.requested_permille == 400);
    complete(&p, 1, 400);
    assert(p.position_ppm == 400000);
}

static void asymmetric_roundtrips(void)
{
    const uint32_t times[][2] = {{20000,30000}, {30000,20000}, {20000,20000}};
    for (unsigned pair = 0; pair < sizeof times / sizeof *times; ++pair) {
        struct motor_position p;
        motor_position_init(&p);
        assert(motor_position_reference(&p, times[pair][0], times[pair][1], 0));
        complete(&p, 1, 500);
        assert(p.position_ppm == 500000);
        for (uint32_t i = 0; i < 10000; ++i) {
            struct motor_position_plan down = plan(&p, 2 + i * 2, 400);
            assert(down.direction == MOTOR_POSITION_CLOSING &&
                   down.required_ticks == times[pair][1] / 10);
            assert(motor_position_observe(&p, down.token, down.required_ticks, true));
            assert(p.position_ppm == 400000 && p.requested_permille == 400);
            struct motor_position_plan up = plan(&p, 3 + i * 2, 500);
            assert(up.direction == MOTOR_POSITION_OPENING &&
                   up.required_ticks == times[pair][0] / 10);
            assert(motor_position_observe(&p, up.token, up.required_ticks, true));
            assert(p.position_ppm == 500000 && p.requested_permille == 500);
        }
    }
}

static void cumulative_not_per_poll_rounding(void)
{
    struct motor_position a, b;
    motor_position_init(&a); motor_position_init(&b);
    assert(motor_position_reference(&a, 3333, 7777, 0));
    assert(motor_position_reference(&b, 3333, 7777, 0));
    struct motor_position_plan out = plan(&a, 1, 1000);
    (void)plan(&b, 1, 1000);
    for (uint32_t t = 0; t <= out.required_ticks; ++t)
        assert(motor_position_observe(&a, 1, t, t == out.required_ticks));
    assert(motor_position_observe(&b, 1, out.required_ticks, true));
    assert(a.position_ppm == b.position_ppm && a.position_ppm == MOTOR_POSITION_FULL);
    struct motor_position_plan down = plan(&a, 2, 400);
    assert(motor_position_observe(&a, 2, 1000, false));
    uint32_t position = a.position_ppm;
    assert(motor_position_observe(&a, 2, 1000, false) && a.position_ppm == position);
    assert(motor_position_observe(&a, 2, down.required_ticks, true));
    assert(a.position_ppm <= 400000 && 400000 - a.position_ppm <= 130);
}

static void quantized_roundtrips_stay_bounded(void)
{
    struct motor_position p;
    motor_position_init(&p);
    assert(motor_position_reference(&p, 20003, 30007, 0));
    complete(&p, 1, 500);
    for (uint32_t i = 0; i < 10000; ++i) {
        complete(&p, 2 + i * 2, 400);
        assert(p.position_ppm <= 400001 && p.position_ppm >= 399966);
        complete(&p, 3 + i * 2, 500);
        assert(p.position_ppm >= 499999 && p.position_ppm <= 500050);
    }
}

static void tiny_targets_and_clean_early_stop(void)
{
    struct motor_position p;
    motor_position_init(&p);
    assert(motor_position_reference(&p, 20000, 30000, 0));
    struct motor_position_plan out = plan(&p, 1, 1);
    assert(out.required_ticks == 20);
    assert(motor_position_observe(&p, 1, 10, true));
    assert(p.position_ppm == 500 && p.requested_permille == 1 && p.valid);
    out = plan(&p, 2, 1);
    assert(out.required_ticks == 10);
    assert(motor_position_observe(&p, 2, out.required_ticks, true));
    assert(p.position_ppm == 1000);
    out = plan(&p, 3, 1);
    assert(out.direction == MOTOR_POSITION_NONE && !out.required_ticks && !p.active);
    assert(!motor_position_observe(&p, 3, 0, true));
    out = plan(&p, 4, 2);
    assert(motor_position_observe(&p, 4, 0, true));
    assert(p.position_ppm == 1000 && p.requested_permille == 2);
}

static void saturation_is_estimate_not_new_reference(void)
{
    struct motor_position p;
    motor_position_init(&p);
    assert(motor_position_reference(&p, 20000, 30000, 0));
    (void)plan(&p, 1, 400);
    assert(motor_position_observe(&p, 1, 20000, true));
    assert(p.position_ppm == MOTOR_POSITION_FULL && p.requested_permille == 400);
    (void)plan(&p, 2, 500);
    assert(motor_position_observe(&p, 2, 30000, true));
    assert(p.position_ppm == 0 && p.requested_permille == 500);
    motor_position_invalidate(&p);
    assert(!p.valid && p.requested_permille == 500);
    assert(!motor_position_observe(&p, 2, 30000, true) && !p.valid);
}

static void stale_tokens_abort_and_unknown_reset(void)
{
    struct motor_position p;
    motor_position_init(&p);
    assert(motor_position_reference(&p, 20000, 30000, 0));
    (void)plan(&p, 10, 500);
    assert(motor_position_observe(&p, 10, 3000, false));
    struct motor_position before = p;
    assert(!motor_position_observe(&p, 9, 10000, true));
    assert(memcmp(&p, &before, sizeof p) == 0);
    struct motor_position_plan out;
    assert(!motor_position_plan(&p, 11, 400, &out));
    assert(memcmp(&p, &before, sizeof p) == 0);
    motor_position_invalidate(&p);
    assert(!p.valid && !p.active && p.position_ppm == 150000 && p.requested_permille == 500);
    assert(!motor_position_observe(&p, 10, 10000, true) && !p.valid);
    assert(motor_position_reference(&p, 20000, 30000, 0));
    before = p;
    assert(!motor_position_plan(&p, 10, 500, &out));
    assert(memcmp(&p, &before, sizeof p) == 0);
    (void)plan(&p, 11, 400);
    before = p;
    assert(!motor_position_observe(&p, 10, 10000, true));
    assert(memcmp(&p, &before, sizeof p) == 0);
    assert(motor_position_observe(&p, 11, 8000, true) && p.position_ppm == 400000);
    motor_position_invalidate(&p); /* Unknown reset cannot restore old position. */
    assert(!motor_position_plan(&p, 12, 500, &out) && !p.valid && p.requested_permille == 500);
    assert(!motor_position_observe(&p, 12, 20000, true) && !p.valid);
}

static void ranges_overflow_and_bad_current_progress(void)
{
    struct motor_position p;
    struct motor_position_plan out;
    motor_position_init(&p);
    const uint32_t bad[] = {0, UINT32_C(0x80000000), UINT32_MAX};
    for (unsigned i = 0; i < sizeof bad / sizeof *bad; ++i) {
        assert(!motor_position_reference(&p, bad[i], 1, 0));
        assert(!motor_position_reference(&p, 1, bad[i], 1000));
    }
    assert(!motor_position_reference(&p, 1, 1, 1));
    assert(motor_position_reference(&p, INT32_MAX, INT32_MAX, 0));
    out = plan(&p, 1, 1000);
    assert(out.required_ticks == INT32_MAX);
    assert(motor_position_observe(&p, 1, INT32_MAX, true) &&
           p.position_ppm == MOTOR_POSITION_FULL);
    out = plan(&p, UINT32_MAX, 0);
    assert(out.required_ticks == INT32_MAX);
    assert(motor_position_observe(&p, UINT32_MAX, INT32_MAX, true) && !p.position_ppm);
    struct motor_position before = p;
    assert(!motor_position_plan(&p, 0, 400, &out));
    assert(!motor_position_plan(&p, 1, 400, &out)); /* Token wrap is rejected. */
    assert(!motor_position_plan(&p, UINT32_MAX, 400, &out));
    assert(memcmp(&p, &before, sizeof p) == 0);
    motor_position_init(&p);
    assert(motor_position_reference(&p, 1, 1, 0));
    before = p;
    assert(!motor_position_plan(&p, 1, 1001, &out));
    assert(memcmp(&p, &before, sizeof p) == 0);
    out = plan(&p, 1, 1);
    assert(out.required_ticks == 1);
    assert(!motor_position_reference(&p, 2, 2, 0));
    assert(motor_position_observe(&p, 1, 1, false));
    assert(!motor_position_observe(&p, 1, 0, true) && !p.valid && !p.active &&
           p.requested_permille == 1 && p.position_ppm == MOTOR_POSITION_FULL);
    assert(motor_position_reference(&p, 2, 3, 1000));
    (void)plan(&p, 2, 400);
    assert(!motor_position_observe(&p, 2, 4, true) && !p.valid && p.requested_permille == 400);
    motor_position_init(NULL); motor_position_invalidate(NULL);
    assert(!motor_position_reference(NULL, 1, 1, 0));
    assert(!motor_position_plan(NULL, 1, 1, &out));
    assert(!motor_position_observe(NULL, 1, 1, true));
}

int main(void)
{
    unknown_and_explicit_reference();
    asymmetric_roundtrips();
    cumulative_not_per_poll_rounding();
    quantized_roundtrips_stay_bounded();
    tiny_targets_and_clean_early_stop();
    saturation_is_estimate_not_new_reference();
    stale_tokens_abort_and_unknown_reset();
    ranges_overflow_and_bad_current_progress();
    puts("PASS:8 normalized-position groups; no physical-position/calibration claim");
    return 0;
}
