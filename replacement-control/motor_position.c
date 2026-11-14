#include "motor_position.h"
#include <limits.h>

static bool durations_valid(uint32_t opening, uint32_t closing)
{
    return opening && closing && opening <= INT32_MAX && closing <= INT32_MAX;
}

void motor_position_init(struct motor_position *p)
{
    if (p) *p = (struct motor_position){0};
}

void motor_position_invalidate(struct motor_position *p)
{
    if (!p) return;
    p->valid = false;
    p->active = false;
    p->direction = MOTOR_POSITION_NONE;
}

bool motor_position_reference(struct motor_position *p, uint32_t opening_ticks,
                              uint32_t closing_ticks, uint16_t endpoint_permille)
{
    if (!p || p->active || !durations_valid(opening_ticks, closing_ticks) ||
        (endpoint_permille != 0 && endpoint_permille != 1000))
        return false;
    p->opening_ticks = opening_ticks;
    p->closing_ticks = closing_ticks;
    p->position_ppm = (uint32_t)endpoint_permille * 1000u;
    p->origin_ppm = p->position_ppm;
    p->on_ticks = 0;
    p->direction = MOTOR_POSITION_NONE;
    p->valid = true;
    return true;
}

bool motor_position_plan(struct motor_position *p, uint32_t token,
                         uint16_t requested_permille, struct motor_position_plan *out)
{
    if (out) *out = (struct motor_position_plan){0};
    if (!p || !out || p->active || !token || token <= p->token ||
        requested_permille > 1000)
        return false;
    p->requested_permille = requested_permille;
    p->requested_known = true;
    if (!p->valid) return false;
    if (!durations_valid(p->opening_ticks, p->closing_ticks) ||
        p->position_ppm > MOTOR_POSITION_FULL) {
        motor_position_invalidate(p);
        return false;
    }
    uint32_t target = (uint32_t)requested_permille * 1000u;
    uint32_t distance = target >= p->position_ppm ? target - p->position_ppm :
                                                    p->position_ppm - target;
    uint8_t direction = target > p->position_ppm ? MOTOR_POSITION_OPENING :
                        target < p->position_ppm ? MOTOR_POSITION_CLOSING :
                                                  MOTOR_POSITION_NONE;
    uint32_t full_ticks = direction == MOTOR_POSITION_CLOSING ? p->closing_ticks :
                                                               p->opening_ticks;
    /* Largest product is1,000,000*INT32_MAX; the ceil cannot exceed full_ticks. */
    uint64_t product = (uint64_t)distance * full_ticks;
    uint32_t budget = (uint32_t)((product + MOTOR_POSITION_FULL - 1u) /
                                MOTOR_POSITION_FULL);
    p->origin_ppm = p->position_ppm;
    p->on_ticks = 0;
    p->token = token;
    p->direction = direction;
    p->active = distance != 0;
    *out = (struct motor_position_plan){token, budget, direction};
    return true;
}

bool motor_position_observe(struct motor_position *p, uint32_t token,
                            uint32_t cumulative_on_ticks, bool stopped)
{
    if (!p || !p->valid || !p->active || token != p->token)
        return false;
    uint32_t full_ticks = p->direction == MOTOR_POSITION_OPENING ? p->opening_ticks :
                                                                  p->closing_ticks;
    if ((p->direction != MOTOR_POSITION_OPENING && p->direction != MOTOR_POSITION_CLOSING) ||
        !durations_valid(p->opening_ticks, p->closing_ticks) ||
        p->origin_ppm > MOTOR_POSITION_FULL || p->position_ppm > MOTOR_POSITION_FULL ||
        cumulative_on_ticks < p->on_ticks || cumulative_on_ticks > full_ticks) {
        motor_position_invalidate(p);
        return false;
    }
    /* Recompute from the move anchor: poll frequency cannot accumulate rounding. */
    uint32_t travel = (uint32_t)((uint64_t)cumulative_on_ticks * MOTOR_POSITION_FULL /
                                 full_ticks);
    if (p->direction == MOTOR_POSITION_OPENING) {
        uint32_t room = MOTOR_POSITION_FULL - p->origin_ppm;
        p->position_ppm = travel >= room ? MOTOR_POSITION_FULL : p->origin_ppm + travel;
    } else {
        p->position_ppm = travel >= p->origin_ppm ? 0 : p->origin_ppm - travel;
    }
    p->on_ticks = cumulative_on_ticks;
    if (stopped) {
        p->active = false;
        p->direction = MOTOR_POSITION_NONE;
    }
    return true;
}
