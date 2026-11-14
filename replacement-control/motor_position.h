#ifndef MOTOR_POSITION_H
#define MOTOR_POSITION_H
#include <stdbool.h>
#include <stdint.h>

enum { MOTOR_POSITION_FULL = 1000000 };
enum motor_position_direction {
    MOTOR_POSITION_NONE, MOTOR_POSITION_OPENING, MOTOR_POSITION_CLOSING
};
struct motor_position {
    uint32_t opening_ticks, closing_ticks;
    uint32_t position_ppm, origin_ppm, on_ticks, token;
    uint16_t requested_permille;
    uint8_t direction;
    bool requested_known, valid, active;
};
struct motor_position_plan {
    uint32_t token, required_ticks;
    uint8_t direction;
};

/* Pure accounting, no hardware or physical-success decision. Serialize calls.
 * valid means an available TIMED ESTIMATE, never an encoder/endpoint reading.
 * Init once per new lifetime, with no old driver results still deliverable. */
void motor_position_init(struct motor_position *);
/* Caller explicitly confirms stopped at CLOSED(0) or OPEN(1000), and supplies
 * independently validated full-travel durations in one tick unit,1..INT32_MAX.
 * Only this operation establishes a reference. It preserves request/token. */
bool motor_position_reference(struct motor_position *, uint32_t opening_ticks,
                              uint32_t closing_ticks, uint16_t endpoint_permille);
/* One active plan only. Tokens are nonzero and strictly increase; no wrap.
 * A valid request while position is unknown is remembered, but returns false.
 * Other rejected plans do not change state. Output is zero on rejection.
 * required_ticks is a ceil model budget, not a motor command/deadline. */
bool motor_position_plan(struct motor_position *, uint32_t token,
                         uint16_t requested_permille, struct motor_position_plan *);
/* Current move's cumulative ENERGIZED ticks, excluding all off/settle/wait time.
 * Duplicate counts are idempotent; decreasing counts or >one full travel makes
 * position unknown. stopped is caller-proved stop, NOT target/endpoint success.
 * True means accepted accounting only. Stale tokens leave current state intact. */
bool motor_position_observe(struct motor_position *, uint32_t token,
                            uint32_t cumulative_on_ticks, bool stopped);
/* Abort/reset/unaccounted movement: call AFTER arranging hardware stop elsewhere.
 * Keep request, numeric history and token; never invent a known zero position. */
void motor_position_invalidate(struct motor_position *);
#endif
