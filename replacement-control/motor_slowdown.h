#ifndef MOTOR_SLOWDOWN_H
#define MOTOR_SLOWDOWN_H
#include <stdbool.h>
#include <stdint.h>

/* Provisional load-change evidence, never an endpoint or position reference. */
struct motor_slowdown {
    uint32_t last_at, high_at, high_min_mv, baseline_mv, low_at;
    uint16_t high_count;
    bool have_sample, low, candidate;
};
void motor_slowdown_reset(struct motor_slowdown *);
/* 100us timestamps and validated pre-stop BEMF, as in the existing driver. */
bool motor_slowdown_observe(struct motor_slowdown *,uint32_t at,uint32_t mv);

/* Closing-seat candidate: sustained load valley followed by sustained recovery.
 * Calibration must repeat it after backing off; it is not a force measurement. */
struct motor_seat {
    uint32_t last_at, window_at, sum;
    uint16_t window_min_mv, plateau_count;
    uint32_t baseline_mv;
    uint16_t trough_mv, plateau_mv;
    uint16_t count, below_count, recovery_count;
    bool have_sample, candidate;
    uint16_t return_min_mv, return_max_mv;
};
void motor_seat_reset(struct motor_seat *);
bool motor_seat_observe(struct motor_seat *,uint32_t at,uint32_t mv);
/* Return means must match the original load and share one retained 100mV band. */
bool motor_seat_return_observe(struct motor_seat *,uint32_t at,uint32_t mv,uint32_t expected_mv);
/* Separate sustained-load candidate; calibration must back off and repeat it. */
bool motor_seat_loaded(const struct motor_seat *);
#endif
