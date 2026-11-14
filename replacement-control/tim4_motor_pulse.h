#ifndef W600_TIM4_MOTOR_PULSE_H
#define W600_TIM4_MOTOR_PULSE_H
#include <stdint.h>

enum tim4_motor_pulse_status {
    TIM4_PULSE_IDLE, TIM4_PULSE_DRIVING, TIM4_PULSE_SETTLING,
    TIM4_PULSE_DONE, TIM4_PULSE_FAILED
};
enum tim4_motor_pulse_error {
    TIM4_PULSE_OK, TIM4_PULSE_CONTEXT, TIM4_PULSE_STATE,
    TIM4_PULSE_DIRECTION, TIM4_PULSE_CONFIG, TIM4_PULSE_PADS,
    TIM4_PULSE_POLL_LIMIT, TIM4_PULSE_TIMER, TIM4_PULSE_ABORTED,
    TIM4_PULSE_CLEANUP
};
/* Explicitly zero before first use; no hidden/static state. Serialize every call
 * in Thread mode with PRIMASK=1. Restore the caller's mask promptly afterwards.
 * Exclusive TIM4 and PB14/15 AF ownership; no other writer may remux these pins.
 * W600_DISPLAY_BENCH additionally requires and preserves UART2/UART3 clocks.
 * DONE means a configured nominal 19ms LOW pulse with LOW observed and terminal
 * HIGH proved, then a separate >=1ms GPIO-HH settle, not measured LOW duration,
 * motion,
 * endpoint, position, calibration, or electrical safety of an unknown driver. */
struct tim4_motor_pulse {
    uint32_t status, error, direction, polls, channel, owned;
    uint32_t saved_afh, seen_low, low_count, settle_count, cleanup_error;
};
enum tim4_motor_pulse_status tim4_motor_pulse_arm(struct tim4_motor_pulse *s,
                                                unsigned direction);
enum tim4_motor_pulse_status tim4_motor_pulse_poll(struct tim4_motor_pulse *s);
enum tim4_motor_pulse_status tim4_motor_pulse_abort(struct tim4_motor_pulse *s);
/* Acknowledges a latched result only when no timer ownership remains and the
 * complete reviewed stopped profile is present. Does not move/repair hardware. */
int tim4_motor_pulse_reset(struct tim4_motor_pulse *s);
#endif
