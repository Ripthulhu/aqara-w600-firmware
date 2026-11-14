#ifndef W600_FEEDBACK_BOARD_H
#define W600_FEEDBACK_BOARD_H
#include <stdbool.h>
#include <stdint.h>

enum w600_feedback_board_error {
    W600_FB_BOARD_STATE = 1u, W600_FB_BOARD_CONTEXT = 2u,
    W600_FB_BOARD_CLOCK = 4u, W600_FB_BOARD_RESET = 8u,
    W600_FB_BOARD_PIN = 16u, W600_FB_BOARD_BGR = 32u,
    W600_FB_BOARD_CONFLICT = 64u
};
struct w600_feedback_board {
    uint32_t owned, error;
    uint32_t cfg, apb1, apb2, reset, mode, pull, pmu;
    bool prepared;
};

/* Zero context once; serialize calls with IRQs masked and motor inhibited.
 * Privileged Thread only; no enabled peripheral IRQ except UART3/4 (27).
 * Its APB1 clock/pins, NVIC enable and pending bytes are left untouched.
 * W600_DISPLAY_BENCH additionally requires and preserves the UART2 clock;
 * its IRQ remains disabled, as in the ordinary profile.
 * Requires the reviewed w600_board_init profile, not inherited stock state.
 * prepare supplies 24MHz PCLK2, ADC/PMU clocks, reset ADC, PB13 analog/no-pull
 * and BGR enabled. It never waits for BGR or ADC readiness.
 * finish cancels owned ADC work by peripheral reset, then restores only owned
 * fields. Call it after every accepted/partial prepare, even after sampler error.
 * Errors latch; failed cleanup retains ownership for an explicit retry. Never
 * zero/reuse an owning context. No function controls the motor or IRQ state. */
bool w600_feedback_board_prepare(struct w600_feedback_board *);
bool w600_feedback_board_finish(struct w600_feedback_board *);
#endif
