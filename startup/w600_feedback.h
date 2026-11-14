#ifndef W600_FEEDBACK_H
#define W600_FEEDBACK_H
#include <stdbool.h>
#include <stdint.h>

enum w600_feedback_status {
    W600_FEEDBACK_BUSY, W600_FEEDBACK_READY,
    W600_FEEDBACK_SAMPLE, W600_FEEDBACK_FAILED
};
enum w600_feedback_error {
    W600_FEEDBACK_OK, W600_FEEDBACK_BAD_STATE, W600_FEEDBACK_CONTEXT,
    W600_FEEDBACK_FACTORY, W600_FEEDBACK_CONFIG, W600_FEEDBACK_TIME,
    W600_FEEDBACK_INIT_TIMEOUT, W600_FEEDBACK_BURST_TIMEOUT,
    W600_FEEDBACK_POLL_LIMIT, W600_FEEDBACK_FLAGS,
    W600_FEEDBACK_CONVERSION, W600_FEEDBACK_RAW, W600_FEEDBACK_ABORTED
};
enum w600_feedback_input { W600_FEEDBACK_MOTOR, W600_FEEDBACK_VBAT };
struct w600_feedback_pair {
    uint16_t bgr, signal;
    uint32_t acquired_at_ms;
};
struct w600_feedback_sample {
    struct w600_feedback_pair pair[3];
    uint32_t oldest_at_ms, completed_at_ms, sequence;
    enum w600_feedback_error error;
    bool valid;
    uint8_t input;
};
struct w600_feedback {
    struct w600_feedback_sample sample;
    uint32_t last_ms, operation_ms, phase_ms, polls;
    uint16_t factory_bgr;
    uint8_t phase, index;
    bool owns_adc;
    uint8_t input;
};

/* Zero-initialize context; serialize calls. Caller supplies a monotonic uint32
 * 1ms clock, verified 48MHz core/24MHz PCLK2, ADC+PMU clocks, PB13 analog,
 * BGR enabled and an exclusively reset ADC. Init/reinit requires MOTOR STOPPED.
 * No function waits or touches GPIO, clocks, SysTick, TIM4 or watchdogs.
 * Caller's motor stop must not wait for ADC abort/cleanup. */
enum w600_feedback_status w600_feedback_init(struct w600_feedback *, uint32_t now_ms);
/* Internal VBAT/3 path only; does not switch board power rails or measure PB2.
 * The VBAT pin's connection to the battery must be verified before reporting it
 * as battery voltage. It is never a capacity/percentage measurement. */
enum w600_feedback_status w600_feedback_init_vbat(struct w600_feedback *, uint32_t now_ms);
bool w600_feedback_vbat_mv(const struct w600_feedback *, uint32_t *millivolts);
enum w600_feedback_status w600_feedback_start(struct w600_feedback *, uint32_t now_ms);
/* SAMPLE is delivered once; only then is out->valid true. out may be NULL.
 * Timestamp is conservative conversion-launch time, not delivery time.
 * Error stays latched until stopped/reset/reinit; raw zero signal is not an endpoint. */
enum w600_feedback_status w600_feedback_poll(struct w600_feedback *, uint32_t now_ms,
                                            struct w600_feedback_sample *out);
void w600_feedback_abort(struct w600_feedback *);
#endif
