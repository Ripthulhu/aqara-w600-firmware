#ifndef W600_MOTOR_SERVICE_H
#define W600_MOTOR_SERVICE_H
#include "app_clock.h"
#include "../startup/w600_feedback_board.h"
#include "../startup/w600_feedback.h"
#include "../replacement-control/motor_homing.h"
enum w600_motor_phase {
    W600_MOTOR_IDLE, W600_MOTOR_PREPARING, W600_MOTOR_DRIVING,
    W600_MOTOR_SETTLING, W600_MOTOR_SAMPLING, W600_MOTOR_RESTORING,
    W600_MOTOR_CLEANUP, W600_MOTOR_DONE, W600_MOTOR_FAILED
};
enum w600_motor_error {
    W600_MOTOR_OK, W600_MOTOR_STATE, W600_MOTOR_CONTEXT, W600_MOTOR_ADC,
    W600_MOTOR_TIMER, W600_MOTOR_TIMING, W600_MOTOR_PINS,
    W600_MOTOR_FEEDBACK, W600_MOTOR_TIMEOUT, W600_MOTOR_ABORTED,
    W600_MOTOR_RELEASE
};
struct w600_motor_service {
    const volatile uint32_t *ticks;
    struct w600_feedback_board board;
    struct w600_feedback feedback;
    struct w600_feedback_sample sample;
    struct motor_homing homing;
    enum w600_motor_phase phase;
    enum w600_motor_error error;
    uint32_t direction, started_ms, arm_cycles, clamp_cycles, next_sample;
    uint32_t expected_cc2, saved_afh, owned, samples, last_mv, last_ms;
    uint32_t polls, stop_confirmed, cleanup_failed, requested_stop, completed_ms;
    uint32_t on_lower, on_upper, on_started_before, on_started_after;
    /* Oldest acquisition time (ms resolution converted to100us), not receipt. */
    uint32_t sample_on_lower, sample_on_upper, sample_at_ticks;
    bool on_active, time_valid;
    uint32_t limit_ms;
};
/* All fields use100us ticks, including wrapping now_ticks. These bracket
 * commanded drive time, not measured current, shaft motion or physical position.
 * The lower count uses consistent quantized edge timestamps for calibration.
 * Sampling/settle periods are excluded; uncertainty includes edge quantization. */
struct w600_motor_time {
    uint32_t now_ticks, on_lower, on_upper, sample_on_lower, sample_on_upper;
};
/* Zero once. Thread calls are serialized with IRQs masked, never from IRQs.
 * Begin only accepts an IDLE service. Explicit rearm accepts clean DONE only,
 * after cleanup, at least100ms stopped, and exclusive ADC/timer release;
 * errors are never cleared there.
 * No automatic movement/retry/reversal. Consume the completed result first.
 * Poll returns without waiting; call it tightly during a sensing window (<1.75ms)
 * and at least every250us otherwise. UART reception IRQs can run between calls.
 * tick runs from the independent1ms SysTick. Only poll releases owned resources.
 * Bench electrical thresholds are not mounted calibration or actual position. */
bool w600_motor_service_init(struct w600_motor_service *, const volatile uint32_t *ticks);
bool w600_motor_service_begin(struct w600_motor_service *, unsigned direction);
/* Explicit per-command250..45000ms hardware/guard cap, never renewed by progress.
 * Expiry is a failed command, not an endpoint or a successful timed finish. */
bool w600_motor_service_begin_bounded(struct w600_motor_service *,unsigned direction,uint32_t limit_ms);
/* Stop an active command normally, without claiming an endpoint or position.
 * Expired leases/timer remain failures. False for inactive/terminal calls.
 * Resources release only through poll. */
bool w600_motor_service_finish(struct w600_motor_service *);
bool w600_motor_service_rearm(struct w600_motor_service *);
/* Serialized Thread snapshot; false clears out and means no usable estimate. */
bool w600_motor_service_time(const struct w600_motor_service *,struct w600_motor_time *out);
void w600_motor_service_poll(struct w600_motor_service *);
void w600_motor_service_tick(struct w600_motor_service *, uint32_t now_ms);
void w600_motor_service_abort(struct w600_motor_service *);
/* Serialized Thread query; includes the3ms lead into a sample edge so slower
 * controller/report calls cannot consume the sensing window. */
bool w600_motor_service_urgent(const struct w600_motor_service *);
#endif
