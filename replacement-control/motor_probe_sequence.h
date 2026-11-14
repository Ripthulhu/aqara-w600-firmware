#ifndef MOTOR_PROBE_SEQUENCE_H
#define MOTOR_PROBE_SEQUENCE_H
#include <stdbool.h>
#include <stdint.h>
#include "tim4_motor_pulse.h"
#include "../startup/w600_feedback.h"

enum motor_probe_state {
    MOTOR_PROBE_IDLE, MOTOR_PROBE_BASELINE, MOTOR_PROBE_PULSE,
    MOTOR_PROBE_SAMPLE, MOTOR_PROBE_OBSERVED, MOTOR_PROBE_STOPPING,
    MOTOR_PROBE_COMPLETE_RAW, MOTOR_PROBE_FAILED
};
enum motor_probe_error {
    MOTOR_PROBE_OK, MOTOR_PROBE_LIMITS, MOTOR_PROBE_TOKEN, MOTOR_PROBE_CLOCK,
    MOTOR_PROBE_TOTAL_TIMEOUT, MOTOR_PROBE_FOREGROUND_TIMEOUT,
    MOTOR_PROBE_OPERATION_TIMEOUT, MOTOR_PROBE_PULSE_ERROR,
    MOTOR_PROBE_SAMPLE_ERROR, MOTOR_PROBE_SAMPLE_STALE,
    MOTOR_PROBE_PULSE_LIMIT, MOTOR_PROBE_ABORTED, MOTOR_PROBE_CLEANUP_ERROR
};
enum motor_probe_action_kind {
    MOTOR_PROBE_NO_ACTION, MOTOR_PROBE_START_SAMPLE,
    MOTOR_PROBE_START_PULSE, MOTOR_PROBE_STOP
};
enum motor_probe_source { MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW = 1 };
enum { MOTOR_PROBE_LOW_TICKS_100KHZ_PER_PULSE = 1900 };
struct motor_probe_key { uint32_t attempt, step, direction; };
struct motor_probe_action {
    enum motor_probe_action_kind kind;
    struct motor_probe_key key;
};
struct motor_probe_limits {
    uint32_t total_ms, foreground_ms, pulse_ms, acquisition_ms;
    uint32_t max_sample_age_ms, max_pulses;
};
struct motor_probe_sequence {
    struct motor_probe_limits limits;
    struct motor_probe_action action;
    struct w600_feedback_sample raw;
    uint32_t factory_word, nominal_low_ticks_100khz, completed_pulses;
    uint32_t last_attempt, started_at_ms, last_now_ms, foreground_deadline_ms;
    uint32_t operation_at_ms, operation_deadline_ms, taken_at_ms;
    enum motor_probe_state state;
    enum motor_probe_error error;
    bool taken, have_sample, cleanup_pending, pulse_released;
};

/* Pure action/evidence adapter: no MMIO, callbacks, thresholds or endpoint output.
 * Valid initialized objects; serialize calls. Independent tick must continue if
 * foreground/acquisition stalls. One monotonic uint32_t millisecond clock, with
 * every observation gap <2^31 ms. A stopped clock is not a wall-clock bound.
 * Expiry publishes STOP only: the independent safety caller must force GPIO
 * off without waiting for foreground pulse/ADC cleanup. This is not a cutoff.
 * Each taken action is consumed ONCE by the caller, never retried automatically.
 * Driver/sampler initialization and real GPIO/timer stop proof remain the caller's
 * responsibility. RAW/COMPLETE_RAW are never homed, calibrated, Ready or position.
 * Initialize once; reset preserves the non-wrapping caller attempt-token floor. */
void motor_probe_sequence_init(struct motor_probe_sequence *);
bool motor_probe_sequence_begin(struct motor_probe_sequence *, uint32_t now_ms,
    uint32_t attempt, unsigned direction, const struct motor_probe_limits *, bool stopped);
void motor_probe_sequence_tick(struct motor_probe_sequence *, uint32_t now_ms);
bool motor_probe_sequence_progress(struct motor_probe_sequence *, uint32_t attempt,
    uint32_t now_ms);
bool motor_probe_sequence_take(struct motor_probe_sequence *, uint32_t now_ms,
    struct motor_probe_action *);
/* Accept the actual fixed-pulse driver DONE plus caller-observed SETTLING/DONE
 * times. off_at is >=20ms after take; settled_at is >=1ms after off_at, <=now.
 * Nominal 1900 LOW timer ticks are credited once; partial/failed pulses get none.
 * This is not measured energized time, actual motion or a feedback-path proof. */
bool motor_probe_sequence_pulse(struct motor_probe_sequence *, const struct motor_probe_key *,
    uint32_t now_ms, const struct tim4_motor_pulse *, uint32_t off_at_ms,
    uint32_t settled_at_ms);
bool motor_probe_sequence_sample(struct motor_probe_sequence *, const struct motor_probe_key *,
    uint32_t now_ms, enum motor_probe_source, uint32_t factory_word,
    const struct w600_feedback_sample *);
/* After each raw observation, explicitly request another pulse or finish. */
bool motor_probe_sequence_next(struct motor_probe_sequence *, const struct motor_probe_key *,
    uint32_t now_ms, bool finish);
void motor_probe_sequence_abort(struct motor_probe_sequence *);
/* Consume STOP first, cancel/release the pulse before aborting ADC/board_finish,
 * then acknowledge in this order. These booleans are caller assertions, not
 * physical or electrical proof. A false release latches failure; later cleanup
 * may clear pending ownership but cannot turn that failure into COMPLETE_RAW. */
bool motor_probe_sequence_pulse_released(struct motor_probe_sequence *, uint32_t attempt,
    uint32_t now_ms, bool stopped_and_released);
bool motor_probe_sequence_adc_released(struct motor_probe_sequence *, uint32_t attempt,
    uint32_t now_ms, bool released);
bool motor_probe_sequence_reset(struct motor_probe_sequence *, bool stopped);
#endif
