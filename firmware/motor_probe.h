#ifndef W600_MOTOR_PROBE_H
#define W600_MOTOR_PROBE_H
#include <stdbool.h>
#include <stdint.h>
#include "../replacement-control/motor_probe_sequence.h"
#include "../startup/w600_feedback_board.h"

enum motor_probe_service_state {
    MOTOR_PROBE_SERVICE_IDLE, MOTOR_PROBE_SERVICE_INITIALIZING,
    MOTOR_PROBE_SERVICE_RUNNING, MOTOR_PROBE_SERVICE_CLEANUP,
    MOTOR_PROBE_SERVICE_COMPLETE_RAW, MOTOR_PROBE_SERVICE_FAILED
};
enum motor_probe_service_error {
    MOTOR_PROBE_SERVICE_OK, MOTOR_PROBE_SERVICE_CONTEXT, MOTOR_PROBE_SERVICE_STATE,
    MOTOR_PROBE_SERVICE_GPIO, MOTOR_PROBE_SERVICE_BOARD, MOTOR_PROBE_SERVICE_ADC,
    MOTOR_PROBE_SERVICE_SEQUENCE, MOTOR_PROBE_SERVICE_PULSE,
    MOTOR_PROBE_SERVICE_CLEANUP_ERROR, MOTOR_PROBE_SERVICE_ABORTED
};
struct motor_probe_snapshot {
    uint32_t version,bytes,state,error,cleanup_pending,begin_used;
    uint32_t started_at_ms,last_tick_ms,finished_at_ms,target_pulses,samples;
    uint32_t completed_pulses,nominal_low_ticks_100khz,sequence_error;
    uint32_t pulse_error,pulse_cleanup_error,board_error,board_owned,sampler_error;
    uint32_t stop_confirmed,cleanup_abort_expected;
    struct w600_feedback_sample sample[3];
    struct tim4_motor_pulse pulse[2];
    uint32_t off_at_ms[2],done_at_ms[2],factory_word;
};
struct motor_probe {
    struct motor_probe_sequence sequence;
    struct w600_feedback_board board;
    struct w600_feedback feedback;
    struct tim4_motor_pulse pulse;
    struct motor_probe_snapshot report;
    struct motor_probe_action action;
    uint32_t initialized,off_at_ms;
    bool adc_started,pulse_active,sample_active,have_off,cleanup_attempted;
};

/* Motor-UNPLUGGED bench service, raw GPIO-HH/selectors-LOW only, no UART writes.
 * Call init once before enabling ticks; no hidden/BSS state. Each initialized
 * object permits ONE begin (one/two dir1 pulses), never a radio-controlled retry.
 * Thread entrypoints require privileged Thread+PRIMASK1; restore IRQs promptly
 * between calls. Read a fresh app tick under that same mask, not before it.
 * tick is serialized with them and may run in SysTick/Thread;
 * it only updates pure sequencing/report state and calls shared GPIO stop.
 * No timer/ADC cleanup from ISR. Fixed100ms whole attempt includes ADC init.
 * All cleanup is foreground, pulse abort/release BEFORE sampler/board finish.
 * A failed cleanup is retained without automatic retry or owning-context reset.
 * COMPLETE_RAW is a bounded bench observation, never endpoint/calibration/Ready.
 * The pulse ledger is NOMINAL1900 LOW ticks at100kHz, not measured motor energy.
 * Snapshot copies only owned RAM; caller serializes it too. */
bool motor_probe_init(struct motor_probe *);
bool motor_probe_begin(struct motor_probe *,uint32_t now_ms,unsigned pulses);
void motor_probe_foreground(struct motor_probe *,uint32_t now_ms);
void motor_probe_tick(struct motor_probe *,uint32_t now_ms);
void motor_probe_abort(struct motor_probe *,uint32_t now_ms);
void motor_probe_snapshot(const struct motor_probe *,volatile struct motor_probe_snapshot *);
#endif
