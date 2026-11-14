#ifndef W600_TEMPERATURE_ADC_H
#define W600_TEMPERATURE_ADC_H
#include "../startup/w600_feedback_board.h"
#include "../startup/w600_feedback.h"
#include "../replacement-control/w600_temperature.h"

enum w600_temperature_adc_status { W600_TADC_IDLE, W600_TADC_BUSY, W600_TADC_SAMPLE, W600_TADC_FAILED };
enum w600_temperature_adc_phase { W600_TADC_OFF, W600_TADC_INIT, W600_TADC_BIAS,
    W600_TADC_CONVERT, W600_TADC_GAP, W600_TADC_CLEANUP, W600_TADC_DONE, W600_TADC_FAULT };
enum w600_temperature_adc_error { W600_TADC_OK, W600_TADC_STATE, W600_TADC_CONTEXT,
    W600_TADC_PINS, W600_TADC_ADC, W600_TADC_TIME, W600_TADC_TIMEOUT,
    W600_TADC_FLAGS, W600_TADC_RAW, W600_TADC_RANGE, W600_TADC_RELEASE, W600_TADC_ABORTED };
struct w600_temperature_adc_sample {
    uint16_t raw[W600_TEMPERATURE_READINGS];
    uint32_t acquired_at_ms[W600_TEMPERATURE_READINGS];
    uint32_t started_at_ms, completed_at_ms, sequence;
    struct w600_temperature_result temperature;
    enum w600_temperature_adc_error error;
    bool valid;
};
struct w600_temperature_adc {
    struct w600_feedback_board board;
    struct w600_feedback feedback;
    struct w600_temperature_adc_sample sample;
    uint32_t owned, saved_amode, saved_apull, saved_bmode, saved_bpull, saved_btype, saved_bodr;
    uint32_t last_ms, phase_ms, polls, calfact;
    enum w600_temperature_adc_phase phase;
    int16_t offset_decic;
    uint8_t count;
    bool delivered, cleanup_failed;
};
/* Zero once, serialized privileged Thread calls with IRQs masked. Caller must
 * arbitrate exclusive ADC ownership against motor/other ADC users; begin rejects
 * active TIM4/ADC/PMU, non-inhibited motor or unreviewed clock/pin profiles.
 * Uses existing board/calibration helpers, but reads rawCH0/PA1 against VDDA.
 * Configures PA1/PB9 BEFORE acquiring ADC-board ownership and releases ADC BEFORE
 * restoring those pins. During ownership PB9LOW biases the sensor, HIGH is idle.
 * Original pins are restored on successful cleanup; no motor pins are written.
 * Two1ms ticks guarantee >=1ms bias/gaps;5ms conversion/100ms total bounds. No retries.
 * DONE may begin another explicit periodic sample; failures stay latched. Never
 * zero a failed owning object. Physical temperature accuracy is not qualified. */
enum w600_temperature_adc_status w600_temperature_adc_begin(struct w600_temperature_adc *,
                                                            uint32_t now_ms,int16_t offset_decic);
/* SAMPLE is delivered once, after successful cleanup; otherwise out->valid=false.
 * out may be NULL. Acquisition timestamps precede conversion launch. */
enum w600_temperature_adc_status w600_temperature_adc_poll(struct w600_temperature_adc *,
                                                           uint32_t now_ms,struct w600_temperature_adc_sample *out);
void w600_temperature_adc_abort(struct w600_temperature_adc *);
#endif
