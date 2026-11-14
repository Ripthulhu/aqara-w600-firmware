#ifndef W600_VBAT_ADC_H
#define W600_VBAT_ADC_H
#include "../startup/w600_feedback_board.h"
#include "../startup/w600_feedback.h"

enum w600_vbat_adc_status { W600_VBAT_ADC_IDLE, W600_VBAT_ADC_BUSY,
    W600_VBAT_ADC_SAMPLE, W600_VBAT_ADC_FAILED };
enum w600_vbat_adc_phase { W600_VBAT_ADC_OFF, W600_VBAT_ADC_INIT,
    W600_VBAT_ADC_CONVERT, W600_VBAT_ADC_CLEANUP, W600_VBAT_ADC_DONE, W600_VBAT_ADC_FAULT };
enum w600_vbat_adc_error { W600_VBAT_ADC_OK, W600_VBAT_ADC_STATE,
    W600_VBAT_ADC_CONTEXT, W600_VBAT_ADC_ACQUIRE, W600_VBAT_ADC_TIME,
    W600_VBAT_ADC_TIMEOUT, W600_VBAT_ADC_POLL_LIMIT, W600_VBAT_ADC_RANGE,
    W600_VBAT_ADC_RELEASE, W600_VBAT_ADC_ABORTED };
struct w600_vbat_adc_sample {
    struct w600_feedback_sample raw;
    uint32_t millivolts, started_at_ms, completed_at_ms, sequence;
    uint16_t factory_bgr;
    enum w600_vbat_adc_error error;
    bool valid;
};
struct w600_vbat_adc {
    struct w600_feedback_board board;
    struct w600_feedback feedback;
    struct w600_vbat_adc_sample sample;
    uint32_t last_ms, polls;
    enum w600_vbat_adc_phase phase;
    bool delivered, cleanup_failed;
};
/* Zero once; serialize privileged Thread calls with IRQs masked. Caller must
 * exclude motor/temperature/other ADC work, STOP and settings writes throughout
 * ownership. Uses internal CH14/VBAT_DIV3 only, with no power-rail switching.
 * Accepts 1000..3600mV as a bench/service policy, not a hardware maximum or
 * battery-capacity claim. Valid SAMPLE is delivered once, after cleanup.
 * begin accepts OFF or clean DONE; FAILED may still require poll for cleanup.
 * Errors latch. Never zero/reuse a context retaining ownership after failure.
 * 100ms total and finite polls bound cooperative work, not stalled bus accesses. */
enum w600_vbat_adc_status w600_vbat_adc_begin(struct w600_vbat_adc *,uint32_t now_ms);
enum w600_vbat_adc_status w600_vbat_adc_poll(struct w600_vbat_adc *,uint32_t now_ms,
                                           struct w600_vbat_adc_sample *out);
void w600_vbat_adc_abort(struct w600_vbat_adc *);
#endif
