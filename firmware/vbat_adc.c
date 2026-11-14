#include "ciu32l071.h"
#include "vbat_adc.h"

static bool v_context(void)
{
    return __get_PRIMASK()==1 && !__get_IPSR() && !__get_CONTROL() &&
        !(NVIC->ISER[0]&~(1u<<UART3_4_IRQn));
}
static bool v_motor_off(void)
{
    return (RCC->IOPEN&7u)==7u && !(RCC->APB1EN&RCC_APB1EN_TIM4EN) &&
        (GPIOB->MODE&0xf0000000u)==0x50000000u &&
        !(GPIOB->OTYPE&0xc000u) && (GPIOB->ODR&0xc000u)==0xc000u &&
        (GPIOB->IDR&0xc000u)==0xc000u &&
        (GPIOA->MODE&0xc0000000u)==0x40000000u &&
        (GPIOC->MODE&0x00300000u)==0x00100000u &&
        !(GPIOA->OTYPE&0x8000u) && !(GPIOC->OTYPE&0x400u) &&
        !(GPIOA->ODR&0x8000u) && !(GPIOC->ODR&0x400u) &&
        !(GPIOA->IDR&0x8000u) && !(GPIOC->IDR&0x400u);
}
static void v_fail(struct w600_vbat_adc *s,enum w600_vbat_adc_error error)
{
    if (!s->sample.error) s->sample.error=error;
    s->sample.valid=false;
    if (s->phase!=W600_VBAT_ADC_FAULT) s->phase=W600_VBAT_ADC_CLEANUP;
}
static void v_cleanup(struct w600_vbat_adc *s,uint32_t now)
{
    s->sample.raw=s->feedback.sample;
    s->sample.factory_bgr=s->feedback.factory_bgr;
    if (!v_context() || !v_motor_off() || (s->feedback.owns_adc && !s->board.owned)) goto failed;
    if (s->feedback.owns_adc) w600_feedback_abort(&s->feedback);
    bool clean=true;
    if (s->board.owned && !w600_feedback_board_finish(&s->board)) clean=false;
    if (!s->board.owned) s->feedback.owns_adc=false;
    if (!clean || s->board.owned) goto failed;
    s->sample.completed_at_ms=now;
    s->sample.valid=s->sample.error==W600_VBAT_ADC_OK;
    s->phase=s->sample.valid ? W600_VBAT_ADC_DONE : W600_VBAT_ADC_FAULT;
    return;
failed:
    s->cleanup_failed=true; s->sample.valid=false;
    if (!s->sample.error) s->sample.error=W600_VBAT_ADC_RELEASE;
    s->phase=W600_VBAT_ADC_FAULT;
}
enum w600_vbat_adc_status w600_vbat_adc_begin(struct w600_vbat_adc *s,uint32_t now)
{
    if (!s) return W600_VBAT_ADC_FAILED;
    if (!v_context() || (s->phase!=W600_VBAT_ADC_OFF && s->phase!=W600_VBAT_ADC_DONE) ||
        s->board.owned || s->board.prepared || s->board.error || s->feedback.owns_adc ||
        s->sample.error || s->cleanup_failed) {
        v_fail(s,W600_VBAT_ADC_STATE); return W600_VBAT_ADC_FAILED;
    }
    uint32_t sequence=s->sample.sequence+1;
    *s=(struct w600_vbat_adc){0};
    s->sample.sequence=sequence; s->sample.started_at_ms=s->last_ms=now;
    if (!v_motor_off() || !w600_feedback_board_prepare(&s->board)) {
        v_fail(s,W600_VBAT_ADC_CONTEXT); return W600_VBAT_ADC_FAILED;
    }
    if (w600_feedback_init_vbat(&s->feedback,now)!=W600_FEEDBACK_BUSY) {
        v_fail(s,W600_VBAT_ADC_ACQUIRE); return W600_VBAT_ADC_FAILED;
    }
    s->phase=W600_VBAT_ADC_INIT;
    return W600_VBAT_ADC_BUSY;
}
static void v_poll(struct w600_vbat_adc *s,uint32_t now)
{
    if (s->phase==W600_VBAT_ADC_OFF || s->phase==W600_VBAT_ADC_DONE || s->phase==W600_VBAT_ADC_FAULT) return;
    bool cleanup=s->phase==W600_VBAT_ADC_CLEANUP;
    if (!cleanup && s->phase!=W600_VBAT_ADC_INIT && s->phase!=W600_VBAT_ADC_CONVERT) {
        v_fail(s,W600_VBAT_ADC_STATE); return;
    }
    if ((uint32_t)(now-s->last_ms)>=0x80000000u) v_fail(s,W600_VBAT_ADC_TIME);
    else {
        s->last_ms=now;
        if (++s->polls>200000u) v_fail(s,W600_VBAT_ADC_POLL_LIMIT);
        else if ((uint32_t)(now-s->sample.started_at_ms)>=100u) v_fail(s,W600_VBAT_ADC_TIMEOUT);
    }
    if (cleanup) { v_cleanup(s,now); return; }
    if (s->phase==W600_VBAT_ADC_CLEANUP) return;
    if (!v_context() || !v_motor_off()) { v_fail(s,W600_VBAT_ADC_CONTEXT); return; }
    enum w600_feedback_status status=w600_feedback_poll(&s->feedback,now,0);
    if (status==W600_FEEDBACK_FAILED) { v_fail(s,W600_VBAT_ADC_ACQUIRE); return; }
    if (s->phase==W600_VBAT_ADC_INIT) {
        if (status!=W600_FEEDBACK_READY) return;
        if (w600_feedback_start(&s->feedback,now)!=W600_FEEDBACK_BUSY) {
            v_fail(s,W600_VBAT_ADC_ACQUIRE); return;
        }
        s->phase=W600_VBAT_ADC_CONVERT;
        return;
    }
    if (s->phase!=W600_VBAT_ADC_CONVERT) { v_fail(s,W600_VBAT_ADC_STATE); return; }
    if (status!=W600_FEEDBACK_SAMPLE) return;
    if (!w600_feedback_vbat_mv(&s->feedback,&s->sample.millivolts)) v_fail(s,W600_VBAT_ADC_ACQUIRE);
    else if (s->sample.millivolts<1000u || s->sample.millivolts>3600u) v_fail(s,W600_VBAT_ADC_RANGE);
    else s->phase=W600_VBAT_ADC_CLEANUP;
}
enum w600_vbat_adc_status w600_vbat_adc_poll(struct w600_vbat_adc *s,uint32_t now,
                                           struct w600_vbat_adc_sample *out)
{
    if (out) out->valid=false;
    if (!s) { if (out) out->error=W600_VBAT_ADC_STATE; return W600_VBAT_ADC_FAILED; }
    v_poll(s,now);
    if (out) out->error=s->sample.error;
    if (s->phase==W600_VBAT_ADC_DONE && !s->delivered) {
        s->delivered=true; if (out) *out=s->sample; return W600_VBAT_ADC_SAMPLE;
    }
    if (s->phase==W600_VBAT_ADC_FAULT) return W600_VBAT_ADC_FAILED;
    return s->phase==W600_VBAT_ADC_OFF || s->phase==W600_VBAT_ADC_DONE ? W600_VBAT_ADC_IDLE : W600_VBAT_ADC_BUSY;
}
void w600_vbat_adc_abort(struct w600_vbat_adc *s)
{
    if (s && s->phase>=W600_VBAT_ADC_INIT && s->phase<=W600_VBAT_ADC_CLEANUP)
        v_fail(s,W600_VBAT_ADC_ABORTED);
}
