#include "ciu32l071.h"
#include <stddef.h>
#include "motor_probe.h"
#include "../startup/w600_motor_stop.h"

#ifndef MOTOR_PROBE_FACTORY_WORD
#define MOTOR_PROBE_FACTORY_WORD (*(volatile const uint32_t *)0x1fff03c0u)
#endif
#define PROBE_INITIALIZED UINT32_C(0x57505242)
_Static_assert(UART3_4_IRQn==27,"Reviewed UART IRQ");
_Static_assert(sizeof(struct motor_probe_snapshot)<=512,"Probe RAM report reserve");

static bool thread_ok(void)
{
    return __get_PRIMASK()==1 && __get_IPSR()==0 && __get_CONTROL()==0 &&
        !(NVIC->ISER[0]&~(1u<<UART3_4_IRQn));
}
static void refresh(struct motor_probe *p)
{
    p->report.sequence_error=(uint32_t)p->sequence.error;
    p->report.completed_pulses=p->sequence.completed_pulses;
    p->report.nominal_low_ticks_100khz=p->sequence.nominal_low_ticks_100khz;
    if(p->pulse.error!=TIM4_PULSE_ABORTED || !p->report.cleanup_abort_expected)
        p->report.pulse_error=p->pulse.error;
    p->report.pulse_cleanup_error=p->pulse.cleanup_error;
    p->report.board_error=p->board.error; p->report.board_owned=p->board.owned;
    if(p->feedback.sample.error!=W600_FEEDBACK_ABORTED)
        p->report.sampler_error=(uint32_t)p->feedback.sample.error;
    p->report.cleanup_pending=(uint32_t)(p->sequence.cleanup_pending ||
        p->pulse.owned || p->board.owned);
}
static void failure(struct motor_probe *p,enum motor_probe_service_error error)
{
    if(!p->report.error) p->report.error=(uint32_t)error;
    motor_probe_sequence_abort(&p->sequence);
    p->report.state=MOTOR_PROBE_SERVICE_FAILED;
    refresh(p);
}
static bool stop(struct motor_probe *p)
{
    p->report.stop_confirmed=(uint32_t)w600_motor_stop();
    if(!p->report.stop_confirmed) failure(p,MOTOR_PROBE_SERVICE_GPIO);
    return p->report.stop_confirmed!=0;
}
static bool pulse_allowed(const struct motor_probe *p)
{
    return p->pulse_active && !p->report.error && p->sequence.state==MOTOR_PROBE_PULSE &&
        p->sequence.taken && p->action.kind==MOTOR_PROBE_START_PULSE &&
        p->action.key.attempt==p->sequence.action.key.attempt &&
        p->action.key.step==p->sequence.action.key.step &&
        p->action.key.direction==1 && p->sequence.action.key.direction==1 &&
        !p->pulse.error && !p->pulse.cleanup_error && p->pulse.direction==1 &&
        p->pulse.channel==2 &&
        ((p->pulse.status==TIM4_PULSE_DRIVING && p->pulse.owned==3) ||
         (p->pulse.status==TIM4_PULSE_SETTLING && p->pulse.owned==1));
}
void motor_probe_tick(struct motor_probe *p,uint32_t now)
{
    if(p->initialized!=PROBE_INITIALIZED) { (void)w600_motor_stop(); return; }
    p->report.last_tick_ms=now;
    if(p->report.begin_used) {
        motor_probe_sequence_tick(&p->sequence,now);
        if(p->sequence.state==MOTOR_PROBE_FAILED && !p->report.error)
            failure(p,MOTOR_PROBE_SERVICE_SEQUENCE);
    }
    if(!pulse_allowed(p)) {
        if(p->pulse_active && !p->report.error) failure(p,MOTOR_PROBE_SERVICE_PULSE);
        (void)stop(p);
    } else p->report.stop_confirmed=0;
    refresh(p);
}
bool motor_probe_init(struct motor_probe *p)
{
    bool off=w600_motor_stop()!=0;
    if(p->initialized==PROBE_INITIALIZED) {
        failure(p,MOTOR_PROBE_SERVICE_STATE); return false;
    }
    *p=(struct motor_probe){0};
    motor_probe_sequence_init(&p->sequence);
    p->report.version=1; p->report.bytes=sizeof p->report;
    p->report.stop_confirmed=(uint32_t)off;
    p->initialized=PROBE_INITIALIZED;
    if(!off || !thread_ok()) {
        failure(p,off ? MOTOR_PROBE_SERVICE_CONTEXT : MOTOR_PROBE_SERVICE_GPIO);
        return false;
    }
    return true;
}
bool motor_probe_begin(struct motor_probe *p,uint32_t now,unsigned pulses)
{
    if(p->initialized!=PROBE_INITIALIZED) { (void)w600_motor_stop(); return false; }
    if(!stop(p) || !thread_ok()) { failure(p,MOTOR_PROBE_SERVICE_CONTEXT); return false; }
    if(p->report.begin_used || p->report.error || p->report.state!=MOTOR_PROBE_SERVICE_IDLE ||
       (pulses!=1 && pulses!=2)) { failure(p,MOTOR_PROBE_SERVICE_STATE); return false; }
    p->report.begin_used=1;
    p->report.started_at_ms=now; p->report.last_tick_ms=now; p->report.target_pulses=pulses;
    const struct motor_probe_limits limits={100,20,40,20,10,pulses};
    if(!motor_probe_sequence_begin(&p->sequence,now,1,1,&limits,true)) {
        failure(p,MOTOR_PROBE_SERVICE_SEQUENCE); return false;
    }
    p->report.state=MOTOR_PROBE_SERVICE_INITIALIZING;
    if(!w600_feedback_board_prepare(&p->board)) {
        failure(p,MOTOR_PROBE_SERVICE_BOARD); return false;
    }
    enum w600_feedback_status status=w600_feedback_init(&p->feedback,now);
    p->adc_started=p->feedback.owns_adc;
    if(status!=W600_FEEDBACK_BUSY) { failure(p,MOTOR_PROBE_SERVICE_ADC); return false; }
    p->report.factory_word=MOTOR_PROBE_FACTORY_WORD;
    refresh(p); return true;
}
static void cleanup(struct motor_probe *p,uint32_t now)
{
    if(p->cleanup_attempted) return;
    p->cleanup_attempted=true;
    if(!stop(p)) return; /* Do not disable a still-AF one-shot on failed GPIO proof. */
    if(p->pulse.status!=TIM4_PULSE_IDLE || p->pulse.owned) {
        bool expected=p->pulse.error==TIM4_PULSE_OK;
        (void)tim4_motor_pulse_abort(&p->pulse);
        p->report.cleanup_abort_expected=(uint32_t)(expected &&
            p->pulse.error==TIM4_PULSE_ABORTED && !p->pulse.cleanup_error && !p->pulse.owned);
    }
    if(p->pulse.owned || p->pulse.cleanup_error || (RCC->APB1EN&RCC_APB1EN_TIM4EN)) {
        failure(p,MOTOR_PROBE_SERVICE_CLEANUP_ERROR); return;
    }
    p->pulse_active=false;
    if(p->sequence.action.kind==MOTOR_PROBE_STOP) {
        if(!motor_probe_sequence_take(&p->sequence,now,&p->action) ||
           p->action.kind!=MOTOR_PROBE_STOP) {
            failure(p,MOTOR_PROBE_SERVICE_SEQUENCE); return;
        }
    }
    if(!motor_probe_sequence_pulse_released(&p->sequence,p->sequence.action.key.attempt,now,true)) {
        failure(p,MOTOR_PROBE_SERVICE_SEQUENCE); return;
    }
    /* No sampler/board call may precede pulse release and verified GPIO stop. */
    if(p->adc_started) w600_feedback_abort(&p->feedback);
    bool finished=w600_feedback_board_finish(&p->board);
    if(!finished) failure(p,MOTOR_PROBE_SERVICE_CLEANUP_ERROR);
    if(p->board.owned) {
        (void)motor_probe_sequence_adc_released(&p->sequence,p->sequence.action.key.attempt,now,false);
        refresh(p); return;
    }
    p->adc_started=p->sample_active=false;
    if(!motor_probe_sequence_adc_released(&p->sequence,p->sequence.action.key.attempt,now,true))
        failure(p,MOTOR_PROBE_SERVICE_SEQUENCE);
    p->report.finished_at_ms=now;
    p->report.state=!p->report.error && p->sequence.state==MOTOR_PROBE_COMPLETE_RAW ?
        MOTOR_PROBE_SERVICE_COMPLETE_RAW : MOTOR_PROBE_SERVICE_FAILED;
    refresh(p);
}
void motor_probe_abort(struct motor_probe *p,uint32_t now)
{
    if(p->initialized!=PROBE_INITIALIZED) { (void)w600_motor_stop(); return; }
    (void)stop(p); failure(p,MOTOR_PROBE_SERVICE_ABORTED);
    if(thread_ok()) cleanup(p,now);
}
static void pulse_poll(struct motor_probe *p,uint32_t now)
{
    enum tim4_motor_pulse_status status=tim4_motor_pulse_poll(&p->pulse);
    if(status==TIM4_PULSE_FAILED) { failure(p,MOTOR_PROBE_SERVICE_PULSE); return; }
    if(status==TIM4_PULSE_SETTLING && !p->have_off) { p->off_at_ms=now; p->have_off=true; }
    if(status==TIM4_PULSE_DONE) {
        unsigned index=p->action.key.step-1u;
        if(index>=2 || !p->have_off ||
           !motor_probe_sequence_pulse(&p->sequence,&p->action.key,now,&p->pulse,p->off_at_ms,now)) {
            failure(p,MOTOR_PROBE_SERVICE_SEQUENCE); return;
        }
        p->report.pulse[index]=p->pulse;
        p->report.off_at_ms[index]=p->off_at_ms; p->report.done_at_ms[index]=now;
        p->pulse_active=false;
    } else if(status!=TIM4_PULSE_DRIVING && status!=TIM4_PULSE_SETTLING)
        failure(p,MOTOR_PROBE_SERVICE_PULSE);
}
void motor_probe_foreground(struct motor_probe *p,uint32_t now)
{
    if(p->initialized!=PROBE_INITIALIZED) { (void)w600_motor_stop(); return; }
    if(!thread_ok()) { (void)stop(p); failure(p,MOTOR_PROBE_SERVICE_CONTEXT); return; }
    motor_probe_tick(p,now);
    if(p->report.error) { cleanup(p,now); return; }
    if(!p->report.begin_used || p->report.state==MOTOR_PROBE_SERVICE_COMPLETE_RAW) return;
    if(!motor_probe_sequence_progress(&p->sequence,1,now)) {
        failure(p,MOTOR_PROBE_SERVICE_SEQUENCE); cleanup(p,now); return;
    }
    if(p->pulse_active) pulse_poll(p,now);
    else if(p->report.state==MOTOR_PROBE_SERVICE_INITIALIZING) {
        enum w600_feedback_status status=w600_feedback_poll(&p->feedback,now,NULL);
        if(status==W600_FEEDBACK_READY) p->report.state=MOTOR_PROBE_SERVICE_RUNNING;
        else if(status!=W600_FEEDBACK_BUSY) failure(p,MOTOR_PROBE_SERVICE_ADC);
    } else if(p->sample_active) {
        enum w600_feedback_status status=w600_feedback_poll(&p->feedback,now,NULL);
        if(status==W600_FEEDBACK_SAMPLE) {
            unsigned index=p->report.samples;
            if(index>=3 || !motor_probe_sequence_sample(&p->sequence,&p->action.key,now,
                MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,p->report.factory_word,&p->feedback.sample))
                failure(p,MOTOR_PROBE_SERVICE_SEQUENCE);
            else {
                p->report.sample[index]=p->feedback.sample; ++p->report.samples;
                p->sample_active=false;
                bool finish=p->sequence.completed_pulses>=p->report.target_pulses;
                if(!motor_probe_sequence_next(&p->sequence,&p->action.key,now,finish))
                    failure(p,MOTOR_PROBE_SERVICE_SEQUENCE);
            }
        } else if(status!=W600_FEEDBACK_BUSY) failure(p,MOTOR_PROBE_SERVICE_ADC);
    } else if(motor_probe_sequence_take(&p->sequence,now,&p->action)) {
        if(p->action.kind==MOTOR_PROBE_START_SAMPLE) {
            if(w600_feedback_start(&p->feedback,now)!=W600_FEEDBACK_BUSY)
                failure(p,MOTOR_PROBE_SERVICE_ADC);
            else p->sample_active=true;
        } else if(p->action.kind==MOTOR_PROBE_START_PULSE) {
            if(!tim4_motor_pulse_reset(&p->pulse) ||
               tim4_motor_pulse_arm(&p->pulse,1)!=TIM4_PULSE_DRIVING)
                failure(p,MOTOR_PROBE_SERVICE_PULSE);
            else { p->pulse_active=true; p->have_off=false; p->report.stop_confirmed=0; }
        } else if(p->action.kind==MOTOR_PROBE_STOP) p->report.state=MOTOR_PROBE_SERVICE_CLEANUP;
        else failure(p,MOTOR_PROBE_SERVICE_SEQUENCE);
    }
    if(p->report.error || p->report.state==MOTOR_PROBE_SERVICE_CLEANUP) cleanup(p,now);
    refresh(p);
}
void motor_probe_snapshot(const struct motor_probe *p,volatile struct motor_probe_snapshot *out)
{
    volatile uint8_t *to=(volatile uint8_t *)out;
    if(p->initialized==PROBE_INITIALIZED) {
        const uint8_t *from=(const uint8_t *)&p->report;
        for(size_t i=0;i<sizeof *out;++i) to[i]=from[i];
    } else {
        for(size_t i=0;i<sizeof *out;++i) to[i]=0;
        out->version=1; out->bytes=sizeof *out;
        out->state=MOTOR_PROBE_SERVICE_FAILED; out->error=MOTOR_PROBE_SERVICE_STATE;
    }
}
