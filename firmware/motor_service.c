#include "ciu32l071.h"
#include "motor_service.h"
#include "motor_trace.h"
#include "../startup/w600_motor_stop.h"
#ifdef W600_FLASH_APPLICATION
void W600_UART3_4_Handler(void);
#endif
#ifndef MOTOR_SERVICE_OBSERVE
#define MOTOR_SERVICE_OBSERVE(reg,value) ((void)0)
#endif
#define WRITE(reg,value) do { (reg)=(value); __DSB(); MOTOR_SERVICE_OBSERVE(&(reg),(uint32_t)(value)); } while (0)
enum { OWN_TIMER=1, OWN_AF=2, END_MS=45000, PERIOD_MS=20, WORK_MARGIN_MS=3,
       CYCLES_MS=48000, SETTLE_CYCLES=24000, LATE_CYCLES=12000 };
static const struct motor_homing_policy policy={45000,100,100,10,MOTOR_HOMING_MOTION_MIN_MV,100,4000,200,20,11,2};
static bool context(void)
{
    return __get_PRIMASK()==1 && !__get_IPSR() && !__get_CONTROL() &&
        !(NVIC->ISER[0]&~(1u<<UART3_4_IRQn));
}
static uint32_t time_ticks(const struct w600_app_time *t)
{
    return t->ms*10u+(t->cycles-t->ms*CYCLES_MS)/4800u;
}
static bool time_bounds(const struct w600_motor_service *s,uint32_t before,uint32_t after,
                         uint32_t *lower,uint32_t *upper)
{
    uint32_t lo=before-s->on_started_after, hi=after-s->on_started_before;
    if (s->limit_ms<250u || s->limit_ms>END_MS) return false;
    uint32_t limit=s->limit_ms*10u;
    if (!s->time_valid || !limit || lo>hi || hi>=limit) return false;
    /* A quantized edge lies anywhere inside its100us counter tick. */
    lo=lo ? lo-1u : 0; ++hi;
    if (s->on_upper>limit-hi) return false;
    *lower=s->on_lower+lo; *upper=s->on_upper+hi;
    return *lower<=*upper;
}
static bool time_off(struct w600_motor_service *s,uint32_t before,uint32_t after)
{
    if (!s->on_active) return s->time_valid;
    s->on_active=false;
    if (!time_bounds(s,before,after,&s->on_lower,&s->on_upper)) {
        s->time_valid=false; return false;
    }
    return true;
}
static void service_stop(void *arg)
{
    struct w600_motor_service *s=arg;
    struct w600_app_time before,after;
    bool timed=s->on_active && s->time_valid && s->ticks && context() && w600_app_clock(s->ticks,&before);
    s->stop_confirmed=(uint32_t)w600_motor_stop();
    if (s->on_active && (!timed || !s->stop_confirmed ||
        !w600_app_clock(s->ticks,&after) || !time_off(s,time_ticks(&before),time_ticks(&after)))) {
        s->time_valid=false; s->on_active=false;
    }
}
static void fail(struct w600_motor_service *s, enum w600_motor_error error)
{
    if (!s->error) s->error=error;
    s->time_valid=false;
    motor_homing_abort(&s->homing);
    service_stop(s);
    s->phase=W600_MOTOR_CLEANUP;
}
static bool selectors(const struct w600_motor_service *s, bool selected)
{
    uint32_t expected_a=selected && s->direction==2 ? 0x8000u : 0;
    uint32_t expected_c=selected && s->direction==1 ? 0x400u : 0;
    return (GPIOA->MODE&0xc0000000u)==0x40000000u &&
        (GPIOC->MODE&0x00300000u)==0x00100000u &&
        !(GPIOA->OTYPE&0x8000u) && !(GPIOC->OTYPE&0x400u) &&
        !(GPIOA->PUPD&0xc0000000u) && !(GPIOC->PUPD&0x00300000u) &&
        (GPIOA->ODR&0x8000u)==expected_a && (GPIOA->IDR&0x8000u)==expected_a &&
        (GPIOC->ODR&0x400u)==expected_c && (GPIOC->IDR&0x400u)==expected_c;
}
static bool timer_config(const struct w600_motor_service *s)
{
    uint32_t ccm=s->direction==2 ? 0x7078u : 0x7870u;
    return context() && s->limit_ms>=250u && s->limit_ms<=END_MS &&
        RCC->CFG==0x40012u && RCC->PLLCFG==0x30260u &&
        (RCC->CSR1&(RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY|RCC_CSR1_RCHDIV))==
            (RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY) &&
        (FLASH->ACR&FLASH_ACR_LATENCY)==FLASH_ACR_LATENCY_2CLK &&
        !(RCC->AHBEN&RCC_AHBEN_DMAEN) &&
        (RCC->APB1EN&(RCC_APB1EN_TIM4EN|RCC_APB1EN_PMUEN))==
            (RCC_APB1EN_TIM4EN|RCC_APB1EN_PMUEN) &&
        (RCC->APB2EN&RCC_APB2EN_ADCEN) &&
        !(RCC->APB1RST&RCC_APB1RST_TIM4_RST) &&
        (TIM4->CR1&~1u)==0x88u && !TIM4->CR2 && !TIM4->SMC && !TIM4->DIER &&
        TIM4->CCM1==ccm && !TIM4->CCM2 && TIM4->CCEN==0x33u &&
        TIM4->PSC==47999u && TIM4->ARR==s->limit_ms-1u &&
        TIM4->CC1==(s->direction==2 ? 1u : s->expected_cc2) &&
        TIM4->CC2==(s->direction==2 ? s->expected_cc2 : 1u) &&
        !TIM4->CC3 && !TIM4->CC4 &&
        (GPIOB->MODE&0xf0000000u)==0xa0000000u &&
        (GPIOB->AFH&0xff000000u)==0x15000000u &&
        !(GPIOB->OTYPE&0xc000u) && !(GPIOB->PUPD&0xf0000000u) &&
        (GPIOB->ODR&0xc000u)==0xc000u;
}
static void compare(struct w600_motor_service *s, uint32_t value)
{
    s->expected_cc2=value;
    if (s->direction==2) WRITE(TIM4->CC2,value);
    else WRITE(TIM4->CC1,value);
}
static bool arm(struct w600_motor_service *s)
{
    if (!s->stop_confirmed || !selectors(s,false) ||
        (RCC->APB1EN&RCC_APB1EN_TIM4EN)) return false;
    s->saved_afh=GPIOB->AFH; s->owned=OWN_TIMER;
    WRITE(RCC->APB1EN,RCC->APB1EN|RCC_APB1EN_TIM4EN);
    if (!(RCC->APB1EN&RCC_APB1EN_TIM4EN)) return false;
    WRITE(TIM4->CR1,0x88); WRITE(TIM4->CCEN,0);
    WRITE(TIM4->CR2,0); WRITE(TIM4->SMC,0); WRITE(TIM4->DIER,0);
    WRITE(TIM4->CCM1,s->direction==2 ? 0x7078u : 0x7870u);
    WRITE(TIM4->CCM2,0); WRITE(TIM4->PSC,47999);
    WRITE(TIM4->ARR,s->limit_ms-1u); WRITE(TIM4->CC1,1); WRITE(TIM4->CC2,1);
    WRITE(TIM4->CC3,0); WRITE(TIM4->CC4,0);
    WRITE(TIM4->EVTG,1); WRITE(TIM4->CNT,0); WRITE(TIM4->SR,0);
    WRITE(TIM4->CCEN,0x33); s->expected_cc2=1;
    s->owned|=OWN_AF;
    WRITE(GPIOB->AFH,(GPIOB->AFH&~0xff000000u)|0x15000000u);
    WRITE(GPIOB->MODE,(GPIOB->MODE&~0xf0000000u)|0xa0000000u);
    if (!timer_config(s) || TIM4->CR1!=0x88 || TIM4->CNT || TIM4->SR ||
        (GPIOB->IDR&0xc000u)!=0xc000u || !selectors(s,false)) return false;
    struct w600_app_time now;
    struct motor_homing_policy bounded=policy;
    bounded.max_move_ticks=s->limit_ms;
    if (!w600_app_clock(s->ticks,&now) || !motor_homing_start(&s->homing,now.ms,&bounded)) return false;
    s->started_ms=now.ms; s->arm_cycles=now.cycles; s->next_sample=1;
    w600_motor_trace_begin(s->started_ms,s->direction);
    s->phase=W600_MOTOR_DRIVING; s->stop_confirmed=0;
    WRITE(TIM4->CR1,0x89);
    return TIM4->CR1==0x89;
}
static void cleanup(struct w600_motor_service *s)
{
    service_stop(s);
    if (!s->stop_confirmed) goto bad;
    if (s->owned&OWN_TIMER) {
        if (!(RCC->APB1EN&RCC_APB1EN_TIM4EN)) goto bad;
        WRITE(TIM4->CR1,0x88); WRITE(TIM4->CCEN,0); WRITE(TIM4->DIER,0);
        if ((TIM4->CR1&1u) || TIM4->CCEN || TIM4->DIER) goto bad;
    }
    if (s->owned&OWN_AF) {
        WRITE(GPIOB->AFH,(GPIOB->AFH&~0xff000000u)|(s->saved_afh&0xff000000u));
        if ((GPIOB->AFH&0xff000000u)!=(s->saved_afh&0xff000000u)) goto bad;
        s->owned&=~OWN_AF;
    }
    if (s->owned&OWN_TIMER) {
        WRITE(RCC->APB1EN,RCC->APB1EN&~RCC_APB1EN_TIM4EN);
        if (RCC->APB1EN&RCC_APB1EN_TIM4EN) goto bad;
        s->owned&=~OWN_TIMER;
    }
    if (s->feedback.owns_adc) w600_feedback_abort(&s->feedback);
    if (s->board.owned && !w600_feedback_board_finish(&s->board)) goto bad;
    if (!s->error && !s->time_valid) s->error=W600_MOTOR_TIMING;
    if (!s->error) s->completed_ms=*s->ticks;
    s->phase=s->error ? W600_MOTOR_FAILED : W600_MOTOR_DONE;
    return;
bad:
    s->cleanup_failed=1;
    if (!s->error) s->error=W600_MOTOR_RELEASE;
    s->phase=W600_MOTOR_FAILED;
}
bool w600_motor_service_init(struct w600_motor_service *s, const volatile uint32_t *ticks)
{
    if (!ticks || !context() || s->phase!=W600_MOTOR_IDLE || s->error || s->owned ||
        s->board.owned || s->feedback.owns_adc) return false;
    s->ticks=ticks;
    if (!motor_homing_init(&s->homing,service_stop,s)) return false;
    return s->stop_confirmed!=0;
}
bool w600_motor_service_begin(struct w600_motor_service *s, unsigned direction)
{ return w600_motor_service_begin_bounded(s,direction,END_MS); }

bool w600_motor_service_begin_bounded(struct w600_motor_service *s,unsigned direction,uint32_t limit_ms)
{
    if (!s->ticks || !context() || s->phase!=W600_MOTOR_IDLE || s->error || s->owned ||
        s->board.owned || s->feedback.owns_adc || !s->stop_confirmed ||
        (direction!=1 && direction!=2) || limit_ms<250u || limit_ms>END_MS) {
        fail(s,W600_MOTOR_STATE); return false;
    }
    struct w600_app_time now;
    if (!w600_app_clock(s->ticks,&now)) { fail(s,W600_MOTOR_TIMING); return false; }
    s->direction=direction; s->limit_ms=limit_ms;
    s->started_ms=s->last_ms=now.ms; s->polls=0;
    s->on_lower=s->on_upper=s->sample_on_lower=s->sample_on_upper=0;
    s->on_active=false; s->time_valid=true;
    if (!w600_feedback_board_prepare(&s->board)) {
        fail(s,W600_MOTOR_CONTEXT); return false;
    }
    if (w600_feedback_init(&s->feedback,now.ms)!=W600_FEEDBACK_BUSY) {
        fail(s,W600_MOTOR_ADC); return false;
    }
    s->phase=W600_MOTOR_PREPARING;
    return true;
}
static bool finish_failure(struct w600_motor_service *s,enum w600_motor_error error)
{
    if (!s->error) s->error=error;
    /* finish may itself be a sequencer stop callback: never dispatch a guard here. */
    w600_motor_service_abort(s);
    return false;
}
bool w600_motor_service_finish(struct w600_motor_service *s)
{
    if (s->phase<W600_MOTOR_PREPARING || s->phase>W600_MOTOR_RESTORING || s->error)
        return false;
    if (!context()) return finish_failure(s,W600_MOTOR_CONTEXT);
    bool driving=s->phase>=W600_MOTOR_DRIVING;
    bool timer_ok=!driving || timer_config(s);
    service_stop(s);
    /* Stop first, but do not turn an unprocessed deadline into a clean finish. */
    if (driving) {
        struct w600_app_time now;
        if (!s->ticks || !w600_app_clock(s->ticks,&now) ||
            (uint32_t)(now.ms-s->last_ms)>=0x80000000u) {
            return finish_failure(s,W600_MOTOR_TIMING);
        }
        enum motor_guard_error due=motor_guard_due(&s->homing.guard,now.ms);
        if (s->homing.guard.error || due) {
            if (!s->homing.guard.error) s->homing.guard.error=due;
            if (!s->homing.error) s->homing.error=MOTOR_HOMING_GUARD_FAILURE;
            return finish_failure(s,W600_MOTOR_FEEDBACK);
        }
        if (s->homing.have_sample && (uint32_t)(now.ms-
            (s->homing.last_sample_at+s->homing.policy.feedback_lease_ticks))<0x80000000u) {
            if (!s->homing.error) s->homing.error=MOTOR_HOMING_STALE_FEEDBACK;
            return finish_failure(s,W600_MOTOR_FEEDBACK);
        }
        if (!s->homing.guard.armed ||
            s->homing.error || (s->homing.state!=MOTOR_HOMING_SEEK_MOTION &&
            s->homing.state!=MOTOR_HOMING_SEEK_ENDPOINT))
            return finish_failure(s,W600_MOTOR_FEEDBACK);
        if (!timer_ok) return finish_failure(s,W600_MOTOR_TIMER);
        if (TIM4->CR1!=0x89u || (TIM4->SR&1u) || TIM4->CNT>=s->limit_ms)
            return finish_failure(s,W600_MOTOR_TIMEOUT);
    }
    if (!s->stop_confirmed) return finish_failure(s,W600_MOTOR_PINS);
    if (!s->time_valid) return finish_failure(s,W600_MOTOR_TIMING);
    s->homing.guard.armed=false;
    s->requested_stop=1;
    s->phase=W600_MOTOR_CLEANUP;
    return true;
}
bool w600_motor_service_rearm(struct w600_motor_service *s)
{
    if (!context() || !s->ticks || s->phase!=W600_MOTOR_DONE || s->error ||
        s->cleanup_failed || s->owned || s->board.owned || s->board.prepared || s->board.error ||
        s->homing.error || s->homing.guard.error || s->homing.guard.armed ||
        (!s->requested_stop && s->homing.state!=MOTOR_HOMING_ENDPOINT_OBSERVED) ||
        !s->stop_confirmed || RCC->CFG!=0x12u ||
        (RCC->APB1EN&(RCC_APB1EN_TIM4EN|RCC_APB1EN_PMUEN)) ||
        (RCC->APB2EN&RCC_APB2EN_ADCEN) || !selectors(s,false)) return false;
    /* One extra tick covers a pending SysTick at the recorded stop. */
    uint32_t elapsed=*s->ticks-s->completed_ms;
    if (elapsed<101u || elapsed>=0x80000000u) return false;
    if (!w600_motor_stop()) { fail(s,W600_MOTOR_PINS); return false; }
    if (!motor_homing_reset(&s->homing,true)) return false;
#ifdef W600_FLASH_APPLICATION
    W600_UART3_4_Handler();
#endif
    /* ADC was reset and gated by successful board cleanup, not merely aborted. */
    s->feedback=(struct w600_feedback){0};
    s->sample=(struct w600_feedback_sample){0};
    s->board=(struct w600_feedback_board){0};
    s->direction=s->started_ms=s->arm_cycles=s->clamp_cycles=s->next_sample=0;
    s->expected_cc2=s->saved_afh=s->samples=s->last_mv=s->last_ms=s->polls=0;
    s->requested_stop=0; s->phase=W600_MOTOR_IDLE;
    s->on_lower=s->on_upper=s->on_started_before=s->on_started_after=0;
    s->sample_on_lower=s->sample_on_upper=s->sample_at_ticks=0; s->on_active=s->time_valid=false;
    s->limit_ms=0;
    return true;
}
void w600_motor_service_tick(struct w600_motor_service *s, uint32_t now_ms)
{
    if (s->phase>=W600_MOTOR_DRIVING && s->phase<=W600_MOTOR_RESTORING) {
        motor_homing_tick(&s->homing,now_ms);
        if (s->homing.state==MOTOR_HOMING_FAILED) fail(s,W600_MOTOR_FEEDBACK);
    }
}
bool w600_motor_service_urgent(const struct w600_motor_service *s)
{
    if (s->phase==W600_MOTOR_DRIVING) {
        if (!(s->owned&OWN_TIMER) || !(RCC->APB1EN&RCC_APB1EN_TIM4EN)) return true;
        uint32_t count=TIM4->CNT;
        /* Reserve time before the edge too: controller work is not instantaneous. */
        return count>=s->next_sample || s->next_sample-count<=WORK_MARGIN_MS;
    }
    return s->phase>=W600_MOTOR_SETTLING && s->phase<=W600_MOTOR_CLEANUP;
}
bool w600_motor_service_time(const struct w600_motor_service *s,struct w600_motor_time *out)
{
    if (!out) return false;
    *out=(struct w600_motor_time){0};
    struct w600_app_time now;
    if (!context() || !s->ticks || !s->time_valid || s->error ||
        !w600_app_clock(s->ticks,&now)) return false;
    uint32_t lower=s->on_lower,upper=s->on_upper;
    if (s->on_active) {
        if (!(s->owned&OWN_TIMER) || !timer_config(s) || !(TIM4->CR1&1u) ||
            !selectors(s,false) || (GPIOB->IDR&0xc000u)!=(s->direction==2 ? 0x8000u : 0x4000u) ||
            !time_bounds(s,time_ticks(&now),time_ticks(&now),&lower,&upper)) return false;
    }
    *out=(struct w600_motor_time){time_ticks(&now),lower,upper,s->sample_on_lower,s->sample_on_upper};
    return true;
}
static uint32_t signal_mv(const struct w600_motor_service *s)
{
    unsigned order[3]={0,1,2};
    for (unsigned i=1;i<3;++i) for (unsigned j=i;j>0;--j) {
        const struct w600_feedback_pair *a=&s->sample.pair[order[j-1]];
        const struct w600_feedback_pair *b=&s->sample.pair[order[j]];
        if ((uint32_t)a->signal*b->bgr<=(uint32_t)b->signal*a->bgr) break;
        unsigned swap=order[j]; order[j]=order[j-1]; order[j-1]=swap;
    }
    const struct w600_feedback_pair *p=&s->sample.pair[order[1]];
    return (uint32_t)(((uint64_t)p->signal*s->feedback.factory_bgr*3300u)/
                      ((uint32_t)p->bgr*4095u));
}
void w600_motor_service_poll(struct w600_motor_service *s)
{
    if (!context()) { fail(s,W600_MOTOR_CONTEXT); return; }
    if (s->phase==W600_MOTOR_IDLE || s->phase==W600_MOTOR_DONE || s->phase==W600_MOTOR_FAILED) return;
    if (s->phase==W600_MOTOR_CLEANUP) { cleanup(s); return; }
    struct w600_app_time now;
    if (!s->ticks || !w600_app_clock(s->ticks,&now)) { fail(s,W600_MOTOR_TIMING); return; }
    if ((uint32_t)(now.ms-s->last_ms)>=0x80000000u || ++s->polls>20000000u) {
        fail(s,W600_MOTOR_TIMING); return;
    }
    s->last_ms=now.ms;
    if (s->phase==W600_MOTOR_PREPARING) {
        enum w600_feedback_status status=w600_feedback_poll(&s->feedback,now.ms,0);
        if (status==W600_FEEDBACK_FAILED) fail(s,W600_MOTOR_ADC);
        else if (status==W600_FEEDBACK_READY && !arm(s)) fail(s,W600_MOTOR_TIMER);
        return;
    }
    w600_motor_service_tick(s,now.ms);
    if (s->phase==W600_MOTOR_CLEANUP) return;
    if (!timer_config(s)) { fail(s,W600_MOTOR_TIMER); return; }
    uint32_t before=TIM4->CNT, cr=TIM4->CR1, pads=GPIOB->IDR&0xc000u;
    uint32_t count=TIM4->CNT, after=TIM4->CR1;
    if (before>count || cr!=after || (before<1u)!=(count<1u)) return;
    if (count<1u && s->next_sample!=1u) { fail(s,W600_MOTOR_TIMER); return; }
    if (!(cr&1u)) { fail(s,W600_MOTOR_TIMEOUT); return; }
    if (count>=s->limit_ms || (TIM4->SR&1u) || (TIM4->SR&~0x1fu)) { fail(s,W600_MOTOR_TIMER); return; }
    bool selected=s->phase>=W600_MOTOR_SETTLING && s->phase<=W600_MOTOR_RESTORING;
    uint32_t expected=count<1u ? 0xc000u : s->expected_cc2==s->limit_ms ?
        (s->direction==2 ? 0x8000u : 0x4000u) : 0u;
    if (pads!=expected || !selectors(s,selected)) { fail(s,W600_MOTOR_PINS); return; }
    uint32_t elapsed=now.cycles-s->arm_cycles;
    if (elapsed>=(s->limit_ms+1000u)*CYCLES_MS) { fail(s,W600_MOTOR_TIMING); return; }
    if (s->phase==W600_MOTOR_DRIVING) {
        if (!motor_homing_foreground_progress(&s->homing,now.ms)) { fail(s,W600_MOTOR_FEEDBACK); return; }
        if (count<s->next_sample) return;
        if (count!=s->next_sample) { fail(s,W600_MOTOR_TIMING); return; }
        /* CNT may cross the edge after the earlier clock snapshot. Retry promptly. */
        if (elapsed<s->next_sample*CYCLES_MS) return;
        if (elapsed-s->next_sample*CYCLES_MS>=LATE_CYCLES) { fail(s,W600_MOTOR_TIMING); return; }
        uint32_t off_before=time_ticks(&now);
        compare(s,1);
        /* Never connect a sensing clamp while a failed compare leaves drive on. */
        if ((s->direction==2 ? TIM4->CC2 : TIM4->CC1)!=1u ||
            (GPIOB->IDR&0xc000u)) { fail(s,W600_MOTOR_TIMER); return; }
        if (s->direction==2) WRITE(GPIOA->BSR,0x8000); else WRITE(GPIOC->BSR,0x400);
        if (!selectors(s,true) || (GPIOB->IDR&0xc000u)) { fail(s,W600_MOTOR_PINS); return; }
        if (!w600_app_clock(s->ticks,&now) ||
            now.cycles-s->arm_cycles-s->next_sample*CYCLES_MS>=LATE_CYCLES) {
            fail(s,W600_MOTOR_TIMING); return;
        }
        if (!time_off(s,off_before,time_ticks(&now))) { fail(s,W600_MOTOR_TIMING); return; }
        s->sample_on_lower=s->on_lower; s->sample_on_upper=s->on_upper;
        s->clamp_cycles=now.cycles; s->phase=W600_MOTOR_SETTLING;
        return;
    }
    uint32_t restore=s->next_sample*CYCLES_MS+CYCLES_MS+CYCLES_MS/2u;
    if (count<s->next_sample || count>s->next_sample+1u || elapsed>=restore+LATE_CYCLES) {
        fail(s,W600_MOTOR_TIMING); return;
    }
    if (s->phase==W600_MOTOR_SETTLING) {
        if ((uint32_t)(now.cycles-s->clamp_cycles)<SETTLE_CYCLES) return;
        if (elapsed>=restore || w600_feedback_start(&s->feedback,now.ms)!=W600_FEEDBACK_BUSY) {
            fail(s,W600_MOTOR_ADC); return;
        }
        s->phase=W600_MOTOR_SAMPLING; return;
    }
    if (s->phase==W600_MOTOR_SAMPLING) {
        if (elapsed>=restore) { fail(s,W600_MOTOR_TIMING); return; }
        enum w600_feedback_status status=w600_feedback_poll(&s->feedback,now.ms,&s->sample);
        if (!w600_app_clock(s->ticks,&now) || now.cycles-s->arm_cycles>=restore) {
            fail(s,W600_MOTOR_TIMING); return;
        }
        if (status==W600_FEEDBACK_BUSY) return;
        if (status!=W600_FEEDBACK_SAMPLE || !s->sample.valid || s->sample.error ||
            s->sample.sequence!=s->samples+1u) { fail(s,W600_MOTOR_ADC); return; }
        for (unsigned i=0;i<3;++i) if (!s->sample.pair[i].bgr || s->sample.pair[i].bgr>4095u ||
            s->sample.pair[i].signal>4095u) { fail(s,W600_MOTOR_ADC); return; }
#ifdef W600_FLASH_APPLICATION
        /* Receive a waiting byte before the longer 64-bit voltage conversion. */
        W600_UART3_4_Handler();
#endif
        s->last_mv=signal_mv(s); s->sample_at_ticks=s->sample.oldest_at_ms*10u; ++s->samples;
        w600_motor_trace_record(s->started_ms,s->samples,s->sample_at_ticks,s->last_mv);
        if (s->next_sample!=1u && !motor_homing_feedback(&s->homing,now.ms,
                s->sample.oldest_at_ms,s->last_mv,true)) { fail(s,W600_MOTOR_FEEDBACK); return; }
        if (s->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED) {
            s->phase=W600_MOTOR_CLEANUP; return;
        }
        s->phase=W600_MOTOR_RESTORING; return;
    }
    if (elapsed<restore) return;
    if (s->direction==2) WRITE(GPIOA->BR,0x8000); else WRITE(GPIOC->BR,0x400);
    if (!selectors(s,false)) { fail(s,W600_MOTOR_PINS); return; }
    uint32_t on_before=time_ticks(&now);
    compare(s,s->limit_ms);
    if ((GPIOB->IDR&0xc000u)!=(s->direction==2 ? 0x8000u : 0x4000u)) {
        fail(s,W600_MOTOR_PINS); return;
    }
    if (!w600_app_clock(s->ticks,&now) || now.cycles-s->arm_cycles>=restore+LATE_CYCLES) {
        fail(s,W600_MOTOR_TIMING); return;
    }
    if (s->on_active || !s->time_valid) { fail(s,W600_MOTOR_TIMING); return; }
    s->on_started_before=on_before; s->on_started_after=time_ticks(&now); s->on_active=true;
    s->next_sample=s->next_sample==1u ? PERIOD_MS : s->next_sample+PERIOD_MS;
    s->phase=W600_MOTOR_DRIVING;
}
void w600_motor_service_abort(struct w600_motor_service *s)
{
    /* Leaf stop: safe from an independent sequencer's guard callback/IRQ. */
    if (!s->error) s->error=W600_MOTOR_ABORTED;
    s->time_valid=false;
    s->homing.guard.armed=false;
    if (!s->homing.guard.error) s->homing.guard.error=MOTOR_GUARD_EXTERNAL_FAULT;
    if (!s->homing.error) s->homing.error=MOTOR_HOMING_ABORTED;
    s->homing.state=MOTOR_HOMING_FAILED;
    service_stop(s); s->phase=W600_MOTOR_CLEANUP;
}
