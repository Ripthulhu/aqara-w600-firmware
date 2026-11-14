#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"
#include "motor_service.h"
static RCC_t rcc;
static FLASH_t flash;
static GPIO_t a,b,c;
static TIM_t timer;
static NVIC_Type nvic;
static uint64_t cycles, armed_at;
static uint64_t observed_on_cycles, observed_on_started, observed_attempt_base;
static bool observed_on;
static uint32_t epoch, tick_ms, mask, ipsr, control;
static unsigned starts, stop_calls, ui_polls, samples, scenario, finish_calls, clamps;
static unsigned allowed_starts=1;
static int clock_valid=1;
static bool poll_active, edge_injected;
static uint32_t stop_delay_cycles;
static uint32_t clock_delay_cycles,clock_delay_skip;
static void advance_hardware(uint32_t delta);
static void pads(void)
{
    a.IDR=a.ODR; b.IDR=b.ODR; c.IDR=c.ODR;
    for (unsigned pin=14;pin<=15;++pin) if (((b.MODE>>(2*pin))&3u)==2u) {
        uint32_t cc=pin==14 ? timer.CC1 : timer.CC2;
        if (timer.CNT>=cc) b.IDR&=~(1u<<pin); else b.IDR|=1u<<pin;
    }
    bool driving=(b.IDR&0xc000u)==0x4000u || (b.IDR&0xc000u)==0x8000u;
    if (driving && !observed_on) observed_on_started=cycles;
    if (!driving && observed_on) observed_on_cycles+=cycles-observed_on_started;
    observed_on=driving;
}
static void written(volatile void *reg,uint32_t value)
{
    if (reg==&a.BSR) a.ODR|=value;
    if (reg==&a.BR) a.ODR&=~value;
    if (reg==&c.BSR) c.ODR|=value;
    if (reg==&c.BR) c.ODR&=~value;
    if (reg==&timer.CR1 && (value&1u)) { ++starts; assert(starts<=allowed_starts); armed_at=cycles; }
    if (scenario==7 && starts && samples && value==1 &&
        (reg==&timer.CC1 || reg==&timer.CC2)) *(volatile uint32_t *)reg=45000;
    if ((reg==&a.BSR && (value&0x8000u)) || (reg==&c.BSR && (value&0x400u))) {
        assert((b.IDR&0xc000u)==0); ++clamps;
    }
    if (scenario==6 && reg==&timer.PSC) timer.PSC=1;
    if (scenario==9 && reg==&rcc.APB1EN && !(value&RCC_APB1EN_TIM4EN)) rcc.APB1EN|=RCC_APB1EN_TIM4EN;
    pads();
}
#undef RCC
#undef FLASH
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef TIM4
#undef NVIC
#define RCC (&rcc)
#define FLASH (&flash)
#define GPIOA (&a)
#define GPIOB (&b)
#define GPIOC (&c)
#define TIM4 (&timer)
#define NVIC (&nvic)
#define __get_PRIMASK() mask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#define __DSB() ((void)0)
#define MOTOR_SERVICE_OBSERVE(reg,value) written(reg,value)
#define W600_FLASH_APPLICATION 1
void W600_UART3_4_Handler(void)
{
    uint32_t outputs=b.IDR&0xc000u;
    assert(mask==1 && !ipsr && (outputs==0 || outputs==0xc000u));
    if (outputs) assert((b.MODE&0xf0000000u)==0x50000000u &&
                        !(a.IDR&0x8000u) && !(c.IDR&0x400u));
}
#include "motor_service.c"
bool w600_app_clock(const volatile uint32_t *ticks, struct w600_app_time *out)
{
    assert(ticks==&tick_ms && mask==1 && !ipsr);
    if(clock_delay_cycles) {
        if(clock_delay_skip) --clock_delay_skip;
        else {uint32_t delay=clock_delay_cycles;clock_delay_cycles=0;advance_hardware(delay);}
    }
    out->ms=epoch+(uint32_t)(cycles/48000u); out->cycles=epoch*48000u+(uint32_t)cycles;
    if (scenario==13 && poll_active && starts==1 && !samples && !timer.CNT && !edge_injected) {
        edge_injected=true;
        cycles=armed_at+48000u; tick_ms=epoch+(uint32_t)(cycles/48000u);
        timer.CNT=1; pads();
    }
    return clock_valid!=0;
}
int w600_motor_stop(void)
{
    ++stop_calls;
    a.ODR&=~0x8000u; c.ODR&=~0x400u;
    a.MODE=(a.MODE&~0xc0000000u)|0x40000000u;
    c.MODE=(c.MODE&~0x00300000u)|0x00100000u;
    b.MODE=(b.MODE&~0xf0000000u)|0x50000000u; b.ODR|=0xc000u;
    pads();
    if (stop_delay_cycles) {
        uint32_t delay=stop_delay_cycles;stop_delay_cycles=0;
        advance_hardware(delay);
    }
    return scenario!=10;
}
bool w600_feedback_board_prepare(struct w600_feedback_board *s)
{
    assert(!observed_on); observed_attempt_base=observed_on_cycles;
    s->owned=1; s->prepared=true;
    rcc.CFG=0x40012; rcc.APB1EN|=RCC_APB1EN_PMUEN; rcc.APB2EN|=RCC_APB2EN_ADCEN;
    return true;
}
bool w600_feedback_board_finish(struct w600_feedback_board *s)
{
    ++finish_calls;
    assert(!(rcc.APB1EN&RCC_APB1EN_TIM4EN));
    assert((b.MODE&0xf0000000u)==0x50000000u && (b.IDR&0xc000u)==0xc000u);
    if (scenario==11) return false;
    s->owned=0; s->prepared=false; rcc.CFG=0x12;
    rcc.APB1EN&=~RCC_APB1EN_PMUEN; rcc.APB2EN&=~RCC_APB2EN_ADCEN;
    return true;
}
enum w600_feedback_status w600_feedback_init(struct w600_feedback *s,uint32_t now)
{
    s->owns_adc=true; s->factory_bgr=1500; s->operation_ms=now;
    return W600_FEEDBACK_BUSY;
}
enum w600_feedback_status w600_feedback_start(struct w600_feedback *s,uint32_t now)
{
    assert((b.IDR&0xc000u)==0);
    assert(((a.IDR&0x8000u)!=0)!=((c.IDR&0x400u)!=0));
    s->phase=1; s->operation_ms=now;
    return W600_FEEDBACK_BUSY;
}
enum w600_feedback_status w600_feedback_poll(struct w600_feedback *s,uint32_t now,struct w600_feedback_sample *out)
{
    if (!s->phase) return W600_FEEDBACK_READY;
    assert(out);
    if (scenario==3) return W600_FEEDBACK_BUSY;
    if (scenario==4) return W600_FEEDBACK_FAILED;
    if (scenario==8) cycles+=48000u;
    ++samples; memset(out,0,sizeof *out);
    out->valid=true; out->sequence=++s->sample.sequence;
    out->oldest_at_ms=scenario==5 ? now-10 : s->operation_ms;
    uint16_t signal=samples<=3 || (samples>=20 && scenario!=2) ? 0 : 1700;
    for (unsigned i=0;i<3;++i) { out->pair[i].bgr=1500; out->pair[i].signal=signal; }
    if (scenario==12) out->pair[0].bgr=0;
    s->phase=0;
    return W600_FEEDBACK_SAMPLE;
}
void w600_feedback_abort(struct w600_feedback *s) { s->phase=0; }
static void fresh(struct w600_motor_service *s)
{
    memset(s,0,sizeof *s); memset(&rcc,0,sizeof rcc); memset(&a,0,sizeof a);
    memset(&b,0,sizeof b); memset(&c,0,sizeof c); memset(&timer,0,sizeof timer);
    memset(&nvic,0,sizeof nvic); cycles=0; starts=stop_calls=ui_polls=samples=finish_calls=0;
    observed_on_cycles=observed_on_started=observed_attempt_base=0; observed_on=false;
    epoch=0; tick_ms=0; mask=1; ipsr=control=0; clock_valid=1; clamps=0; allowed_starts=1;
    poll_active=edge_injected=false;stop_delay_cycles=clock_delay_cycles=clock_delay_skip=0;
    rcc.IOPEN=7; rcc.CFG=0x12; rcc.PLLCFG=0x30260;
    rcc.CSR1=RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY; flash.ACR=FLASH_ACR_LATENCY_2CLK;
    rcc.APB1EN=RCC_APB1EN_UART2EN|RCC_APB1EN_UART3EN; nvic.ISER[0]=1u<<27;
    assert(w600_motor_service_init(s,&tick_ms));
}
static void advance_hardware(uint32_t delta)
{
    cycles+=delta; tick_ms=epoch+(uint32_t)(cycles/48000u);
    if (timer.CR1&1u) {
        uint32_t count=(uint32_t)((cycles-armed_at)/48000u);
        if (count>timer.ARR) {timer.CNT=0; timer.CR1&=~1u; timer.SR|=1u;}
        else timer.CNT=count;
    }
    pads();
}
static void advance(struct w600_motor_service *s,uint32_t delta)
{
    uint32_t old=tick_ms;
    advance_hardware(delta);
    if (tick_ms!=old) { mask=0; ipsr=15; w600_motor_service_tick(s,tick_ms); ipsr=0; mask=1; }
}
static void run(struct w600_motor_service *s)
{
    struct w600_motor_time previous={0}; bool have_time=false;
    /* The pre-edge reservation adds fast polls; retain the45s hardware cutoff. */
    for (unsigned n=0;n<4000000 && s->phase!=W600_MOTOR_DONE && s->phase!=W600_MOTOR_FAILED;++n) {
        bool urgent=w600_motor_service_urgent(s);
        if (!urgent) ++ui_polls;
        advance(s,urgent ? 240u : 2400u);
        poll_active=true; w600_motor_service_poll(s); poll_active=false;
        if (s->samples) {
            assert(w600_motor_trace.magic==W600_MOTOR_TRACE_MAGIC);
            assert(w600_motor_trace.generation==s->started_ms && w600_motor_trace.direction==s->direction);
            assert(w600_motor_trace.count==s->samples && !w600_motor_trace.dropped && !w600_motor_trace.invalid);
            assert(w600_motor_trace.samples[s->samples-1].acquired_at_ticks==s->sample_at_ticks);
            assert(w600_motor_trace.samples[s->samples-1].millivolts==s->last_mv);
        }
        struct w600_motor_time time={UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX,UINT32_MAX};
        if (s->time_valid && !s->error) {
            assert(w600_motor_service_time(s,&time));
            uint64_t actual=observed_on_cycles-observed_attempt_base+(observed_on ? cycles-observed_on_started : 0);
            if (!((uint64_t)time.on_lower*4800u<=actual && actual<=(uint64_t)time.on_upper*4800u))
                fprintf(stderr,"time mismatch scenario=%u phase=%u ms=%u lower=%u upper=%u cycles=%llu active=%u\n",
                    scenario,s->phase,tick_ms,time.on_lower,time.on_upper,(unsigned long long)actual,s->on_active);
            assert((uint64_t)time.on_lower*4800u<=actual && actual<=(uint64_t)time.on_upper*4800u);
            assert(time.on_lower<=time.on_upper && time.sample_on_lower<=time.sample_on_upper);
            assert(time.sample_on_lower<=time.on_lower && time.sample_on_upper<=time.on_upper);
            if (have_time) {
                assert(time.on_lower>=previous.on_lower);
                assert(time.on_lower-previous.on_lower<=(uint32_t)(time.now_ticks-previous.now_ticks));
            }
            previous=time; have_time=true;
        } else {
            assert(!w600_motor_service_time(s,&time));
            assert(!time.now_ticks && !time.on_lower && !time.on_upper &&
                   !time.sample_on_lower && !time.sample_on_upper);
        }
    }
    assert(s->phase==W600_MOTOR_DONE || s->phase==W600_MOTOR_FAILED);
    assert((b.IDR&0xc000u)==0xc000u && !(a.IDR&0x8000u) && !(c.IDR&0x400u));
    assert((rcc.APB1EN&(RCC_APB1EN_UART2EN|RCC_APB1EN_UART3EN))==
        (RCC_APB1EN_UART2EN|RCC_APB1EN_UART3EN));
}
static void bounded_commands(void)
{
    struct w600_motor_service s;
    const uint32_t bad[]={0,1,249,45001,UINT32_MAX};
    for(unsigned i=0;i<sizeof bad/sizeof *bad;++i) {
        scenario=0;fresh(&s);
        assert(!w600_motor_service_begin_bounded(&s,2,bad[i]));run(&s);
        assert(!starts && s.error==W600_MOTOR_STATE && !s.owned && !s.board.owned);
    }
    const uint32_t limits[]={250,2000,5000,45000};
    for(uint32_t limit=250;limit<=45000;++limit) {
        scenario=0;fresh(&s);
        assert(w600_motor_service_begin_bounded(&s,1u+(limit&1u),limit));
        w600_motor_service_poll(&s);
        assert(timer.ARR==limit-1u && s.homing.guard.move_deadline==s.started_ms+limit);
        assert(w600_motor_service_finish(&s));run(&s);
        assert(s.phase==W600_MOTOR_DONE && s.requested_stop && !s.error);
    }
    for(unsigned i=0;i<sizeof limits/sizeof *limits;++i)
    for(unsigned dir=1;dir<=2;++dir) for(unsigned wrap=0;wrap<2;++wrap) {
        uint32_t limit=limits[i];scenario=2;fresh(&s);
        epoch=wrap ? UINT32_MAX-200u : 0;tick_ms=epoch;
        assert(w600_motor_service_begin_bounded(&s,dir,limit));
        w600_motor_service_poll(&s);
        assert(starts==1 && timer.ARR==limit-1u && s.homing.guard.move_deadline==s.started_ms+limit);
        run(&s);
        assert(s.phase==W600_MOTOR_FAILED && s.stop_confirmed && !s.owned && !s.board.owned);
        assert(s.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT);
        assert(s.homing.state!=MOTOR_HOMING_ENDPOINT_OBSERVED && !w600_motor_service_rearm(&s));
        assert((uint32_t)(tick_ms-s.started_ms)==limit);

        scenario=2;fresh(&s);
        assert(w600_motor_service_begin_bounded(&s,dir,limit));
        while(!(s.phase==W600_MOTOR_DRIVING && samples && (b.IDR&0xc000u)==(dir==2 ? 0x8000u : 0x4000u))) {
            advance(&s,240u);w600_motor_service_poll(&s);
        }
        uint64_t at=armed_at+(uint64_t)(limit-1u)*48000u;
        advance_hardware((uint32_t)(at-cycles));
        assert((timer.CR1&1u) && (b.IDR&0xc000u)==(dir==2 ? 0x8000u : 0x4000u));
        advance_hardware(48000u);
        assert(!(timer.CR1&1u) && (b.IDR&0xc000u)==0xc000u && s.homing.guard.armed);
        w600_motor_service_poll(&s);run(&s);
        assert(s.phase==W600_MOTOR_FAILED && s.stop_confirmed && !w600_motor_service_rearm(&s));
    }
    scenario=0;fresh(&s);allowed_starts=2;
    assert(w600_motor_service_begin_bounded(&s,2,2000));run(&s);
    assert(s.phase==W600_MOTOR_DONE && s.homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED);
    advance(&s,101u*48000u);assert(w600_motor_service_rearm(&s) && !s.limit_ms);
    samples=0;
    assert(w600_motor_service_begin_bounded(&s,1,5000));w600_motor_service_poll(&s);
    assert(timer.ARR==4999 && s.limit_ms==5000);
    assert(w600_motor_service_finish(&s));run(&s);
    assert(s.phase==W600_MOTOR_DONE && s.requested_stop && s.homing.state!=MOTOR_HOMING_ENDPOINT_OBSERVED);
    scenario=0;fresh(&s);assert(w600_motor_service_begin_bounded(&s,2,2000));
    w600_motor_service_poll(&s);
    assert(!w600_motor_service_begin_bounded(&s,1,5000));run(&s);
    assert(starts==1 && s.error==W600_MOTOR_STATE && !w600_motor_service_rearm(&s));
    puts("PASS bounded motor commands: limits/rejection, both directions/wrap, nonrenewed guard, independent timer cutoff without CPU/IRQ, clean endpoint/requested stop, explicit rearm and no fault retry");
}
static void finish_after_expiry(void)
{
    const uint32_t limits[]={250,2000,5000,45000};
    struct w600_motor_service s;
    for(unsigned i=0;i<sizeof limits/sizeof *limits;++i)
    for(unsigned dir=1;dir<=2;++dir) for(unsigned wrap=0;wrap<2;++wrap) {
        scenario=2;fresh(&s);
        epoch=wrap ? UINT32_MAX-200u : 0;tick_ms=epoch;
        assert(w600_motor_service_begin_bounded(&s,dir,limits[i]));
        w600_motor_service_poll(&s);
        advance_hardware(limits[i]*48000u);
        assert(!(timer.CR1&1u) && s.homing.guard.armed);
        /* Finish must not erase an expiry that no CPU/IRQ has processed yet. */
        assert(!w600_motor_service_finish(&s));
        run(&s);
        assert(s.phase==W600_MOTOR_FAILED && !s.requested_stop && s.stop_confirmed);
        assert(s.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT);
        assert(!w600_motor_service_rearm(&s) && starts==1);

        scenario=2;fresh(&s);
        epoch=wrap ? UINT32_MAX-200u : 0;tick_ms=epoch;
        assert(w600_motor_service_begin_bounded(&s,dir,limits[i]));
        w600_motor_service_poll(&s);
        advance_hardware((limits[i]-1u)*48000u);
        stop_delay_cycles=48000u;
        assert(!w600_motor_service_finish(&s));run(&s);
        assert(s.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT && !s.requested_stop);
    }
    for(unsigned fault=0;fault<11;++fault) {
        scenario=2;fresh(&s);
        assert(w600_motor_service_begin_bounded(&s,2,2000));
        w600_motor_service_poll(&s);
        if(fault==0) advance_hardware(100u*48000u);
        if(fault==1) {
            advance_hardware(99u*48000u);
            assert(motor_homing_foreground_progress(&s.homing,tick_ms));
            advance_hardware(48000u);
        }
        if(fault==2) clock_valid=0;
        if(fault==3) timer.ARR=500;
        if(fault==4) timer.CR1&=~1u;
        if(fault==5) s.homing.guard.armed=false;
        if(fault==6) s.homing.state=MOTOR_HOMING_IDLE;
        if(fault==7) timer.SR|=1u;
        if(fault==8) timer.CNT=2000;
        if(fault==9) timer.PSC=1;
        if(fault==10) timer.CC1=2;
        assert(!w600_motor_service_finish(&s));
        clock_valid=1;run(&s);
        assert(s.phase==W600_MOTOR_FAILED && s.stop_confirmed && !s.requested_stop);
        assert(!w600_motor_service_rearm(&s) && starts==1);
        if(fault==0) assert(s.homing.guard.error==MOTOR_GUARD_FOREGROUND_TIMEOUT);
        if(fault==1) assert(s.homing.guard.error==MOTOR_GUARD_ADC_TIMEOUT);
        if(fault==2) assert(s.error==W600_MOTOR_TIMING);
        if(fault==3) assert(s.error==W600_MOTOR_TIMER);
        if(fault==4) assert(s.error==W600_MOTOR_TIMEOUT);
    }
    for(unsigned dir=1;dir<=2;++dir) for(unsigned wrap=0;wrap<2;++wrap)
    for(unsigned delay=0;delay<4;++delay) {
        scenario=2;fresh(&s);
        epoch=wrap ? UINT32_MAX-200u : 0;tick_ms=epoch;
        assert(w600_motor_service_begin_bounded(&s,dir,2000));
        while(!(s.phase==W600_MOTOR_DRIVING && samples && s.on_active)) {
            advance(&s,240u);w600_motor_service_poll(&s);
        }
        advance_hardware((uint32_t)(armed_at+1999ull*48000u-cycles));
        if(delay<3) {
            clock_delay_cycles=48000u;clock_delay_skip=delay;
            assert(!w600_motor_service_finish(&s));run(&s);
            assert(s.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT && !s.requested_stop);
        } else {
            /* Isolate the exact pre-cap finish boundary from separate lease tests. */
            s.homing.guard.foreground_deadline=s.homing.guard.adc_deadline=s.started_ms+2000u;
            assert(w600_motor_service_finish(&s));run(&s);
            assert(s.phase==W600_MOTOR_DONE && s.requested_stop && !s.error);
        }
    }
    for(unsigned phase=W600_MOTOR_DRIVING;phase<=W600_MOTOR_RESTORING;++phase) {
        scenario=2;fresh(&s);assert(w600_motor_service_begin_bounded(&s,2,2000));
        while((unsigned)s.phase!=phase) {advance(&s,240u);w600_motor_service_poll(&s);}
        assert(w600_motor_service_finish(&s));run(&s);
        assert(s.phase==W600_MOTOR_DONE && s.requested_stop && !s.error);
    }
    puts("PASS finish cannot hide timer expiry before poll/tick");
}
static void pure_deadline_query(void)
{
    struct w600_motor_service s;
    for(unsigned wrap=0;wrap<2;++wrap) {
        scenario=0;fresh(&s);
        uint32_t start=wrap ? UINT32_MAX-50u : 0;
        struct motor_guard g=s.homing.guard;
        g.armed=true;g.move_deadline=start+2000u;
        g.foreground_deadline=start+100u;g.adc_deadline=start+80u;
        const uint32_t at[]={0,79,80,99,100,1999,2000};
        const enum motor_guard_error expected[]={MOTOR_GUARD_OK,MOTOR_GUARD_OK,
            MOTOR_GUARD_ADC_TIMEOUT,MOTOR_GUARD_ADC_TIMEOUT,MOTOR_GUARD_FOREGROUND_TIMEOUT,
            MOTOR_GUARD_FOREGROUND_TIMEOUT,MOTOR_GUARD_MOVE_TIMEOUT};
        for(unsigned i=0;i<sizeof at/sizeof *at;++i) {
            struct motor_guard saved=g;unsigned calls=stop_calls;
            assert(motor_guard_due(&g,start+at[i])==expected[i]);
            assert(!memcmp(&g,&saved,sizeof g) && stop_calls==calls);
        }
        g.armed=false;
        struct motor_guard saved=g;unsigned calls=stop_calls;
        assert(motor_guard_due(&g,start+2000u)==MOTOR_GUARD_OK);
        assert(!memcmp(&g,&saved,sizeof g) && stop_calls==calls);
    }
    puts("PASS pure deadline query: boundaries, wrap, precedence, no mutation/callback");
}
int main(void)
{
    assert(policy.motion_min==MOTOR_HOMING_MOTION_MIN_MV && policy.endpoint_max==100);
    pure_deadline_query();
    finish_after_expiry();
    bounded_commands();
    struct w600_motor_service s;
    scenario=0; fresh(&s);
    assert(!w600_motor_service_urgent(&s));
    s.phase=W600_MOTOR_DRIVING; s.owned=OWN_TIMER; s.next_sample=20;
    rcc.APB1EN|=RCC_APB1EN_TIM4EN;
    for(unsigned count=0;count<=21;++count) {
        timer.CNT=count;
        assert(w600_motor_service_urgent(&s)==(count>=17));
    }
    s.next_sample=1; timer.CNT=0;
    assert(w600_motor_service_urgent(&s));
    s.owned=0; rcc.APB1EN&=~RCC_APB1EN_TIM4EN;
    assert(w600_motor_service_urgent(&s));
    s.owned=OWN_TIMER;
    assert(w600_motor_service_urgent(&s));
    scenario=13; fresh(&s); assert(w600_motor_service_begin(&s,2)); run(&s);
    assert(edge_injected && s.phase==W600_MOTOR_DONE && !s.error && s.samples==21);
    for (unsigned dir=1;dir<=2;++dir) for (unsigned wrap=0;wrap<3;++wrap) {
        scenario=0; fresh(&s); epoch=wrap==1 ? UINT32_MAX-200 : wrap==2 ? 89478 : 0;
        tick_ms=epoch;
        assert(w600_motor_service_begin(&s,dir)); run(&s);
        assert(s.phase==W600_MOTOR_DONE && !s.error && s.samples==21 && starts==1);
        assert(s.homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED && s.stop_confirmed);
        assert(ui_polls>100 && finish_calls==1 && !s.owned && !s.board.owned);
        assert(!(rcc.APB1EN&(RCC_APB1EN_TIM4EN|RCC_APB1EN_PMUEN)));
        struct w600_motor_time time;
        assert(w600_motor_service_time(&s,&time));
        assert(time.on_upper<(uint32_t)(tick_ms-s.started_ms)*10u);
        assert(!s.on_active && s.time_valid);
    }
    for (scenario=2;scenario<=12;++scenario) {
        if (scenario==10) continue;
        fresh(&s); assert(w600_motor_service_begin(&s,2)); run(&s);
        assert(s.phase==W600_MOTOR_FAILED && s.error && starts==(scenario==6 ? 0u : 1u));
        if (scenario==9 || scenario==11) assert(s.cleanup_failed && (s.owned || s.board.owned));
        if (scenario==7) assert(clamps==1 && s.error==W600_MOTOR_TIMER);
        if (scenario==8) assert(s.samples==0 && s.error==W600_MOTOR_TIMING);
    }
    scenario=0; fresh(&s); assert(!w600_motor_service_begin(&s,3));
    w600_motor_service_poll(&s); assert(!starts && s.phase==W600_MOTOR_FAILED);
    fresh(&s); assert(w600_motor_service_begin(&s,1));
    w600_motor_service_poll(&s); assert(starts==1);
    advance(&s,2u*48000u); w600_motor_service_poll(&s); run(&s);
    assert(s.error==W600_MOTOR_TIMING);
    fresh(&s); assert(w600_motor_service_begin(&s,1)); w600_motor_service_poll(&s);
    advance(&s,100u*48000u); assert(s.phase==W600_MOTOR_CLEANUP && s.stop_confirmed);
    w600_motor_service_poll(&s); assert(s.phase==W600_MOTOR_FAILED);
    fresh(&s); assert(w600_motor_service_begin(&s,1)); w600_motor_service_poll(&s);
    assert(!w600_motor_service_begin(&s,1)); run(&s); assert(starts==1);
    fresh(&s); assert(w600_motor_service_begin(&s,1)); w600_motor_service_poll(&s);
    w600_motor_service_abort(&s); run(&s); assert(s.error==W600_MOTOR_ABORTED);
    fresh(&s); assert(w600_motor_service_begin(&s,1)); w600_motor_service_poll(&s);
    clock_valid=0; run(&s); assert(s.error==W600_MOTOR_TIMING);
    fresh(&s); assert(w600_motor_service_begin(&s,1)); w600_motor_service_poll(&s);
    /* Independent timer still returns both bridge outputs HIGH without CPU service. */
    cycles=armed_at+45000ull*48000; timer.CNT=0; timer.CR1&=~1u; timer.SR|=1u; pads();
    assert((b.IDR&0xc000u)==0xc000u);
    /* Explicit clean completion/rearm supports repeated commands, not retries. */
    fresh(&s); allowed_starts=3;
    epoch=UINT32_MAX-500u; tick_ms=epoch;
    assert(!w600_motor_service_rearm(&s));
    assert(!w600_motor_service_finish(&s));
    assert(w600_motor_service_begin(&s,2)); run(&s);
    assert(s.phase==W600_MOTOR_DONE && !s.requested_stop && starts==1);
    assert(!w600_motor_service_rearm(&s));
    advance(&s,100u*48000u); assert(!w600_motor_service_rearm(&s));
    advance(&s,48000u);
    for (unsigned fault=0;fault<6;++fault) {
        struct w600_motor_service saved=s;
        if (fault==0) mask=0;
        if (fault==1) rcc.APB2EN|=RCC_APB2EN_ADCEN;
        if (fault==2) rcc.APB1EN|=RCC_APB1EN_PMUEN;
        if (fault==3) s.board.owned=1;
        if (fault==4) s.cleanup_failed=1;
        if (fault==5) s.error=W600_MOTOR_ADC;
        struct w600_motor_service rejected=s;
        assert(!w600_motor_service_rearm(&s));
        assert(!memcmp(&s,&rejected,sizeof s) && starts==1);
        s=saved; mask=1;
        rcc.APB2EN&=~RCC_APB2EN_ADCEN; rcc.APB1EN&=~RCC_APB1EN_PMUEN;
    }
    assert(w600_motor_service_rearm(&s));
    assert(s.phase==W600_MOTOR_IDLE && !s.samples && !s.feedback.owns_adc && !s.requested_stop);
    samples=0;
    assert(w600_motor_service_begin(&s,1)); w600_motor_service_poll(&s);
    assert(starts==2 && !w600_motor_service_rearm(&s));
    advance(&s,100u);
    assert(w600_motor_service_finish(&s));
    assert(s.stop_confirmed && s.phase==W600_MOTOR_CLEANUP && s.requested_stop && !s.error);
    assert(!w600_motor_service_rearm(&s));
    run(&s);
    assert(s.phase==W600_MOTOR_DONE && !s.error && s.homing.state!=MOTOR_HOMING_ENDPOINT_OBSERVED);
    assert(!w600_motor_service_finish(&s));
    advance(&s,101u*48000u);
    assert(w600_motor_service_rearm(&s)); samples=0;
    assert(w600_motor_service_begin(&s,2)); run(&s);
    assert(starts==3 && s.phase==W600_MOTOR_DONE && s.samples==21 && !s.requested_stop);
    advance(&s,101u*48000u);
    scenario=10;
    assert(!w600_motor_service_rearm(&s));
    assert(s.error==W600_MOTOR_PINS && !s.stop_confirmed && starts==3);
    scenario=0;
    fresh(&s); assert(w600_motor_service_begin(&s,1));
    assert(w600_motor_service_finish(&s)); run(&s);
    assert(!starts && s.requested_stop && s.phase==W600_MOTOR_DONE);
    advance(&s,101u*48000u);
    assert(w600_motor_service_rearm(&s));
    fresh(&s); assert(w600_motor_service_begin(&s,1));
    w600_motor_service_abort(&s); run(&s);
    struct w600_motor_service failed=s;
    assert(!w600_motor_service_rearm(&s) && !memcmp(&s,&failed,sizeof s));
    puts("PASS motor service: both directions, endpoint, ms/cycle wrap, ADC/timing faults, leases, repeated/requested stops, cleanup/cutoff, commanded-on brackets versus independent pad-time integral, lower-count monotonic/time bounds");
}
