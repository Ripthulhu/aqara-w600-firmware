/* Also force-included as the MMIO declarations in separately compiled sources. */
#ifndef MOTOR_PROBE_TEST_MMIO
#define MOTOR_PROBE_TEST_MMIO
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"
extern RCC_t probe_rcc;
extern FLASH_t probe_flash;
extern GPIO_t probe_a,probe_b,probe_c;
extern PMU_t probe_pmu;
extern ADC_t probe_adc;
extern TIM_t probe_timer;
extern NVIC_Type probe_nvic;
extern uint32_t probe_primask,probe_ipsr,probe_control,probe_factory;
ADC_t *probe_adc_regs(void);
PMU_t *probe_pmu_regs(void);
TIM_t *probe_timer_regs(void);
void probe_barrier(void);
void probe_stop_write(volatile uint32_t *,uint32_t);
void probe_pulse_write(volatile uint32_t *,uint32_t);
void probe_board_observe(unsigned);
void probe_adc_clear(uint32_t);
void probe_adc_command(uint32_t);
uint32_t probe_adc_data(void);
#undef RCC
#undef FLASH
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef PMU
#undef ADC
#undef TIM4
#undef NVIC
#define RCC (&probe_rcc)
#define FLASH (&probe_flash)
#define GPIOA (&probe_a)
#define GPIOB (&probe_b)
#define GPIOC (&probe_c)
#define PMU probe_pmu_regs()
#define ADC probe_adc_regs()
#define TIM4 probe_timer_regs()
#define NVIC (&probe_nvic)
#define __get_PRIMASK() probe_primask
#define __get_IPSR() probe_ipsr
#define __get_CONTROL() probe_control
#define __DSB() probe_barrier()
#define W600_MOTOR_STOP_WRITE(reg,value) probe_stop_write(&(reg),(uint32_t)(value))
#define TIM4_PULSE_OBSERVE(reg,value) probe_pulse_write(reg,value)
#define FB_BOARD_OBSERVE(stage) probe_board_observe(stage)
#define FEEDBACK_FACTORY_BGR probe_factory
#define MOTOR_PROBE_FACTORY_WORD probe_factory
#define FEEDBACK_CLEAR_FLAGS(mask) probe_adc_clear(mask)
#define FEEDBACK_COMMAND(value) probe_adc_command(value)
#define FEEDBACK_READ_DATA() probe_adc_data()
#endif

#ifndef MOTOR_PROBE_TEST_HEADER_ONLY
#include "motor_probe.h"
RCC_t probe_rcc;
FLASH_t probe_flash;
GPIO_t probe_a,probe_b,probe_c;
PMU_t probe_pmu;
ADC_t probe_adc;
TIM_t probe_timer;
NVIC_Type probe_nvic;
uint32_t probe_primask,probe_ipsr,probe_control,probe_factory;
static struct motor_probe p;
static unsigned cases,adc_reads,adc_accesses,timer_accesses,pulse_starts,cleanup_resets;
static bool in_isr,stall_adc,stall_timer,ignore_remux,fail_cleanup_reset;

ADC_t *probe_adc_regs(void)
{ assert(!in_isr && (probe_rcc.APB2EN&RCC_APB2EN_ADCEN)); ++adc_accesses; return &probe_adc; }
PMU_t *probe_pmu_regs(void)
{ assert(!in_isr && (probe_rcc.APB1EN&RCC_APB1EN_PMUEN)); return &probe_pmu; }
TIM_t *probe_timer_regs(void)
{ assert(!in_isr && (probe_rcc.APB1EN&RCC_APB1EN_TIM4EN)); ++timer_accesses; return &probe_timer; }
static void pads(void)
{
    probe_a.IDR=(probe_a.IDR&~0x8000u)|(probe_a.ODR&0x8000u);
    probe_c.IDR=(probe_c.IDR&~0x400u)|(probe_c.ODR&0x400u);
    uint32_t value=probe_b.ODR&0xc000u;
    if((probe_b.MODE&0xf0000000u)==0x90000000u && (probe_timer.CCEN&16u) &&
       probe_timer.CNT>=100u) value&=~0x8000u;
    probe_b.IDR=(probe_b.IDR&~0xc000u)|value;
}
void probe_barrier(void) { pads(); }
void probe_stop_write(volatile uint32_t *reg,uint32_t value)
{
    assert(reg==&probe_rcc.IOPEN || (probe_rcc.IOPEN&7u)==7u);
    if(reg==&probe_c.BR) { assert(value==0x400u); probe_c.ODR&=~value; }
    else if(reg==&probe_a.BR) { assert(value==0x8000u); probe_a.ODR&=~value; }
    else if(reg==&probe_b.BSR) {
        assert(!(probe_a.IDR&0x8000u) && !(probe_c.IDR&0x400u)); probe_b.ODR|=value;
    } else if(reg==&probe_b.MODE && ignore_remux && (value&0xf0000000u)==0x50000000u) {
        /* An ignored AF->GPIO remux must leave the one-shot fallback running. */
    } else *reg=value;
    pads();
}
void probe_pulse_write(volatile uint32_t *reg,uint32_t value)
{
    assert(!in_isr);
    if(reg==&probe_timer.EVTG) { probe_timer.CNT=0; probe_timer.SR|=1u; }
    if(reg==&probe_timer.CR1 && value==0x89u && probe_timer.ARR==1999) ++pulse_starts;
    if(reg==&probe_timer.CR1 && !(value&1u))
        assert((probe_b.MODE&0xf0000000u)==0x50000000u && (probe_b.IDR&0xc000u)==0xc000u);
    pads();
}
void probe_board_observe(unsigned stage)
{
    assert(!in_isr);
    if(stage==10 && fail_cleanup_reset) probe_rcc.APB2RST&=~RCC_APB2RST_ADC_RST;
    if((stage==4 || stage==10) && (probe_rcc.APB2RST&RCC_APB2RST_ADC_RST)) {
        assert(!(probe_rcc.APB1EN&RCC_APB1EN_TIM4EN));
        assert((probe_b.MODE&0xf0000000u)==0x50000000u);
        memset(&probe_adc,0,sizeof probe_adc);
        if(stage==10) ++cleanup_resets;
    }
}
void probe_adc_clear(uint32_t mask)
{ assert(!in_isr); probe_adc.ISR&=~mask; }
void probe_adc_command(uint32_t command)
{
    assert(!in_isr);
    if(command==ADC_CR_ADEN) probe_adc.CR=ADC_CR_ADEN;
    else if(command==ADC_CR_CALEN) {
        probe_adc.CR=ADC_CR_ADEN; probe_adc.ISR=ADC_ISR_EOCAL;
    } else if(command==ADC_CR_START) {
        assert((probe_b.MODE&0xf0000000u)==0x50000000u && (probe_b.IDR&0xc000u)==0xc000u);
        assert(!(probe_a.IDR&0x8000u) && !(probe_c.IDR&0x400u));
        probe_adc.CR=ADC_CR_ADEN|ADC_CR_START;
        if(!stall_adc) {
            probe_adc.CR=ADC_CR_ADEN; probe_adc.ISR=ADC_ISR_EOC|ADC_ISR_EOS;
            probe_adc.DR=probe_adc.CHCFG==ADC_CHCFG_CHN13 ? 1495u : 0u;
        }
    } else {
        assert(command==ADC_CR_STOP && !(probe_rcc.APB1EN&RCC_APB1EN_TIM4EN));
        assert((probe_b.IDR&0xc000u)==0xc000u);
        probe_adc.CR=ADC_CR_ADEN;
    }
}
uint32_t probe_adc_data(void)
{
    assert(!in_isr && (probe_adc.ISR&ADC_ISR_EOC)); ++adc_reads;
    probe_adc.ISR&=~ADC_ISR_EOC; return probe_adc.DR;
}
static void setup(void)
{
    memset(&probe_rcc,0,sizeof probe_rcc); memset(&probe_flash,0,sizeof probe_flash);
    memset(&probe_a,0,sizeof probe_a); memset(&probe_b,0,sizeof probe_b); memset(&probe_c,0,sizeof probe_c);
    memset(&probe_adc,0,sizeof probe_adc); memset(&probe_pmu,0,sizeof probe_pmu);
    memset(&probe_timer,0,sizeof probe_timer); memset(&probe_nvic,0,sizeof probe_nvic);
    memset(&p,0xa5,sizeof p);
    probe_rcc.CFG=0x12; probe_rcc.PLLCFG=0x30260;
    probe_rcc.CSR1=RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY;
    probe_rcc.AHBEN=0x100; probe_rcc.IOPEN=15; probe_rcc.APB1EN=RCC_APB1EN_UART3EN;
    probe_flash.ACR=2; probe_pmu.SR=PMU_SR_BGR_READY;
    probe_nvic.ISER[0]=1u<<UART3_4_IRQn;
    probe_primask=1; probe_ipsr=probe_control=0; probe_factory=0xfa2805d7u;
    adc_reads=adc_accesses=timer_accesses=pulse_starts=cleanup_resets=0;
    in_isr=stall_adc=stall_timer=ignore_remux=fail_cleanup_reset=false;
    assert(motor_probe_init(&p)); assert(p.report.state==MOTOR_PROBE_SERVICE_IDLE);
}
static void advance_timer(void)
{
    if(!stall_timer && (probe_rcc.APB1EN&RCC_APB1EN_TIM4EN) && (probe_timer.CR1&1u)) {
        uint32_t next=probe_timer.CNT+100u;
        if(next>probe_timer.ARR) {
            probe_timer.CNT=0; probe_timer.CR1&=~1u; probe_timer.SR=0x1fu;
        } else { probe_timer.CNT=next; probe_timer.SR|=0x1eu; }
    }
    pads();
}
static void tick(uint32_t now)
{
    advance_timer();
    TIM_t before_timer=probe_timer; ADC_t before_adc=probe_adc;
    uint32_t apb1=probe_rcc.APB1EN,apb2=probe_rcc.APB2EN,cfg=probe_rcc.CFG;
    unsigned before_adc_access=adc_accesses,before_timer_access=timer_accesses;
    in_isr=true; probe_ipsr=15;
    motor_probe_tick(&p,now);
    probe_ipsr=0; in_isr=false;
    assert(!memcmp(&probe_timer,&before_timer,sizeof before_timer));
    assert(!memcmp(&probe_adc,&before_adc,sizeof before_adc));
    assert(adc_accesses==before_adc_access && timer_accesses==before_timer_access);
    assert(probe_rcc.APB1EN==apb1 && probe_rcc.APB2EN==apb2 && probe_rcc.CFG==cfg);
}
static bool terminal(void)
{
    return p.report.state==MOTOR_PROBE_SERVICE_COMPLETE_RAW ||
        (p.report.state==MOTOR_PROBE_SERVICE_FAILED && p.cleanup_attempted);
}
static uint32_t run(uint32_t start,unsigned count)
{
    assert(motor_probe_begin(&p,start,count));
    for(uint32_t elapsed=0;elapsed<150;++elapsed) {
        uint32_t now=start+elapsed;
        if(elapsed) tick(now);
        for(unsigned poll=0;poll<32;++poll) {
            motor_probe_foreground(&p,now);
            if(terminal()) return elapsed;
        }
    }
    assert(0 && "service did not terminate"); return 0;
}
static void complete(unsigned count,uint32_t start)
{
    setup(); uint32_t elapsed=run(start,count);
    assert(elapsed<100 && p.report.state==MOTOR_PROBE_SERVICE_COMPLETE_RAW && !p.report.error);
    assert(p.report.samples==count+1 && p.report.completed_pulses==count);
    assert(p.report.nominal_low_ticks_100khz==1900*count && pulse_starts==count);
    assert(p.report.cleanup_abort_expected && !p.report.pulse_error);
    assert(!p.report.cleanup_pending && !p.board.owned && !p.pulse.owned && !p.adc_started);
    assert(probe_rcc.CFG==0x12 && probe_rcc.APB1EN==RCC_APB1EN_UART3EN && !probe_rcc.APB2EN);
    assert(probe_nvic.ISER[0]==(1u<<UART3_4_IRQn) && cleanup_resets==1);
    assert(adc_reads==6*(count+1));
    for(unsigned i=0;i<count;++i) {
        assert(p.report.pulse[i].status==TIM4_PULSE_DONE && p.report.pulse[i].direction==1);
        assert(p.report.done_at_ms[i]-p.report.off_at_ms[i]>=1);
    }
    struct motor_probe_snapshot snapshot;
    motor_probe_snapshot(&p,&snapshot); assert(!memcmp(&snapshot,&p.report,sizeof snapshot));
    assert(!motor_probe_begin(&p,start+elapsed+1,count)); /* Single begin for this object. */
    assert(p.report.error==MOTOR_PROBE_SERVICE_STATE); ++cases;
}
static uint32_t start_low(void)
{
    assert(motor_probe_begin(&p,0,2));
    for(uint32_t now=0;now<30;++now) {
        if(now) tick(now);
        for(unsigned i=0;i<32;++i) motor_probe_foreground(&p,now);
        if(p.pulse_active && (probe_b.IDR&0xc000u)!=0xc000u) return now;
    }
    assert(0); return 0;
}
int main(void)
{
    complete(1,0); complete(2,0); complete(2,UINT32_MAX-10);
    setup(); uint32_t now=start_low(); unsigned access=adc_accesses;
    stall_timer=true; /* Preserve AF LOW until independent ISR GPIO stop. */
    for(unsigned i=1;i<=20;++i) tick(now+i); /* No foreground progress. */
    assert(p.report.error==MOTOR_PROBE_SERVICE_SEQUENCE && p.report.sequence_error==MOTOR_PROBE_FOREGROUND_TIMEOUT);
    assert((probe_b.MODE&0xf0000000u)==0x50000000u && (probe_b.IDR&0xc000u)==0xc000u);
    assert(adc_accesses==access && p.board.owned); motor_probe_foreground(&p,now+20);
    assert(!p.report.cleanup_pending && p.report.state==MOTOR_PROBE_SERVICE_FAILED); ++cases;
    setup(); stall_adc=true; (void)run(0,1);
    assert(p.report.error==MOTOR_PROBE_SERVICE_ADC && p.report.sampler_error==W600_FEEDBACK_BURST_TIMEOUT);
    assert(!p.report.cleanup_pending && !pulse_starts); ++cases;
    setup(); assert(motor_probe_begin(&p,0,2));
    for(uint32_t t=0;t<100 && !terminal();++t) {
        if(t) tick(t);
        for(unsigned i=0;i<32 && !terminal();++i) {
            if(p.sequence.completed_pulses==1) stall_adc=true;
            motor_probe_foreground(&p,t);
        }
    }
    assert(p.report.error==MOTOR_PROBE_SERVICE_ADC && p.report.samples==1 &&
        p.report.completed_pulses==1 && p.report.nominal_low_ticks_100khz==1900);
    assert(!p.report.cleanup_pending && pulse_starts==1); ++cases;
    setup(); stall_timer=true; (void)run(0,1);
    assert(p.report.error && p.report.state==MOTOR_PROBE_SERVICE_FAILED && !p.report.completed_pulses);
    assert(!p.report.cleanup_pending); ++cases;
    setup(); now=start_low(); ignore_remux=true;
    for(unsigned i=1;i<=20;++i) tick(now+i);
    uint32_t timer_cr=probe_timer.CR1,adc_cr=probe_adc.CR;
    motor_probe_foreground(&p,now+20);
    assert(p.report.error && p.report.cleanup_pending && p.pulse.owned && p.board.owned);
    assert(probe_timer.CR1==timer_cr && probe_adc.CR==adc_cr);
    bool attempted=p.cleanup_attempted;
    ignore_remux=false; motor_probe_foreground(&p,now+21);
    assert(p.cleanup_attempted==attempted && p.report.cleanup_pending); ++cases;
    setup(); now=start_low(); ignore_remux=true;
    timer_cr=probe_timer.CR1; adc_cr=probe_adc.CR;
    assert(timer_cr&1u); motor_probe_abort(&p,now);
    assert(p.report.error==MOTOR_PROBE_SERVICE_GPIO && p.report.cleanup_pending);
    assert(probe_timer.CR1==timer_cr && probe_adc.CR==adc_cr && p.pulse.owned && p.board.owned);
    assert((probe_b.MODE&0xf0000000u)==0x90000000u); ++cases;
    setup(); fail_cleanup_reset=true; (void)run(0,1);
    assert(p.report.error==MOTOR_PROBE_SERVICE_CLEANUP_ERROR && p.report.cleanup_pending && p.board.owned);
    assert(!p.pulse.owned && !(probe_rcc.APB1EN&RCC_APB1EN_TIM4EN)); ++cases;
    setup(); now=start_low(); motor_probe_abort(&p,now);
    assert(p.report.error==MOTOR_PROBE_SERVICE_ABORTED && !p.report.cleanup_pending);
    assert(!p.report.completed_pulses && !p.report.nominal_low_ticks_100khz); ++cases;
    setup(); assert(motor_probe_begin(&p,0,1));
    motor_probe_abort(&p,0); assert(!p.report.cleanup_pending && !pulse_starts); ++cases;
    setup(); now=start_low(); uint32_t owned=p.pulse.owned;
    assert(!motor_probe_init(&p)); assert(p.pulse.owned==owned && p.report.error);
    motor_probe_foreground(&p,now); assert(!p.report.cleanup_pending); ++cases;
    setup(); probe_nvic.ISER[0]|=1u; assert(!motor_probe_begin(&p,0,1));
    assert(p.report.error==MOTOR_PROBE_SERVICE_CONTEXT && !p.board.owned && !pulse_starts); ++cases;
    setup(); probe_factory=0; assert(!motor_probe_begin(&p,0,1));
    motor_probe_foreground(&p,0);
    assert(p.report.error==MOTOR_PROBE_SERVICE_ADC && !p.report.cleanup_pending && !adc_reads); ++cases;
    setup(); memset(&p,0xa5,sizeof p); unsigned old_access=adc_accesses;
    tick(1); assert(adc_accesses==old_access);
    assert(motor_probe_init(&p)); ++cases;
    printf("PASS %u actual-source integrated motor-probe cases; motor UNPLUGGED scope only\n",cases);
}
#endif
