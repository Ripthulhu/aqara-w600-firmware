#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"
static RCC_t rcc;
static FLASH_t flash;
static DBG_t debug_regs;
static GPIO_t a,b,c;
static TIM_t timer;
static NVIC_Type nvic;
static uint32_t primask,ipsr,control;
static unsigned cases,writes,fail_write,remux_fail,stop_writes;
static unsigned read_edge;
static unsigned fail_gate_off, stuck_selector;
static volatile uint32_t *stop_ignore;
static uint32_t *write_log[256];
static void model_write(volatile uint32_t *reg,uint32_t value);
static uint32_t model_read(volatile uint32_t *reg);
static void stop_model_write(volatile uint32_t *reg,uint32_t value);
static TIM_t *timer_regs(void) { assert(rcc.APB1EN & 4); return &timer; }
#undef RCC
#undef FLASH
#undef DBG
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef TIM4
#undef NVIC
#define RCC (&rcc)
#define FLASH (&flash)
#define DBG (&debug_regs)
#define GPIOA (&a)
#define GPIOB (&b)
#define GPIOC (&c)
#define TIM4 timer_regs()
#define NVIC (&nvic)
#define __get_PRIMASK() primask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#define __DSB() ((void)0)
#define TIM4_PULSE_OBSERVE(reg,value) model_write(reg,value)
#define TIM4_PULSE_READ(reg) model_read(&(reg))
#define W600_MOTOR_STOP_READ(reg) model_read(&(reg))
#define W600_MOTOR_STOP_WRITE(reg,value) stop_model_write(&(reg),(uint32_t)(value))
#include "../startup/w600_motor_stop.c"
#include "tim4_motor_pulse.c"
static struct tim4_motor_pulse s;
static RCC_t before_rcc;
static GPIO_t before_a,before_b,before_c;
static FLASH_t before_flash;
static DBG_t before_debug;
static void pad(volatile uint32_t *reg,uint32_t v) { *reg=v; }
static void model_pads(void)
{
    uint32_t state=b.ODR & HIGH;
    unsigned mode=(b.MODE >> 28)&15;
    if (mode==6 || mode==9) {
        unsigned channel=mode==6?1u:2u;
        if ((timer.CCEN & (channel==1?1u:16u)) && timer.CNT >= 100u)
            state=channel==1?0x8000u:0x4000u;
    }
    pad(&b.IDR,(b.IDR&~HIGH)|state);
    pad(&a.IDR,(a.IDR&~0x8000u)|(a.ODR&0x8000u));
    pad(&c.IDR,(c.IDR&~0x400u)|(c.ODR&0x400u));
    if(stuck_selector) pad(&c.IDR,c.IDR|0x400u);
}
static uint32_t model_read(volatile uint32_t *reg)
{
    uint32_t value=*reg;
    if(reg==&timer.CNT && read_edge) {
        unsigned edge=read_edge;read_edge=0;
        if(edge==1) timer.CNT=100;
        else { timer.CNT=0;timer.CR1=0x88;timer.SR=0x1f; }
        model_pads();
    }
    return value;
}
static void model_write(volatile uint32_t *reg,uint32_t value)
{
    ++writes;
    if(writes<256) write_log[writes]=(uint32_t *)(uintptr_t)reg;
    if(reg==&b.BSR) b.ODR|=value;
    if(reg==&a.BR) a.ODR&=~value;
    if(reg==&c.BR) c.ODR&=~value;
    if(reg==&timer.EVTG) { timer.CNT=0;timer.SR|=1; }
    if(reg==&timer.CR1 && !(value&1)) {
        ++stop_writes;
        /* Every stop/disable must occur only after actual GPIO HH remux. */
        assert((b.MODE & MODES)==GPIO_HH && (b.IDR&HIGH)==HIGH);
    }
    if(remux_fail && reg==&b.MODE && (value&MODES)==GPIO_HH)
        b.MODE=(value&~MODES)|active_mode(&s);
    if(fail_gate_off && reg==&rcc.APB1EN && !(value&4u)) rcc.APB1EN|=4u;
    if(writes==fail_write) {
        if(reg==&rcc.APB1EN) rcc.APB1EN ^= 4;
        else if(reg==&b.MODE) b.MODE ^= 0x10000000;
        else if(reg==&b.AFH) b.AFH ^= af_mask(&s);
        else if(reg==&timer.CR1) timer.CR1 ^= 1;
        else *reg=value^1u;
    }
    model_pads();
}
static void stop_model_write(volatile uint32_t *reg,uint32_t value)
{
    if(reg==stop_ignore) return;
    *reg=value;model_write(reg,value);
}
static void snapshot(void)
{
    before_rcc=rcc;before_a=a;before_b=b;before_c=c;
    before_flash=flash;before_debug=debug_regs;
}
static void unchanged_scoped(void)
{
    assert(rcc.CFG==before_rcc.CFG && rcc.PLLCFG==before_rcc.PLLCFG);
    assert(rcc.APB2EN==before_rcc.APB2EN && rcc.APB2RST==before_rcc.APB2RST);
    assert(rcc.AHBEN==before_rcc.AHBEN && rcc.IOPEN==before_rcc.IOPEN);
    assert((rcc.APB1EN&~4u)==before_rcc.APB1EN);
    assert((b.MODE&~MODES)==(before_b.MODE&~MODES));
    assert(b.OTYPE==before_b.OTYPE && b.PUPD==before_b.PUPD);
    assert((b.AFH&~af_mask(&s))==(before_b.AFH&~af_mask(&s)));
    assert((b.ODR&~HIGH)==(before_b.ODR&~HIGH));
    assert(a.MODE==before_a.MODE && a.OTYPE==before_a.OTYPE && a.PUPD==before_a.PUPD);
    assert(c.MODE==before_c.MODE && c.OTYPE==before_c.OTYPE && c.PUPD==before_c.PUPD);
    assert(a.ODR==before_a.ODR && c.ODR==before_c.ODR);
    assert(!memcmp(&flash,&before_flash,sizeof flash));
    assert(!memcmp(&debug_regs,&before_debug,sizeof debug_regs));
}
static void setup(unsigned adc,unsigned debug)
{
    memset(&rcc,0,sizeof rcc);memset(&flash,0,sizeof flash);
    memset(&debug_regs,0,sizeof debug_regs);memset(&timer,0xa5,sizeof timer);
    memset(&nvic,0,sizeof nvic);
    memset(&s,0,sizeof s);memset(&a,0x55,sizeof a);memset(&b,0x55,sizeof b);
    memset(&c,0x55,sizeof c);memset(write_log,0,sizeof write_log);
    rcc.CFG=adc?0x40012:0x12;rcc.PLLCFG=0x30260;
    rcc.CSR1=RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY;
    rcc.APB1EN=RCC_APB1EN_UART3EN|(adc?RCC_APB1EN_PMUEN:0);
#ifdef W600_DISPLAY_BENCH
    rcc.APB1EN|=RCC_APB1EN_UART2EN;
#endif
    rcc.APB2EN=(debug?RCC_APB2EN_DBGEN:0)|(adc?RCC_APB2EN_ADCEN:0);
    rcc.AHBEN=0x100;rcc.IOPEN=15;flash.ACR=2;
    b.MODE=(b.MODE&~MODES)|GPIO_HH;b.OTYPE&=~HIGH;b.ODR|=HIGH;
    b.PUPD&=~MODES;
    if(adc) { b.MODE|=3u<<26;b.PUPD&=~(3u<<26); }
    a.MODE=(a.MODE&~0xc0000000u)|0x40000000u;a.OTYPE&=~0x8000u;a.ODR&=~0x8000u;
    c.MODE=(c.MODE&~0x00300000u)|0x00100000u;c.OTYPE&=~0x400u;c.ODR&=~0x400u;
    a.PUPD&=~0xc0000000u;c.PUPD&=~0x00300000u;
    primask=1;ipsr=control=0;writes=fail_write=remux_fail=stop_writes=read_edge=0;
    fail_gate_off=stuck_selector=0;stop_ignore=NULL;
    model_pads();snapshot();
}
static void low_sample(void)
{
    timer.CNT=200;timer.SR=0x1e;model_pads();
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_DRIVING && s.seen_low);
    unchanged_scoped();
}
static void terminal(void)
{
    timer.CNT=0;timer.CR1=0x88;timer.SR=0x1f;model_pads();
}
static void settled(void)
{
    terminal();assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_DONE);
    assert(!s.owned && !(rcc.APB1EN&4));
    assert((b.MODE&MODES)==GPIO_HH && b.AFH==before_b.AFH);
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_DONE);
    unchanged_scoped();++cases;
}
int main(void)
{
    for(unsigned adc=0;adc<2;++adc) for(unsigned dbg=0;dbg<1;++dbg)
    for(unsigned dir=1;dir<=2;++dir) {
        setup(adc,dbg);assert(tim4_motor_pulse_arm(&s,dir)==TIM4_PULSE_DRIVING);
        assert(timer.ARR==1999 && timer.PSC==479 && timer.CNT==0);
        assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_DRIVING);low_sample();
        terminal();assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING);
        assert(timer.ARR==99 && timer.CCEN==0 && timer.CNT==0 && timer.SR==0);
        timer.CNT=50;assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING);
        assert(s.settle_count==50);settled();assert(tim4_motor_pulse_reset(&s));
        assert(s.status==TIM4_PULSE_IDLE && !s.error);
    }
    for(unsigned bad=0;bad<21;++bad) {
        setup(1,0);
        switch(bad) {
        case 0:rcc.CFG|=0x100;break;case 1:rcc.PLLCFG^=1;break;
        case 2:rcc.CSR1&=~RCC_CSR1_PLLRDY;break;
        case 3:rcc.AHBEN|=RCC_AHBEN_DMAEN;break;
        case 4:rcc.APB1EN|=RCC_APB1EN_TIM4EN;break;
        case 5:rcc.APB1EN|=RCC_APB1EN_TIM3EN;break;
        case 6:rcc.APB2EN|=RCC_APB2EN_SPI1EN;break;
        case 7:rcc.APB1RST|=4;break;case 8:rcc.APB2EN|=RCC_APB2EN_DBGEN;break;
        case 9:rcc.IOPEN&=~1u;break;case 10:a.ODR|=0x8000;model_pads();break;
        case 11:c.OTYPE|=0x400;break;case 12:b.OTYPE|=HIGH;break;
        case 13:primask=0;break;case 14:ipsr=15;break;case 15:control=1;break;
        case 16:flash.ACR=0;break;case 17:b.MODE&=~MODES;break;
        case 18:nvic.ISER[0]=1u<<TIM4_IRQn;break;
        case 19:nvic.ISPR[0]=1u<<TIM4_IRQn;break;
        default:rcc.APB1EN^=RCC_APB1EN_UART2EN;break;
        }
        assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_FAILED && s.error);
        assert(!s.owned);++cases;
    }
#ifdef W600_DISPLAY_BENCH
    setup(1,0);rcc.APB1EN&=~RCC_APB1EN_UART3EN;
    assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_FAILED && !s.owned);++cases;
#else
    setup(1,0);rcc.APB1EN&=~RCC_APB1EN_UART3EN;snapshot();
    assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);
    assert(tim4_motor_pulse_abort(&s)==TIM4_PULSE_FAILED && !s.owned);
    unchanged_scoped();++cases;
#endif
    for(unsigned phase=0;phase<2;++phase) {
        setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);low_sample();
        if(phase) { terminal();assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING); }
        rcc.APB1EN^=RCC_APB1EN_UART2EN;
        before_rcc.APB1EN^=RCC_APB1EN_UART2EN;
        assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_FAILED &&
               s.error==TIM4_PULSE_CONTEXT && !s.owned);
        unchanged_scoped();++cases;
    }
    for(unsigned dir=0;dir<4;dir+=3) {
        setup(0,0);assert(tim4_motor_pulse_arm(&s,dir)==TIM4_PULSE_FAILED);
        assert(s.error==TIM4_PULSE_DIRECTION);++cases;
    }
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);
    assert(!tim4_motor_pulse_reset(&s));low_sample();
    assert(tim4_motor_pulse_arm(&s,2)==TIM4_PULSE_FAILED);
    assert(s.error==TIM4_PULSE_STATE && !s.owned);++cases;
    for(unsigned phase=0;phase<5;++phase) {
        setup(1,0);
        if(phase) assert(tim4_motor_pulse_arm(&s,2)==TIM4_PULSE_DRIVING);
        if(phase>=2) low_sample();
        if(phase==3) terminal();
        if(phase==4) { terminal();assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING); }
        assert(tim4_motor_pulse_abort(&s)==TIM4_PULSE_FAILED && s.error==TIM4_PULSE_ABORTED);
        assert(!s.owned && pins(GPIO_HH,HIGH));unchanged_scoped();++cases;
    }
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);
    terminal();assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_FAILED && !s.seen_low);++cases;
    for(unsigned phase=0;phase<2;++phase) for(unsigned failure=0;failure<5;++failure) {
        setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);low_sample();
        if(phase) { terminal();assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING); }
        switch(failure) {
        case 0:timer.CR1=0x88;timer.SR=0;break;
        case 1:s.polls=LIMIT;break;case 2:timer.DIER=1;break;
        case 3:timer.CNT=phase?100:2000;break;
        default:timer.SR=1;break;
        }
        assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_FAILED && s.error && !s.owned);
        assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_FAILED);++cases;
    }
    /* Model ignored writes at every successful arm step and terminal/cleanup. */
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);
    unsigned arm_writes=writes;
    for(unsigned nth=1;nth<=arm_writes;++nth) {
        setup(1,0);fail_write=nth;
        enum tim4_motor_pulse_status st=tim4_motor_pulse_arm(&s,1);
        if(st==TIM4_PULSE_DRIVING) { /* An unused event write may not alter readback. */
            tim4_motor_pulse_abort(&s);
        } else assert(st==TIM4_PULSE_FAILED && s.error);
        ++cases;
    }
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);low_sample();
    remux_fail=1;unsigned stopped=stop_writes;
    assert(tim4_motor_pulse_abort(&s)==TIM4_PULSE_FAILED && s.owned);
    assert(stop_writes==stopped && (timer.CR1&1));
    remux_fail=0;assert(tim4_motor_pulse_abort(&s)==TIM4_PULSE_FAILED && !s.owned);
    assert(s.error==TIM4_PULSE_ABORTED);++cases;
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);low_sample();
    terminal();assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING);
    terminal();fail_gate_off=1;
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_FAILED && s.cleanup_error);
    fail_gate_off=0;assert(tim4_motor_pulse_abort(&s)==TIM4_PULSE_FAILED && !s.owned);++cases;
    for(unsigned failure=0;failure<3;++failure) {
        setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);low_sample();
        if(failure==0) { c.ODR|=0x400u;stop_ignore=&c.BR;model_pads(); }
        if(failure==1) { stuck_selector=1;model_pads(); }
        if(failure==2) { c.MODE^=0x00300000u;stop_ignore=&c.MODE; }
        unsigned stops=stop_writes;
        assert(tim4_motor_pulse_abort(&s)==TIM4_PULSE_FAILED);
        assert(s.error==TIM4_PULSE_ABORTED && s.cleanup_error && s.owned);
        assert(stop_writes==stops && (timer.CR1&1u) && (rcc.APB1EN&4u));
        assert((b.MODE&MODES)==active_mode(&s));
        stop_ignore=NULL;stuck_selector=0;
        assert(tim4_motor_pulse_abort(&s)==TIM4_PULSE_FAILED && !s.owned);
        assert(s.error==TIM4_PULSE_ABORTED && s.cleanup_error);++cases;
    }
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);
    rcc.APB1EN&=~4u;
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_FAILED && s.owned);
    assert(pins(GPIO_HH,HIGH));++cases;
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);
    timer.CNT=99;model_pads();read_edge=1;
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_DRIVING && !s.error && !s.seen_low);
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_DRIVING && s.seen_low);++cases;
    read_edge=2;
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_DRIVING && !s.error);
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING);++cases;
    timer.CNT=98;model_pads();read_edge=2;
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_SETTLING && !s.error);
    settled();
    setup(1,0);assert(tim4_motor_pulse_arm(&s,1)==TIM4_PULSE_DRIVING);low_sample();
    timer.SR=0x40;
    assert(tim4_motor_pulse_poll(&s)==TIM4_PULSE_FAILED && s.error==TIM4_PULSE_TIMER);++cases;
    printf("PASS %u actual-source TIM4 pulse cases; no hardware validation\n",cases);
}
