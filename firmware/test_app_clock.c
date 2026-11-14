#include <assert.h>
#include <stdio.h>
#include "ciu32l071.h"
static SysTick_Type systick;
static SCB_Type scb;
static NVIC_Type nvic;
static unsigned mask, ipsr, control, reads, wrap_at;
static unsigned barriers, waits;
static unsigned resume_writes, reload_stuck;
static void idle_barrier(void) { ++barriers; }
static void idle_wait(void)
{
    assert(mask==1 && !ipsr && !control && !scb.SCR && barriers==1);
    ++waits;
    scb.ICSR=SCB_ICSR_PENDSTSET_Msk;
}
static uint32_t read_clock(volatile const uint32_t *reg)
{
    ++reads;
    if (reads==wrap_at) { scb.ICSR=SCB_ICSR_PENDSTSET_Msk; systick.VAL=47990; }
    return *reg;
}
static void write_clock(volatile uint32_t *reg,uint32_t value)
{
    ++resume_writes;
    if (reg==&scb.ICSR) {
        assert(value==SCB_ICSR_PENDSTCLR_Msk);
        scb.ICSR&=~SCB_ICSR_PENDSTSET_Msk;
    } else if (reg==&systick.VAL) { assert(value==0); systick.VAL=0; }
    else {
        assert(reg==&systick.CTRL && value==7);
        systick.CTRL=value;
        if (!reload_stuck) systick.VAL=47999;
    }
}
#undef SysTick
#undef SCB
#undef NVIC
#define SysTick (&systick)
#define SCB (&scb)
#define NVIC (&nvic)
#define __get_PRIMASK() mask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#define APP_CLOCK_READ(reg) read_clock(&(reg))
#define APP_CLOCK_WRITE(reg,value) write_clock(&(reg),(value))
#undef __DSB
#undef __WFI
#undef __ISB
#define __DSB() idle_barrier()
#define __WFI() idle_wait()
#define __ISB() idle_barrier()
#include "app_clock.c"
int main(void)
{
    volatile uint32_t ms=100;
    struct w600_app_time out={0};
    mask=1; systick.CTRL=7; systick.LOAD=47999;
    for (uint32_t value=0;value<48000;++value) {
        systick.VAL=value; scb.ICSR=0; reads=0;
        assert(w600_app_clock(&ms,&out));
        assert(out.ms==100 && out.cycles==100u*48000+47999-value);
        scb.ICSR=SCB_ICSR_PENDSTSET_Msk;
        assert(w600_app_clock(&ms,&out));
        assert(out.ms==101 && out.cycles==101u*48000+47999-value);
    }
    /* Wrap while reading pending/counter: don't pair a new ms with an old VAL. */
    for (wrap_at=3;wrap_at<=5;++wrap_at) {
        scb.ICSR=0; systick.VAL=2; reads=0;
        assert(w600_app_clock(&ms,&out));
        assert(out.ms==101 && out.cycles==101u*48000+9);
    }
    wrap_at=0; scb.ICSR=SCB_ICSR_PENDSTSET_Msk; ms=UINT32_MAX; systick.VAL=47999;
    assert(w600_app_clock(&ms,&out) && out.ms==0 && out.cycles==0);
    scb.ICSR=0; ms=89478; systick.VAL=47999;
    assert(w600_app_clock(&ms,&out)); uint32_t before=out.cycles;
    ++ms; assert(w600_app_clock(&ms,&out) && out.cycles-before==48000);
    mask=0; assert(!w600_app_clock(&ms,&out)); mask=1;
    ipsr=15; assert(!w600_app_clock(&ms,&out)); ipsr=0;
    control=1; assert(!w600_app_clock(&ms,&out)); control=0;
    systick.CTRL=5; assert(!w600_app_clock(&ms,&out)); systick.CTRL=7;
    systick.LOAD=48000; assert(!w600_app_clock(&ms,&out)); systick.LOAD=47999;
    systick.VAL=48000; assert(!w600_app_clock(&ms,&out));
    puts("PASS app clock: all48000counter values, pending wraps, ms/cycle rollover, invalid context/profile");

    systick.VAL=47999; scb.ICSR=0; nvic.ISER[0]=1u<<27;
    assert(w600_app_idle() && waits==1 && barriers==2 && mask==1);
    assert(scb.ICSR==SCB_ICSR_PENDSTSET_Msk);
    assert(!w600_app_idle()); scb.ICSR=0;
    scb.ICSR=SCB_ICSR_PENDSVSET_Msk; assert(!w600_app_idle()); scb.ICSR=0;
    scb.ICSR=SCB_ICSR_NMIPENDSET_Msk; assert(!w600_app_idle()); scb.ICSR=0;
    mask=0; assert(!w600_app_idle()); mask=1;
    ipsr=15; assert(!w600_app_idle()); ipsr=0;
    control=1; assert(!w600_app_idle()); control=0;
    for (unsigned bit=0;bit<32;++bit) {
        scb.SCR=1u<<bit; assert(!w600_app_idle()); scb.SCR=0;
        nvic.ISPR[0]=1u<<bit; assert(!w600_app_idle()); nvic.ISPR[0]=0;
        nvic.ISER[0]=(1u<<27)^(1u<<bit); assert(!w600_app_idle()); nvic.ISER[0]=1u<<27;
    }
    for (unsigned enabled=0;enabled<7;++enabled) {
        systick.CTRL=enabled; assert(!w600_app_idle());
    }
    systick.CTRL=7; systick.LOAD=48000; assert(!w600_app_idle()); systick.LOAD=47999;
    assert(waits==1 && barriers==2 && mask==1 && !scb.SCR && !scb.ICSR);
    puts("PASS shallow idle: masked wake, barriers, unchanged mask, pending IRQs, all SCR/IRQ bits, SysTick profile");

    const uint32_t rates[]={16000,31997,32000,64000};
    for (unsigned r=0;r<sizeof rates/sizeof rates[0];++r) {
        uint32_t rate=rates[r];
        for (unsigned phase=0;phase<48000;phase+=997) for (unsigned elapsed=0;elapsed<=rate;elapsed+=113) {
            ms=UINT32_MAX-100; systick.CTRL=0;scb.ICSR=SCB_ICSR_PENDSTSET_Msk;resume_writes=0;
            struct w600_app_time before_sleep={ms,ms*48000u+phase};
            uint64_t numerator=(uint64_t)elapsed*48000000u+(uint64_t)phase*rate;
            uint64_t divisor=(uint64_t)rate*48000u;
            uint32_t delta=(uint32_t)((numerator+divisor-1)/divisor);
            assert(w600_app_resume(&ms,&before_sleep,elapsed,rate));
            assert(ms==before_sleep.ms+delta && resume_writes==3 && !scb.ICSR && mask==1);
            assert(systick.LOAD==47999 && systick.CTRL==7);
            assert(w600_app_clock(&ms,&out));
            assert((uint32_t)(out.cycles-before_sleep.cycles)<=48048000u);
            assert((uint64_t)delta*divisor>=numerator && (uint64_t)delta*divisor-numerator<divisor);
        }
    }
    for (unsigned bad=0;bad<14;++bad) {
        ms=100; systick.CTRL=0;systick.LOAD=47999;scb.ICSR=0;resume_writes=0;
        struct w600_app_time before_sleep={100,4800000};
        uint32_t rate=32000,elapsed=16000;
        switch(bad) {
        case 0:mask=0;break;case 1:ipsr=15;break;case 2:control=1;break;
        case 3:scb.SCR=4;break;case 4:systick.CTRL=7;break;case 5:systick.LOAD=48000;break;
        case 6:nvic.ISER[0]|=1u<<7;break;case 7:rate=15999;break;case 8:rate=64001;break;
        case 9:elapsed=32001;break;case 10:before_sleep.cycles+=48000;break;
        case 11:before_sleep.ms=102;break;case 12:before_sleep.ms=99;break;
        case 13:before_sleep.cycles-=1;break;
        }
        assert(!w600_app_resume(&ms,&before_sleep,elapsed,rate) && ms==100 && !resume_writes);
        mask=1;ipsr=control=0;scb.SCR=0;nvic.ISER[0]=1u<<27;
    }
    ms=100;systick.CTRL=0;systick.LOAD=47999;reload_stuck=1;
    struct w600_app_time before_sleep={101,101u*48000u+1u};
    assert(!w600_app_resume(&ms,&before_sleep,1,32000) && ms==102);
    assert(!w600_app_resume(NULL,&before_sleep,0,32000) && !w600_app_resume(&ms,NULL,0,32000));
    puts("PASS clock resume: phase/frequency grid, bounded upward rounding, millisecond/cycle wrap, pending tick, context rejection, stalled reload");
}
