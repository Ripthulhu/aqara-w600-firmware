#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"
#include "app_clock.h"
static RCC_t rcc;
static PMU_t pmu;
static FLASH_t flash;
static EXTI_t exti;
static GPIO_t gpioa,gpiob;
static USART_t uart;
static LPTIM_t timer;
static SysTick_Type systick;
static SCB_Type scb;
static NVIC_Type nvic;
static uint32_t mask,ipsr,control,writes,ignore_at,waits,resumes,wake_kind,elapsed_count;
static uint32_t point_race,clock_fault,inject_at,foreign_after,initial_imr;
static volatile uint32_t ticks;
static bool clock_valid,restore_valid;
static uint32_t read_reg(volatile const uint32_t *reg);
static void write_reg(volatile uint32_t *reg,uint32_t value);
static void wait_stop(void);
#undef RCC
#undef PMU
#undef FLASH
#undef EXTI
#undef GPIOA
#undef GPIOB
#undef UART3
#undef LPTIM2
#undef SysTick
#undef SCB
#undef NVIC
#define RCC (&rcc)
#define PMU (&pmu)
#define FLASH (&flash)
#define EXTI (&exti)
#define GPIOA (&gpioa)
#define GPIOB (&gpiob)
#define UART3 (&uart)
#define LPTIM2 (&timer)
#define SysTick (&systick)
#define SCB (&scb)
#define NVIC (&nvic)
#define __get_PRIMASK() mask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#undef __WFI
#undef __DSB
#undef __ISB
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#define __WFI() wait_stop()
#define STOP_READ(reg) read_reg(&(reg))
#define STOP_WRITE(reg,value) write_reg(&(reg),(value))
#include "stop_mode.c"

bool w600_app_clock(const volatile uint32_t *p,struct w600_app_time *out)
{
    assert(p==&ticks && mask==1);
    if (!clock_valid) return false;
    *out=(struct w600_app_time){ticks,ticks*48000u+12345u};return true;
}
bool w600_app_resume(volatile uint32_t *p,const struct w600_app_time *before,uint32_t elapsed,uint32_t hz)
{
    assert(p==&ticks && mask==1 && !scb.SCR && systick.CTRL==0 && normal_clock());
    assert(nvic.ISER[0]==UART_IRQ && !(nvic.ISPR[0]&WAKE_IRQS));
    assert(before->ms==ticks && elapsed==elapsed_count && hz==32000u);
    ++resumes;ticks+=500;systick.CTRL=7;return restore_valid;
}
static uint32_t read_reg(volatile const uint32_t *reg)
{
    uintptr_t addr=(uintptr_t)reg;
    if (addr>=(uintptr_t)&pmu && addr<(uintptr_t)&pmu+sizeof pmu)
        assert(rcc.APB1EN&RCC_APB1EN_PMUEN);
    if (reg==&timer.CNT && point_race) ++timer.CNT;
    return *reg;
}
static void write_reg(volatile uint32_t *reg,uint32_t value)
{
    ++writes;
    if (writes==inject_at) { gpioa.IDR&=~(1u<<10);exti.PIR|=1u<<10;nvic.ISPR[0]|=1u<<7; }
    if (writes==ignore_at) return;
    if (reg==&nvic.ISER[0]) nvic.ISER[0]|=value;
    else if (reg==&nvic.ICER[0]) nvic.ISER[0]&=~value;
    else if (reg==&nvic.ICPR[0]) nvic.ISPR[0]&=~value;
    else if (reg==&exti.PIR) { assert(!(value&~0x3ffffu));exti.PIR&=~value; }
    else if (reg==&timer.ICR) timer.ISR&=~value;
    else if (reg==&rcc.CSR1) {
        rcc.CSR1=value;
        if (!clock_fault && value&RCC_CSR1_PLLON) rcc.CSR1|=RCC_CSR1_PLLRDY;
    } else if (reg==&rcc.CFG) {
        rcc.CFG=value;
        if (clock_fault!=2 && value==2u) rcc.CFG=0x12;
    } else *reg=value;
    if (reg==&nvic.ICPR[0] && foreign_after) nvic.ISPR[0]|=1u<<27;
}
static void wait_stop(void)
{
    assert(mask==1 && !ipsr && !control && scb.SCR==4u && systick.CTRL==0);
    assert(nvic.ISER[0]==(UART_IRQ|WAKE_IRQS) && timer.IER==2u);
    assert(exti.IMR==(initial_imr|GPIO_LINES|TIMER_LINE));
    assert(exti.EXTICR[0]==0x12345018u && exti.EXTICR[1]==0x87654031u);
    ++waits;
    if (wake_kind!=6u) {
        rcc.CFG=0;rcc.PLLCFG=0x20260u;rcc.CSR1=0x500u;
    }
    uint32_t count=timer.CNT;
    if (!wake_kind) { timer.CNT=timer.ARR;timer.ISR=2;elapsed_count=timer.ARR-count;nvic.ISPR[0]|=1u<<30; }
    else if (wake_kind==1u) { timer.CNT=16;timer.ISR=2;elapsed_count=timer.ARR+1u-count+16u;nvic.ISPR[0]|=1u<<30; }
    else {
        timer.CNT+=320;elapsed_count=320;
        if (wake_kind==2u) { gpioa.IDR&=~(1u<<10);exti.PIR|=1u<<10;nvic.ISPR[0]|=1u<<7; }
        if (wake_kind==3u) { gpiob.IDR&=~(1u<<8);exti.PIR|=1u<<8;nvic.ISPR[0]|=1u<<7; }
        if (wake_kind==4u) { gpioa.IDR&=~(1u<<2);exti.PIR|=1u<<2;nvic.ISPR[0]|=1u<<6; }
        if (wake_kind==5u) { gpiob.IDR&=~(1u<<1);exti.PIR|=1u<<1;nvic.ISPR[0]|=1u<<5; }
    }
}
static struct w600_sleep_clock setup(void)
{
    memset(&rcc,0,sizeof rcc);memset(&pmu,0,sizeof pmu);memset(&flash,0,sizeof flash);
    memset(&exti,0,sizeof exti);memset(&gpioa,0,sizeof gpioa);memset(&gpiob,0,sizeof gpiob);
    memset(&uart,0,sizeof uart);memset(&timer,0,sizeof timer);memset(&systick,0,sizeof systick);
    memset(&scb,0,sizeof scb);memset(&nvic,0,sizeof nvic);
    mask=1;ipsr=control=writes=ignore_at=waits=resumes=wake_kind=elapsed_count=0;
    point_race=clock_fault=inject_at=foreign_after=0;clock_valid=restore_valid=true;ticks=100;
    rcc.CFG=0x12;rcc.PLLCFG=0x30260;rcc.CSR1=0x03000500;rcc.CSR2=3;
    rcc.APB1EN=UARTS|RCC_APB1EN_LPTIM2EN;rcc.CLKSEL=RCC_CLKSEL_LPTIM2_SEL_RCL;rcc.IOPEN=15;
    flash.ACR=2;flash.OPTR2=0x7121;nvic.ISER[0]=UART_IRQ;
    gpioa.IDR=(1u<<2)|(1u<<10)|(1u<<11);gpiob.IDR=(1u<<1)|(1u<<8);gpioa.MODE=1u<<22;
    uart.ISR=USART_ISR_TC;timer.ARR=15999;timer.CNT=40;timer.CR=1;
    systick.CTRL=7;systick.LOAD=47999;
    exti.EXTICR[0]=0x12345008;exti.EXTICR[1]=0x87654030;
    initial_imr=0xefc10000;
    exti.RTSR=0x10002;exti.FTSR=0x10004;exti.IMR=initial_imr;exti.EMR=0x20000;
    return (struct w600_sleep_clock){.phase=W600_SC_ARMED,.owned=3,.hz=32000,.windows=2,.arming=1,.ticks=&ticks};
}
static void cleaned(void)
{
    assert(mask==1 && !scb.SCR && nvic.ISER[0]==UART_IRQ && !(nvic.ISPR[0]&WAKE_IRQS));
    assert(!timer.IER && exti.EXTICR[0]==0x12345008 && exti.EXTICR[1]==0x87654030);
    assert(exti.RTSR==0x10002 && exti.FTSR==0x10004 && exti.IMR==initial_imr && exti.EMR==0x20000);
    assert(!(rcc.APB1EN&RCC_APB1EN_PMUEN));
}
int main(void)
{
    unsigned cases=0,baseline=0;
    for (unsigned masked=0;masked<2;++masked) for (unsigned kind=0;kind<7;++kind) {
        struct w600_sleep_clock s=setup();struct w600_stop_report report={0};wake_kind=kind;foreign_after=1;
        if (masked) { initial_imr&=~TIMER_LINE;exti.IMR=initial_imr; }
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_WOKE);
        assert(waits==1 && resumes==1 && report.entered==1 && report.woke==1 && !report.error);
        assert(report.elapsed==elapsed_count && normal_clock() && systick.CTRL==7);
        assert(report.radio_wakes==(kind==2u));
        assert(report.button_wakes==(kind>=3u && kind<=5u));
        assert(report.timer_wakes==(kind<=1u));
        assert(report.last_radio_at==(kind==2u ? ticks : 0));
        assert(nvic.ISPR[0]&UART_IRQ);cleaned();baseline=writes;++cases;
    }
    {
        struct w600_stop_report report={0};
        struct w600_sleep_clock s=setup();wake_kind=2;
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_WOKE);
        uint32_t radio_at=report.last_radio_at;
        s=setup();wake_kind=0;
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_WOKE);
        assert(report.wake_lines==TIMER_LINE && report.radio_wakes==1 && report.timer_wakes==1);
        assert(report.woke==2 && report.last_radio_at==radio_at);
        s=setup();wake_kind=3;
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_WOKE);
        assert(report.radio_wakes==1 && report.button_wakes==1 && report.timer_wakes==1);
        s=setup();gpioa.IDR&=~(1u<<10);
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_SKIPPED);
        assert(report.woke==3 && report.radio_wakes==1 && report.last_radio_at==radio_at);
        ++cases;
    }
    for (unsigned bad=0;bad<30;++bad) {
        struct w600_sleep_clock s=setup();struct w600_stop_report report={0};
        switch(bad) {
        case 0:mask=0;break;case 1:ipsr=15;break;case 2:control=1;break;case 3:scb.SCR=4;break;
        case 4:nvic.ISER[0]|=1u<<7;break;case 5:nvic.ISPR[0]=1;break;case 6:scb.ICSR=SCB_ICSR_PENDSTSET_Msk;break;
        case 7:systick.CTRL=0;break;case 8:systick.LOAD=48000;break;case 9:rcc.CFG=0;break;
        case 10:s.phase=W600_SC_DONE;break;case 11:s.owned=0;break;case 12:s.error=1;break;
        case 13:s.hz=15999;break;case 14:s.windows=1;break;case 15:rcc.APB1EN|=1;break;
        case 16:rcc.CLKSEL=0;break;case 17:rcc.CSR2=1;break;case 18:flash.OPTR2&=~1u;break;
        case 19:gpioa.MODE|=1u<<20;break;case 20:exti.PIR=1;break;case 21:exti.IMR|=1u<<8;break;
        case 22:timer.IER=2;break;case 23:timer.ARR=1;break;case 24:timer.ISR=2;break;
        case 25:gpioa.IDR&=~(1u<<10);break;case 26:uart.ISR|=USART_ISR_RXNE;break;
        case 27:s.ticks=NULL;break;case 28:s.arming=0;break;case 29:uart.ISR|=USART_ISR_BUSY;break;
        }
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_SKIPPED && !writes && !waits && !resumes);++cases;
    }
    for (unsigned early=0;early<5;++early) {
        struct w600_sleep_clock s=setup();struct w600_stop_report report={0};
        if (early==0) pmu.CR3=1;
        if (early==1) clock_valid=false;
        if (early==2) point_race=1;
        if (early==3) timer.CNT=10000;
        if (early==4) inject_at=9;
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_SKIPPED && !waits && !resumes);
        cleaned();++cases;
    }
    for (unsigned fault=1;fault<=3;++fault) {
        struct w600_sleep_clock s=setup();struct w600_stop_report report={0};
        if (fault<3) clock_fault=fault;else restore_valid=false;
        assert(w600_stop_mode(&s,&ticks,&report)==W600_STOP_FAILED && waits==1 && report.error);
        cleaned();++cases;
    }
    for (unsigned ignored=1;ignored<=baseline;++ignored) {
        struct w600_sleep_clock s=setup();struct w600_stop_report report={0};ignore_at=ignored;
        enum w600_stop_result result=w600_stop_mode(&s,&ticks,&report);
        if (result!=W600_STOP_FAILED) { cleaned();assert(normal_clock() && systick.CTRL==7); }
        else assert(report.error);
        ++cases;
    }
    puts("PASS STOP: timer match/wrap, radio/three button wakes, pending entry, clock restoration, IRQ ownership, context/race/clock/write failures; mocked hardware only");
    printf("PASS %u STOP cases; no physical power or radio timing claim\n",cases);
}
