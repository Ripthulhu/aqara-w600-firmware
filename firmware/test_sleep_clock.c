#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"
#include "app_clock.h"
static RCC_t rcc;
static LPTIM_t timer;
static FLASH_t flash;
static IWDG_t watchdog;
static NVIC_Type nvic;
static uint32_t mask,ipsr,control,factory_value,ms,cycles,hz,enabled_at,writes,ignore_at,race;
static volatile uint32_t ticks;
static bool clock_valid,running;
static uint32_t read_reg(volatile const uint32_t *reg);
static void write_reg(volatile uint32_t *reg,uint32_t value);
#undef RCC
#undef LPTIM2
#undef FLASH
#undef IWDG
#undef NVIC
#define RCC (&rcc)
#define LPTIM2 (&timer)
#define FLASH (&flash)
#define IWDG (&watchdog)
#define NVIC (&nvic)
#define __get_PRIMASK() mask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#define SLEEP_CLOCK_FACTORY factory_value
#define SLEEP_CLOCK_READ(reg) read_reg(&(reg))
#define SLEEP_CLOCK_WRITE(reg,value) write_reg(&(reg),(value))
#include "sleep_clock.c"

bool w600_app_clock(const volatile uint32_t *tick,struct w600_app_time *out)
{
    assert(tick==&ticks);
    if (!clock_valid) return false;
    *out=(struct w600_app_time){ms,cycles}; return true;
}
static bool timer_reg(volatile const uint32_t *reg)
{
    uintptr_t address=(uintptr_t)reg;
    return address>=(uintptr_t)&timer && address<(uintptr_t)&timer+sizeof timer;
}
static uint32_t read_reg(volatile const uint32_t *reg)
{
    if (timer_reg(reg)) assert(rcc.APB1EN&RCC_APB1EN_LPTIM2EN);
    if (reg==&timer.CNT && race) ++timer.CNT;
    return *reg;
}
static void write_reg(volatile uint32_t *reg,uint32_t value)
{
    ++writes;
    uint32_t allowed=0;
    if (reg==&rcc.CSR2) allowed=RCC_CSR2_RCLON|RCC_CSR2_RMVF;
    else if (reg==&rcc.APB1EN) allowed=RCC_APB1EN_LPTIM2EN;
    else if (reg==&rcc.APB1RST) allowed=RCC_APB1RST_LPTIM2_RST;
    else if (reg==&rcc.CLKSEL) allowed=RCC_CLKSEL_LPTIM2_SEL;
    else {
        assert(timer_reg(reg) && (rcc.APB1EN&RCC_APB1EN_LPTIM2EN));
        assert(reg==&timer.CR || reg==&timer.IER || reg==&timer.CFG || reg==&timer.ARR);
        allowed=UINT32_MAX;
        if (reg==&timer.ARR) assert((timer.CR&1u) && ms-enabled_at>=2u);
    }
    assert(!((*reg^value)&~allowed));
    if (reg==&rcc.CSR2) assert(!(value&RCC_CSR2_RMVF));
    if (writes==ignore_at) return;
    *reg=value;
    if (reg==&rcc.CSR2 && !(value&RCC_CSR2_RCLON)) rcc.CSR2&=~RCC_CSR2_RCLRDY;
    if (reg==&rcc.APB1RST && value&RCC_APB1RST_LPTIM2_RST) {
        memset(&timer,0,sizeof timer); timer.ARR=1; running=false;
    }
    if (reg==&timer.CR) {
        if (value==1u) enabled_at=ms;
        if (value&4u) { running=true; timer.CR&=~4u; }
        if (!(value&1u)) running=false;
    }
}
static struct w600_sleep_clock setup(void)
{
    memset(&rcc,0,sizeof rcc);memset(&timer,0,sizeof timer);
    memset(&flash,0,sizeof flash);memset(&watchdog,0,sizeof watchdog);memset(&nvic,0,sizeof nvic);
    mask=1;ipsr=control=ms=cycles=enabled_at=writes=ignore_at=race=0;
    factory_value=0xfff60009;hz=32000;clock_valid=true;running=false;
    rcc.CFG=0x12;rcc.PLLCFG=0x30260;rcc.CSR1=0x03000500;
    rcc.CSR2=0x1c000000;rcc.APB1EN=UARTS;rcc.RCLCAL=9;flash.ACR=2;
    nvic.ISER[0]=1u<<27;timer.ARR=1;
    return (struct w600_sleep_clock){0};
}
static void advance(unsigned count)
{
    ms+=count;cycles+=count*48000u;
    if (rcc.CSR2&RCC_CSR2_RCLON) rcc.CSR2|=RCC_CSR2_RCLRDY;
    if (running) {
        uint32_t total=timer.CNT+count*hz/1000u;
        if (total>timer.ARR) timer.ISR|=2u;
        timer.CNT=total%(timer.ARR+1u);
    }
}
static void measuring(struct w600_sleep_clock *s)
{
    assert(w600_sleep_clock_begin(s,&ticks));
    advance(1);w600_sleep_clock_poll(s);assert(s->phase==W600_SC_ENABLE_WAIT);
    advance(1);w600_sleep_clock_poll(s);assert(s->phase==W600_SC_ENABLE_WAIT);
    advance(1);w600_sleep_clock_poll(s);assert(s->phase==W600_SC_COUNTER_WAIT);
    advance(1);w600_sleep_clock_poll(s);assert(s->phase==W600_SC_MEASURE);
}
static void released(const struct w600_sleep_clock *s)
{
    assert(!s->owned && !s->cleanup_failed && !running && rcc.APB1EN==UARTS &&
        !rcc.APB1RST && !rcc.CLKSEL && rcc.CSR2==0x1c000000);
    assert(nvic.ISER[0]==1u<<27 && !nvic.ISPR[0] && mask==1);
}
int main(void)
{
    unsigned cases=0,baseline_writes=0;
    const unsigned rates[]={16000,20000,32000,40000,64000};
    for (unsigned i=0;i<sizeof rates/sizeof rates[0];++i) {
        unsigned rate=rates[i];
        struct w600_sleep_clock s=setup();hz=rate;
        if (rate==40000) { ms=UINT32_MAX-100u;cycles=UINT32_MAX-100000u; }
        measuring(&s);
        advance(199);w600_sleep_clock_poll(&s);assert(!s.windows && !s.hz);
        advance(1);w600_sleep_clock_poll(&s);assert(s.windows==1 && !s.hz && s.first_hz==rate);
        advance(200);w600_sleep_clock_poll(&s);
        assert(s.phase==W600_SC_DONE && !s.error && s.hz==rate && s.windows==2);
        released(&s);baseline_writes=writes;
        assert(!w600_sleep_clock_begin(&s,&ticks));++cases;
    }
    for (unsigned bad=0;bad<17;++bad) {
        struct w600_sleep_clock s=setup();
        switch(bad) {
        case 0:mask=0;break;case 1:ipsr=15;break;case 2:control=1;break;
        case 3:nvic.ISER[0]|=1u<<7;break;case 4:nvic.ISPR[0]=1u<<30;break;
        case 5:rcc.APB1EN|=4u;break;case 6:rcc.APB2EN=1;break;
        case 7:rcc.CLKSEL=1;break;case 8:rcc.CSR2|=2;break;
        case 9:rcc.AWCR=0x8200;break;case 10:watchdog.CFG=1;break;
        case 11:factory_value^=1u<<16;break;case 12:rcc.RCLCAL=8;break;
        case 13:clock_valid=false;break;case 14:rcc.CFG=0;break;
        case 15:rcc.PLLCFG=0;break;case 16:flash.ACR=0;break;
        }
        assert(!w600_sleep_clock_begin(&s,&ticks) && !writes && !s.owned && s.error);++cases;
    }
    for (unsigned phase=0;phase<4;++phase) {
        struct w600_sleep_clock s=setup();assert(w600_sleep_clock_begin(&s,&ticks));
        if (phase>0) { advance(1);w600_sleep_clock_poll(&s); }
        if (phase>1) { advance(2);w600_sleep_clock_poll(&s); }
        if (phase>2) { advance(1);w600_sleep_clock_poll(&s); }
        assert(w600_sleep_clock_cancel(&s) && s.phase==W600_SC_OFF);
        released(&s);++cases;
    }
    for (unsigned retained=1;retained<=3;retained+=2) {
        struct w600_sleep_clock s=setup();rcc.CSR2|=retained;
        measuring(&s);advance(200);w600_sleep_clock_poll(&s);
        advance(200);w600_sleep_clock_poll(&s);
        assert(s.phase==W600_SC_DONE && !s.error && s.hz==32000 && s.windows==2);
        released(&s);++cases;
        for (unsigned consumer=0;consumer<5;++consumer) {
            s=setup();rcc.CSR2|=retained;
            if (consumer==0) watchdog.CFG=1;
            else if (consumer==1) rcc.AWCR=RCC_AWCR_RTCEN;
            else if (consumer==2) rcc.AWCR=RCC_AWCR_RTCSEL_RCL;
            else if (consumer==3) rcc.AWCR=RCC_AWCR_LXTALON;
            else rcc.APB1EN|=RCC_APB1EN_LPTIM2EN;
            assert(!w600_sleep_clock_begin(&s,&ticks) && !writes && !s.owned && s.error==W600_SC_CONTEXT);
            ++cases;
        }
    }
    for (unsigned fault=0;fault<7;++fault) {
        struct w600_sleep_clock s=setup();measuring(&s);
        switch(fault) {
        case 0:advance(500);break;
        case 1:race=1;break;
        case 2:timer.CNT=65536u;break;
        case 3:timer.ISR=2;break;
        case 4:rcc.CSR2&=~RCC_CSR2_RCLRDY;break;
        case 5:--ms;break;
        case 6:rcc.APB1EN=UARTS;break;
        }
        w600_sleep_clock_poll(&s);assert(s.phase==W600_SC_FAILED && s.error && !s.hz);
        released(&s);++cases;
    }
    struct w600_sleep_clock s=setup();measuring(&s);
    advance(200);w600_sleep_clock_poll(&s);hz=28000;
    advance(200);w600_sleep_clock_poll(&s);
    assert(s.error==W600_SC_RATE && !s.hz);released(&s);++cases;
    s=setup();assert(w600_sleep_clock_begin(&s,&ticks));ms=10;
    w600_sleep_clock_poll(&s);assert(s.error==W600_SC_TIMEOUT);released(&s);++cases;
    s=setup();measuring(&s);nvic.ISER[0]|=1u<<7;
    w600_sleep_clock_poll(&s);assert(s.cleanup_failed && s.owned && !s.hz);
    nvic.ISER[0]=1u<<27; s.cleanup_failed=0;
    assert(w600_sleep_clock_cancel(&s));released(&s);++cases;
    for (unsigned fault=1;fault<=baseline_writes;++fault) {
        s=setup();ignore_at=fault;
        (void)w600_sleep_clock_begin(&s,&ticks);
        for (unsigned elapsed=0;elapsed<600 && s.phase!=W600_SC_DONE && s.phase!=W600_SC_FAILED;++elapsed) {
            advance(1);w600_sleep_clock_poll(&s);
        }
        assert(s.phase==W600_SC_DONE || s.phase==W600_SC_FAILED);
        if (s.cleanup_failed) assert(s.owned && s.error && !s.hz);
        else released(&s);
        ++cases;
    }
    s=setup();assert(!w600_sleep_clock_arm(&s,&ticks) && !writes);++cases;
    for (unsigned stop=0;stop<5;++stop) {
        s=setup();measuring(&s);advance(200);w600_sleep_clock_poll(&s);
        advance(200);w600_sleep_clock_poll(&s);assert(s.hz==32000);
        assert(w600_sleep_clock_arm(&s,&ticks));
        if (stop>=1) { advance(1);w600_sleep_clock_poll(&s); }
        if (stop>=2) { advance(2);w600_sleep_clock_poll(&s);assert(timer.ARR==15999u); }
        if (stop>=3) {
            advance(1);w600_sleep_clock_poll(&s);assert(s.phase==W600_SC_ARMED && s.hz==32000);
        }
        if (stop>=4) {
            advance(500);assert(timer.ISR==2 && running);
            w600_sleep_clock_poll(&s);assert(s.phase==W600_SC_ARMED);
        }
        assert(w600_sleep_clock_cancel(&s) && s.phase==W600_SC_DONE && s.hz==32000 && s.windows==2);
        released(&s);++cases;
    }
    assert(!w600_sleep_clock_begin(NULL,&ticks) && !w600_sleep_clock_cancel(NULL));
    printf("PASS %u slow-clock cases: two rate windows, wraps, ownership, cancellation, context/trim/time/rate failures; mocked hardware only\n",cases);
}
