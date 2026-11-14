#include "ciu32l071.h"
#include "sleep_clock.h"
#include "app_clock.h"
#ifndef SLEEP_CLOCK_READ
#define SLEEP_CLOCK_READ(reg) (reg)
#endif
#ifndef SLEEP_CLOCK_WRITE
#define SLEEP_CLOCK_WRITE(reg,value) do { (reg)=(value); __DSB(); } while (0)
#endif
#ifndef SLEEP_CLOCK_FACTORY
#define SLEEP_CLOCK_FACTORY (*(volatile const uint32_t *)RCL_CAL)
#endif
#define RD(reg) SLEEP_CLOCK_READ(reg)
#define WR(reg,value) SLEEP_CLOCK_WRITE(reg,(uint32_t)(value))
enum { OWN_RCL=1, OWN_TIMER=2, UARTS=0x60000u, WINDOW_CYCLES=9600000u };
_Static_assert(RCC_APB1EN_LPTIM2EN==0x40000000u && RCC_APB1RST_LPTIM2_RST==0x40000000u &&
    RCC_CLKSEL_LPTIM2_SEL_RCL==0x100000u && RCC_CSR2_RMVF==0x10000u &&
    LPTIM2_IRQn==30 && LPTIM_CR_ENABLE==1 && LPTIM_CR_CNTSTRT==4,
    "Reviewed CIU32 low-power timer profile");

static bool context(void)
{
    return __get_PRIMASK()==1 && !__get_IPSR() && !__get_CONTROL() &&
        RD(NVIC->ISER[0])==(1u<<27) && !(RD(NVIC->ISPR[0])&(1u<<30));
}
static bool profile(const struct w600_sleep_clock *s)
{
    uint32_t gate=(s->owned&OWN_TIMER) ? RCC_APB1EN_LPTIM2EN : 0;
    uint32_t select=(s->owned&OWN_TIMER) ? RCC_CLKSEL_LPTIM2_SEL_RCL : 0;
    return context() && RD(RCC->APB1EN)==(UARTS|gate) && !RD(RCC->APB2EN) &&
        RD(RCC->CLKSEL)==select && !(RD(RCC->APB1RST)&RCC_APB1RST_LPTIM2_RST) &&
        RD(RCC->CFG)==0x12u && RD(RCC->PLLCFG)==0x30260u &&
        (RD(RCC->CSR1)&(RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY|RCC_CSR1_RCHDIV))==
            (RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY) &&
        (RD(FLASH->ACR)&FLASH_ACR_LATENCY)==FLASH_ACR_LATENCY_2CLK &&
        (RD(RCC->CSR2)&RCC_CSR2_RCLON)!=0;
}
static bool release(struct w600_sleep_clock *s)
{
    if (!s->owned) return true;
    if (!context()) { s->cleanup_failed=1; return false; }
    bool ok=true;
    if (s->owned&OWN_TIMER) {
        if (RD(RCC->APB1EN)&RCC_APB1EN_LPTIM2EN) {
            WR(LPTIM2->IER,0); WR(LPTIM2->CR,0);
        }
        WR(RCC->APB1RST,RD(RCC->APB1RST)|RCC_APB1RST_LPTIM2_RST);
        if (!(RD(RCC->APB1RST)&RCC_APB1RST_LPTIM2_RST)) ok=false;
        WR(RCC->APB1EN,RD(RCC->APB1EN)&~RCC_APB1EN_LPTIM2EN);
        WR(RCC->APB1RST,RD(RCC->APB1RST)&~RCC_APB1RST_LPTIM2_RST);
        WR(RCC->CLKSEL,RD(RCC->CLKSEL)&~RCC_CLKSEL_LPTIM2_SEL);
        if ((RD(RCC->APB1EN)&RCC_APB1EN_LPTIM2EN) ||
            (RD(RCC->APB1RST)&RCC_APB1RST_LPTIM2_RST) ||
            (RD(RCC->CLKSEL)&RCC_CLKSEL_LPTIM2_SEL)) ok=false;
    }
    if (s->owned&OWN_RCL) {
        WR(RCC->CSR2,RD(RCC->CSR2)&~(RCC_CSR2_RCLON|RCC_CSR2_RMVF));
        if (RD(RCC->CSR2)&RCC_CSR2_RCLON) ok=false;
    }
    if (ok) s->owned=0;
    else s->cleanup_failed=1;
    return ok;
}
static void fail(struct w600_sleep_clock *s,uint32_t error)
{
    if (!s->error) s->error=error;
    s->hz=0; s->phase=W600_SC_FAILED;
    (void)release(s);
}
static bool start(struct w600_sleep_clock *s,const volatile uint32_t *ticks,bool arming)
{
    if (!s || !ticks || s->phase!=(arming ? W600_SC_DONE : W600_SC_OFF) ||
        s->error || s->owned || s->cleanup_failed ||
        (arming && (s->hz<16000u || s->hz>64000u || s->windows!=2u))) return false;
    struct w600_app_time now;
    uint32_t rcl=RD(RCC->CSR2)&(RCC_CSR2_RCLON|RCC_CSR2_RCLRDY);
    if (!context() || !w600_app_clock(ticks,&now) || RD(RCC->APB1EN)!=UARTS ||
        RD(RCC->APB2EN) || RD(RCC->CLKSEL) ||
        RD(RCC->CFG)!=0x12u || RD(RCC->PLLCFG)!=0x30260u ||
        (RD(RCC->CSR1)&(RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY|RCC_CSR1_RCHDIV))!=
            (RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY) ||
        (RD(FLASH->ACR)&FLASH_ACR_LATENCY)!=FLASH_ACR_LATENCY_2CLK ||
        (RD(RCC->APB1RST)&RCC_APB1RST_LPTIM2_RST) ||
        rcl==RCC_CSR2_RCLRDY ||
        (RD(RCC->AWCR)&(RCC_AWCR_RTCEN|RCC_AWCR_RTCSEL|RCC_AWCR_LXTALON)) ||
        RD(IWDG->CFG) || RD(IWDG->CNT) || RD(IWDG->WINR)) {
        s->error=W600_SC_CONTEXT; s->phase=W600_SC_FAILED; return false;
    }
    uint32_t factory=SLEEP_CLOCK_FACTORY,trim=factory&0xffffu;
    if ((factory>>16)!=(trim^0xffffu) || trim>31u || RD(RCC->RCLCAL)!=trim) {
        s->error=W600_SC_FACTORY; s->phase=W600_SC_FAILED; return false;
    }
    uint32_t hz=arming ? s->hz : 0;
    /* A software reset can leave RCL running; all other consumers are excluded above. */
    *s=(struct w600_sleep_clock){.phase=W600_SC_RCL_WAIT,.owned=OWN_RCL,
        .phase_ms=now.ms,.last_ms=now.ms,.ticks=ticks,.arming=arming,
        .hz=hz,.windows=arming ? 2u : 0u};
    WR(RCC->CSR2,(RD(RCC->CSR2)&~RCC_CSR2_RMVF)|RCC_CSR2_RCLON);
    if (!(RD(RCC->CSR2)&RCC_CSR2_RCLON)) { fail(s,W600_SC_HARDWARE); return false; }
    return true;
}
bool w600_sleep_clock_begin(struct w600_sleep_clock *s,const volatile uint32_t *ticks)
{ return start(s,ticks,false); }
bool w600_sleep_clock_arm(struct w600_sleep_clock *s,const volatile uint32_t *ticks)
{ return start(s,ticks,true); }
static bool point(struct w600_sleep_clock *s,struct w600_app_time *now,uint32_t *count)
{
    for (unsigned i=0;i<8;++i) {
        uint32_t before=RD(LPTIM2->CNT);
        if (!w600_app_clock(s->ticks,now)) return false;
        uint32_t after=RD(LPTIM2->CNT);
        if (before==after) { *count=after; return after<=65535u; }
    }
    return false;
}
void w600_sleep_clock_poll(struct w600_sleep_clock *s)
{
    if (!s || s->phase==W600_SC_OFF || s->phase==W600_SC_DONE ||
        s->phase==W600_SC_FAILED || s->phase==W600_SC_ARMED) return;
    struct w600_app_time now;
    if (!profile(s) || !w600_app_clock(s->ticks,&now)) { fail(s,W600_SC_CONTEXT); return; }
    if ((uint32_t)(now.ms-s->last_ms)>=0x80000000u) { fail(s,W600_SC_TIME); return; }
    s->last_ms=now.ms;
    uint32_t age=now.ms-s->phase_ms;
    if ((s->phase==W600_SC_MEASURE && age>=500u) ||
        (s->phase!=W600_SC_MEASURE && age>=10u)) { fail(s,W600_SC_TIMEOUT); return; }
    if (s->phase==W600_SC_RCL_WAIT) {
        if (!(RD(RCC->CSR2)&RCC_CSR2_RCLRDY)) return;
        s->owned|=OWN_TIMER;
        WR(RCC->CLKSEL,RCC_CLKSEL_LPTIM2_SEL_RCL);
        WR(RCC->APB1EN,UARTS|RCC_APB1EN_LPTIM2EN);
        if (!profile(s)) { fail(s,W600_SC_HARDWARE); return; }
        WR(RCC->APB1RST,RD(RCC->APB1RST)|RCC_APB1RST_LPTIM2_RST);
        if (!(RD(RCC->APB1RST)&RCC_APB1RST_LPTIM2_RST)) { fail(s,W600_SC_HARDWARE); return; }
        WR(RCC->APB1RST,RD(RCC->APB1RST)&~RCC_APB1RST_LPTIM2_RST);
        if (!profile(s)) { fail(s,W600_SC_HARDWARE); return; }
        WR(LPTIM2->IER,0); WR(LPTIM2->CFG,0); WR(LPTIM2->CR,LPTIM_CR_ENABLE);
        if (RD(LPTIM2->IER) || RD(LPTIM2->CFG) || !(RD(LPTIM2->CR)&LPTIM_CR_ENABLE)) {
            fail(s,W600_SC_HARDWARE); return;
        }
        s->phase=W600_SC_ENABLE_WAIT; s->phase_ms=now.ms; return;
    }
    if (!(RD(RCC->CSR2)&RCC_CSR2_RCLRDY) || RD(LPTIM2->IER) || RD(LPTIM2->CFG) || RD(LPTIM2->ISR) ||
        !(RD(LPTIM2->CR)&LPTIM_CR_ENABLE)) { fail(s,W600_SC_HARDWARE); return; }
    if (s->phase==W600_SC_ENABLE_WAIT) {
        if (age<2u) return; /* Let the enable cross both clock domains before ARR. */
        uint32_t reload=s->arming ? s->hz/2u-1u : 65535u;
        WR(LPTIM2->ARR,reload); WR(LPTIM2->CR,LPTIM_CR_ENABLE|LPTIM_CR_CNTSTRT);
        if (RD(LPTIM2->ARR)!=reload) { fail(s,W600_SC_HARDWARE); return; }
        s->phase=W600_SC_COUNTER_WAIT; s->phase_ms=now.ms; return;
    }
    uint32_t count;
    if (!point(s,&now,&count)) { fail(s,W600_SC_TIME); return; }
    if (s->phase==W600_SC_COUNTER_WAIT) {
        if (!count) return;
        s->start_count=count; s->start_cycles=now.cycles;
        s->phase=s->arming ? W600_SC_ARMED : W600_SC_MEASURE;
        s->phase_ms=now.ms; return;
    }
    uint32_t cycles=now.cycles-s->start_cycles;
    if (cycles<WINDOW_CYCLES) return;
    if (cycles>24000000u) { fail(s,W600_SC_TIME); return; }
    uint32_t delta=(count-s->start_count)&65535u,kcycles=cycles/1000u;
    uint32_t hz=(delta*48000u+kcycles/2u)/kcycles;
    if (hz<16000u || hz>64000u) { fail(s,W600_SC_RATE); return; }
    s->sample_cycles=cycles; s->sample_count=delta;
    if (!s->windows) {
        s->first_hz=hz; s->windows=1; s->start_count=count;
        s->start_cycles=now.cycles; s->phase_ms=now.ms; return;
    }
    uint32_t difference=hz>s->first_hz ? hz-s->first_hz : s->first_hz-hz;
    uint32_t lower=hz<s->first_hz ? hz : s->first_hz;
    if (difference*100u>lower*2u) { fail(s,W600_SC_RATE); return; }
    s->windows=2; s->hz=(hz+s->first_hz+1u)/2u;
    if (!release(s)) { s->hz=0; s->error=W600_SC_RELEASE; s->phase=W600_SC_FAILED; return; }
    s->phase=W600_SC_DONE;
}
bool w600_sleep_clock_cancel(struct w600_sleep_clock *s)
{
    if (!s) return false;
    if (!release(s)) {
        if (!s->error) s->error=W600_SC_RELEASE;
        s->phase=W600_SC_FAILED; s->hz=0; return false;
    }
    if (s->phase!=W600_SC_DONE && !s->error) {
        if (s->arming) s->phase=W600_SC_DONE;
        else { s->phase=W600_SC_OFF; s->hz=0; s->windows=0; }
    }
    return true;
}
