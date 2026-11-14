#include "ciu32l071.h"
#include "app_clock.h"
#ifndef APP_CLOCK_READ
#define APP_CLOCK_READ(reg) (reg)
#endif
#ifndef APP_CLOCK_WRITE
#define APP_CLOCK_WRITE(reg,value) ((reg)=(value))
#endif
bool w600_app_clock(const volatile uint32_t *ticks, struct w600_app_time *out)
{
    if (__get_PRIMASK()!=1 || __get_IPSR() || __get_CONTROL() ||
        (APP_CLOCK_READ(SysTick->CTRL)&7u)!=7u ||
        APP_CLOCK_READ(SysTick->LOAD)!=47999u) return false;
    uint32_t ms=*ticks;
    uint32_t before=APP_CLOCK_READ(SCB->ICSR)&SCB_ICSR_PENDSTSET_Msk;
    uint32_t value=APP_CLOCK_READ(SysTick->VAL);
    uint32_t after=APP_CLOCK_READ(SCB->ICSR)&SCB_ICSR_PENDSTSET_Msk;
    if (before && !after) return false;
    /* A wrap between the pending-bit and counter reads needs the new counter. */
    if (before!=after) value=APP_CLOCK_READ(SysTick->VAL);
    if (value>47999u) return false;
    if (after) ++ms;
    out->ms=ms;
    out->cycles=ms*48000u+(47999u-value);
    return true;
}

bool w600_app_idle(void)
{
    if (__get_PRIMASK()!=1 || __get_IPSR() || __get_CONTROL() || SCB->SCR ||
        (SysTick->CTRL&7u)!=7u || SysTick->LOAD!=47999u ||
        NVIC->ISER[0]!=(1u<<27) || NVIC->ISPR[0] ||
        (SCB->ICSR&(SCB_ICSR_PENDSTSET_Msk|SCB_ICSR_PENDSVSET_Msk|SCB_ICSR_NMIPENDSET_Msk)))
        return false;
    /* A pending masked IRQ wakes WFI; keeping the check masked avoids lost work. */
    __DSB(); __WFI(); __ISB();
    return true;
}

bool w600_app_resume(volatile uint32_t *ticks,const struct w600_app_time *before,
                     uint32_t elapsed_slow_ticks,uint32_t slow_hz)
{
    if (!ticks || !before || __get_PRIMASK()!=1 || __get_IPSR() || __get_CONTROL() ||
        SCB->SCR || (SysTick->CTRL&7u) || SysTick->LOAD!=47999u ||
        NVIC->ISER[0]!=(1u<<27) || slow_hz<16000u || slow_hz>64000u ||
        elapsed_slow_ticks>slow_hz || (uint32_t)(before->ms-*ticks)>1u) return false;
    uint32_t phase=before->cycles-before->ms*48000u;
    if (phase>=48000u) return false;
    uint32_t denominator=slow_hz*48000u;
    uint64_t elapsed=(uint64_t)elapsed_slow_ticks*48000000u+(uint64_t)phase*slow_hz;
    uint32_t advance=(uint32_t)((elapsed+denominator-1u)/denominator);
    if (advance>1001u) return false;
    *ticks=before->ms+advance;
    APP_CLOCK_WRITE(SCB->ICSR,SCB_ICSR_PENDSTCLR_Msk);
    APP_CLOCK_WRITE(SysTick->VAL,0);
    __DSB();
    APP_CLOCK_WRITE(SysTick->CTRL,7u);
    __DSB(); __ISB();
    /* VAL reads zero just after a clear, before its first clock reload. */
    for (unsigned i=0;i<16;++i) {
        uint32_t value=APP_CLOCK_READ(SysTick->VAL);
        if (value && value<=47999u) return true;
    }
    return false;
}
