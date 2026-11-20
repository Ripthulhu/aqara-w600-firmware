#include "ciu32l071.h"
#include "stop_mode.h"
#include "app_clock.h"
#ifndef STOP_READ
#define STOP_READ(reg) (reg)
#endif
#ifndef STOP_WRITE
#define STOP_WRITE(reg,value) do { (reg)=(value); __DSB(); } while (0)
#endif
#define RD(reg) STOP_READ(reg)
#define WR(reg,value) STOP_WRITE(reg,(uint32_t)(value))
enum { GPIO_LINES=(1u<<1)|(1u<<2)|(1u<<8)|(1u<<10),
       TIMER_LINE=1u<<23, WAKE_IRQS=(1u<<5)|(1u<<6)|(1u<<7)|(1u<<30),
       UART_IRQ=1u<<27, UARTS=0x60000u };
_Static_assert(EXTI0_1_IRQn==5 && EXTI2_3_IRQn==6 && EXTI4_15_IRQn==7 &&
    LPTIM2_IRQn==30 && RCC_APB1EN_PMUEN==0x10000000u &&
    SCB_SCR_SLEEPDEEP_Msk==4u,"Reviewed STOP wake profile");

static bool lines_idle(void)
{
    return (RD(GPIOA->IDR)&((1u<<2)|(1u<<10)|(1u<<11)))==((1u<<2)|(1u<<10)|(1u<<11)) &&
        (RD(GPIOB->IDR)&((1u<<1)|(1u<<8)))==((1u<<1)|(1u<<8)) &&
        !(RD(UART3->ISR)&(15u|USART_ISR_RXNE|USART_ISR_BUSY)) &&
        (RD(UART3->ISR)&USART_ISR_TC);
}
static bool normal_clock(void)
{
    return RD(RCC->CFG)==0x12u && RD(RCC->PLLCFG)==0x30260u &&
        (RD(RCC->CSR1)&(RCC_CSR1_RCHRDY|RCC_CSR1_RCHDIV|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY))==
            (RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY) &&
        (RD(FLASH->ACR)&FLASH_ACR_LATENCY)==FLASH_ACR_LATENCY_2CLK;
}
static bool restore_clock(void)
{
    if (normal_clock()) return true; /* A pending IRQ can prevent STOP entry. */
    if (RD(RCC->CFG) || RD(RCC->PLLCFG)!=0x20260u ||
        (RD(RCC->CSR1)&(RCC_CSR1_RCHRDY|RCC_CSR1_RCHDIV))!=RCC_CSR1_RCHRDY ||
        (RD(FLASH->ACR)&FLASH_ACR_LATENCY)!=FLASH_ACR_LATENCY_2CLK) return false;
    WR(RCC->CSR1,RD(RCC->CSR1)|RCC_CSR1_PLLON);
    unsigned n;
    for (n=0;n<4096u && !(RD(RCC->CSR1)&RCC_CSR1_PLLRDY);++n) {}
    if (n==4096u) return false;
    WR(RCC->PLLCFG,RD(RCC->PLLCFG)|RCC_PLLCFG_PLLEN);
    WR(RCC->CFG,RCC_CFG_SYSW_PLL);
    for (n=0;n<4096u && (RD(RCC->CFG)&RCC_CFG_SYSWS)!=RCC_CFG_SYSWS_PLL;++n) {}
    return n!=4096u && normal_clock();
}
static bool point(const volatile uint32_t *ticks,struct w600_app_time *time,uint32_t *count)
{
    for (unsigned i=0;i<8u;++i) {
        uint32_t a=RD(LPTIM2->CNT);
        if (!w600_app_clock(ticks,time)) return false;
        uint32_t b=RD(LPTIM2->CNT);
        if (a==b) { *count=b; return true; }
    }
    return false;
}
enum w600_stop_result w600_stop_mode(struct w600_sleep_clock *s,volatile uint32_t *ticks,
                                     struct w600_stop_report *report)
{
    if (!s || !ticks || !report || report->error) return W600_STOP_SKIPPED;
    report->wake_lines=0;
    if (__get_PRIMASK()!=1 || __get_IPSR() || __get_CONTROL() || RD(SCB->SCR) ||
        RD(NVIC->ISER[0])!=UART_IRQ || RD(NVIC->ISPR[0]) ||
        RD(SCB->ICSR)&(SCB_ICSR_PENDSTSET_Msk|SCB_ICSR_PENDSVSET_Msk|SCB_ICSR_NMIPENDSET_Msk) ||
        (RD(SysTick->CTRL)&7u)!=7u || RD(SysTick->LOAD)!=47999u || !normal_clock() ||
        s->phase!=W600_SC_ARMED || s->owned!=3u || s->error || s->cleanup_failed ||
        s->hz<16000u || s->hz>64000u || s->windows!=2u || s->ticks!=ticks || !s->arming ||
        RD(RCC->APB1EN)!=(UARTS|RCC_APB1EN_LPTIM2EN) || RD(RCC->APB2EN) ||
        RD(RCC->CLKSEL)!=RCC_CLKSEL_LPTIM2_SEL_RCL ||
        (RD(RCC->CSR2)&3u)!=3u || !(RD(FLASH->OPTR2)&FLASH_OPTR2_RST_STOP) ||
        (RD(RCC->IOPEN)&3u)!=3u ||
        (RD(GPIOA->MODE)&((3u<<4)|(3u<<20)|(3u<<22)))!=(1u<<22) ||
        (RD(GPIOB->MODE)&((3u<<2)|(3u<<16))) ||
        RD(EXTI->PIR) || ((RD(EXTI->IMR)|RD(EXTI->EMR))&GPIO_LINES) ||
        RD(LPTIM2->IER) || RD(LPTIM2->CFG) || RD(LPTIM2->CR)!=1u ||
        RD(LPTIM2->ARR)!=s->hz/2u-1u || RD(LPTIM2->ISR) || !lines_idle())
        return W600_STOP_SKIPPED;

    WR(RCC->APB1EN,RD(RCC->APB1EN)|RCC_APB1EN_PMUEN);
    bool pmu_ok=(RD(RCC->APB1EN)&RCC_APB1EN_PMUEN) && !RD(PMU->CR1) && !RD(PMU->CR2) && !RD(PMU->CR3);
    WR(RCC->APB1EN,RD(RCC->APB1EN)&~RCC_APB1EN_PMUEN);
    if (RD(RCC->APB1EN)!=(UARTS|RCC_APB1EN_LPTIM2EN)) { report->error=1; return W600_STOP_FAILED; }
    if (!pmu_ok) return W600_STOP_SKIPPED;

    uint32_t ext0=RD(EXTI->EXTICR[0]),ext1=RD(EXTI->EXTICR[1]);
    uint32_t rising=RD(EXTI->RTSR),falling=RD(EXTI->FTSR),imr=RD(EXTI->IMR);
    WR(EXTI->EXTICR[0],(ext0&~0x770u)|0x10u);
    WR(EXTI->EXTICR[1],(ext1&~0x707u)|1u);
    WR(EXTI->RTSR,rising&~GPIO_LINES);
    WR(EXTI->FTSR,falling|GPIO_LINES);
    WR(EXTI->IMR,imr|GPIO_LINES|TIMER_LINE);
    WR(LPTIM2->IER,LPTIM_IER_ARRM_IE);
    WR(NVIC->ISER[0],WAKE_IRQS);
    bool configured=RD(EXTI->EXTICR[0])==((ext0&~0x770u)|0x10u) &&
        RD(EXTI->EXTICR[1])==((ext1&~0x707u)|1u) &&
        RD(EXTI->RTSR)==(rising&~GPIO_LINES) && RD(EXTI->FTSR)==(falling|GPIO_LINES) &&
        RD(EXTI->IMR)==(imr|GPIO_LINES|TIMER_LINE) &&
        RD(LPTIM2->IER)==LPTIM_IER_ARRM_IE && RD(NVIC->ISER[0])==(UART_IRQ|WAKE_IRQS);
    struct w600_app_time before;
    uint32_t start=0,elapsed=0;
    bool entered=false,ok=true;
    if (!configured) { report->error=2; ok=false; }
    else if (point(ticks,&before,&start) && start<RD(LPTIM2->ARR)/2u &&
        !RD(LPTIM2->ISR) && !RD(EXTI->PIR) && !RD(NVIC->ISPR[0]) &&
        !(RD(SCB->ICSR)&(SCB_ICSR_PENDSTSET_Msk|SCB_ICSR_PENDSVSET_Msk|SCB_ICSR_NMIPENDSET_Msk)) && lines_idle()) {
        WR(SysTick->CTRL,0);
        WR(SCB->SCR,SCB_SCR_SLEEPDEEP_Msk);
        if (!RD(SysTick->CTRL) && RD(SCB->SCR)==SCB_SCR_SLEEPDEEP_Msk) {
            ++report->entered; entered=true;
            __DSB(); __WFI(); __ISB();
        } else { report->error=3; ok=false; }
        WR(SCB->SCR,0);
        if (!restore_clock() || RD(SCB->SCR)) { report->error=4; ok=false; }
        uint32_t end=RD(LPTIM2->CNT),flags=RD(LPTIM2->ISR),again=RD(LPTIM2->CNT);
        for (unsigned i=0;i<8u && again!=end;++i) {
            end=again;flags=RD(LPTIM2->ISR);again=RD(LPTIM2->CNT);
        }
        uint32_t top=RD(LPTIM2->ARR);
        if (end!=again || end>top || (flags&~2u) || (!(flags&2u) && end<start)) {
            report->error=5;ok=false;
        } else {
            /* ARRM asserts at ARR, one tick before the counter wraps to zero. */
            elapsed=end-start;
            if ((flags&2u) && end!=top) elapsed=top+1u-start+end;
            if (elapsed>s->hz) { report->error=5;ok=false; }
        }
    }
    WR(LPTIM2->IER,0);
    WR(EXTI->IMR,imr);
    report->wake_lines=(RD(EXTI->PIR)&GPIO_LINES)|((RD(LPTIM2->ISR)&2u) ? TIMER_LINE : 0u);
    WR(EXTI->PIR,GPIO_LINES);
    WR(LPTIM2->ICR,LPTIM_ICR_ARRM_CF);
    WR(NVIC->ICER[0],WAKE_IRQS);
    WR(NVIC->ICPR[0],WAKE_IRQS);
    WR(EXTI->EXTICR[0],ext0);WR(EXTI->EXTICR[1],ext1);
    WR(EXTI->RTSR,rising);WR(EXTI->FTSR,falling);
    if (RD(LPTIM2->IER) || RD(EXTI->IMR)!=imr || RD(NVIC->ISER[0])!=UART_IRQ ||
        RD(EXTI->EXTICR[0])!=ext0 || RD(EXTI->EXTICR[1])!=ext1 ||
        RD(EXTI->RTSR)!=rising || RD(EXTI->FTSR)!=falling || (RD(NVIC->ISPR[0])&WAKE_IRQS)) {
        report->error=6;ok=false;
    }
    if (!(RD(SysTick->CTRL)&7u)) {
        if (!ok) return W600_STOP_FAILED;
        if (!w600_app_resume(ticks,&before,elapsed,s->hz)) { report->error=7; return W600_STOP_FAILED; }
    }
    if (!ok) return W600_STOP_FAILED;
    /* Also hold awake if PA10 interrupted setup and prevented STOP entry. */
    if (report->wake_lines&(1u<<10)) { ++report->radio_wakes; report->last_radio_at=*ticks; }
    if (entered) {
        ++report->woke; report->elapsed=elapsed;
        if (report->wake_lines&((1u<<1)|(1u<<2)|(1u<<8))) ++report->button_wakes;
        if (report->wake_lines&TIMER_LINE) ++report->timer_wakes;
        return W600_STOP_WOKE;
    }
    return W600_STOP_SKIPPED;
}
