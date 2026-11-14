#include <stddef.h>
#include "ciu32l071.h"
#include "ciu32_startup.h"
#include "w600_board.h"
#ifndef BOARD_OBSERVE
#define BOARD_OBSERVE(stage) ((void)(stage))
#endif

_Static_assert(sizeof(struct w600_board_status) == 128, "Board mailbox layout");
enum { BOARD_LATCHED=100, BOARD_GATED, BOARD_RESET_ON, BOARD_RESET_OFF,
       BOARD_PINS, BOARD_CONFIGURED, BOARD_WAKE_LATCHED, BOARD_SETTLE_ARMED };

static void board_hold_off(void)
{
    GPIOC->BR=1u<<10; GPIOA->BR=1u<<15; GPIOB->BSR=0xc000u;
}
static int board_off(void)
{
    return (RCC->IOPEN&7u)==7u && (GPIOB->MODE&0xf0000000u)==0x50000000u &&
        (GPIOA->MODE&(3u<<30))==(1u<<30) &&
        (GPIOC->MODE&(3u<<20))==(1u<<20) &&
        !(GPIOB->OTYPE&0xc000u) && !(GPIOA->OTYPE&(1u<<15)) &&
        !(GPIOC->OTYPE&(1u<<10)) &&
        (GPIOB->ODR&0xc000u)==0xc000u && (GPIOB->IDR&0xc000u)==0xc000u &&
        !(GPIOA->ODR&(1u<<15)) && !(GPIOC->ODR&(1u<<10)) &&
        !(GPIOA->IDR&(1u<<15)) && !(GPIOC->IDR&(1u<<10));
}
static int uart_wait(USART_t *uart,uint32_t mask,uint32_t value,unsigned stage)
{
    for(unsigned i=0;i<100000;++i) {
        board_hold_off(); BOARD_OBSERVE(stage);
        if((uart->ISR&mask)==value) return 1;
    }
    return 0;
}
static int stop_uart(USART_t *uart,uint32_t expected_brr,volatile uint32_t *last_isr)
{
    if(uart->CR1&USART_CR1_UE) {
        if(RCC->CFG!=0x12 || RCC->PLLCFG!=0x30260 ||
           (FLASH->ACR&FLASH_ACR_LATENCY)!=FLASH_ACR_LATENCY_2CLK ||
           uart->BRR!=expected_brr || uart->CR2 || uart->CR3 ||
           (uart->CR1&USART_CR1_OVS8)) return W600_BOARD_CONTEXT;
        if(!uart_wait(uart,USART_ISR_TXE,USART_ISR_TXE,W600_BOARD_UART_BUSY))
            return W600_BOARD_UART_BUSY;
        /* Stock clears TC after sending; TXE plus a known-clock settle is needed. */
        SysTick->LOAD=480000u-1u; SysTick->VAL=0;
        SysTick->CTRL=SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_ENABLE_Msk;
        BOARD_OBSERVE(BOARD_SETTLE_ARMED);
        unsigned i=2000000;
        if(SysTick->LOAD==479999u &&
           (SysTick->CTRL&7u)==(SysTick_CTRL_CLKSOURCE_Msk|SysTick_CTRL_ENABLE_Msk)) {
            for(i=0;i<2000000;++i) {
                board_hold_off(); BOARD_OBSERVE(W600_BOARD_UART_SETTLE);
                if(SysTick->CTRL&SysTick_CTRL_COUNTFLAG_Msk) break;
            }
        }
        SysTick->CTRL=0; SysTick->LOAD=0; SysTick->VAL=0;
        if(i==2000000) return W600_BOARD_UART_SETTLE;
    }
    *last_isr=uart->ISR;
    uart->CR1=0;
    if(!uart_wait(uart,USART_ISR_TEACK|USART_ISR_REACK,0,W600_BOARD_UART_STOP) ||
       uart->CR1!=0) return W600_BOARD_UART_STOP;
    return W600_BOARD_OK;
}
int w600_board_init(volatile struct w600_board_status *s)
{
    volatile uint32_t *words=(volatile uint32_t *)s;
    for(unsigned i=0;i<sizeof(*s)/4;++i) words[i]=0;
    s->magic=0x57364244u; s->version=1; s->size=sizeof(*s); s->stage=1;
    s->clock_result=0xffffffffu;
    int owns=0, own_uart=0;
    if(!__get_PRIMASK() || __get_IPSR()!=0 || __get_CONTROL()!=0 ||
       NVIC->ISER[0] || (SysTick->CTRL&SysTick_CTRL_ENABLE_Msk) || !board_off() ||
       (RCC->APB1EN&(RCC_APB1EN_LPUART1EN|RCC_APB1EN_LPUART2EN)) ||
       ((RCC->APB1EN&RCC_APB1EN_UART4EN) && UART4->CR1!=0) ||
       (RCC->APB2EN&RCC_APB2EN_USART1EN) ||
       ((RCC->AHBEN&RCC_AHBEN_DMAEN) &&
        ((DMA->channel[0].CC|DMA->channel[1].CC)&DMA_CC_EN))) {
        s->error=W600_BOARD_CONTEXT; goto finish;
    }
    owns=1;
    board_hold_off(); BOARD_OBSERVE(BOARD_LATCHED);
    s->cfg_before=RCC->CFG; s->apb1_before=RCC->APB1EN;
    s->apb2_before=RCC->APB2EN; s->ahb_before=RCC->AHBEN;
    if(RCC->APB1EN&RCC_APB1EN_UART2EN) {
        s->uart2_before=UART2->CR1;
        s->error=stop_uart(UART2,2500,&s->uart2_isr);
        if(s->error) goto finish;
    }
    if(RCC->APB1EN&RCC_APB1EN_UART3EN) {
        s->uart3_before=UART3->CR1;
        s->error=stop_uart(UART3,417,&s->uart3_isr);
        if(s->error) goto finish;
    }
    RCC->APB1EN=0; RCC->APB2EN&=RCC_APB2EN_DBGEN;
    RCC->AHBEN&=~RCC_AHBEN_DMAEN;
    __DSB(); BOARD_OBSERVE(BOARD_GATED);
    if(RCC->APB1EN || (RCC->APB2EN&~RCC_APB2EN_DBGEN) ||
       (RCC->AHBEN&RCC_AHBEN_DMAEN)) { s->error=W600_BOARD_CLOCK_GATE; goto finish; }
    s->clock_result=ciu32_startup_48mhz();
    if(s->clock_result!=CIU32_STARTUP_OK) { s->error=W600_BOARD_CLOCK; goto finish; }
    s->stage=2;
    RCC->IOPEN|=RCC_IOPEN_GPIODEN;
    RCC->APB1EN|=RCC_APB1EN_UART3EN;
    (void)RCC->APB1EN; __DSB();
    if(!(RCC->IOPEN&RCC_IOPEN_GPIODEN) || !(RCC->APB1EN&RCC_APB1EN_UART3EN)) {
        s->error=W600_BOARD_CLOCK_GATE; goto finish;
    }
    RCC->APB1RST|=RCC_APB1RST_UART3_RST;
    __DSB(); BOARD_OBSERVE(BOARD_RESET_ON);
    if(!(RCC->APB1RST&RCC_APB1RST_UART3_RST)) { s->error=W600_BOARD_UART_RESET; goto finish; }
    RCC->APB1RST&=~RCC_APB1RST_UART3_RST;
    __DSB(); BOARD_OBSERVE(BOARD_RESET_OFF);
    if(RCC->APB1RST&RCC_APB1RST_UART3_RST) { s->error=W600_BOARD_UART_RESET; goto finish; }
    own_uart=1;

    GPIOA->BSR=1u<<11;
    BOARD_OBSERVE(BOARD_WAKE_LATCHED);
    GPIOA->OTYPE&=~(1u<<11);
    GPIOA->PUPD=(GPIOA->PUPD&~(15u<<20))|(5u<<20);
    GPIOA->MODE=(GPIOA->MODE&~(15u<<20))|(1u<<22);
    GPIOD->OTYPE&=~3u;
    GPIOD->PUPD=(GPIOD->PUPD&~15u)|5u;
    GPIOD->AFL=(GPIOD->AFL&~255u)|0x77u;
    GPIOD->MODE=(GPIOD->MODE&~15u)|10u;
    __DSB(); BOARD_OBSERVE(BOARD_PINS);
    if((GPIOA->MODE&(15u<<20))!=(1u<<22) || (GPIOA->OTYPE&(1u<<11)) ||
       (GPIOA->PUPD&(15u<<20))!=(5u<<20) || !(GPIOA->ODR&(1u<<11)) ||
       !(GPIOA->IDR&(1u<<11)) || (GPIOD->MODE&15u)!=10u ||
       (GPIOD->AFL&255u)!=0x77u || (GPIOD->OTYPE&3u) || (GPIOD->PUPD&15u)!=5u) {
        s->error=W600_BOARD_PIN_CONFIG; goto finish;
    }
    UART3->CR1=0; UART3->CR2=0; UART3->CR3=0; UART3->BRR=0x1a1;
    UART3->ICR=15u;
    BOARD_OBSERVE(BOARD_CONFIGURED);
    if(UART3->CR1 || UART3->CR2 || UART3->CR3 || UART3->BRR!=0x1a1) {
        s->error=W600_BOARD_UART_CONFIG; goto finish;
    }
    UART3->CR1=USART_CR1_UE|USART_CR1_RE|USART_CR1_TE;
    if(!uart_wait(UART3,USART_ISR_TEACK|USART_ISR_REACK,
                  USART_ISR_TEACK|USART_ISR_REACK,W600_BOARD_UART_READY) || UART3->CR1!=0xd) {
        s->error=W600_BOARD_UART_READY; goto finish;
    }
    s->uart3_cr1=UART3->CR1; s->uart3_cr2=UART3->CR2;
    s->uart3_cr3=UART3->CR3; s->uart3_brr=UART3->BRR;
finish:
    if(owns) board_hold_off();
    s->cfg_after=RCC->CFG; s->final_off=board_off();
    if(!s->final_off && !s->error) s->error=W600_BOARD_PIN_CONFIG;
    if(own_uart && s->error) {
        UART3->CR1=0;
        GPIOA->BSR=1u<<11;
        __DSB();
    }
    s->stage=s->error ? 0xe0u : 3u;
    __DMB();
    return !s->error;
}
