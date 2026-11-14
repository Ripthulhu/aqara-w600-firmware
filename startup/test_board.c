/* Reuse the exact clock source/model rather than inventing a second clock fake. */
#define main clock_tests_main
#include "test_startup.c"
#undef main

static GPIO_t gpiod;
static USART_t uart2,uart3,uart4;
static DMA_t dma;
static unsigned board_counts[128];
static int board_fault, bad_tick_config, stuck_ue, stuck_reset, lose_motor_mode;
static void board_observe(unsigned stage);
static USART_t *get_uart(unsigned n)
{
    assert(rcc.APB1EN&(1u<<(n+15u)));
    return n==2 ? &uart2 : n==3 ? &uart3 : &uart4;
}
#undef GPIOD
#undef UART2
#undef UART3
#undef UART4
#undef DMA
#define GPIOD (&gpiod)
#define UART2 get_uart(2)
#define UART3 get_uart(3)
#define UART4 get_uart(4)
#define DMA (&dma)
#define __DMB() ((void)0)
#define BOARD_OBSERVE(stage) board_observe(stage)
#include "w600_board.c"

static void set_isr(USART_t *uart,uint32_t value)
{ *(volatile uint32_t *)&uart->ISR=value; }
static void board_observe(unsigned stage)
{
    assert(stage<128); ++board_counts[stage];
    apply_gpio(&gpioa); apply_gpio(&gpiob); apply_gpio(&gpioc); apply_gpio(&gpiod);
    assert(board_off());
    if(stage==W600_BOARD_UART_BUSY) {
        if(board_fault!=(int)stage) {
            set_isr(&uart2,uart2.ISR|USART_ISR_TXE);
            set_isr(&uart3,uart3.ISR|USART_ISR_TXE);
        }
        assert(!(uart2.ISR&USART_ISR_TC) && !(uart3.ISR&USART_ISR_TC));
    } else if(stage==W600_BOARD_UART_SETTLE) {
        assert(rcc.CFG==0x12 && rcc.PLLCFG==0x30260);
        assert(systick.LOAD==479999u && systick.CTRL==5u);
        if(board_fault!=(int)stage) systick.CTRL|=SysTick_CTRL_COUNTFLAG_Msk;
    } else if(stage==BOARD_SETTLE_ARMED) {
        if(bad_tick_config) systick.LOAD=0;
    } else if(stage==W600_BOARD_UART_STOP) {
        assert(systick.CTRL==0);
        uint32_t ack=USART_ISR_TEACK|USART_ISR_REACK;
        set_isr(&uart2,board_fault==(int)stage ? ack : 0);
        set_isr(&uart3,board_fault==(int)stage ? ack : 0);
        if(stuck_ue) uart2.CR1=0xd;
    } else if(stage==BOARD_GATED) {
        assert(systick.CTRL==0 && !(uart2.CR1|uart3.CR1));
        if(board_fault==W600_BOARD_CLOCK_GATE) rcc.APB1EN=1;
    } else if(stage==BOARD_RESET_ON) {
        assert(rcc.APB1EN==RCC_APB1EN_UART3EN && (rcc.APB1RST&RCC_APB1RST_UART3_RST));
        uart3.CR1=uart3.CR2=uart3.CR3=uart3.BRR=0; set_isr(&uart3,0);
        if(board_fault==W600_BOARD_UART_RESET) rcc.APB1RST=0;
    } else if(stage==BOARD_RESET_OFF) {
        assert(!(rcc.APB1RST&RCC_APB1RST_UART3_RST));
        if(stuck_reset) rcc.APB1RST|=RCC_APB1RST_UART3_RST;
    } else if(stage==BOARD_WAKE_LATCHED) {
        assert(gpioa.ODR&(1u<<11));
        assert((gpioa.MODE&(3u<<22))==(3u<<22));
    } else if(stage==BOARD_PINS) {
        if(board_fault==W600_BOARD_PIN_CONFIG) gpiod.AFL^=1;
    } else if(stage==BOARD_CONFIGURED) {
        if(board_fault==W600_BOARD_UART_CONFIG) uart3.BRR=1;
    } else if(stage==W600_BOARD_UART_READY) {
        assert(uart3.CR1==0xd);
        if(board_fault!=(int)stage) set_isr(&uart3,USART_ISR_TEACK|USART_ISR_REACK|USART_ISR_TXE);
        if(lose_motor_mode) gpiob.MODE=0;
    }
}
static void reset_board(void)
{
    reset_model(1);
    memset(&gpiod,0,sizeof gpiod); memset(&uart2,0,sizeof uart2);
    memset(&uart3,0,sizeof uart3); memset(&uart4,0,sizeof uart4);
    memset(&dma,0,sizeof dma); memset(board_counts,0,sizeof board_counts);
    rcc.IOPEN=7; rcc.APB1EN=RCC_APB1EN_UART2EN|RCC_APB1EN_UART3EN|RCC_APB1EN_UART4EN;
    rcc.APB2EN=RCC_APB2EN_DBGEN; rcc.AHBEN=RCC_AHBEN_DMAEN;
    gpioa.MODE=(gpioa.MODE&~(3u<<30))|(1u<<30);
    gpioc.MODE=(gpioc.MODE&~(3u<<20))|(1u<<20);
    gpiob.MODE=(gpiob.MODE&~0xf0000000u)|0x50000000u;
    gpioa.OTYPE=gpiob.OTYPE=gpioc.OTYPE=0;
    gpioa.ODR&=~(1u<<15); gpioc.ODR&=~(1u<<10); gpiob.ODR|=0xc000u;
    apply_gpio(&gpioa); apply_gpio(&gpiob); apply_gpio(&gpioc);
    initial_mode[0]=gpioa.MODE; initial_mode[1]=gpiob.MODE; initial_mode[2]=gpioc.MODE;
    uart2.CR1=uart3.CR1=0xd; uart2.BRR=2500; uart3.BRR=417;
    set_isr(&uart2,USART_ISR_TEACK|USART_ISR_REACK);
    set_isr(&uart3,USART_ISR_TEACK|USART_ISR_REACK);
    board_fault=-1; bad_tick_config=stuck_ue=stuck_reset=lose_motor_mode=0;
}
int main(void)
{
    clock_tests_main();
    struct w600_board_status s;
    reset_board();
    assert(w600_board_init(&s) && s.stage==3 && s.error==0 && s.clock_result==0);
    assert(s.final_off && uart3.CR1==0xd && uart3.BRR==417 && !uart3.CR2 && !uart3.CR3);
    assert(rcc.APB1EN==RCC_APB1EN_UART3EN && rcc.APB2EN==RCC_APB2EN_DBGEN);
    assert(!systick.CTRL && !nvic.ISER[0] && primask==1);
    assert(board_counts[W600_BOARD_UART_SETTLE]==2 && !(s.uart2_isr&USART_ISR_TC));
    reset_board();
    rcc.APB1EN=0; rcc.AHBEN=0;
    uart2.CR1=uart3.CR1=0;
    assert(w600_board_init(&s) && board_counts[W600_BOARD_UART_SETTLE]==0);
    for(unsigned i=0;i<5;++i) {
        reset_board();
        if(i==0) uart4.CR1=1;
        if(i==1) dma.channel[0].CC=DMA_CC_EN;
        if(i==2) primask=0;
        if(i==3) nvic.ISER[0]=1;
        if(i==4) gpioa.MODE&=~(3u<<30);
        RCC_t before=rcc;
        assert(!w600_board_init(&s) && s.error==W600_BOARD_CONTEXT);
        assert(!memcmp(&before,&rcc,sizeof rcc) && s.clock_result==0xffffffffu);
    }
    for(unsigned i=0;i<5;++i) {
        reset_board();
        if(i==0) uart2.BRR=1;
        if(i==1) uart2.CR2=1;
        if(i==2) uart2.CR3=1;
        if(i==3) uart2.CR1|=USART_CR1_OVS8;
        if(i==4) rcc.CFG=0;
        assert(!w600_board_init(&s) && s.error==W600_BOARD_CONTEXT);
        assert(s.clock_result==0xffffffffu && !board_counts[BOARD_GATED]);
    }
    const unsigned errors[]={W600_BOARD_UART_BUSY,W600_BOARD_UART_STOP,
        W600_BOARD_UART_SETTLE,W600_BOARD_CLOCK_GATE,W600_BOARD_UART_RESET,
        W600_BOARD_PIN_CONFIG,W600_BOARD_UART_CONFIG,W600_BOARD_UART_READY};
    for(unsigned i=0;i<sizeof errors/sizeof errors[0];++i) {
        reset_board(); board_fault=(int)errors[i];
        assert(!w600_board_init(&s) && s.error==errors[i] && s.stage==0xe0 && s.final_off);
        if(errors[i]==W600_BOARD_UART_BUSY || errors[i]==W600_BOARD_UART_STOP ||
           errors[i]==W600_BOARD_UART_READY) assert(board_counts[errors[i]]==100000);
        if(errors[i]==W600_BOARD_UART_SETTLE) assert(board_counts[errors[i]]==2000000 && !systick.CTRL);
    }
    reset_board(); fault=CIU32_STARTUP_PLL_TIMEOUT;
    assert(!w600_board_init(&s) && s.error==W600_BOARD_CLOCK &&
           s.clock_result==CIU32_STARTUP_PLL_TIMEOUT && s.final_off);
    reset_board(); bad_tick_config=1;
    assert(!w600_board_init(&s) && s.error==W600_BOARD_UART_SETTLE &&
           !board_counts[W600_BOARD_UART_SETTLE] && !systick.CTRL);
    reset_board(); stuck_ue=1;
    assert(!w600_board_init(&s) && s.error==W600_BOARD_UART_STOP && !board_counts[BOARD_GATED]);
    reset_board(); stuck_reset=1;
    assert(!w600_board_init(&s) && s.error==W600_BOARD_UART_RESET && !board_counts[BOARD_PINS]);
    reset_board(); lose_motor_mode=1;
    assert(!w600_board_init(&s) && s.error==W600_BOARD_PIN_CONFIG && !s.final_off && !uart3.CR1);
    puts("Board init: 25 scenarios passed, plus 27 actual clock-source scenarios");
}
