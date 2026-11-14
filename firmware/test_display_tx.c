#include "display_tx.h"
#include "display_tx_test_mmio.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

struct display_test_rcc display_test_rcc;
struct display_test_nvic display_test_nvic;
uint32_t display_test_mask, display_test_ipsr, display_test_control;
static struct display_test_gpio gpio;
static struct display_test_uart uart;
static struct display_test_flash flash;
static struct w600_display_tx s;
static uint8_t output[32];
static unsigned output_size, icr_count, ignore_icr, auto_tc, cases;
static unsigned gpio_reads, uart_reads, flash_reads;
static const uint8_t frame[4] = {0x37, 0x7b, 0xa5, 0xe9};

struct display_test_gpio *display_test_gpio(void)
{
    assert(display_test_rcc.IOPEN & RCC_IOPEN_GPIOBEN);
    ++gpio_reads; return &gpio;
}
struct display_test_uart *display_test_uart(void)
{
    assert(display_test_rcc.APB1EN & RCC_APB1EN_UART2EN);
    ++uart_reads; return &uart;
}
struct display_test_flash *display_test_flash(void)
{
    assert(display_test_rcc.AHBEN & RCC_AHBEN_FLASHEN);
    ++flash_reads; return &flash;
}
void display_test_icr(uint32_t bits)
{
    assert(display_test_rcc.APB1EN & RCC_APB1EN_UART2EN);
    assert(bits == USART_ICR_TCCF);
    ++icr_count;
    if (!ignore_icr) uart.ISR &= ~bits;
}
void display_test_tdr(uint8_t byte)
{
    assert(display_test_rcc.APB1EN & RCC_APB1EN_UART2EN);
    assert(uart.ISR & USART_ISR_TXE);
    assert(output_size < sizeof output);
    output[output_size++] = byte;
    uart.ISR &= ~(uint32_t)USART_ISR_TC;
    if (auto_tc) uart.ISR |= USART_ISR_TC;
}

static void prepared(void)
{
    memset(&s, 0, sizeof s); memset(&uart, 0, sizeof uart);
    memset(&gpio, 0, sizeof gpio); memset(&display_test_nvic, 0, sizeof display_test_nvic);
    display_test_rcc = (struct display_test_rcc){
        .CFG=0x12, .PLLCFG=0x30260, .CSR1=0x03000400,
        .AHBEN=0x100, .IOPEN=2, .APB1EN=0x60000};
    gpio.MODE=2u<<12; gpio.OTYPE=1u<<6; gpio.PUPD=2u<<12; gpio.AFL=1u<<24;
    flash.ACR=2; uart.CR1=0x1609; uart.BRR=2500;
    uart.ISR=USART_ISR_TEACK | USART_ISR_TXE | USART_ISR_TC;
    display_test_mask=1; display_test_ipsr=display_test_control=0;
    output_size=icr_count=ignore_icr=0; auto_tc=1;
    gpio_reads=uart_reads=flash_reads=0;
}
static void setup(uint32_t now)
{
    prepared(); assert(w600_display_tx_init(&s, now));
    assert(s.initialized && !s.error && s.phase==W600_DISPLAY_TX_IDLE);
    assert(!output_size && !icr_count && uart.CR1==0x1609 && !uart.CR3);
    display_test_mask=0;
}
static bool poll(uint32_t now)
{
    unsigned before=output_size;
    bool ok=w600_display_tx_poll(&s, now);
    assert(output_size-before<=1);
    return ok;
}
static void five(uint32_t now)
{
    assert(w600_display_tx_submit(&s, frame, now));
    for (unsigned i=0; i<5; ++i) assert(poll(now));
    assert(output_size==5 && s.phase==W600_DISPLAY_TX_DATA_TC);
}
static void failed(enum w600_display_tx_error error, uint32_t now)
{
    unsigned writes=output_size+icr_count;
    assert(s.error==error && s.phase==W600_DISPLAY_TX_FAILED);
    assert(!poll(now)); assert(!w600_display_tx_submit(&s, frame, now));
    assert(!w600_display_tx_init(&s, now));
    assert(output_size+icr_count==writes && s.error==error);
}

int main(void)
{
    setup(0);
    uint8_t copied[4]; memcpy(copied, frame, 4);
    assert(w600_display_tx_submit(&s, copied, 0)); memset(copied, 0, 4);
    assert(!output_size && icr_count==1);
    for (unsigned i=0; i<5; ++i) assert(poll(i));
    assert(poll(10) && s.phase==W600_DISPLAY_TX_GAP);
    assert(poll(14) && output_size==5);
    assert(poll(15) && output_size==6 && icr_count==2);
    assert(poll(16) && output_size==7 && s.phase==W600_DISPLAY_TX_FINAL_TC);
    assert(poll(17) && s.phase==W600_DISPLAY_TX_IDLE);
    const uint8_t golden[7]={8,0x37,0x7b,0xa5,0xe9,0x18,0xff};
    assert(!memcmp(output, golden, sizeof golden));
    assert(poll(18) && output_size==7);
    assert(w600_display_tx_submit(&s, frame, 18)); ++cases;

    setup(0); display_test_nvic.ISER[0]=display_test_nvic.ISPR[0]=1u<<27;
    assert(w600_display_tx_submit(&s, frame, 0) && poll(0)); ++cases;

    setup(0); five(0);
    struct w600_display_tx before=s;
    assert(!w600_display_tx_submit(&s, frame, UINT32_MAX));
    assert(!w600_display_tx_submit(&s, NULL, UINT32_MAX));
    assert(!w600_display_tx_init(&s, UINT32_MAX));
    assert(!memcmp(&s, &before, sizeof s) && output_size==5); ++cases;
    setup(0); before=s;
    assert(!w600_display_tx_submit(&s, NULL, 0));
    assert(!memcmp(&s, &before, sizeof s));
    assert(!w600_display_tx_init(NULL, 0));
    assert(!w600_display_tx_submit(NULL, frame, 0));
    assert(!w600_display_tx_poll(NULL, 0)); ++cases;

    /* Both qualified bus profiles keep APB1 at48MHz. */
    prepared(); display_test_rcc.CFG=0x40012;
    assert(w600_display_tx_init(&s, 0)); ++cases;
    /* Each context failure is checked before any transfer write. */
    for (unsigned which=0; which<24; ++which) {
        setup(0);
        switch (which) {
        case 0: display_test_ipsr=15; break;
        case 1: display_test_control=1; break;
        case 2: display_test_nvic.ISER[0]=1u<<26; break;
        case 3: display_test_nvic.ISPR[0]=1u<<26; break;
        case 4: display_test_rcc.CFG=0; break;
        case 5: display_test_rcc.PLLCFG=0x20260; break;
        case 6: display_test_rcc.CSR1&=~(uint32_t)RCC_CSR1_RCHRDY; break;
        case 7: display_test_rcc.CSR1&=~(uint32_t)RCC_CSR1_PLLON; break;
        case 8: display_test_rcc.CSR1&=~(uint32_t)RCC_CSR1_PLLRDY; break;
        case 9: display_test_rcc.CSR1|=0x800; break;
        case 10: display_test_rcc.AHBEN=0; break;
        case 11: flash.ACR=0; break;
        case 12: display_test_rcc.IOPEN=0; break;
        case 13: gpio.MODE=0; break;
        case 14: gpio.OTYPE=0; break;
        case 15: gpio.PUPD=0; break;
        case 16: gpio.AFL=0; break;
        case 17: display_test_rcc.APB1EN=0; break;
        case 18: uart.CR1|=4; break;
        case 19: uart.CR2=0x2000; break;
        case 20: uart.CR3=0x80; break;
        case 21: uart.BRR=2499; break;
        case 22: uart.ISR&=~(uint32_t)USART_ISR_TEACK; break;
        default: uart.ISR|=USART_ISR_REACK; break;
        }
        assert(!w600_display_tx_submit(&s, frame, 0));
        assert(!output_size && !icr_count);
        failed(W600_DISPLAY_TX_CONTEXT, 1); ++cases;
    }
    prepared(); display_test_mask=0;
    assert(!w600_display_tx_init(&s, 0) && !s.initialized);
    assert(s.error==W600_DISPLAY_TX_CONTEXT && !icr_count); ++cases;
    for (unsigned bit=0; bit<6; ++bit) {
        if (bit==4) continue;
        setup(0); uart.ISR|=1u<<bit;
        assert(!w600_display_tx_submit(&s, frame, 0));
        failed(W600_DISPLAY_TX_UART, 1); ++cases;
    }

    setup(0); ignore_icr=1;
    assert(!w600_display_tx_submit(&s, frame, 0) && !output_size);
    failed(W600_DISPLAY_TX_UART, 1); ++cases;
    setup(0); five(0); assert(poll(1)); ignore_icr=1;
    assert(!poll(6) && output_size==5);
    failed(W600_DISPLAY_TX_UART, 7); ++cases;

    /* No stale initial TC can complete a data phase whose final byte is busy. */
    setup(0); auto_tc=0; five(0);
    assert(poll(1) && s.phase==W600_DISPLAY_TX_DATA_TC);
    assert(poll(49) && output_size==5);
    assert(!poll(50)); failed(W600_DISPLAY_TX_TIMEOUT, 51); ++cases;
    setup(0); five(0); assert(poll(1)); assert(poll(6));
    auto_tc=0; assert(poll(7) && output_size==7);
    assert(poll(8) && s.phase==W600_DISPLAY_TX_FINAL_TC);
    assert(!poll(50)); failed(W600_DISPLAY_TX_TIMEOUT, 51); ++cases;
    setup(0); assert(w600_display_tx_submit(&s, frame, 0));
    uart.ISR&=~(uint32_t)USART_ISR_TXE;
    assert(poll(49) && !output_size); assert(!poll(50));
    failed(W600_DISPLAY_TX_TIMEOUT, 51); ++cases;
    setup(0); assert(w600_display_tx_submit(&s, frame, 0));
    assert(!poll(50) && !output_size && icr_count==1); ++cases;
    setup(0); five(0); assert(poll(1)); assert(poll(6));
    uart.ISR&=~(uint32_t)USART_ISR_TXE;
    assert(poll(49) && output_size==6); assert(!poll(50));
    failed(W600_DISPLAY_TX_TIMEOUT, 51); ++cases;
    setup(0); five(0); assert(poll(41)); assert(poll(46)); assert(poll(47));
    assert(poll(49) && s.phase==W600_DISPLAY_TX_IDLE); ++cases;
    setup(0); five(0); assert(poll(41)); assert(poll(46)); assert(poll(47));
    assert(!poll(50) && output_size==7);
    failed(W600_DISPLAY_TX_TIMEOUT, 51); ++cases;

    setup(UINT32_MAX-3u); five(UINT32_MAX-3u);
    assert(poll(UINT32_MAX-2u)); assert(poll(1) && output_size==5);
    assert(poll(2) && output_size==6); assert(poll(3)); assert(poll(4));
    assert(s.phase==W600_DISPLAY_TX_IDLE && !s.error); ++cases;
    setup(10); assert(w600_display_tx_submit(&s, frame, 10));
    assert(!poll(9)); failed(W600_DISPLAY_TX_CLOCK, 10); ++cases;
    setup(10); assert(!w600_display_tx_submit(&s, frame, 9));
    failed(W600_DISPLAY_TX_CLOCK, 10); ++cases;
    setup(0); assert(w600_display_tx_submit(&s, frame, 0));
    assert(!poll(UINT32_C(0x80000000))); failed(W600_DISPLAY_TX_CLOCK, 1); ++cases;
    setup(0); assert(w600_display_tx_submit(&s, frame, 0));
    uart.ISR&=~(uint32_t)USART_ISR_TXE;
    for (unsigned i=1; i<W600_DISPLAY_TX_STALLED_POLLS; ++i) assert(poll(0));
    assert(!poll(0)); failed(W600_DISPLAY_TX_CLOCK, 1); ++cases;
    setup(0);
    for (unsigned i=0; i<W600_DISPLAY_TX_STALLED_POLLS+1u; ++i) assert(poll(0));
    assert(!s.error && !output_size && !icr_count); ++cases;

    /* Context loss after a partial wire write must not disable/repair UART2. */
    setup(0); assert(w600_display_tx_submit(&s, frame, 0)); assert(poll(0));
    display_test_rcc.APB1EN=0; unsigned reads=uart_reads;
    assert(!poll(1) && uart_reads==reads && output_size==1 && uart.CR1==0x1609);
    failed(W600_DISPLAY_TX_CONTEXT, 2); ++cases;
    setup(0); assert(w600_display_tx_submit(&s, frame, 0)); assert(poll(0));
    display_test_ipsr=15; assert(!poll(1)); display_test_ipsr=0;
    uart.ISR|=8; failed(W600_DISPLAY_TX_CONTEXT, 2); ++cases;
    for (unsigned which=0; which<4; ++which) {
        setup(0); assert(w600_display_tx_submit(&s, frame, 0));
        s.phase=which<2 ? W600_DISPLAY_TX_DATA : W600_DISPLAY_TX_CONTROL;
        s.position=(uint8_t)(which%2 ? 255u : (which<2 ? 5u : 2u));
        unsigned writes=icr_count;
        assert(!poll(0) && !output_size && icr_count==writes);
        failed(W600_DISPLAY_TX_CONTEXT, 1); ++cases;
    }
    printf("display_tx: %u cases PASS\n", cases);
    return 0;
}
