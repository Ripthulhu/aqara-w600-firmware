#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"

static RCC_t rcc;
static GPIO_t a, b, c;
static USART_t uart, radio;
static FLASH_t flash;
static NVIC_Type nvic;
static unsigned mask, ipsr, control, writes, ignore_at, corrupt_at, drop_at, drop_uart_at, last_latch;
static unsigned gpio_reads, uart_reads, ack_reads, ack_mode, stuck_pa4, protect_cr1, cases;
static uint32_t read_reg(volatile const uint32_t *reg);
static void write_reg(volatile uint32_t *reg, uint32_t value);
#undef RCC
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef UART2
#undef UART3
#undef FLASH
#undef NVIC
#define RCC (&rcc)
#define GPIOA (&a)
#define GPIOB (&b)
#define GPIOC (&c)
#define UART2 (&uart)
#define UART3 (&radio)
#define FLASH (&flash)
#define NVIC (&nvic)
#define __get_PRIMASK() mask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#define W600_DISPLAY_BOARD_READ(reg) read_reg(&(reg))
#define W600_DISPLAY_BOARD_WRITE(reg, value) write_reg(&(reg), (uint32_t)(value))
#include "w600_display_board.c"

static bool belongs(volatile const uint32_t *reg, const void *base, size_t size)
{
    uintptr_t p = (uintptr_t)reg, start = (uintptr_t)base;
    return p >= start && p < start + size;
}
static void pads(void)
{
    a.IDR = a.ODR | (stuck_pa4 ? PA4 : 0);
    b.IDR = b.ODR;
    c.IDR = c.ODR;
}
static uint32_t read_reg(volatile const uint32_t *reg)
{
    if (belongs(reg, &a, sizeof a) || belongs(reg, &b, sizeof b) || belongs(reg, &c, sizeof c)) {
        unsigned gate = belongs(reg, &a, sizeof a) ? 1u : belongs(reg, &b, sizeof b) ? 2u : 4u;
        assert(rcc.IOPEN & gate);
        assert(reg != &a.BR && reg != &a.BSR);
        ++gpio_reads;
    } else if (belongs(reg, &uart, sizeof uart)) {
        assert((rcc.APB1EN & RCC_APB1EN_UART2EN) && !(rcc.APB1RST & RCC_APB1RST_UART2_RST));
        ++uart_reads;
        if (reg == &uart.ISR) {
            ++ack_reads;
            if (ack_mode == 1) return uart.ISR & ~(uint32_t)ACK;
            if (ack_mode == 2) return uart.ISR | ACK;
        }
    } else if (belongs(reg, &flash, sizeof flash)) assert(rcc.AHBEN & RCC_AHBEN_FLASHEN);
    else assert(belongs(reg, &rcc, sizeof rcc) || belongs(reg, &nvic, sizeof nvic));
    return *reg;
}
static void write_reg(volatile uint32_t *reg, uint32_t value)
{
    ++writes;
    uint32_t allowed = UINT32_MAX;
    if (reg == &rcc.APB1EN) allowed = RCC_APB1EN_UART2EN;
    else if (reg == &rcc.APB1RST) allowed = RCC_APB1RST_UART2_RST;
    else if (reg == &a.BR || reg == &a.BSR) allowed = PA4;
    else if (reg == &a.MODE || reg == &a.PUPD) allowed = A4;
    else if (reg == &a.OTYPE) allowed = PA4;
    else if (reg == &b.MODE || reg == &b.PUPD) allowed = B6;
    else if (reg == &b.OTYPE) allowed = PB6;
    else if (reg == &b.AFL) allowed = B6_AF;
    else assert(reg == &uart.CR1 || reg == &uart.CR2 || reg == &uart.CR3 || reg == &uart.BRR);
    if (reg == &a.BR || reg == &a.BSR) assert(value == PA4);
    else assert(!((*reg ^ value) & ~allowed));
    if (belongs(reg, &uart, sizeof uart))
        assert((rcc.APB1EN & RCC_APB1EN_UART2EN) && !(rcc.APB1RST & RCC_APB1RST_UART2_RST));
    if (belongs(reg, &a, sizeof a) || belongs(reg, &b, sizeof b)) assert((rcc.IOPEN & 3u) == 3u);
    if (reg == &a.MODE && (value & A4) == (1u << 8)) {
        assert(!(a.OTYPE & PA4) && (a.PUPD & A4) == (1u << 8));
        assert(last_latch < 2 && !!(a.ODR & PA4) == last_latch);
    }
    if (writes == corrupt_at) value ^= allowed & (0u - allowed);
    if (writes != ignore_at) {
        if (reg == &a.BR) { a.ODR &= ~value; last_latch = 0; }
        else if (reg == &a.BSR) { a.ODR |= value; last_latch = 1; }
        else {
            if (reg == &uart.CR1 && protect_cr1 && (uart.CR1 & USART_CR1_UE))
                value = (value & ~0x1600u) | (uart.CR1 & 0x1600u);
            *reg = value;
        }
        if (reg == &rcc.APB1RST && (value & RCC_APB1RST_UART2_RST)) memset(&uart, 0, sizeof uart);
        if (reg == &uart.CR1)
            uart.ISR = (uart.ISR & ~(uint32_t)ACK) |
                ((value & (USART_CR1_UE | USART_CR1_TE)) == (USART_CR1_UE | USART_CR1_TE) ? USART_ISR_TEACK : 0u);
    }
    pads();
    if (writes == drop_at) rcc.IOPEN &= ~1u;
    if (writes == drop_uart_at) rcc.APB1EN &= ~(uint32_t)RCC_APB1EN_UART2EN;
}
static void setup(void)
{
    memset(&rcc, 0xa5, sizeof rcc); memset(&a, 0x96, sizeof a);
    memset(&b, 0x69, sizeof b); memset(&c, 0x5a, sizeof c);
    memset(&uart, 0x39, sizeof uart); memset(&radio, 0x8c, sizeof radio);
    memset(&nvic, 0, sizeof nvic); memset(&flash, 0, sizeof flash);
    lifetime = COLD; mask = 1; ipsr = control = 0;
    writes = ignore_at = corrupt_at = drop_at = drop_uart_at = gpio_reads = uart_reads = ack_reads = ack_mode = stuck_pa4 = protect_cr1 = 0;
    last_latch = 2;
    rcc.CFG = 0x12; rcc.PLLCFG = 0x30260; rcc.CSR1 = 0x03000400;
    rcc.IOPEN = 0x1f; rcc.AHBEN = RCC_AHBEN_FLASHEN;
    rcc.APB1EN = RCC_APB1EN_UART3EN; rcc.APB1RST = RCC_APB1RST_TIM5_RST;
    flash.ACR = 2;
    a.MODE = (a.MODE & ~(A4 | 0xc00u | 0xc0000000u)) | A4 | 0x400u | 0x40000000u;
    b.MODE = (b.MODE & ~(B6 | 0x03000003u | 0xf0000000u)) | B6 | 0x01000001u | 0x50000000u;
    c.MODE = (c.MODE & ~0x00300000u) | 0x00100000u;
    a.OTYPE &= ~(PA4 | 0x20u | 0x8000u); b.OTYPE &= ~(PB6 | 0x1001u | 0xc000u); c.OTYPE &= ~0x400u;
    a.PUPD = (a.PUPD & ~(A4 | 0xc00u | 0xc0000000u)) | 0x800u;
    b.PUPD = (b.PUPD & ~(B6 | 0x03000003u | 0xf0000000u)) | 0x01000002u;
    c.PUPD &= ~0x00300000u;
    a.AFL &= ~(15u << 16); b.AFL &= ~B6_AF;
    a.ODR = (a.ODR & ~0x8020u) | 0x20u;
    b.ODR = (b.ODR & ~0xd001u) | 0xc001u; c.ODR &= ~0x400u;
    pads(); assert(context());
}
static void stopped(void)
{
    assert(!(rcc.APB1EN & RCC_APB1EN_UART2EN) && !(rcc.APB1RST & RCC_APB1RST_UART2_RST));
    assert((b.MODE & B6) == B6 && !(b.OTYPE & PB6) && !(b.PUPD & B6) && !(b.AFL & B6_AF));
    assert((a.MODE & A4) == (1u << 8) && !(a.OTYPE & PA4) && (a.PUPD & A4) == (1u << 8));
    assert((a.ODR & PA4) && (a.IDR & PA4));
}
static void rejected(void)
{
    RCC_t rr = rcc; GPIO_t aa = a, bb = b, cc = c; USART_t uu = uart, radio_old = radio;
    assert(!w600_display_board_init() && !writes && lifetime == COLD);
    assert(w600_display_board_stop() && !writes);
    assert(!memcmp(&rr, &rcc, sizeof rr) && !memcmp(&aa, &a, sizeof aa) && !memcmp(&bb, &b, sizeof bb));
    assert(!memcmp(&cc, &c, sizeof cc) && !memcmp(&uu, &uart, sizeof uu) && !memcmp(&radio_old, &radio, sizeof radio));
    ++cases;
}
int main(void)
{
    setup(); USART_t radio_old = radio; GPIO_t cc = c;
    assert(w600_display_board_init()); unsigned init_writes = writes;
    assert(uart.CR1 == 0x1609 && !uart.CR2 && !uart.CR3 && uart.BRR == 2500);
    assert((b.MODE & B6) == (2u << 12) && (b.OTYPE & PB6) && (b.PUPD & B6) == (2u << 12));
    assert((b.AFL & B6_AF) == (1u << 24) && !(a.ODR & PA4) && !(a.IDR & PA4));
    assert(!w600_display_board_init() && writes == init_writes);
    assert(w600_display_board_stop()); stopped(); unsigned all_writes = writes;
    assert(!w600_display_board_init() && w600_display_board_stop() && writes == all_writes);
    assert(!memcmp(&radio_old, &radio, sizeof radio) && !memcmp(&cc, &c, sizeof c)); ++cases;
    setup(); nvic.ISER[0] = nvic.ISPR[0] = 1u << 27; assert(w600_display_board_init()); ++cases;
    setup(); protect_cr1 = 1; assert(w600_display_board_init());
    assert(w600_display_board_stop()); stopped(); assert(!uart.CR1); ++cases;
    for (unsigned fault = 0; fault < 44; ++fault) {
        setup();
        switch (fault) {
        case 0: mask = 0; break; case 1: ipsr = 15; break; case 2: control = 1; break;
        case 3: nvic.ISER[0] = 1u << 26; break; case 4: nvic.ISPR[0] = 1u << 26; break;
        case 5: rcc.IOPEN &= ~1u; break; case 6: rcc.IOPEN &= ~2u; break; case 7: rcc.IOPEN &= ~4u; break;
        case 8: rcc.APB1EN |= RCC_APB1EN_UART2EN; break; case 9: rcc.APB1RST |= RCC_APB1RST_UART2_RST; break;
        case 10: rcc.CFG = 0x40012; break; case 11: rcc.PLLCFG = 0; break; case 12: rcc.CSR1 |= 0x800u; break;
        case 13: rcc.AHBEN = 0; break; case 14: flash.ACR = 0; break;
        case 15: a.MODE &= ~A4; break; case 16: b.MODE &= ~B6; break;
        case 17: a.OTYPE |= PA4; break; case 18: b.OTYPE |= PB6; break;
        case 19: a.PUPD |= A4; break; case 20: b.PUPD |= B6; break;
        case 21: a.AFL |= 1u << 16; break; case 22: b.AFL |= 1u << 24; break;
        case 23: a.ODR &= ~0x20u; break; case 24: b.ODR |= 0x1000u; break;
        case 25: a.IDR &= ~0x20u; break; case 26: b.IDR &= ~1u; break;
        case 27: c.MODE &= ~0x00300000u; break; case 28: c.ODR |= 0x400u; break;
        case 29: b.IDR &= ~0xc000u; break;
        case 30: rcc.CSR1 &= ~(uint32_t)RCC_CSR1_RCHRDY; break;
        case 31: rcc.CSR1 &= ~(uint32_t)RCC_CSR1_PLLON; break;
        case 32: rcc.CSR1 &= ~(uint32_t)RCC_CSR1_PLLRDY; break;
        case 33: a.MODE &= ~0xc00u; break; case 34: b.MODE &= ~3u; break;
        case 35: b.MODE &= ~0x03000000u; break; case 36: b.OTYPE |= 0xc000u; break;
        case 37: a.PUPD |= 0xc0000000u; break; case 38: c.IDR |= 0x400u; break;
        case 39: c.OTYPE |= 0x400u; break; case 40: b.PUPD |= 0xf0000000u; break;
        case 41: a.IDR |= 0x8000u; break; case 42: a.PUPD &= ~0xc00u; break;
        default: b.PUPD &= ~0x03000003u; break;
        }
        rejected();
    }
    for (unsigned n = 1; n <= init_writes; ++n) {
        setup(); ignore_at = n; bool ok = w600_display_board_init();
        /* Rewriting an already-correct reset value need not fail. */
        (void)ok; ignore_at = 0;
        assert(w600_display_board_stop()); stopped(); ++cases;
        setup(); corrupt_at = n; assert(!w600_display_board_init()); corrupt_at = 0;
        assert(w600_display_board_stop()); stopped(); ++cases;
        setup(); drop_at = n; assert(!w600_display_board_init()); drop_at = 0;
        rcc.IOPEN |= 1u; assert(w600_display_board_stop()); stopped(); ++cases;
        setup(); drop_uart_at = n; assert(!w600_display_board_init()); drop_uart_at = 0;
        assert(w600_display_board_stop()); stopped(); ++cases;
    }
    setup(); ack_mode = 2; assert(!w600_display_board_init() && ack_reads == POLL_LIMIT);
    ack_mode = 0; assert(w600_display_board_stop()); stopped(); ++cases;
    setup(); ack_mode = 1; assert(!w600_display_board_init() && ack_reads >= POLL_LIMIT);
    ack_mode = 0; assert(w600_display_board_stop()); stopped(); ++cases;
    setup(); stuck_pa4 = 1; assert(!w600_display_board_init());
    stuck_pa4 = 0; assert(w600_display_board_stop()); stopped(); ++cases;
    setup(); assert(w600_display_board_init()); ack_mode = 2;
    assert(!w600_display_board_stop()); stopped(); ack_mode = 0;
    assert(w600_display_board_stop()); ++cases;
    for (unsigned n = 1; n <= all_writes - init_writes; ++n) {
        setup(); assert(w600_display_board_init()); ignore_at = writes + n;
        (void)w600_display_board_stop(); ignore_at = 0;
        assert(w600_display_board_stop()); stopped(); ++cases;
        setup(); assert(w600_display_board_init()); drop_at = writes + n;
        assert(!w600_display_board_stop()); drop_at = 0; rcc.IOPEN |= 1u;
        assert(w600_display_board_stop()); stopped(); ++cases;
    }
    setup(); assert(w600_display_board_init()); unsigned before = writes;
    mask = 0; assert(!w600_display_board_stop() && writes == before); mask = 1;
    rcc.APB1EN &= ~(uint32_t)RCC_APB1EN_UART2EN; before = uart_reads;
    assert(w600_display_board_stop() && uart_reads == before); stopped(); ++cases;
    printf("display_board: %u actual-source cases PASS\n", cases);
}
