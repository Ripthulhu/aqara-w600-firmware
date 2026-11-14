#include "ciu32l071.h"
#include "w600_display_board.h"
#ifndef W600_DISPLAY_BOARD_READ
#define W600_DISPLAY_BOARD_READ(reg) (reg)
#endif
#ifndef W600_DISPLAY_BOARD_WRITE
#define W600_DISPLAY_BOARD_WRITE(reg, value) do { (reg) = (value); __DSB(); } while (0)
#endif
#define RD(reg) W600_DISPLAY_BOARD_READ(reg)
#define WR(reg, value) W600_DISPLAY_BOARD_WRITE(reg, value)
#define PA4 (1u << 4)
#define PB6 (1u << 6)
#define A4 (3u << 8)
#define B6 (3u << 12)
#define B6_AF (15u << 24)
#define ACK (USART_ISR_TEACK | USART_ISR_REACK)
#define TX_CR1 (USART_CR1_UE | USART_CR1_TE | USART_CR1_WL | USART_CR1_PEN | USART_CR1_PTS)
enum { COLD, OWNED, STOPPED, POLL_LIMIT = 100000 };
static unsigned char lifetime;

_Static_assert(TX_CR1 == 0x1609u && UART2_IRQn == 26 &&
    RCC_APB1EN_UART2EN == 0x20000u && RCC_APB1RST_UART2_RST == 0x20000u &&
    USART_ISR_TEACK == 0x200000u && USART_ISR_REACK == 0x400000u,
    "Reviewed CIU32 UART2 profile");

static bool access_ok(bool uart_access)
{
    return __get_PRIMASK() == 1 && !__get_IPSR() && !__get_CONTROL() &&
        !(RD(NVIC->ISER[0]) & (1u << UART2_IRQn)) &&
        !(RD(NVIC->ISPR[0]) & (1u << UART2_IRQn)) &&
        (RD(RCC->IOPEN) & 3u) == 3u &&
        (!uart_access || ((RD(RCC->APB1EN) & RCC_APB1EN_UART2EN) &&
                   !(RD(RCC->APB1RST) & RCC_APB1RST_UART2_RST)));
}
static bool field(volatile uint32_t *reg, uint32_t bits, uint32_t value, bool uart_access)
{
    if (!access_ok(uart_access)) return false;
    uint32_t expected = (RD(*reg) & ~bits) | value;
    WR(*reg, expected);
    return access_ok(uart_access) && RD(*reg) == expected;
}
static bool latch(bool high)
{
    if (!access_ok(false)) return false;
    uint32_t expected = high ? RD(GPIOA->ODR) | PA4 : RD(GPIOA->ODR) & ~PA4;
    if (high) WR(GPIOA->BSR, PA4);
    else WR(GPIOA->BR, PA4);
    return access_ok(false) && RD(GPIOA->ODR) == expected;
}
static bool wait_ack(uint32_t expected)
{
    for (unsigned i = 0; i < POLL_LIMIT; ++i) {
        if (!access_ok(true)) return false;
        if ((RD(UART2->ISR) & ACK) == expected) return true;
    }
    return false;
}
static bool context(void)
{
    return access_ok(false) && (RD(RCC->IOPEN) & 7u) == 7u &&
        !(RD(RCC->APB1EN) & RCC_APB1EN_UART2EN) &&
        !(RD(RCC->APB1RST) & RCC_APB1RST_UART2_RST) &&
        RD(RCC->CFG) == 0x12u && RD(RCC->PLLCFG) == 0x30260u &&
        (RD(RCC->CSR1) & (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON |
            RCC_CSR1_PLLRDY | RCC_CSR1_RCHDIV)) ==
            (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY) &&
        (RD(RCC->AHBEN) & RCC_AHBEN_FLASHEN) &&
        (RD(FLASH->ACR) & FLASH_ACR_LATENCY) == FLASH_ACR_LATENCY_2CLK &&
        (RD(GPIOA->MODE) & (A4 | 0xc00u | 0xc0000000u)) == (A4 | 0x400u | 0x40000000u) &&
        (RD(GPIOB->MODE) & (B6 | 0x03000003u | 0xf0000000u)) == (B6 | 0x01000001u | 0x50000000u) &&
        (RD(GPIOC->MODE) & 0x00300000u) == 0x00100000u &&
        !(RD(GPIOA->OTYPE) & (PA4 | 0x20u | 0x8000u)) &&
        !(RD(GPIOB->OTYPE) & (PB6 | 0x1001u | 0xc000u)) &&
        !(RD(GPIOC->OTYPE) & 0x400u) &&
        (RD(GPIOA->PUPD) & (A4 | 0xc00u | 0xc0000000u)) == 0x800u &&
        (RD(GPIOB->PUPD) & (B6 | 0x03000003u | 0xf0000000u)) == 0x01000002u &&
        !(RD(GPIOC->PUPD) & 0x00300000u) &&
        !(RD(GPIOA->AFL) & (15u << 16)) && !(RD(GPIOB->AFL) & B6_AF) &&
        (RD(GPIOA->ODR) & 0x8020u) == 0x20u &&
        (RD(GPIOA->IDR) & 0x8020u) == 0x20u &&
        (RD(GPIOB->ODR) & 0xd001u) == 0xc001u &&
        (RD(GPIOB->IDR) & 0xd001u) == 0xc001u &&
        !(RD(GPIOC->ODR) & 0x400u) && !(RD(GPIOC->IDR) & 0x400u);
}
bool w600_display_board_init(void)
{
    if (lifetime != COLD || !context()) return false;
    lifetime = OWNED;
    if (!field(&RCC->APB1EN, RCC_APB1EN_UART2EN, RCC_APB1EN_UART2EN, false) ||
        !field(&RCC->APB1RST, RCC_APB1RST_UART2_RST, RCC_APB1RST_UART2_RST, false) ||
        !field(&RCC->APB1RST, RCC_APB1RST_UART2_RST, 0, false) ||
        !field(&UART2->CR1, UINT32_MAX, 0, true) || !wait_ack(0) ||
        !field(&UART2->CR2, UINT32_MAX, 0, true) ||
        !field(&UART2->CR3, UINT32_MAX, 0, true) ||
        !field(&UART2->BRR, UINT32_MAX, 2500, true) ||
        !field(&GPIOB->OTYPE, PB6, PB6, false) ||
        !field(&GPIOB->PUPD, B6, 2u << 12, false) ||
        !field(&GPIOB->AFL, B6_AF, 1u << 24, false) ||
        !field(&GPIOB->MODE, B6, 2u << 12, false) ||
        !latch(false) || !field(&GPIOA->OTYPE, PA4, 0, false) ||
        !field(&GPIOA->PUPD, A4, 1u << 8, false) ||
        !field(&GPIOA->MODE, A4, 1u << 8, false) ||
        !field(&UART2->CR1, UINT32_MAX, TX_CR1, true) || !wait_ack(USART_ISR_TEACK))
        return false;
    return access_ok(true) && !(RD(GPIOA->ODR) & PA4) && !(RD(GPIOA->IDR) & PA4) &&
        !(RD(UART2->ISR) & (15u | USART_ISR_RXNE));
}
bool w600_display_board_stop(void)
{
    if (lifetime != OWNED) return true;
    if (!access_ok(false)) return false;
    bool ok = true;
    if ((RD(RCC->APB1EN) & RCC_APB1EN_UART2EN) &&
        !(RD(RCC->APB1RST) & RCC_APB1RST_UART2_RST)) {
        /* Format bits are write-protected until UE is cleared. */
        if (!field(&UART2->CR1, USART_CR1_UE, 0, true)) ok = false;
        if (!wait_ack(0)) ok = false;
        if (!field(&UART2->CR1, UINT32_MAX, 0, true)) ok = false;
    }
    if (!field(&RCC->APB1RST, RCC_APB1RST_UART2_RST, RCC_APB1RST_UART2_RST, false)) ok = false;
    if (!field(&RCC->APB1EN, RCC_APB1EN_UART2EN, 0, false)) ok = false;
    if (!field(&RCC->APB1RST, RCC_APB1RST_UART2_RST, 0, false)) ok = false;
    if (!field(&GPIOB->MODE, B6, B6, false) ||
        !field(&GPIOB->OTYPE, PB6, 0, false) ||
        !field(&GPIOB->PUPD, B6, 0, false) ||
        !field(&GPIOB->AFL, B6_AF, 0, false)) ok = false;
    if (!latch(true) || !field(&GPIOA->OTYPE, PA4, 0, false) ||
        !field(&GPIOA->PUPD, A4, 1u << 8, false) ||
        !field(&GPIOA->MODE, A4, 1u << 8, false)) ok = false;
    if (!access_ok(false) || !(RD(GPIOA->ODR) & PA4) || !(RD(GPIOA->IDR) & PA4)) ok = false;
    if (ok) lifetime = STOPPED;
    return ok;
}
