#include "display_tx.h"
#ifdef W600_DISPLAY_TX_TEST
#include "display_tx_test_mmio.h"
#else
#include "ciu32l071.h"
#define DISPLAY_TX_ICR(bits) (UART2->ICR = (bits))
#define DISPLAY_TX_TDR(byte) (UART2->TDR = (byte))
#endif

#define HALF UINT32_C(0x80000000)
#define PB6 (1u << 6)
#define TX_CR1 (USART_CR1_WL | USART_CR1_PEN | USART_CR1_PTS | \
                USART_CR1_TE | USART_CR1_UE)
_Static_assert(TX_CR1 == 0x1609u && USART_ISR_TEACK == 0x200000u &&
               USART_ISR_REACK == 0x400000u && UART2_IRQn == 26 &&
               RCC_APB1EN_UART2EN == 0x20000u && RCC_CSR1_RCHRDY == 0x400u &&
               RCC_CSR1_RCHDIV == 0x1800u && FLASH_ACR_LATENCY == 3u,
               "Reviewed CIU32 UART2 profile");

static bool fail(struct w600_display_tx *s, enum w600_display_tx_error error)
{
    if (!s->error) s->error = error;
    s->phase = W600_DISPLAY_TX_FAILED;
    return false;
}

static enum w600_display_tx_error context(uint32_t *status)
{
    if (__get_IPSR() || __get_CONTROL() ||
        (NVIC->ISER[0] & (1u << UART2_IRQn)) ||
        (NVIC->ISPR[0] & (1u << UART2_IRQn)) ||
        (RCC->CFG != 0x12u && RCC->CFG != 0x40012u) ||
        RCC->PLLCFG != 0x30260u ||
        (RCC->CSR1 & (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON |
                       RCC_CSR1_PLLRDY | RCC_CSR1_RCHDIV)) !=
            (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY) ||
        !(RCC->AHBEN & RCC_AHBEN_FLASHEN) ||
        (FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2CLK ||
        !(RCC->IOPEN & RCC_IOPEN_GPIOBEN) ||
        (GPIOB->MODE & (3u << 12)) != (2u << 12) ||
        (GPIOB->OTYPE & PB6) != PB6 ||
        (GPIOB->PUPD & (3u << 12)) != (2u << 12) ||
        (GPIOB->AFL & (0xfu << 24)) != (1u << 24) ||
        !(RCC->APB1EN & RCC_APB1EN_UART2EN) ||
        UART2->CR1 != TX_CR1 || UART2->CR2 || UART2->CR3 ||
        UART2->BRR != 2500u) return W600_DISPLAY_TX_CONTEXT;
    *status = UART2->ISR;
    if (!(*status & USART_ISR_TEACK) || (*status & USART_ISR_REACK))
        return W600_DISPLAY_TX_CONTEXT;
    if (*status & (15u | USART_ISR_RXNE)) return W600_DISPLAY_TX_UART;
    return W600_DISPLAY_TX_OK;
}

static bool clock_ok(struct w600_display_tx *s, uint32_t now, bool poll)
{
    uint32_t delta = now - s->last_now;
    if (delta >= HALF) return fail(s, W600_DISPLAY_TX_CLOCK);
    if (delta) s->stalled_polls = 0;
    else if (poll && s->phase != W600_DISPLAY_TX_IDLE &&
             ++s->stalled_polls >= W600_DISPLAY_TX_STALLED_POLLS)
        return fail(s, W600_DISPLAY_TX_CLOCK);
    s->last_now = now;
    return true;
}

static bool fresh_tc(struct w600_display_tx *s, uint32_t *status)
{
    DISPLAY_TX_ICR(USART_ICR_TCCF);
    __DSB();
    enum w600_display_tx_error error = context(status);
    if (error) return fail(s, error);
    return !(*status & USART_ISR_TC) || fail(s, W600_DISPLAY_TX_UART);
}

bool w600_display_tx_init(struct w600_display_tx *s, uint32_t now)
{
    if (!s || s->initialized || s->error) return false;
    if (__get_PRIMASK() != 1) return fail(s, W600_DISPLAY_TX_CONTEXT);
    uint32_t status;
    enum w600_display_tx_error error = context(&status);
    if (error) return fail(s, error);
    s->last_now = now; s->phase = W600_DISPLAY_TX_IDLE;
    s->initialized = true;
    return true;
}

bool w600_display_tx_submit(struct w600_display_tx *s, const uint8_t frame[4],
                            uint32_t now)
{
    if (!s || !frame || !s->initialized || s->error ||
        s->phase != W600_DISPLAY_TX_IDLE) return false;
    if (!clock_ok(s, now, false)) return false;
    uint32_t status;
    enum w600_display_tx_error error = context(&status);
    if (error) return fail(s, error);
    if (!fresh_tc(s, &status)) return false;
    for (unsigned i = 0; i < 4; ++i) s->frame[i] = frame[i];
    s->started_at = now; s->stalled_polls = 0; s->position = 0;
    s->phase = W600_DISPLAY_TX_DATA;
    return true;
}

bool w600_display_tx_poll(struct w600_display_tx *s, uint32_t now)
{
    if (!s || !s->initialized || s->error) return false;
    if ((s->phase == W600_DISPLAY_TX_DATA && s->position > 4) ||
        (s->phase == W600_DISPLAY_TX_CONTROL && s->position > 1))
        return fail(s, W600_DISPLAY_TX_CONTEXT);
    if (!clock_ok(s, now, true)) return false;
    uint32_t status;
    enum w600_display_tx_error error = context(&status);
    if (error) return fail(s, error);
    if (s->phase == W600_DISPLAY_TX_IDLE) return true;
    if ((uint32_t)(now - s->started_at) >= 50u)
        return fail(s, W600_DISPLAY_TX_TIMEOUT);
    if (s->phase == W600_DISPLAY_TX_DATA_TC) {
        if (status & USART_ISR_TC) {
            s->gap_at = now; s->phase = W600_DISPLAY_TX_GAP;
        }
        return true;
    }
    if (s->phase == W600_DISPLAY_TX_GAP) {
        if ((uint32_t)(now - s->gap_at) < 5u) return true;
        if (!fresh_tc(s, &status)) return false;
        s->position = 0; s->phase = W600_DISPLAY_TX_CONTROL;
    }
    if (s->phase == W600_DISPLAY_TX_DATA || s->phase == W600_DISPLAY_TX_CONTROL) {
        if (!(status & USART_ISR_TXE)) return true;
        uint8_t byte;
        if (s->phase == W600_DISPLAY_TX_DATA) {
            byte = s->position ? s->frame[s->position - 1u] : 8u;
            DISPLAY_TX_TDR(byte);
            if (++s->position == 5) s->phase = W600_DISPLAY_TX_DATA_TC;
        } else {
            byte = s->position ? 0xffu : 0x18u;
            DISPLAY_TX_TDR(byte);
            if (++s->position == 2) s->phase = W600_DISPLAY_TX_FINAL_TC;
        }
        return true;
    }
    if (s->phase == W600_DISPLAY_TX_FINAL_TC) {
        if (status & USART_ISR_TC) s->phase = W600_DISPLAY_TX_IDLE;
        return true;
    }
    return fail(s, W600_DISPLAY_TX_CONTEXT);
}
