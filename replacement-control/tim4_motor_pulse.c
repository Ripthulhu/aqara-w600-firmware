#include "ciu32l071.h"
#include "tim4_motor_pulse.h"
#include "../startup/w600_motor_stop.h"
#ifndef TIM4_PULSE_OBSERVE
#define TIM4_PULSE_OBSERVE(reg, value) ((void)0)
#endif
#ifndef TIM4_PULSE_READ
#define TIM4_PULSE_READ(reg) (reg)
#endif
#define WRITE(reg, value) do { (reg) = (value); __DSB(); \
    TIM4_PULSE_OBSERVE(&(reg), (uint32_t)(value)); } while (0)
#define HIGH 0xc000u
#define MODES 0xf0000000u
#define GPIO_HH 0x50000000u
#define LIMIT 100000u
#ifdef W600_DISPLAY_BENCH
#define PULSE_APB1_UARTS (RCC_APB1EN_UART2EN | RCC_APB1EN_UART3EN)
#else
#define PULSE_APB1_UARTS RCC_APB1EN_UART3EN
#endif
enum { OWN_TIMER = 1, OWN_AF = 2 };
_Static_assert(RCC_APB1EN_TIM4EN == 4 && RCC_APB1RST_TIM4_RST == 4 &&
               RCC_CFG_P2PRE_2 == 0x40000, "Reviewed CIU32 TIM4 mapping");

static int cpu_ok(void)
{
    return __get_PRIMASK() == 1 && __get_IPSR() == 0 && __get_CONTROL() == 0 &&
        !(NVIC->ISER[0] & (1u << TIM4_IRQn)) &&
        !(NVIC->ISPR[0] & (1u << TIM4_IRQn));
}
static int clock_ok(unsigned owned)
{
    return
#ifdef W600_DISPLAY_BENCH
        (RCC->APB1EN & PULSE_APB1_UARTS) == PULSE_APB1_UARTS &&
#endif
        (RCC->CFG == 0x12 || RCC->CFG == 0x40012) &&
        RCC->PLLCFG == 0x30260 &&
        (RCC->CSR1 & (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON |
                      RCC_CSR1_PLLRDY | RCC_CSR1_RCHDIV)) ==
            (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY) &&
        (RCC->AHBEN & RCC_AHBEN_FLASHEN) &&
        (FLASH->ACR & FLASH_ACR_LATENCY) == FLASH_ACR_LATENCY_2CLK &&
        !(RCC->AHBEN & RCC_AHBEN_DMAEN) &&
        !(RCC->APB1EN & ~(PULSE_APB1_UARTS | RCC_APB1EN_PMUEN |
                         (owned ? RCC_APB1EN_TIM4EN : 0u))) &&
        (RCC->APB1EN & RCC_APB1EN_TIM4EN) == (owned ? RCC_APB1EN_TIM4EN : 0u) &&
        !(RCC->APB2EN & ~RCC_APB2EN_ADCEN) &&
        !(RCC->APB1RST & RCC_APB1RST_TIM4_RST);
}
static int pins(unsigned mode, unsigned expected)
{
    return (RCC->IOPEN & 7u) == 7u &&
        (GPIOB->MODE & MODES) == mode && !(GPIOB->OTYPE & HIGH) &&
        (GPIOB->ODR & HIGH) == HIGH && (GPIOB->IDR & HIGH) == expected &&
        (GPIOA->MODE & 0xc0000000u) == 0x40000000u &&
        (GPIOC->MODE & 0x00300000u) == 0x00100000u &&
        !(GPIOA->OTYPE & 0x8000u) && !(GPIOC->OTYPE & 0x400u) &&
        !(GPIOA->ODR & 0x8000u) && !(GPIOC->ODR & 0x400u) &&
        !(GPIOA->IDR & 0x8000u) && !(GPIOC->IDR & 0x400u);
}
static uint32_t af_mask(const struct tim4_motor_pulse *s)
{
    return s->channel == 1 ? 0x0f000000u : 0xf0000000u;
}
static uint32_t active_mode(const struct tim4_motor_pulse *s)
{
    return s->channel == 1 ? 0x60000000u : 0x90000000u;
}
static int off(struct tim4_motor_pulse *s)
{
    if (!w600_motor_stop()) return 0;
    /* Never disable a still-LOW AF output if remux/readback failed. Its original
     * one-shot remains the independent bound, unless its clock itself fails. */
    if (s->owned & OWN_TIMER) {
        if (!(RCC->APB1EN & RCC_APB1EN_TIM4EN)) return 0;
        WRITE(TIM4->CR1, 0x88u); WRITE(TIM4->CCEN, 0u); WRITE(TIM4->DIER, 0u);
        if ((TIM4->CR1 & 1u) || TIM4->CCEN || TIM4->DIER) return 0;
    }
    if (s->owned & OWN_AF) {
        uint32_t mask = af_mask(s);
        WRITE(GPIOB->AFH, (GPIOB->AFH & ~mask) | (s->saved_afh & mask));
        if ((GPIOB->AFH & mask) != (s->saved_afh & mask)) return 0;
        s->owned &= ~OWN_AF;
    }
    return 1;
}
static int release_timer(struct tim4_motor_pulse *s)
{
    if (!off(s)) return 0;
    if (s->owned & OWN_TIMER) {
        WRITE(RCC->APB1EN, RCC->APB1EN & ~RCC_APB1EN_TIM4EN);
        if (RCC->APB1EN & RCC_APB1EN_TIM4EN) return 0;
        s->owned &= ~OWN_TIMER;
    }
    return !s->owned && pins(GPIO_HH, HIGH);
}
static enum tim4_motor_pulse_status fail(struct tim4_motor_pulse *s, unsigned err)
{
    if (!s->error) s->error = err;
    s->status = TIM4_PULSE_FAILED;
    if (!release_timer(s)) s->cleanup_error = 1;
    return TIM4_PULSE_FAILED;
}
static int timer_config(const struct tim4_motor_pulse *s, unsigned settling)
{
    uint32_t ccm = settling ? 0u : (s->channel == 1 ? 0x4078u : 0x7840u);
    uint32_t enable = settling ? 0u : (s->channel == 1 ? 3u : 0x30u);
    return (TIM4->CR1 & ~1u) == 0x88 && !TIM4->CR2 && !TIM4->SMC &&
        !TIM4->DIER && TIM4->CCM1 == ccm && !TIM4->CCM2 &&
        TIM4->CCEN == enable && TIM4->PSC == 479 &&
        TIM4->ARR == (settling ? 99u : 1999u) &&
        TIM4->CC1 == ((!settling && s->channel == 1) ? 100u : 0u) &&
        TIM4->CC2 == ((!settling && s->channel == 2) ? 100u : 0u) &&
        !TIM4->CC3 && !TIM4->CC4;
}
static int configure(const struct tim4_motor_pulse *s, unsigned settling)
{
    WRITE(TIM4->CR1, 0x88u); WRITE(TIM4->CCEN, 0u);
    WRITE(TIM4->CR2, 0u); WRITE(TIM4->SMC, 0u); WRITE(TIM4->DIER, 0u);
    WRITE(TIM4->CCM1, settling ? 0u : (s->channel == 1 ? 0x4078u : 0x7840u));
    WRITE(TIM4->CCM2, 0u); WRITE(TIM4->PSC, 479u);
    WRITE(TIM4->ARR, settling ? 99u : 1999u);
    WRITE(TIM4->CC1, (!settling && s->channel == 1) ? 100u : 0u);
    WRITE(TIM4->CC2, (!settling && s->channel == 2) ? 100u : 0u);
    WRITE(TIM4->CC3, 0u); WRITE(TIM4->CC4, 0u);
    WRITE(TIM4->EVTG, 1u); WRITE(TIM4->CNT, 0u); WRITE(TIM4->SR, 0u);
    WRITE(TIM4->CCEN, settling ? 0u : (s->channel == 1 ? 3u : 0x30u));
    return timer_config(s, settling) && !(TIM4->CR1 & 1u) &&
        TIM4->CNT == 0 && TIM4->SR == 0;
}
enum tim4_motor_pulse_status tim4_motor_pulse_arm(struct tim4_motor_pulse *s,
                                                unsigned direction)
{
    if (s->status != TIM4_PULSE_IDLE || s->error || s->owned)
        return fail(s, TIM4_PULSE_STATE);
    if (direction != 1 && direction != 2) return fail(s, TIM4_PULSE_DIRECTION);
    if (!cpu_ok() || !clock_ok(0) || !pins(GPIO_HH, HIGH))
        return fail(s, TIM4_PULSE_CONTEXT);
    s->direction = direction; s->channel = direction == 1 ? 2u : 1u;
    s->saved_afh = GPIOB->AFH; s->seen_low = s->polls = 0;
    s->low_count = s->settle_count = 0; s->owned = OWN_TIMER;
    WRITE(RCC->APB1EN, RCC->APB1EN | RCC_APB1EN_TIM4EN);
    if (!(RCC->APB1EN & RCC_APB1EN_TIM4EN) || !configure(s, 0))
        return fail(s, TIM4_PULSE_CONFIG);
    uint32_t mask = af_mask(s);
    s->owned |= OWN_AF;
    WRITE(GPIOB->AFH, (GPIOB->AFH & ~mask) |
          (s->channel == 1 ? 0x05000000u : 0x10000000u));
    WRITE(GPIOB->MODE, (GPIOB->MODE & ~MODES) | active_mode(s));
    if ((GPIOB->AFH & mask) != (s->channel == 1 ? 0x05000000u : 0x10000000u) ||
        !pins(active_mode(s), HIGH)) return fail(s, TIM4_PULSE_PADS);
    s->status = TIM4_PULSE_DRIVING;
    WRITE(TIM4->CR1, 0x89u);
    if (!(TIM4->CR1 & 1u)) return fail(s, TIM4_PULSE_TIMER);
    return TIM4_PULSE_DRIVING;
}
enum tim4_motor_pulse_status tim4_motor_pulse_poll(struct tim4_motor_pulse *s)
{
    if (s->status == TIM4_PULSE_IDLE || s->status == TIM4_PULSE_FAILED)
        return (enum tim4_motor_pulse_status)s->status;
    if (s->status == TIM4_PULSE_DONE) {
        if (!cpu_ok() || !clock_ok(0) || !pins(GPIO_HH, HIGH))
            return fail(s, TIM4_PULSE_CONTEXT);
        return TIM4_PULSE_DONE;
    }
    unsigned settling = s->status == TIM4_PULSE_SETTLING;
    if (!cpu_ok() || !clock_ok(1) || !timer_config(s, settling))
        return fail(s, TIM4_PULSE_CONTEXT);
    if (++s->polls > LIMIT) return fail(s, TIM4_PULSE_POLL_LIMIT);
    uint32_t cr = TIM4_PULSE_READ(TIM4->CR1);
    uint32_t count = TIM4_PULSE_READ(TIM4->CNT);
    uint32_t sr = TIM4_PULSE_READ(TIM4->SR);
    uint32_t expected = settling || count < 100u ? HIGH :
        (s->channel == 1 ? 0x8000u : 0x4000u);
    int pad_ok = pins(settling ? GPIO_HH : active_mode(s), expected);
    uint32_t after = TIM4_PULSE_READ(TIM4->CNT);
    uint32_t cr_after = TIM4_PULSE_READ(TIM4->CR1);
    uint32_t sr_after = TIM4_PULSE_READ(TIM4->SR);
    /* A single bounded observation can straddle compare/update. Defer it rather
     * than label that edge a fault or accept mismatched counter/pad evidence. */
    if (cr != cr_after || ((sr ^ sr_after) & 1u) || after < count ||
        (!settling && ((count < 100u) != (after < 100u))))
        return (enum tim4_motor_pulse_status)s->status;
    if ((sr | sr_after) & ~0x1fu) return fail(s, TIM4_PULSE_TIMER);
    if (!pad_ok) return fail(s, TIM4_PULSE_PADS);
    if (!(cr & 1u)) {
        if (count != 0 || after != 0 || !(sr & 1u))
            return fail(s, TIM4_PULSE_TIMER);
        if (settling) {
            if (!release_timer(s)) {
                s->cleanup_error = 1;
                return fail(s, TIM4_PULSE_CLEANUP);
            }
            s->status = TIM4_PULSE_DONE;
            return TIM4_PULSE_DONE;
        }
        if (!s->seen_low) return fail(s, TIM4_PULSE_TIMER);
        if (!off(s) || !configure(s, 1) || !pins(GPIO_HH, HIGH)) {
            s->cleanup_error = 1;
            return fail(s, TIM4_PULSE_CLEANUP);
        }
        s->status = TIM4_PULSE_SETTLING; s->polls = 0;
        WRITE(TIM4->CR1, 0x89u);
        if (!(TIM4->CR1 & 1u)) return fail(s, TIM4_PULSE_TIMER);
        return TIM4_PULSE_SETTLING;
    }
    if ((sr & 1u) || count > (settling ? 99u : 1999u)) return fail(s, TIM4_PULSE_TIMER);
    if (settling) {
        s->settle_count = count;
    } else {
        if (count >= 100u) { s->seen_low = 1; s->low_count = count; }
    }
    return (enum tim4_motor_pulse_status)s->status;
}
enum tim4_motor_pulse_status tim4_motor_pulse_abort(struct tim4_motor_pulse *s)
{
    return fail(s, TIM4_PULSE_ABORTED);
}
int tim4_motor_pulse_reset(struct tim4_motor_pulse *s)
{
    if (s->owned || s->status == TIM4_PULSE_DRIVING || s->status == TIM4_PULSE_SETTLING ||
        !cpu_ok() || !clock_ok(0) || !pins(GPIO_HH, HIGH)) return 0;
    volatile struct tim4_motor_pulse *v = s;
    v->status = TIM4_PULSE_IDLE; v->error = 0; v->direction = 0; v->polls = 0;
    v->channel = 0; v->owned = 0; v->saved_afh = 0; v->seen_low = 0;
    v->low_count = 0; v->settle_count = 0; v->cleanup_error = 0;
    return 1;
}
