#include "ciu32l071.h"
#include "w600_feedback_board.h"
#ifndef FB_BOARD_OBSERVE
#define FB_BOARD_OBSERVE(stage) ((void)(stage))
#endif

enum { OWN_CONTEXT = 1u, OWN_DIV = 2u, OWN_PMU_CLOCK = 4u,
       OWN_ADC_CLOCK = 8u, OWN_PIN = 16u, OWN_BGR = 32u };
enum { SET_DIV = 1, SET_PMU_CLOCK, SET_ADC_CLOCK, RESET_ON, RESET_OFF,
       SET_MODE, SET_PULL, SET_BGR, CLEAN_ADC_CLOCK, CLEAN_RESET_ON,
       CLEAN_RESET_OFF, CLEAN_MODE, CLEAN_PULL, CLEAN_PMU_CLOCK, CLEAN_BGR,
       CLEAN_ADC_GATE, CLEAN_PMU_GATE, CLEAN_DIV };
#define PB13_FIELDS (3u << 26)
#ifdef W600_DISPLAY_BENCH
#define FB_APB1_UARTS (RCC_APB1EN_UART2EN | RCC_APB1EN_UART3EN)
#else
#define FB_APB1_UARTS RCC_APB1EN_UART3EN
#endif

_Static_assert(RCC_CFG_P2PRE_POS == 16 && RCC_CFG_P2PRE_2 == 0x40000u &&
    RCC_APB2RST_ADC_RST == 0x100000u && PMU_CR1_BGR_EN == 0x8000u,
    "Reviewed vendor clock and BGR masks");
_Static_assert(UART3_4_IRQn == 27, "Reviewed radio-only IRQ exception");

static bool motor_off(void)
{
    return (RCC->IOPEN & 7u) == 7u &&
        (GPIOB->MODE & 0xf0000000u) == 0x50000000u &&
        (GPIOA->MODE & (3u << 30)) == (1u << 30) &&
        (GPIOC->MODE & (3u << 20)) == (1u << 20) &&
        !(GPIOB->OTYPE & 0xc000u) && !(GPIOA->OTYPE & (1u << 15)) &&
        !(GPIOC->OTYPE & (1u << 10)) &&
        (GPIOB->ODR & 0xc000u) == 0xc000u &&
        (GPIOB->IDR & 0xc000u) == 0xc000u &&
        !(GPIOA->ODR & (1u << 15)) && !(GPIOA->IDR & (1u << 15)) &&
        !(GPIOC->ODR & (1u << 10)) && !(GPIOC->IDR & (1u << 10));
}
static bool cpu_ok(void)
{
    return __get_PRIMASK() == 1 && __get_IPSR() == 0 &&
        __get_CONTROL() == 0 && !(NVIC->ISER[0] & ~(1u << UART3_4_IRQn));
}
static bool profile_ok(void)
{
    return cpu_ok() && motor_off() && RCC->CFG == 0x12u &&
        RCC->PLLCFG == 0x30260u &&
        (RCC->CSR1 & (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON |
                       RCC_CSR1_PLLRDY | RCC_CSR1_RCHDIV)) ==
            (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY) &&
        (FLASH->ACR & FLASH_ACR_LATENCY) == FLASH_ACR_LATENCY_2CLK &&
        RCC->APB1EN == FB_APB1_UARTS &&
        !(RCC->APB2EN & ~RCC_APB2EN_DBGEN) &&
        !(RCC->AHBEN & RCC_AHBEN_DMAEN) &&
        !(RCC->APB2RST & RCC_APB2RST_ADC_RST);
}
static bool error(struct w600_feedback_board *s, uint32_t flag)
{
    s->error |= flag;
    return false;
}
static void observe(unsigned stage)
{
    __DSB();
    FB_BOARD_OBSERVE(stage);
}
static bool conflicts(const struct w600_feedback_board *s)
{
    return !cpu_ok() || !motor_off() ||
        (RCC->CFG & ~RCC_CFG_P2PRE) != (s->cfg & ~RCC_CFG_P2PRE) ||
        (RCC->APB1EN & ~RCC_APB1EN_PMUEN) != (s->apb1 & ~RCC_APB1EN_PMUEN) ||
        (RCC->APB2EN & ~RCC_APB2EN_ADCEN) != (s->apb2 & ~RCC_APB2EN_ADCEN) ||
        (RCC->APB2RST & ~RCC_APB2RST_ADC_RST) != (s->reset & ~RCC_APB2RST_ADC_RST) ||
        (GPIOB->MODE & ~PB13_FIELDS) != (s->mode & ~PB13_FIELDS) ||
        (GPIOB->PUPD & ~PB13_FIELDS) != (s->pull & ~PB13_FIELDS) ||
        ((s->owned & OWN_BGR) && (RCC->APB1EN & RCC_APB1EN_PMUEN) &&
         (PMU->CR1 & ~PMU_CR1_BGR_EN) != (s->pmu & ~PMU_CR1_BGR_EN));
}

bool w600_feedback_board_prepare(struct w600_feedback_board *s)
{
    if (s->owned || s->prepared || s->error) return error(s, W600_FB_BOARD_STATE);
    if (!profile_ok()) return error(s, W600_FB_BOARD_CONTEXT);
    s->cfg = RCC->CFG; s->apb1 = RCC->APB1EN; s->apb2 = RCC->APB2EN;
    s->reset = RCC->APB2RST; s->mode = GPIOB->MODE; s->pull = GPIOB->PUPD;
    s->owned = OWN_CONTEXT | OWN_DIV;
    RCC->CFG = (RCC->CFG & ~RCC_CFG_P2PRE) | RCC_CFG_P2PRE_2;
    observe(SET_DIV);
    if (RCC->CFG != 0x40012u) return error(s, W600_FB_BOARD_CLOCK);
    s->owned |= OWN_PMU_CLOCK;
    RCC->APB1EN |= RCC_APB1EN_PMUEN;
    observe(SET_PMU_CLOCK);
    if (!(RCC->APB1EN & RCC_APB1EN_PMUEN)) return error(s, W600_FB_BOARD_CLOCK);
    s->pmu = PMU->CR1; /* PMU was gated at entry; only now is this read valid. */
    s->owned |= OWN_ADC_CLOCK;
    RCC->APB2EN |= RCC_APB2EN_ADCEN;
    observe(SET_ADC_CLOCK);
    if (!(RCC->APB2EN & RCC_APB2EN_ADCEN)) return error(s, W600_FB_BOARD_CLOCK);
    RCC->APB2RST |= RCC_APB2RST_ADC_RST;
    observe(RESET_ON);
    if (!(RCC->APB2RST & RCC_APB2RST_ADC_RST)) return error(s, W600_FB_BOARD_RESET);
    RCC->APB2RST &= ~RCC_APB2RST_ADC_RST;
    observe(RESET_OFF);
    if (RCC->APB2RST & RCC_APB2RST_ADC_RST) return error(s, W600_FB_BOARD_RESET);
    s->owned |= OWN_PIN;
    GPIOB->MODE = (GPIOB->MODE & ~PB13_FIELDS) | PB13_FIELDS;
    observe(SET_MODE);
    if ((GPIOB->MODE & PB13_FIELDS) != PB13_FIELDS) return error(s, W600_FB_BOARD_PIN);
    GPIOB->PUPD &= ~PB13_FIELDS;
    observe(SET_PULL);
    if (GPIOB->PUPD & PB13_FIELDS) return error(s, W600_FB_BOARD_PIN);
    s->owned |= OWN_BGR;
    PMU->CR1 |= PMU_CR1_BGR_EN;
    observe(SET_BGR);
    if (!(PMU->CR1 & PMU_CR1_BGR_EN)) return error(s, W600_FB_BOARD_BGR);
    if (conflicts(s)) return error(s, W600_FB_BOARD_CONFLICT);
    s->prepared = true;
    return true;
}

bool w600_feedback_board_finish(struct w600_feedback_board *s)
{
    if (!s->owned) return s->error == 0;
    bool clean = true;
    s->prepared = false;
    if (conflicts(s)) s->error |= W600_FB_BOARD_CONFLICT;
    if (s->owned & OWN_ADC_CLOCK) {
        RCC->APB2EN |= RCC_APB2EN_ADCEN;
        observe(CLEAN_ADC_CLOCK);
        if (!(RCC->APB2EN & RCC_APB2EN_ADCEN)) { s->error |= W600_FB_BOARD_CLOCK; clean = false; }
        RCC->APB2RST |= RCC_APB2RST_ADC_RST;
        observe(CLEAN_RESET_ON);
        if (!(RCC->APB2RST & RCC_APB2RST_ADC_RST)) { s->error |= W600_FB_BOARD_RESET; clean = false; }
        RCC->APB2RST &= ~RCC_APB2RST_ADC_RST;
        observe(CLEAN_RESET_OFF);
        if (RCC->APB2RST & RCC_APB2RST_ADC_RST) { s->error |= W600_FB_BOARD_RESET; clean = false; }
    }
    if (s->owned & OWN_PIN) {
        if (RCC->IOPEN & RCC_IOPEN_GPIOBEN) {
            GPIOB->MODE = (GPIOB->MODE & ~PB13_FIELDS) | (s->mode & PB13_FIELDS);
            observe(CLEAN_MODE);
            if ((GPIOB->MODE & PB13_FIELDS) != (s->mode & PB13_FIELDS)) { s->error |= W600_FB_BOARD_PIN; clean = false; }
            GPIOB->PUPD = (GPIOB->PUPD & ~PB13_FIELDS) | (s->pull & PB13_FIELDS);
            observe(CLEAN_PULL);
            if ((GPIOB->PUPD & PB13_FIELDS) != (s->pull & PB13_FIELDS)) { s->error |= W600_FB_BOARD_PIN; clean = false; }
        } else { s->error |= W600_FB_BOARD_CLOCK; clean = false; }
    }
    if (s->owned & OWN_BGR) {
        RCC->APB1EN |= RCC_APB1EN_PMUEN;
        observe(CLEAN_PMU_CLOCK);
        if (RCC->APB1EN & RCC_APB1EN_PMUEN) {
            PMU->CR1 = (PMU->CR1 & ~PMU_CR1_BGR_EN) | (s->pmu & PMU_CR1_BGR_EN);
            observe(CLEAN_BGR);
            if ((PMU->CR1 & PMU_CR1_BGR_EN) != (s->pmu & PMU_CR1_BGR_EN)) { s->error |= W600_FB_BOARD_BGR; clean = false; }
        } else { s->error |= W600_FB_BOARD_CLOCK; clean = false; }
    }
    if (s->owned & OWN_ADC_CLOCK) {
        RCC->APB2EN = (RCC->APB2EN & ~RCC_APB2EN_ADCEN) | (s->apb2 & RCC_APB2EN_ADCEN);
        observe(CLEAN_ADC_GATE);
        if ((RCC->APB2EN & RCC_APB2EN_ADCEN) != (s->apb2 & RCC_APB2EN_ADCEN)) { s->error |= W600_FB_BOARD_CLOCK; clean = false; }
    }
    if (s->owned & OWN_PMU_CLOCK) {
        RCC->APB1EN = (RCC->APB1EN & ~RCC_APB1EN_PMUEN) | (s->apb1 & RCC_APB1EN_PMUEN);
        observe(CLEAN_PMU_GATE);
        if ((RCC->APB1EN & RCC_APB1EN_PMUEN) != (s->apb1 & RCC_APB1EN_PMUEN)) { s->error |= W600_FB_BOARD_CLOCK; clean = false; }
    }
    if (s->owned & OWN_DIV) {
        RCC->CFG = (RCC->CFG & ~RCC_CFG_P2PRE) | (s->cfg & RCC_CFG_P2PRE);
        observe(CLEAN_DIV);
        if ((RCC->CFG & RCC_CFG_P2PRE) != (s->cfg & RCC_CFG_P2PRE)) { s->error |= W600_FB_BOARD_CLOCK; clean = false; }
    }
    if (conflicts(s)) s->error |= W600_FB_BOARD_CONFLICT;
    if (clean) s->owned = 0;
    return s->error == 0;
}
