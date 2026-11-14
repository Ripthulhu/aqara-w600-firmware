#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"

static RCC_t rcc;
static PMU_t pmu;
static FLASH_t flash;
static GPIO_t gpioa, gpiob, gpioc;
static NVIC_Type nvic;
static ADC_t adc;
static uint32_t primask, ipsr, control;
static unsigned fault, observed[32], resets, cases;
static uint32_t debug_baseline;
#ifdef W600_DISPLAY_BENCH
#define TEST_APB1_UARTS (RCC_APB1EN_UART2EN | RCC_APB1EN_UART3EN)
#else
#define TEST_APB1_UARTS RCC_APB1EN_UART3EN
#endif
static void model_observe(unsigned stage);
static PMU_t *pmu_regs(void)
{
    assert(rcc.APB1EN & RCC_APB1EN_PMUEN);
    return &pmu;
}
static GPIO_t *gpiob_regs(void)
{
    assert(rcc.IOPEN & RCC_IOPEN_GPIOBEN);
    return &gpiob;
}
#undef RCC
#undef PMU
#undef FLASH
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef NVIC
#define RCC (&rcc)
#define PMU pmu_regs()
#define FLASH (&flash)
#define GPIOA (&gpioa)
#define GPIOB gpiob_regs()
#define GPIOC (&gpioc)
#define NVIC (&nvic)
#define __get_PRIMASK() primask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#define __DSB() ((void)0)
#define FB_BOARD_OBSERVE(stage) model_observe(stage)
#include "w600_feedback_board.c"

struct snapshot {
    RCC_t rcc; PMU_t pmu; FLASH_t flash;
    GPIO_t a, b, c; NVIC_Type nvic;
    uint32_t primask, ipsr, control;
};
static struct snapshot before;
static struct w600_feedback_board s;

static void snapshot(struct snapshot *out)
{
    memset(out, 0, sizeof *out);
    out->rcc = rcc; out->pmu = pmu; out->flash = flash;
    out->a = gpioa; out->b = gpiob; out->c = gpioc; out->nvic = nvic;
    out->primask = primask; out->ipsr = ipsr; out->control = control;
}
static void unchanged(void)
{
    struct snapshot now; snapshot(&now);
    assert(!memcmp(&before, &now, sizeof now));
}
static void unrelated_unchanged(void)
{
    struct snapshot now; snapshot(&now);
    now.rcc.CFG = (now.rcc.CFG & ~RCC_CFG_P2PRE) | (before.rcc.CFG & RCC_CFG_P2PRE);
    now.rcc.APB1EN = (now.rcc.APB1EN & ~RCC_APB1EN_PMUEN) | (before.rcc.APB1EN & RCC_APB1EN_PMUEN);
    now.rcc.APB2EN = (now.rcc.APB2EN & ~RCC_APB2EN_ADCEN) | (before.rcc.APB2EN & RCC_APB2EN_ADCEN);
    now.rcc.APB2RST = (now.rcc.APB2RST & ~RCC_APB2RST_ADC_RST) | (before.rcc.APB2RST & RCC_APB2RST_ADC_RST);
    now.b.MODE = (now.b.MODE & ~PB13_FIELDS) | (before.b.MODE & PB13_FIELDS);
    now.b.PUPD = (now.b.PUPD & ~PB13_FIELDS) | (before.b.PUPD & PB13_FIELDS);
    now.pmu.CR1 = (now.pmu.CR1 & ~PMU_CR1_BGR_EN) | (before.pmu.CR1 & PMU_CR1_BGR_EN);
    assert(!memcmp(&before, &now, sizeof now));
}
static void model_observe(unsigned stage)
{
    assert(stage > 0 && stage < 32); ++observed[stage];
    if (fault == stage) {
        switch (stage) {
        case SET_DIV: rcc.CFG &= ~RCC_CFG_P2PRE; break;
        case SET_PMU_CLOCK: case CLEAN_PMU_CLOCK: rcc.APB1EN &= ~RCC_APB1EN_PMUEN; break;
        case SET_ADC_CLOCK: case CLEAN_ADC_CLOCK: rcc.APB2EN &= ~RCC_APB2EN_ADCEN; break;
        case RESET_ON: case CLEAN_RESET_ON: rcc.APB2RST &= ~RCC_APB2RST_ADC_RST; break;
        case RESET_OFF: case CLEAN_RESET_OFF: rcc.APB2RST |= RCC_APB2RST_ADC_RST; break;
        case SET_MODE: gpiob.MODE &= ~PB13_FIELDS; break;
        case SET_PULL: gpiob.PUPD |= PB13_FIELDS; break;
        case SET_BGR: pmu.CR1 &= ~PMU_CR1_BGR_EN; break;
        case CLEAN_MODE: gpiob.MODE ^= PB13_FIELDS; break;
        case CLEAN_PULL: gpiob.PUPD ^= PB13_FIELDS; break;
        case CLEAN_BGR: pmu.CR1 ^= PMU_CR1_BGR_EN; break;
        case CLEAN_ADC_GATE: rcc.APB2EN |= RCC_APB2EN_ADCEN; break;
        case CLEAN_PMU_GATE: rcc.APB1EN |= RCC_APB1EN_PMUEN; break;
        case CLEAN_DIV: rcc.CFG |= RCC_CFG_P2PRE_2; break;
        default: assert(0);
        }
    }
    if ((stage == RESET_ON || stage == CLEAN_RESET_ON) &&
        (rcc.APB2RST & RCC_APB2RST_ADC_RST)) {
        memset(&adc, 0, sizeof adc); ++resets;
    }
    unrelated_unchanged();
}
static void reset_model(void)
{
    memset(&rcc, 0xa5, sizeof rcc); memset(&pmu, 0x5a, sizeof pmu);
    memset(&flash, 0x35, sizeof flash); memset(&gpioa, 0x96, sizeof gpioa);
    memset(&gpiob, 0x69, sizeof gpiob); memset(&gpioc, 0x96, sizeof gpioc);
    memset(&nvic, 0, sizeof nvic); memset(&s, 0, sizeof s);
    memset(&adc, 0xff, sizeof adc); memset(observed, 0, sizeof observed);
    rcc.CFG = 0x12; rcc.PLLCFG = 0x30260;
    rcc.CSR1 = RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY;
    rcc.APB1EN = TEST_APB1_UARTS; rcc.APB2EN = debug_baseline;
    rcc.AHBEN &= ~RCC_AHBEN_DMAEN; rcc.APB2RST &= ~RCC_APB2RST_ADC_RST;
    rcc.IOPEN |= 7u; flash.ACR = FLASH_ACR_LATENCY_2CLK;
    gpiob.MODE = (gpiob.MODE & ~0xf0000000u) | 0x50000000u;
    gpioa.MODE = (gpioa.MODE & ~(3u << 30)) | (1u << 30);
    gpioc.MODE = (gpioc.MODE & ~(3u << 20)) | (1u << 20);
    gpiob.OTYPE &= ~0xc000u; gpioa.OTYPE &= ~(1u << 15); gpioc.OTYPE &= ~(1u << 10);
    gpiob.ODR |= 0xc000u; *(volatile uint32_t *)&gpiob.IDR |= 0xc000u;
    gpioa.ODR &= ~(1u << 15); *(volatile uint32_t *)&gpioa.IDR &= ~(1u << 15);
    gpioc.ODR &= ~(1u << 10); *(volatile uint32_t *)&gpioc.IDR &= ~(1u << 10);
    gpiob.PUPD = (gpiob.PUPD & ~PB13_FIELDS) | (1u << 26);
    pmu.CR1 &= ~PMU_CR1_BGR_EN;
    primask = 1; ipsr = control = fault = resets = 0;
    snapshot(&before);
}
static void prepared(void)
{
    assert(w600_feedback_board_prepare(&s) && s.prepared && !s.error);
    assert(rcc.CFG == 0x40012 && rcc.APB1EN == (TEST_APB1_UARTS | RCC_APB1EN_PMUEN));
    assert(rcc.APB2EN == (debug_baseline | RCC_APB2EN_ADCEN));
    assert((gpiob.MODE & PB13_FIELDS) == PB13_FIELDS && !(gpiob.PUPD & PB13_FIELDS));
    assert(pmu.CR1 & PMU_CR1_BGR_EN);
    assert(!adc.CR && !adc.CFG1 && !adc.CFG2 && !adc.ISR && resets == 1);
    unrelated_unchanged();
}
static void run_profile(uint32_t debug)
{
    debug_baseline = debug;
    reset_model(); assert(w600_feedback_board_finish(&s)); unchanged(); ++cases;
    for (unsigned pending = 0; pending < 2; ++pending) {
        reset_model(); nvic.ISER[0] = 1u << UART3_4_IRQn;
        nvic.ISPR[0] = pending ? 1u << UART3_4_IRQn : 0;
        snapshot(&before); prepared();
        assert(w600_feedback_board_finish(&s) && !s.owned);
        unchanged(); ++cases;
    }
    reset_model(); nvic.ISER[0] = (1u << UART3_4_IRQn) | 1u;
    snapshot(&before);
    assert(!w600_feedback_board_prepare(&s) && !s.owned &&
        s.error == W600_FB_BOARD_CONTEXT);
    unchanged(); ++cases;
    reset_model(); nvic.ISER[0] = 1u << UART3_4_IRQn;
    nvic.ISPR[0] = 1u << UART3_4_IRQn;
    snapshot(&before); prepared();
    nvic.ISER[0] |= 1u; before.nvic.ISER[0] |= 1u;
    assert(!w600_feedback_board_finish(&s) && !s.owned &&
        s.error == W600_FB_BOARD_CONFLICT);
    unchanged(); ++cases;
    for (unsigned bgr = 0; bgr < 2; ++bgr) for (unsigned mode = 0; mode < 4; ++mode) {
        reset_model();
        pmu.CR1 |= bgr ? PMU_CR1_BGR_EN : 0;
        gpiob.MODE = (gpiob.MODE & ~PB13_FIELDS) | (mode << 26);
        snapshot(&before); prepared();
        adc.CR = ADC_CR_ADEN | ADC_CR_START | ADC_CR_CALEN;
        assert(w600_feedback_board_finish(&s) && !s.owned && !s.prepared && resets == 2);
        assert(!adc.CR); unchanged();
        unsigned count = observed[CLEAN_DIV];
        assert(w600_feedback_board_finish(&s) && observed[CLEAN_DIV] == count); ++cases;
    }
    for (unsigned bad = 0; bad < 23; ++bad) {
        reset_model();
        switch (bad) {
        case 0: primask = 0; break;
        case 1: ipsr = 15; break;
        case 2: control = 2; break;
        case 3: nvic.ISER[0] = 1; break;
        case 4: rcc.CFG |= RCC_CFG_P2PRE_2; break;
        case 5: rcc.PLLCFG ^= 1; break;
        case 6: rcc.CSR1 &= ~RCC_CSR1_PLLRDY; break;
        case 7: rcc.CSR1 |= RCC_CSR1_RCHDIV_2; break;
        case 8: flash.ACR = 0; break;
        case 9: rcc.APB1EN |= RCC_APB1EN_PMUEN; break;
        case 10: rcc.APB2EN |= RCC_APB2EN_ADCEN; break;
        case 11: rcc.AHBEN |= RCC_AHBEN_DMAEN; break;
        case 12: rcc.APB2RST |= RCC_APB2RST_ADC_RST; break;
        case 13: rcc.IOPEN &= ~1u; break;
        case 14: gpiob.MODE &= ~0xf0000000u; break;
        case 15: gpiob.OTYPE |= 0xc000u; break;
        case 16: *(volatile uint32_t *)&gpiob.IDR &= ~0xc000u; break;
        case 17: gpioa.ODR |= 1u << 15; break;
        case 18: rcc.APB2EN |= RCC_APB2EN_SPI1EN; break;
        case 19: rcc.APB2EN |= RCC_APB2EN_USART1EN; break;
        case 20: rcc.APB1EN ^= RCC_APB1EN_UART2EN; break;
        case 21: rcc.APB1EN &= ~RCC_APB1EN_UART3EN; break;
        default: rcc.APB1EN |= RCC_APB1EN_TIM3EN; break;
        }
        snapshot(&before);
        assert(!w600_feedback_board_prepare(&s) && s.error == W600_FB_BOARD_CONTEXT);
        assert(!s.owned && !w600_feedback_board_finish(&s)); unchanged();
        for (unsigned i = 0; i < 32; ++i) assert(!observed[i]);
        ++cases;
    }
    for (unsigned stage = SET_DIV; stage <= SET_BGR; ++stage) {
        reset_model(); fault = stage;
        assert(!w600_feedback_board_prepare(&s) && s.error && !s.prepared);
        uint32_t error_before = s.error; fault = 0;
        assert(!w600_feedback_board_finish(&s) && s.error == error_before && !s.owned);
        unchanged(); ++cases;
    }
    for (unsigned stage = CLEAN_ADC_CLOCK; stage <= CLEAN_DIV; ++stage) {
        reset_model(); prepared(); fault = stage;
        assert(!w600_feedback_board_finish(&s) && s.error && s.owned && !s.prepared);
        assert(observed[CLEAN_DIV] == 1 && observed[CLEAN_PMU_GATE] == 1);
        uint32_t error_before = s.error; fault = 0;
        assert(!w600_feedback_board_finish(&s) && s.error == error_before && !s.owned);
        unchanged(); ++cases;
    }
    reset_model(); prepared();
    struct snapshot active; snapshot(&active);
    assert(!w600_feedback_board_prepare(&s) && s.error == W600_FB_BOARD_STATE);
    struct snapshot again; snapshot(&again); assert(!memcmp(&active, &again, sizeof active));
    assert(!w600_feedback_board_finish(&s) && !s.owned); unchanged(); ++cases;
    for (unsigned field = 0; field < 8; ++field) {
        reset_model(); prepared();
        switch (field) {
        case 0: rcc.CFG |= RCC_CFG_P1PRE_2; before.rcc.CFG |= RCC_CFG_P1PRE_2; break;
        case 1: rcc.APB1EN |= RCC_APB1EN_TIM4EN; before.rcc.APB1EN |= RCC_APB1EN_TIM4EN; break;
        case 2: rcc.APB2EN ^= RCC_APB2EN_DBGEN; before.rcc.APB2EN ^= RCC_APB2EN_DBGEN; break;
        case 3: rcc.APB2RST ^= 1u; before.rcc.APB2RST ^= 1u; break;
        case 4: gpiob.MODE ^= 3u; before.b.MODE ^= 3u; break;
        case 5: gpiob.PUPD ^= 3u; before.b.PUPD ^= 3u; break;
        case 6: pmu.CR1 ^= 1u; before.pmu.CR1 ^= 1u; break;
        default: rcc.APB1EN ^= RCC_APB1EN_UART2EN; before.rcc.APB1EN ^= RCC_APB1EN_UART2EN; break;
        }
        assert(!w600_feedback_board_finish(&s) && s.error == W600_FB_BOARD_CONFLICT && !s.owned);
        unchanged(); ++cases;
    }
    reset_model(); prepared();
    rcc.IOPEN &= ~RCC_IOPEN_GPIOBEN; before.rcc.IOPEN &= ~RCC_IOPEN_GPIOBEN;
    assert(!w600_feedback_board_finish(&s) && s.owned &&
           s.error == (W600_FB_BOARD_CONFLICT | W600_FB_BOARD_CLOCK));
    assert(!observed[CLEAN_MODE] && observed[CLEAN_DIV] == 1);
    rcc.IOPEN |= RCC_IOPEN_GPIOBEN; before.rcc.IOPEN |= RCC_IOPEN_GPIOBEN;
    assert(!w600_feedback_board_finish(&s) && !s.owned); unchanged(); ++cases;
    reset_model(); prepared(); primask = 0; before.primask = 0;
    assert(!w600_feedback_board_finish(&s) && !s.owned && s.error == W600_FB_BOARD_CONFLICT);
    unchanged(); ++cases;
}

int main(void)
{
    run_profile(0);
    run_profile(RCC_APB2EN_DBGEN);
    printf("ALL %u ACTUAL-SOURCE FEEDBACK BOARD CASES PASSED; no hardware validation\n", cases);
}
