#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"

static RCC_t rcc, saved_rcc;
static PMU_t pmu, saved_pmu;
static GPIO_t gpiob, saved_gpio;
static FLASH_t flash, saved_flash;
static ADC_t adc;
static uint32_t factory;
static unsigned accesses, reads, commands, starts, stops, clears;
static bool clear_ignored, start_ignored, enable_ignored, cal_ignored;
static bool read_ignored, overrun_on_read;
static ADC_t *adc_regs(void) { assert(rcc.APB2EN & RCC_APB2EN_ADCEN); ++accesses; return &adc; }
static PMU_t *pmu_regs(void) { assert(rcc.APB1EN & RCC_APB1EN_PMUEN); return &pmu; }
static GPIO_t *gpio_regs(void) { assert(rcc.IOPEN & RCC_IOPEN_GPIOBEN); return &gpiob; }
static void clear_flags(uint32_t mask);
static void command(uint32_t value);
static uint32_t read_data(void);
#undef RCC
#undef PMU
#undef GPIOB
#undef FLASH
#undef ADC
#define RCC (&rcc)
#define PMU pmu_regs()
#define GPIOB gpio_regs()
#define FLASH (&flash)
#define ADC adc_regs()
#define FEEDBACK_FACTORY_BGR factory
#define FEEDBACK_CLEAR_FLAGS(mask) clear_flags(mask)
#define FEEDBACK_COMMAND(value) command(value)
#define FEEDBACK_READ_DATA() read_data()
#include "w600_feedback.c"

static struct w600_feedback s;
static struct w600_feedback_sample out;
static unsigned cases;

static void clear_flags(uint32_t mask)
{
    assert(mask == FB_FLAGS); ++clears;
    if (!clear_ignored) adc.ISR &= ~mask;
}
static void command(uint32_t value)
{
    ++commands;
    if (value == ADC_CR_ADEN) { if (!enable_ignored) adc.CR = ADC_CR_ADEN; }
    else if (value == ADC_CR_CALEN) {
        assert(adc.CR == ADC_CR_ADEN && !adc.CHCFG && !(adc.ISR & FB_FLAGS));
        if (!cal_ignored) adc.CR |= ADC_CR_CALEN;
    } else if (value == ADC_CR_START) {
        assert(adc.CR == ADC_CR_ADEN && !(adc.ISR & FB_FLAGS));
        assert(adc.CHCFG == ADC_CHCFG_CHN13 || adc.CHCFG == signal_channel(&s));
        assert(adc.CFG2 == input_config(s.input));
        ++starts;
        if (!start_ignored) adc.CR |= ADC_CR_START;
    } else {
        assert(value == ADC_CR_STOP && (adc.CR & ADC_CR_START));
        ++stops; adc.CR = ADC_CR_ADEN;
    }
}
static uint32_t read_data(void)
{
    assert(adc.CR == ADC_CR_ADEN && (adc.ISR & (ADC_ISR_EOC | ADC_ISR_EOS)) ==
           (ADC_ISR_EOC | ADC_ISR_EOS));
    ++reads;
    if (!read_ignored) adc.ISR &= ~ADC_ISR_EOC;
    if (overrun_on_read) adc.ISR |= ADC_ISR_OVRN;
    return adc.DR;
}
static void save_external(void)
{
    saved_rcc = rcc; saved_pmu = pmu; saved_gpio = gpiob; saved_flash = flash;
}
static void unchanged(void)
{
    assert(!memcmp(&rcc, &saved_rcc, sizeof rcc));
    assert(!memcmp(&pmu, &saved_pmu, sizeof pmu));
    assert(!memcmp(&gpiob, &saved_gpio, sizeof gpiob));
    assert(!memcmp(&flash, &saved_flash, sizeof flash));
}
static enum w600_feedback_status poll_at(uint32_t now)
{
    unsigned before = accesses;
    out.valid = true;
    enum w600_feedback_status result = w600_feedback_poll(&s, now, &out);
    assert(accesses - before < 64); unchanged();
    if (result != W600_FEEDBACK_SAMPLE) assert(!out.valid);
    if (result == W600_FEEDBACK_FAILED) assert(out.error == s.sample.error);
    return result;
}
static void reset_model(void)
{
    memset(&rcc, 0, sizeof rcc); memset(&pmu, 0, sizeof pmu);
    memset(&gpiob, 0, sizeof gpiob); memset(&flash, 0, sizeof flash);
    memset(&adc, 0, sizeof adc); memset(&s, 0, sizeof s); memset(&out, 0, sizeof out);
    rcc.CFG = 0x40012; rcc.PLLCFG = 0x30260;
    rcc.CSR1 = RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY;
    rcc.APB1EN = RCC_APB1EN_PMUEN; rcc.APB2EN = RCC_APB2EN_ADCEN;
    rcc.IOPEN = RCC_IOPEN_GPIOBEN;
    flash.ACR = FLASH_ACR_LATENCY_2CLK;
    gpiob.MODE = 3u << 26;
    pmu.CR1 = PMU_CR1_BGR_EN; pmu.SR = PMU_SR_BGR_READY;
    factory = 0xfa2805d7;
    accesses = reads = commands = starts = stops = clears = 0;
    clear_ignored = start_ignored = enable_ignored = cal_ignored = false;
    read_ignored = overrun_on_read = false;
    save_external();
}
static void calibrated_input(uint32_t now, bool vbat)
{
    assert((vbat ? w600_feedback_init_vbat(&s, now) : w600_feedback_init(&s, now)) == W600_FEEDBACK_BUSY);
    unchanged();
    assert(adc.CFG2 == (0x80010101u | (vbat ? 0x40000u : 0u)) && adc.CFG1 == 0x4000 && adc.SAMPT == 0x77);
    assert(s.input == (vbat ? W600_FEEDBACK_VBAT : W600_FEEDBACK_MOTOR));
    assert((adc.SAMPT & 15u) == 7u && ((adc.SAMPT >> 4) & 15u) == 7u);
    assert(!commands && !reads);
    assert(poll_at(now + 1) == W600_FEEDBACK_BUSY && !commands);
    assert(poll_at(now + 2) == W600_FEEDBACK_BUSY && adc.CR == ADC_CR_ADEN);
    assert(poll_at(now + 3) == W600_FEEDBACK_BUSY && commands == 1);
    assert(poll_at(now + 4) == W600_FEEDBACK_BUSY && (adc.CR & ADC_CR_CALEN));
    adc.CR = ADC_CR_ADEN; adc.ISR = ADC_ISR_EOCAL; adc.CALFACT = 13;
    assert(poll_at(now + 4) == W600_FEEDBACK_READY && !reads);
}
static void calibrated(uint32_t now) { calibrated_input(now, false); }
static enum w600_feedback_status finish_raw(uint32_t raw, uint32_t now)
{
    adc.CR = ADC_CR_ADEN; adc.DR = raw; adc.ISR = ADC_ISR_EOC | ADC_ISR_EOS;
    return poll_at(now);
}
static void burst(uint32_t now, uint16_t signal)
{
    assert(w600_feedback_start(&s, now) == W600_FEEDBACK_BUSY); unchanged();
    for (unsigned i = 0; i < 3; ++i) {
        assert(finish_raw(1495 + i, now + i) == W600_FEEDBACK_BUSY);
        assert(finish_raw(signal + i, now + i) ==
               (i == 2 ? W600_FEEDBACK_SAMPLE : W600_FEEDBACK_BUSY));
    }
    assert(out.valid && out.oldest_at_ms == now && out.completed_at_ms == now + 2);
    for (unsigned i = 0; i < 3; ++i) {
        assert(out.pair[i].bgr == 1495 + i && out.pair[i].signal == signal + i);
        assert((uint32_t)(out.pair[i].acquired_at_ms - now) <= i);
    }
    assert(!adc.CHCFG && !(adc.ISR & FB_FLAGS) && adc.CR == ADC_CR_ADEN);
}
static void failed(enum w600_feedback_error error)
{
    assert(s.phase == FB_FAILED && !s.sample.valid && s.sample.error == error);
    unsigned before = commands;
    assert(poll_at(s.last_ms) == W600_FEEDBACK_FAILED);
    assert(w600_feedback_start(&s, s.last_ms) == W600_FEEDBACK_FAILED);
    w600_feedback_abort(&s);
    assert(commands == before && s.sample.error == error); ++cases;
}
static void vbat_cases(void)
{
    uint32_t mv = 123456;
    reset_model(); calibrated_input(0, true);
    assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    for (unsigned i = 0; i < 3; ++i) {
        assert(adc.CHCFG == ADC_CHCFG_CHN13);
        assert(finish_raw(1495, 20 + i) == W600_FEEDBACK_BUSY);
        assert(adc.CHCFG == ADC_CHCFG_CHN14);
        assert(finish_raw(1365, 20 + i) == (i == 2 ? W600_FEEDBACK_SAMPLE : W600_FEEDBACK_BUSY));
    }
    assert(out.input == W600_FEEDBACK_VBAT && reads == 6 && starts == 6);
    assert(w600_feedback_vbat_mv(&s, &mv) && mv == 3300);
    unchanged(); ++cases;
    struct w600_feedback good = s;
    for (unsigned fault = 0; fault < 10; ++fault) {
        s = good; mv = 123456;
        switch (fault) {
        case 0: s.sample.valid = false; break;
        case 1: s.sample.error = W600_FEEDBACK_RAW; break;
        case 2: s.input = W600_FEEDBACK_MOTOR; break;
        case 3: s.sample.input = W600_FEEDBACK_MOTOR; break;
        case 4: s.factory_bgr = 0; break;
        case 5: s.factory_bgr = 4096; break;
        case 6: s.sample.pair[0].bgr = 0; break;
        case 7: s.sample.pair[1].bgr = 4096; break;
        case 8: s.sample.pair[2].signal = 4096; break;
        default: s.input = 2; break;
        }
        assert(!w600_feedback_vbat_mv(&s, &mv) && mv == 123456); ++cases;
    }
    assert(!w600_feedback_vbat_mv(NULL, &mv) && !w600_feedback_vbat_mv(&good, NULL)); ++cases;
    s = good;
    for (unsigned i = 0; i < 3; ++i) s.sample.pair[i].signal = 0;
    assert(w600_feedback_vbat_mv(&s, &mv) && mv == 0); ++cases;
    s = good; s.factory_bgr = 4095;
    for (unsigned i = 0; i < 3; ++i) { s.sample.pair[i].bgr = 1; s.sample.pair[i].signal = 4095; }
    assert(w600_feedback_vbat_mv(&s, &mv) && mv == 40540500); ++cases;
    for (unsigned fault = 0; fault < 5; ++fault) {
        reset_model(); calibrated_input(0, true);
        assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
        switch (fault) {
        case 0: adc.CFG2 |= ADC_CFG2_VDDA_DIV3; break;
        case 1: adc.CFG2 &= ~ADC_CFG2_VBAT_DIV3; break;
        case 2: adc.CFG2 ^= ADC_CFG2_VBAT_DIV3 | ADC_CFG2_VDDA_DIV3; break;
        case 3: s.input = 2; break;
        default: adc.CHCFG = ADC_CHCFG_CHN16; break;
        }
        assert(poll_at(21) == W600_FEEDBACK_FAILED && !reads && stops == 1);
        failed(fault == 4 ? W600_FEEDBACK_CONVERSION : W600_FEEDBACK_CONFIG);
    }
    reset_model(); calibrated_input(0, true);
    assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    assert(poll_at(25) == W600_FEEDBACK_FAILED && !reads); failed(W600_FEEDBACK_BURST_TIMEOUT);
    reset_model(); calibrated_input(UINT32_MAX - 8, true); burst(UINT32_MAX - 2, 1365); ++cases;
    reset_model(); calibrated_input(0, true);
    assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    w600_feedback_abort(&s); assert(stops == 1); failed(W600_FEEDBACK_ABORTED);
    memset(&adc, 0, sizeof adc);
    assert(w600_feedback_init(&s, 30) == W600_FEEDBACK_BUSY && s.input == W600_FEEDBACK_MOTOR &&
           adc.CFG2 == FB_CFG2); ++cases;
    reset_model(); adc.CFG2 = ADC_CFG2_VDDA_DIV3;
    assert(w600_feedback_init_vbat(&s, 0) == W600_FEEDBACK_FAILED && !commands);
    failed(W600_FEEDBACK_CONFIG);
}
int main(void)
{
    reset_model(); calibrated(100); burst(200, 0);
    assert(out.sequence == 1 && reads == 6);
    assert(poll_at(202) == W600_FEEDBACK_READY);
    burst(203, 100); assert(out.sequence == 2 && reads == 12); ++cases;
    reset_model(); calibrated(UINT32_MAX - 8); burst(UINT32_MAX - 2, 20); ++cases;

    for (unsigned fault = 0; fault < 12; ++fault) {
        reset_model();
        switch (fault) {
        case 0: rcc.CFG = 0x12; break;
        case 1: rcc.PLLCFG ^= 1; break;
        case 2: rcc.CSR1 &= ~RCC_CSR1_PLLRDY; break;
        case 3: flash.ACR = 0; break;
        case 4: rcc.APB1EN = 0; break;
        case 5: rcc.APB2EN = 0; break;
        case 6: rcc.IOPEN = 0; break;
        case 7: pmu.CR1 = 0; break;
        case 8: gpiob.MODE = 0; break;
        case 9: gpiob.PUPD = 1u << 26; break;
        case 10: rcc.CSR1 |= RCC_CSR1_RCHDIV_2; break;
        default: rcc.CFG |= RCC_CFG_P1PRE_2; break;
        }
        save_external();
        assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_FAILED);
        assert(!commands && !clears && !accesses); unchanged(); failed(W600_FEEDBACK_CONTEXT);
    }
    for (unsigned fault = 0; fault < 9; ++fault) {
        reset_model();
        volatile uint32_t *reg[] = {&adc.CR, &adc.CFG1, &adc.CFG2, &adc.ISR,
            &adc.IER, &adc.SAMPT, &adc.CHCFG, &adc.AWDG1CR, &adc.CALFACT};
        *reg[fault] = 1;
        assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_FAILED && !commands);
        failed(W600_FEEDBACK_CONFIG);
    }
    const uint32_t bad_factory[] = {0, 0xffffffff, 0xfa2905d7, 0xffff0000, 0xefff1000};
    for (unsigned i = 0; i < sizeof bad_factory / sizeof *bad_factory; ++i) {
        reset_model(); factory = bad_factory[i];
        assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_FAILED && !commands);
        failed(W600_FEEDBACK_FACTORY);
    }
    reset_model(); pmu.SR = 0; save_external();
    assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_BUSY);
    assert(poll_at(2) == W600_FEEDBACK_BUSY && !commands);
    assert(poll_at(10) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_INIT_TIMEOUT);
    reset_model(); enable_ignored = true;
    assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_BUSY);
    assert(poll_at(2) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_CONFIG);
    reset_model(); cal_ignored = true;
    assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_BUSY);
    assert(poll_at(2) == W600_FEEDBACK_BUSY && poll_at(4) == W600_FEEDBACK_BUSY);
    assert(poll_at(10) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_INIT_TIMEOUT);
    reset_model();
    assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_BUSY);
    assert(poll_at(2) == W600_FEEDBACK_BUSY && poll_at(4) == W600_FEEDBACK_BUSY);
    adc.ISR = ADC_ISR_EOCAL;
    assert(poll_at(5) == W600_FEEDBACK_BUSY);
    assert(poll_at(10) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_INIT_TIMEOUT);
    reset_model();
    assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_BUSY);
    assert(poll_at(2) == W600_FEEDBACK_BUSY);
    adc.ISR = ADC_ISR_EOCAL; clear_ignored = true;
    assert(poll_at(4) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_FLAGS);
    reset_model();
    assert(w600_feedback_init(&s, 0) == W600_FEEDBACK_BUSY);
    for (unsigned i = 0; i < FB_POLL_LIMIT; ++i) assert(poll_at(0) == W600_FEEDBACK_BUSY);
    assert(poll_at(0) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_POLL_LIMIT);

    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    assert(poll_at(19) == W600_FEEDBACK_FAILED && stops == 1); failed(W600_FEEDBACK_TIME);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    assert(poll_at(25) == W600_FEEDBACK_FAILED && !reads); failed(W600_FEEDBACK_BURST_TIMEOUT);
    reset_model(); calibrated(0); start_ignored = true;
    assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    assert(poll_at(25) == W600_FEEDBACK_FAILED && !reads); failed(W600_FEEDBACK_BURST_TIMEOUT);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    for (unsigned i = 0; i < 5; ++i) assert(finish_raw(100, 20 + i) == W600_FEEDBACK_BUSY);
    assert(finish_raw(100, 25) == W600_FEEDBACK_FAILED && reads == 5);
    failed(W600_FEEDBACK_BURST_TIMEOUT);
    for (unsigned fault = 0; fault < 9; ++fault) {
        reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
        enum w600_feedback_error expected = W600_FEEDBACK_FLAGS;
        switch (fault) {
        case 0: adc.ISR = ADC_ISR_OVRN; break;
        case 1: adc.ISR = ADC_ISR_EOCAL; break;
        case 2: adc.ISR = ADC_ISR_AWDG1; break;
        case 3: adc.CHCFG |= ADC_CHCFG_CHN0; expected = W600_FEEDBACK_CONVERSION; break;
        case 4: adc.CR |= ADC_CR_CALEN; expected = W600_FEEDBACK_CONVERSION; break;
        case 5: adc.CFG2 &= ~ADC_CFG2_LFTRG; expected = W600_FEEDBACK_CONFIG; break;
        case 6: adc.IER = ADC_ISR_EOC; expected = W600_FEEDBACK_CONFIG; break;
        case 7: adc.CFG1 |= ADC_CFG1_DMAEN; expected = W600_FEEDBACK_CONFIG; break;
        default: adc.SAMPT = 0; expected = W600_FEEDBACK_CONFIG; break;
        }
        assert(poll_at(21) == W600_FEEDBACK_FAILED && !reads); failed(expected);
    }
    for (unsigned fault = 0; fault < 4; ++fault) {
        reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
        read_ignored = fault == 0; overrun_on_read = fault == 1;
        assert(finish_raw(fault == 2 ? 0 : fault == 3 ? 4096 : 100, 20) == W600_FEEDBACK_FAILED);
        assert(reads == 1); failed(fault < 2 ? W600_FEEDBACK_FLAGS : W600_FEEDBACK_RAW);
    }
    reset_model(); calibrated(0); adc.ISR = ADC_ISR_EOC; clear_ignored = true;
    assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_FAILED && !starts && !reads);
    failed(W600_FEEDBACK_FLAGS);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    w600_feedback_abort(&s); assert(stops == 1); unchanged(); failed(W600_FEEDBACK_ABORTED);
    memset(&adc, 0, sizeof adc);
    assert(w600_feedback_init(&s, 40) == W600_FEEDBACK_BUSY && s.sample.error == W600_FEEDBACK_OK); ++cases;
    reset_model(); assert(poll_at(0) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_BAD_STATE);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_BAD_STATE);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    for (unsigned i = 0; i < FB_POLL_LIMIT; ++i) assert(poll_at(20) == W600_FEEDBACK_BUSY);
    assert(poll_at(20) == W600_FEEDBACK_FAILED && stops == 1); failed(W600_FEEDBACK_POLL_LIMIT);
    for (unsigned partial = 0; partial < 3; ++partial) {
        reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
        adc.ISR = partial == 0 ? ADC_ISR_EOC : partial == 1 ? ADC_ISR_EOS : ADC_ISR_EOC | ADC_ISR_EOS;
        adc.CR = partial == 2 ? ADC_CR_ADEN | ADC_CR_START : ADC_CR_ADEN;
        assert(poll_at(21) == W600_FEEDBACK_BUSY && !reads);
        assert(poll_at(25) == W600_FEEDBACK_FAILED && !reads); failed(W600_FEEDBACK_BURST_TIMEOUT);
    }
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    rcc.APB2EN = 0; save_external(); unsigned before = accesses;
    assert(poll_at(21) == W600_FEEDBACK_FAILED && accesses == before);
    failed(W600_FEEDBACK_CONTEXT);
    reset_model(); calibrated(0); pmu.SR = 0; save_external();
    assert(poll_at(4) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_CONVERSION);
    reset_model(); calibrated(0); s.phase = 99;
    assert(poll_at(4) == W600_FEEDBACK_FAILED); failed(W600_FEEDBACK_BAD_STATE);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    s.index = 3;
    assert(poll_at(20) == W600_FEEDBACK_FAILED && !reads); failed(W600_FEEDBACK_BAD_STATE);
    reset_model(); calibrated(0);
    assert(w600_feedback_poll(&s, 4, NULL) == W600_FEEDBACK_READY); ++cases;
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    assert(w600_feedback_init(&s, 20) == W600_FEEDBACK_FAILED && stops == 1);
    failed(W600_FEEDBACK_BAD_STATE);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    w600_feedback_abort(&s); adc.CR = 0; factory = 0;
    adc.CFG1 = adc.CFG2 = adc.ISR = adc.IER = adc.SAMPT = adc.CHCFG = adc.AWDG1CR = adc.CALFACT = 0;
    assert(w600_feedback_init(&s, 30) == W600_FEEDBACK_FAILED);
    failed(W600_FEEDBACK_FACTORY);
    reset_model(); calibrated(0); assert(w600_feedback_start(&s, 20) == W600_FEEDBACK_BUSY);
    for (unsigned i = 0; i < 5; ++i) assert(finish_raw(100, 20) == W600_FEEDBACK_BUSY);
    adc.CR = ADC_CR_ADEN; adc.DR = 100; adc.ISR = ADC_ISR_EOC | ADC_ISR_EOS;
    assert(w600_feedback_poll(&s, 20, &s.sample) == W600_FEEDBACK_SAMPLE && s.sample.valid);
    unchanged(); ++cases;
    vbat_cases();
    printf("ALL %u ACTUAL-SOURCE FEEDBACK CASES PASSED; no hardware validation\n", cases);
}
