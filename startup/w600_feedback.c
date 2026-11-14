#include <stddef.h>
#include "ciu32l071.h"
#include "w600_feedback.h"

#ifndef FEEDBACK_FACTORY_BGR
#define FEEDBACK_FACTORY_BGR (*(volatile const uint32_t *)0x1fff03c0u)
#endif
#ifndef FEEDBACK_CLEAR_FLAGS
#define FEEDBACK_CLEAR_FLAGS(mask) (ADC->ISR = (mask))
#endif
#ifndef FEEDBACK_COMMAND
#define FEEDBACK_COMMAND(command) (ADC->CR = (command))
#endif
#ifndef FEEDBACK_READ_DATA
#define FEEDBACK_READ_DATA() (ADC->DR)
#endif

enum { FB_OFF, FB_BGR_WAIT, FB_ADC_WAIT, FB_CAL_WAIT, FB_READY,
       FB_BGR_CONVERT, FB_SIGNAL_CONVERT, FB_FAILED };
enum { FB_INIT_MS = 10, FB_BURST_MS = 5, FB_POLL_LIMIT = 100000,
       FB_FLAGS = ADC_ISR_EOCAL | ADC_ISR_AWDG1 | ADC_ISR_OVRN |
                  ADC_ISR_EOS | ADC_ISR_EOC | ADC_ISR_EOSAMP,
       FB_CFG1 = ADC_CFG1_WAIT_MOD,
       FB_CFG2 = ADC_CFG2_CKSRC_PCLK_DIV4 | ADC_CFG2_LFTRG |
                 ADC_CFG2_VBGREN | ADC_CFG2_REF,
       FB_SAMPT = ADC_SAMPT_SAMPT1_119CYCLES | ADC_SAMPT_SAMPT2_119CYCLES };

_Static_assert(offsetof(ADC_t, ISR) == 0x10 && offsetof(ADC_t, CHCFG) == 0x1c &&
               offsetof(ADC_t, DR) == 0x40, "Vendor ADC layout");
_Static_assert(ADC_CR_ADEN == 1 && ADC_CR_START == 4 && ADC_ISR_EOC == 4 &&
               ADC_ISR_OVRN == 16 && ADC_CFG2_LFTRG == 0x100, "ADC masks");
_Static_assert(FB_SAMPT == 0x77, "Both sampling groups use119cycles");

static bool context_ok(void)
{
    if (RCC->CFG != 0x40012u || RCC->PLLCFG != 0x30260u ||
        (RCC->CSR1 & (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY |
                       RCC_CSR1_RCHDIV)) !=
            (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY) ||
        (FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2CLK ||
        !(RCC->APB1EN & RCC_APB1EN_PMUEN) ||
        !(RCC->APB2EN & RCC_APB2EN_ADCEN) || !(RCC->IOPEN & RCC_IOPEN_GPIOBEN))
        return false;
    return (PMU->CR1 & PMU_CR1_BGR_EN) &&
           (GPIOB->MODE & (3u << 26)) == (3u << 26) &&
           !(GPIOB->PUPD & (3u << 26));
}

static uint32_t signal_channel(const struct w600_feedback *s)
{
    return s->input == W600_FEEDBACK_VBAT ? ADC_CHCFG_CHN14 : ADC_CHCFG_CHN16;
}

static uint32_t input_config(uint8_t input)
{
    return FB_CFG2 | (input == W600_FEEDBACK_VBAT ? ADC_CFG2_VBAT_DIV3 : 0u);
}

static bool config_ok(const struct w600_feedback *s)
{
    return s->input <= W600_FEEDBACK_VBAT && ADC->CFG1 == FB_CFG1 &&
           ADC->CFG2 == input_config(s->input) &&
           ADC->SAMPT == FB_SAMPT && ADC->IER == 0 && ADC->AWDG1CR == 0;
}

static enum w600_feedback_status fail(struct w600_feedback *s,
                                      enum w600_feedback_error error, bool stop)
{
    if (s->phase != FB_FAILED) s->sample.error = error;
    s->sample.valid = false;
    s->phase = FB_FAILED;
    if (stop && s->owns_adc && (RCC->APB2EN & RCC_APB2EN_ADCEN)) {
        const uint32_t cr = ADC->CR;
        if ((cr & (ADC_CR_ADEN | ADC_CR_START | ADC_CR_ADDIS | ADC_CR_CALEN)) ==
            (ADC_CR_ADEN | ADC_CR_START)) FEEDBACK_COMMAND(ADC_CR_STOP);
    }
    return W600_FEEDBACK_FAILED;
}

static bool time_ok(struct w600_feedback *s, uint32_t now)
{
    if ((uint32_t)(now - s->last_ms) >= 0x80000000u) return false;
    s->last_ms = now;
    return true;
}

static bool launch(struct w600_feedback *s, uint32_t channel, uint32_t now)
{
    if (ADC->CR != ADC_CR_ADEN) return false;
    FEEDBACK_CLEAR_FLAGS(FB_FLAGS);
    if (ADC->ISR & FB_FLAGS) return false;
    ADC->CHCFG = channel;
    if (ADC->CHCFG != channel) return false;
    if (channel == ADC_CHCFG_CHN13) s->sample.pair[s->index].acquired_at_ms = now;
    FEEDBACK_COMMAND(ADC_CR_START);
    return true;
}

static enum w600_feedback_status init_input(struct w600_feedback *s, uint32_t now, uint8_t input)
{
    if (s->phase != FB_OFF && s->phase != FB_FAILED)
        return fail(s, W600_FEEDBACK_BAD_STATE, context_ok());
    s->phase = FB_OFF;
    s->owns_adc = false;
    s->sample.valid = false;
    s->sample.error = W600_FEEDBACK_OK;
    s->input = s->sample.input = input;
    s->last_ms = s->operation_ms = s->phase_ms = now;
    s->polls = 0; s->index = 0;
    if (!context_ok()) return fail(s, W600_FEEDBACK_CONTEXT, false);
    if (ADC->CR || ADC->CFG1 || ADC->CFG2 || ADC->ISR || ADC->IER ||
        ADC->SAMPT || ADC->CHCFG || ADC->AWDG1CR || ADC->CALFACT)
        return fail(s, W600_FEEDBACK_CONFIG, false);
    const uint32_t factory_word = FEEDBACK_FACTORY_BGR;
    const uint32_t code = factory_word & 0xffffu;
    if ((((factory_word >> 16) ^ code) & 0xffffu) != 0xffffu || code == 0 || code > 4095)
        return fail(s, W600_FEEDBACK_FACTORY, false);
    s->factory_bgr = (uint16_t)code;
    s->owns_adc = true;
    ADC->CFG1 = FB_CFG1; ADC->CFG2 = input_config(s->input);
    ADC->SAMPT = FB_SAMPT; ADC->CHCFG = 0;
    if (!config_ok(s)) return fail(s, W600_FEEDBACK_CONFIG, false);
    s->phase = FB_BGR_WAIT;
    return W600_FEEDBACK_BUSY;
}

enum w600_feedback_status w600_feedback_init(struct w600_feedback *s, uint32_t now)
{
    return init_input(s, now, W600_FEEDBACK_MOTOR);
}

enum w600_feedback_status w600_feedback_init_vbat(struct w600_feedback *s, uint32_t now)
{
    return init_input(s, now, W600_FEEDBACK_VBAT);
}

bool w600_feedback_vbat_mv(const struct w600_feedback *s, uint32_t *millivolts)
{
    if (!s || !millivolts || !s->sample.valid || s->sample.error ||
        s->input != W600_FEEDBACK_VBAT || s->sample.input != W600_FEEDBACK_VBAT ||
        !s->factory_bgr || s->factory_bgr > 4095) return false;
    uint32_t sum = 0;
    for (unsigned i = 0; i < 3; ++i) {
        const struct w600_feedback_pair *p = &s->sample.pair[i];
        if (!p->bgr || p->bgr > 4095 || p->signal > 4095) return false;
        sum += (uint32_t)(((uint64_t)p->signal * s->factory_bgr * 9900u) /
                         ((uint32_t)p->bgr * 4095u));
    }
    *millivolts = sum / 3u;
    return true;
}

enum w600_feedback_status w600_feedback_start(struct w600_feedback *s, uint32_t now)
{
    if (s->phase == FB_FAILED) return W600_FEEDBACK_FAILED;
    if (s->phase != FB_READY) return fail(s, W600_FEEDBACK_BAD_STATE, context_ok());
    if (!time_ok(s, now)) return fail(s, W600_FEEDBACK_TIME, true);
    if (!context_ok()) return fail(s, W600_FEEDBACK_CONTEXT, false);
    if (!config_ok(s) || ADC->CR != ADC_CR_ADEN || ADC->CHCFG != 0 ||
        !(PMU->SR & PMU_SR_BGR_READY)) return fail(s, W600_FEEDBACK_CONFIG, true);
    s->sample.valid = false; s->sample.error = W600_FEEDBACK_OK;
    s->operation_ms = now; s->polls = 0; s->index = 0;
    s->sample.oldest_at_ms = now;
    if (!launch(s, ADC_CHCFG_CHN13, now)) return fail(s, W600_FEEDBACK_FLAGS, true);
    s->phase = FB_BGR_CONVERT;
    return W600_FEEDBACK_BUSY;
}

static enum w600_feedback_status poll_step(struct w600_feedback *s, uint32_t now)
{
    if (s->phase == FB_FAILED) return W600_FEEDBACK_FAILED;
    if (s->phase == FB_OFF || s->phase > FB_SIGNAL_CONVERT)
        return fail(s, W600_FEEDBACK_BAD_STATE, false);
    if (!time_ok(s, now)) return fail(s, W600_FEEDBACK_TIME, true);
    if (!context_ok()) return fail(s, W600_FEEDBACK_CONTEXT, false);
    if (!config_ok(s)) return fail(s, W600_FEEDBACK_CONFIG, true);
    if (s->phase == FB_READY) {
        if (ADC->CR != ADC_CR_ADEN || ADC->CHCFG || !(PMU->SR & PMU_SR_BGR_READY))
            return fail(s, W600_FEEDBACK_CONVERSION, true);
        return W600_FEEDBACK_READY;
    }
    if (++s->polls > FB_POLL_LIMIT) return fail(s, W600_FEEDBACK_POLL_LIMIT, true);
    const bool initializing = s->phase <= FB_CAL_WAIT;
    if ((uint32_t)(now - s->operation_ms) >= (initializing ? FB_INIT_MS : FB_BURST_MS))
        return fail(s, initializing ? W600_FEEDBACK_INIT_TIMEOUT :
                    W600_FEEDBACK_BURST_TIMEOUT, true);
    if (s->phase == FB_BGR_WAIT) {
        if (ADC->CR != 0 || ADC->CHCFG != 0) return fail(s, W600_FEEDBACK_CONFIG, true);
        if ((uint32_t)(now - s->phase_ms) < 2 || !(PMU->SR & PMU_SR_BGR_READY))
            return W600_FEEDBACK_BUSY;
        FEEDBACK_COMMAND(ADC_CR_ADEN);
        if (ADC->CR != ADC_CR_ADEN) return fail(s, W600_FEEDBACK_CONFIG, true);
        s->phase_ms = now; s->phase = FB_ADC_WAIT;
        return W600_FEEDBACK_BUSY;
    }
    if (s->phase == FB_ADC_WAIT) {
        if (ADC->CR != ADC_CR_ADEN || ADC->CHCFG != 0)
            return fail(s, W600_FEEDBACK_CONFIG, true);
        if (!(PMU->SR & PMU_SR_BGR_READY))
            return fail(s, W600_FEEDBACK_CONVERSION, true);
        if ((uint32_t)(now - s->phase_ms) < 2) return W600_FEEDBACK_BUSY;
        FEEDBACK_CLEAR_FLAGS(FB_FLAGS);
        if (ADC->ISR & FB_FLAGS) return fail(s, W600_FEEDBACK_FLAGS, true);
        FEEDBACK_COMMAND(ADC_CR_CALEN);
        s->phase = FB_CAL_WAIT;
        return W600_FEEDBACK_BUSY;
    }
    if (s->phase == FB_CAL_WAIT) {
        const uint32_t cr = ADC->CR, status = ADC->ISR;
        if ((cr & ~ADC_CR_CALEN) != ADC_CR_ADEN || ADC->CHCFG != 0 ||
            (status & (FB_FLAGS & ~ADC_ISR_EOCAL)) || !(PMU->SR & PMU_SR_BGR_READY))
            return fail(s, W600_FEEDBACK_CONVERSION, true);
        if (!(status & ADC_ISR_EOCAL) || (cr & ADC_CR_CALEN)) return W600_FEEDBACK_BUSY;
        if (ADC->CALFACT & ~ADC_CALFACT_CALFACT) return fail(s, W600_FEEDBACK_RAW, true);
        FEEDBACK_CLEAR_FLAGS(FB_FLAGS);
        if (ADC->ISR & FB_FLAGS) return fail(s, W600_FEEDBACK_FLAGS, true);
        s->phase = FB_READY;
        return W600_FEEDBACK_READY;
    }
    if (s->index >= 3) return fail(s, W600_FEEDBACK_BAD_STATE, true);
    const uint32_t channel = s->phase == FB_BGR_CONVERT ? ADC_CHCFG_CHN13 : signal_channel(s);
    const uint32_t cr = ADC->CR, status = ADC->ISR;
    if ((cr & ~ADC_CR_START) != ADC_CR_ADEN || ADC->CHCFG != channel ||
        !(PMU->SR & PMU_SR_BGR_READY)) return fail(s, W600_FEEDBACK_CONVERSION, true);
    if (status & (ADC_ISR_OVRN | ADC_ISR_AWDG1 | ADC_ISR_EOCAL))
        return fail(s, W600_FEEDBACK_FLAGS, true);
    if ((status & (ADC_ISR_EOC | ADC_ISR_EOS)) != (ADC_ISR_EOC | ADC_ISR_EOS) ||
        (cr & ADC_CR_START)) return W600_FEEDBACK_BUSY;
    const uint32_t raw = FEEDBACK_READ_DATA();
    if ((ADC->ISR & (ADC_ISR_EOC | ADC_ISR_OVRN)) || ADC->CR != ADC_CR_ADEN)
        return fail(s, W600_FEEDBACK_FLAGS, true);
    if (raw > 4095 || (s->phase == FB_BGR_CONVERT && raw == 0))
        return fail(s, W600_FEEDBACK_RAW, true);
    if (s->phase == FB_BGR_CONVERT) {
        s->sample.pair[s->index].bgr = (uint16_t)raw;
        if (!launch(s, signal_channel(s), now)) return fail(s, W600_FEEDBACK_FLAGS, true);
        s->phase = FB_SIGNAL_CONVERT;
        return W600_FEEDBACK_BUSY;
    }
    s->sample.pair[s->index].signal = (uint16_t)raw;
    if (++s->index < 3) {
        if (!launch(s, ADC_CHCFG_CHN13, now)) return fail(s, W600_FEEDBACK_FLAGS, true);
        s->phase = FB_BGR_CONVERT;
        return W600_FEEDBACK_BUSY;
    }
    ADC->CHCFG = 0;
    FEEDBACK_CLEAR_FLAGS(FB_FLAGS);
    if (ADC->CHCFG || (ADC->ISR & FB_FLAGS)) return fail(s, W600_FEEDBACK_FLAGS, true);
    s->sample.completed_at_ms = now;
    ++s->sample.sequence;
    s->sample.valid = true;
    s->phase = FB_READY;
    return W600_FEEDBACK_SAMPLE;
}

enum w600_feedback_status w600_feedback_poll(struct w600_feedback *s, uint32_t now,
                                            struct w600_feedback_sample *out)
{
    const enum w600_feedback_status status = poll_step(s, now);
    if (out) {
        if (status == W600_FEEDBACK_SAMPLE) *out = s->sample;
        else { out->valid = false; out->error = s->sample.error; }
    }
    return status;
}

void w600_feedback_abort(struct w600_feedback *s)
{
    if (s->phase != FB_FAILED)
        (void)fail(s, W600_FEEDBACK_ABORTED, context_ok());
}
