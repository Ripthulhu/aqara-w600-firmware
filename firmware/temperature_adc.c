#include "ciu32l071.h"
#include "temperature_adc.h"
#ifndef TEMP_ADC_OBSERVE
#define TEMP_ADC_OBSERVE(reg,value) ((void)0)
#endif
#ifndef TEMP_ADC_CLEAR_FLAGS
#define TEMP_ADC_CLEAR_FLAGS(mask) (ADC->ISR=(mask))
#endif
#ifndef TEMP_ADC_COMMAND
#define TEMP_ADC_COMMAND(command) (ADC->CR=(command))
#endif
#ifndef TEMP_ADC_READ_DATA
#define TEMP_ADC_READ_DATA() (ADC->DR)
#endif
#define TWRITE(reg,value) do { (reg)=(value); __DSB(); TEMP_ADC_OBSERVE(&(reg),(uint32_t)(value)); } while (0)
#define T_A1 (3u<<2)
#define T_B9 (3u<<18)
#define T_PIN 0x200u
#ifdef W600_DISPLAY_BENCH
#define T_UARTS (RCC_APB1EN_UART2EN|RCC_APB1EN_UART3EN)
#else
#define T_UARTS RCC_APB1EN_UART3EN
#endif
enum { T_OWN_PINS=1, T_FLAGS=ADC_ISR_EOCAL|ADC_ISR_AWDG1|ADC_ISR_OVRN|ADC_ISR_EOS|ADC_ISR_EOC|ADC_ISR_EOSAMP };
static bool t_context(void)
{
    return __get_PRIMASK()==1 && !__get_IPSR() && !__get_CONTROL() &&
        !(NVIC->ISER[0]&~(1u<<UART3_4_IRQn));
}
static bool t_motor_off(void)
{
    return (RCC->IOPEN&7u)==7u && !(RCC->APB1EN&RCC_APB1EN_TIM4EN) &&
        (GPIOB->MODE&0xf0000000u)==0x50000000u &&
        !(GPIOB->OTYPE&0xc000u) && !(GPIOB->PUPD&0xf0000000u) &&
        (GPIOB->ODR&0xc000u)==0xc000u && (GPIOB->IDR&0xc000u)==0xc000u &&
        (GPIOA->MODE&0xc0000000u)==0x40000000u &&
        (GPIOC->MODE&0x00300000u)==0x00100000u &&
        !(GPIOA->OTYPE&0x8000u) && !(GPIOC->OTYPE&0x400u) &&
        !(GPIOA->PUPD&0xc0000000u) && !(GPIOC->PUPD&0x00300000u) &&
        !(GPIOA->ODR&0x8000u) && !(GPIOC->ODR&0x400u) &&
        !(GPIOA->IDR&0x8000u) && !(GPIOC->IDR&0x400u);
}
static bool t_clock(void)
{
    return RCC->PLLCFG==0x30260 &&
        (RCC->CSR1&(RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY|RCC_CSR1_RCHDIV))==
            (RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY) &&
        (FLASH->ACR&FLASH_ACR_LATENCY)==FLASH_ACR_LATENCY_2CLK &&
        !(RCC->AHBEN&RCC_AHBEN_DMAEN);
}
static bool t_pins(bool bias)
{
    uint32_t high=bias ? 0 : T_PIN;
    return (RCC->IOPEN&3u)==3u && (GPIOA->MODE&T_A1)==T_A1 && !(GPIOA->PUPD&T_A1) &&
        (GPIOB->MODE&T_B9)==(1u<<18) && (GPIOB->PUPD&T_B9)==(1u<<18) &&
        !(GPIOB->OTYPE&T_PIN) && (GPIOB->ODR&T_PIN)==high && (GPIOB->IDR&T_PIN)==high;
}
static void t_fail(struct w600_temperature_adc *s,enum w600_temperature_adc_error error)
{
    if (!s->sample.error) s->sample.error=error;
    s->sample.valid=false;
    s->phase=W600_TADC_CLEANUP;
}
static bool t_initial(void)
{
    if (!t_context() || (RCC->IOPEN&7u)!=7u) return false;
    uint32_t mode=GPIOB->MODE&T_B9;
    bool idle=(mode==T_B9 && !(GPIOB->PUPD&T_B9)) ||
        (mode==(1u<<18) && (GPIOB->PUPD&T_B9)==(1u<<18) &&
         !(GPIOB->OTYPE&T_PIN) && (GPIOB->ODR&T_PIN) && (GPIOB->IDR&T_PIN));
    return t_context() && t_motor_off() && t_clock() && RCC->CFG==0x12 &&
        RCC->APB1EN==T_UARTS && !(RCC->APB2EN&~RCC_APB2EN_DBGEN) &&
        !(RCC->APB2RST&RCC_APB2RST_ADC_RST) &&
        (GPIOA->MODE&T_A1)==T_A1 && !(GPIOA->PUPD&T_A1) && idle;
}
static bool t_prepare_pins(struct w600_temperature_adc *s)
{
    s->saved_amode=GPIOA->MODE; s->saved_apull=GPIOA->PUPD;
    s->saved_bmode=GPIOB->MODE; s->saved_bpull=GPIOB->PUPD;
    s->saved_btype=GPIOB->OTYPE; s->saved_bodr=GPIOB->ODR; s->owned=T_OWN_PINS;
    TWRITE(GPIOB->BSR,T_PIN);
    TWRITE(GPIOB->OTYPE,GPIOB->OTYPE&~T_PIN);
    TWRITE(GPIOB->PUPD,(GPIOB->PUPD&~T_B9)|(1u<<18));
    TWRITE(GPIOA->PUPD,GPIOA->PUPD&~T_A1);
    TWRITE(GPIOA->MODE,(GPIOA->MODE&~T_A1)|T_A1);
    TWRITE(GPIOB->MODE,(GPIOB->MODE&~T_B9)|(1u<<18));
    return t_pins(false);
}
static bool t_adc_config(void)
{
    return t_clock() && RCC->CFG==0x40012 && RCC->APB1EN==(T_UARTS|RCC_APB1EN_PMUEN) &&
        (RCC->APB2EN&RCC_APB2EN_ADCEN) && !(RCC->APB2EN&~(RCC_APB2EN_ADCEN|RCC_APB2EN_DBGEN)) &&
        (GPIOB->MODE&(3u<<26))==(3u<<26) && !(GPIOB->PUPD&(3u<<26)) &&
        (PMU->CR1&PMU_CR1_BGR_EN) && (PMU->SR&PMU_SR_BGR_READY) &&
        ADC->CFG1==ADC_CFG1_WAIT_MOD &&
        ADC->CFG2==(ADC_CFG2_CKSRC_PCLK_DIV4|ADC_CFG2_LFTRG|ADC_CFG2_VBGREN|ADC_CFG2_REF) &&
        ADC->SAMPT==0x77 && !ADC->IER && !ADC->AWDG1CR;
}
static bool t_launch(struct w600_temperature_adc *s,uint32_t now)
{
    if (ADC->CR!=ADC_CR_ADEN || ADC->CHCFG) return false;
    TEMP_ADC_CLEAR_FLAGS(T_FLAGS);
    if (ADC->ISR&T_FLAGS) return false;
    TWRITE(ADC->CHCFG,ADC_CHCFG_CHN0);
    if (ADC->CHCFG!=ADC_CHCFG_CHN0) return false;
    s->sample.acquired_at_ms[s->count]=now;
    TEMP_ADC_COMMAND(ADC_CR_START);
    s->phase_ms=now; s->phase=W600_TADC_CONVERT;
    return (ADC->CR&~ADC_CR_START)==ADC_CR_ADEN;
}
static bool t_restore_pins(struct w600_temperature_adc *s)
{
    if (!(s->owned&T_OWN_PINS)) return true;
    if ((RCC->IOPEN&3u)!=3u) return false;
    TWRITE(GPIOB->BSR,T_PIN);
    TWRITE(GPIOB->MODE,(GPIOB->MODE&~T_B9)|(s->saved_bmode&T_B9));
    TWRITE(GPIOB->OTYPE,(GPIOB->OTYPE&~T_PIN)|(s->saved_btype&T_PIN));
    TWRITE(GPIOB->PUPD,(GPIOB->PUPD&~T_B9)|(s->saved_bpull&T_B9));
    TWRITE(GPIOA->MODE,(GPIOA->MODE&~T_A1)|(s->saved_amode&T_A1));
    TWRITE(GPIOA->PUPD,(GPIOA->PUPD&~T_A1)|(s->saved_apull&T_A1));
    if (s->saved_bodr&T_PIN) TWRITE(GPIOB->BSR,T_PIN); else TWRITE(GPIOB->BR,T_PIN);
    bool clean=(GPIOB->MODE&T_B9)==(s->saved_bmode&T_B9) &&
        (GPIOB->PUPD&T_B9)==(s->saved_bpull&T_B9) &&
        (GPIOB->OTYPE&T_PIN)==(s->saved_btype&T_PIN) &&
        (GPIOB->ODR&T_PIN)==(s->saved_bodr&T_PIN) &&
        (GPIOA->MODE&T_A1)==(s->saved_amode&T_A1) &&
        (GPIOA->PUPD&T_A1)==(s->saved_apull&T_A1);
    if (clean) s->owned=0;
    return clean;
}
static void t_cleanup(struct w600_temperature_adc *s,uint32_t now)
{
    if (t_context() && s->owned && (RCC->IOPEN&RCC_IOPEN_GPIOBEN)) TWRITE(GPIOB->BSR,T_PIN);
    bool clean=t_context() && t_motor_off();
    if (!clean) goto failed;
    if (s->feedback.owns_adc) w600_feedback_abort(&s->feedback);
    if (s->board.owned && !w600_feedback_board_finish(&s->board)) clean=false;
    if (s->board.owned) goto failed;
    if (!t_restore_pins(s)) clean=false;
    if (!clean) goto failed;
    s->sample.completed_at_ms=now;
    s->sample.valid=s->sample.error==W600_TADC_OK;
    s->phase=s->sample.valid ? W600_TADC_DONE : W600_TADC_FAULT;
    return;
failed:
    s->cleanup_failed=true; s->sample.valid=false;
    if (!s->sample.error) s->sample.error=W600_TADC_RELEASE;
    s->phase=W600_TADC_FAULT;
}
enum w600_temperature_adc_status w600_temperature_adc_begin(struct w600_temperature_adc *s,uint32_t now,int16_t offset)
{
    if (!t_context() || (s->phase!=W600_TADC_OFF && s->phase!=W600_TADC_DONE) ||
        s->owned || s->board.owned || s->sample.error || offset < -50 || offset > 50) {
        t_fail(s,W600_TADC_STATE); return W600_TADC_FAILED;
    }
    uint32_t sequence=s->sample.sequence+1;
    *s=(struct w600_temperature_adc){0};
    s->sample.sequence=sequence; s->sample.started_at_ms=s->last_ms=now;
    s->offset_decic=offset;
    if (!t_initial()) { t_fail(s,W600_TADC_CONTEXT); return W600_TADC_FAILED; }
    if (!t_prepare_pins(s)) { t_fail(s,W600_TADC_PINS); return W600_TADC_FAILED; }
    if (!w600_feedback_board_prepare(&s->board)) { t_fail(s,W600_TADC_ADC); return W600_TADC_FAILED; }
    if (w600_feedback_init(&s->feedback,now)!=W600_FEEDBACK_BUSY) { t_fail(s,W600_TADC_ADC); return W600_TADC_FAILED; }
    s->phase=W600_TADC_INIT;
    return W600_TADC_BUSY;
}
static void t_poll(struct w600_temperature_adc *s,uint32_t now)
{
    if (!t_context()) { t_fail(s,W600_TADC_CONTEXT); return; }
    if (s->phase==W600_TADC_OFF || s->phase==W600_TADC_DONE || s->phase==W600_TADC_FAULT) return;
    if (s->phase==W600_TADC_CLEANUP) { t_cleanup(s,now); return; }
    if ((uint32_t)(now-s->last_ms)>=0x80000000u || ++s->polls>200000u) { t_fail(s,W600_TADC_TIME); return; }
    s->last_ms=now;
    if (s->count>=W600_TEMPERATURE_READINGS) { t_fail(s,W600_TADC_STATE); return; }
    if ((uint32_t)(now-s->sample.started_at_ms)>=100) { t_fail(s,W600_TADC_TIMEOUT); return; }
    if (!t_motor_off() || !t_pins(s->phase!=W600_TADC_INIT)) { t_fail(s,W600_TADC_PINS); return; }
    if (s->phase==W600_TADC_INIT) {
        enum w600_feedback_status status=w600_feedback_poll(&s->feedback,now,0);
        if (status==W600_FEEDBACK_FAILED) { t_fail(s,W600_TADC_ADC); return; }
        if (status!=W600_FEEDBACK_READY) return;
        s->calfact=ADC->CALFACT;
        TWRITE(GPIOB->BR,T_PIN);
        if (!t_pins(true)) { t_fail(s,W600_TADC_PINS); return; }
        s->phase_ms=now; s->phase=W600_TADC_BIAS; return;
    }
    if (!t_adc_config() || ADC->CALFACT!=s->calfact) { t_fail(s,W600_TADC_ADC); return; }
    if (s->phase==W600_TADC_BIAS || s->phase==W600_TADC_GAP) {
        /* Two coarse ticks guarantee at least1ms even near a tick boundary. */
        if ((uint32_t)(now-s->phase_ms)<2) return;
        if (!t_launch(s,now)) t_fail(s,W600_TADC_FLAGS);
        return;
    }
    if ((uint32_t)(now-s->phase_ms)>=5) { t_fail(s,W600_TADC_TIMEOUT); return; }
    uint32_t cr=ADC->CR, flags=ADC->ISR;
    if ((cr&~ADC_CR_START)!=ADC_CR_ADEN || ADC->CHCFG!=ADC_CHCFG_CHN0 ||
        (flags&(ADC_ISR_OVRN|ADC_ISR_AWDG1|ADC_ISR_EOCAL))) { t_fail(s,W600_TADC_FLAGS); return; }
    if ((cr&ADC_CR_START) || (flags&(ADC_ISR_EOC|ADC_ISR_EOS))!=(ADC_ISR_EOC|ADC_ISR_EOS)) return;
    uint32_t raw=TEMP_ADC_READ_DATA();
    if ((ADC->ISR&(ADC_ISR_EOC|ADC_ISR_OVRN)) || ADC->CR!=ADC_CR_ADEN || raw>4095) {
        t_fail(s,raw>4095 ? W600_TADC_RAW : W600_TADC_FLAGS); return;
    }
    s->sample.raw[s->count++]=(uint16_t)raw;
    TWRITE(ADC->CHCFG,0); TEMP_ADC_CLEAR_FLAGS(T_FLAGS);
    if (ADC->CHCFG || (ADC->ISR&T_FLAGS)) { t_fail(s,W600_TADC_FLAGS); return; }
    if (s->count==W600_TEMPERATURE_READINGS) {
        TWRITE(GPIOB->BSR,T_PIN);
        if (!t_pins(false)) { t_fail(s,W600_TADC_PINS); return; }
        if (w600_temperature_convert(s->sample.raw,s->offset_decic,&s->sample.temperature)!=W600_TEMPERATURE_OK)
            t_fail(s,W600_TADC_RANGE);
        else s->phase=W600_TADC_CLEANUP;
    } else { s->phase_ms=now; s->phase=W600_TADC_GAP; }
}
enum w600_temperature_adc_status w600_temperature_adc_poll(struct w600_temperature_adc *s,uint32_t now,
                                                            struct w600_temperature_adc_sample *out)
{
    t_poll(s,now);
    if (out) { out->valid=false; out->error=s->sample.error; }
    if (s->phase==W600_TADC_DONE && !s->delivered) {
        s->delivered=true; if (out) *out=s->sample; return W600_TADC_SAMPLE;
    }
    if (s->phase==W600_TADC_FAULT) return W600_TADC_FAILED;
    return s->phase==W600_TADC_OFF || s->phase==W600_TADC_DONE ? W600_TADC_IDLE : W600_TADC_BUSY;
}
void w600_temperature_adc_abort(struct w600_temperature_adc *s)
{
    if (s->phase>=W600_TADC_INIT && s->phase<=W600_TADC_CLEANUP) t_fail(s,W600_TADC_ABORTED);
}
