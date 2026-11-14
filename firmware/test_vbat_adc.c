#include <assert.h>
#include <stdbool.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"
static RCC_t rcc;
static PMU_t pmu;
static ADC_t adc;
static GPIO_t a,b,c,initial_a,initial_b,initial_c;
static FLASH_t flash;
static NVIC_Type nvic;
static uint32_t mask,ipsr,control,factory;
static unsigned starts,reads,stops,resets,fault,cases;
static uint32_t signal_code,bgr_code;
static void flags_clear(uint32_t);
static void adc_command(uint32_t);
static uint32_t adc_data(void);
static void board_observe(unsigned);
static ADC_t *adc_regs(void) { assert(rcc.APB2EN&RCC_APB2EN_ADCEN); return &adc; }
static PMU_t *pmu_regs(void) { assert(rcc.APB1EN&RCC_APB1EN_PMUEN); return &pmu; }
#undef RCC
#undef PMU
#undef ADC
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef FLASH
#undef NVIC
#define RCC (&rcc)
#define PMU pmu_regs()
#define ADC adc_regs()
#define GPIOA (&a)
#define GPIOB (&b)
#define GPIOC (&c)
#define FLASH (&flash)
#define NVIC (&nvic)
#define __get_PRIMASK() mask
#define __get_IPSR() ipsr
#define __get_CONTROL() control
#define __DSB() ((void)0)
#define FB_BOARD_OBSERVE(stage) board_observe(stage)
#define FEEDBACK_FACTORY_BGR factory
#define FEEDBACK_CLEAR_FLAGS(bits) flags_clear(bits)
#define FEEDBACK_COMMAND(value) adc_command(value)
#define FEEDBACK_READ_DATA() adc_data()
#include "../startup/w600_feedback_board.c"
#include "../startup/w600_feedback.c"
#include "vbat_adc.c"
#ifdef W600_DISPLAY_BENCH
#define V_UARTS (RCC_APB1EN_UART2EN|RCC_APB1EN_UART3EN)
#else
#define V_UARTS RCC_APB1EN_UART3EN
#endif
static void pads(void) { a.IDR=a.ODR; b.IDR=b.ODR; c.IDR=c.ODR; }
static void flags_clear(uint32_t bits) { assert(bits==FB_FLAGS); adc.ISR&=~bits; }
static void adc_command(uint32_t command)
{
    if (command==ADC_CR_ADEN) adc.CR=ADC_CR_ADEN;
    else if (command==ADC_CR_CALEN) { assert(adc.CR==ADC_CR_ADEN); adc.CR|=ADC_CR_CALEN; }
    else if (command==ADC_CR_START) {
        assert(adc.CR==ADC_CR_ADEN && (adc.CHCFG==ADC_CHCFG_CHN13 || adc.CHCFG==ADC_CHCFG_CHN14));
        assert((adc.CFG2&(ADC_CFG2_VBAT_DIV3|ADC_CFG2_VDDA_DIV3))==ADC_CFG2_VBAT_DIV3);
        ++starts; adc.CR|=ADC_CR_START;
    } else { assert(command==ADC_CR_STOP); ++stops; adc.CR=ADC_CR_ADEN; }
}
static uint32_t adc_data(void)
{
    assert(adc.CR==ADC_CR_ADEN && (adc.ISR&(ADC_ISR_EOC|ADC_ISR_EOS))==(ADC_ISR_EOC|ADC_ISR_EOS));
    ++reads; adc.ISR&=~ADC_ISR_EOC;
    if (fault==6) adc.ISR|=ADC_ISR_OVRN;
    return adc.DR;
}
static void board_observe(unsigned stage)
{
    if (stage==RESET_ON || stage==CLEAN_RESET_ON) { memset(&adc,0,sizeof adc); ++resets; }
    if (stage==SET_BGR && fault!=1) pmu.SR=PMU_SR_BGR_READY;
    if (stage==CLEAN_BGR && !(pmu.CR1&PMU_CR1_BGR_EN)) pmu.SR=0;
    if (stage==CLEAN_ADC_GATE && fault==7) rcc.APB2EN|=RCC_APB2EN_ADCEN;
    if (stage==SET_ADC_CLOCK && fault==8) rcc.APB2EN&=~RCC_APB2EN_ADCEN;
    assert(a.MODE==initial_a.MODE && a.ODR==initial_a.ODR && c.MODE==initial_c.MODE && c.ODR==initial_c.ODR);
    assert((b.MODE&~PB13_FIELDS)==(initial_b.MODE&~PB13_FIELDS) && b.ODR==initial_b.ODR);
    pads();
}
static void fresh(struct w600_vbat_adc *s)
{
    memset(s,0,sizeof *s); memset(&rcc,0,sizeof rcc); memset(&pmu,0,sizeof pmu);
    memset(&adc,0,sizeof adc); memset(&a,0,sizeof a); memset(&b,0,sizeof b); memset(&c,0,sizeof c);
    memset(&nvic,0,sizeof nvic); memset(&flash,0,sizeof flash);
    rcc.IOPEN=7; rcc.CFG=0x12; rcc.PLLCFG=0x30260; rcc.AHBEN=RCC_AHBEN_FLASHEN;
    rcc.CSR1=RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY; rcc.APB1EN=V_UARTS;
    flash.ACR=FLASH_ACR_LATENCY_2CLK; mask=1; ipsr=control=0; factory=0xfa2805d7;
    a.MODE=0x7fffffffu; b.MODE=0x5fffffffu; c.MODE=0xffdfffffu;
    b.ODR=0xc000u; b.PUPD=1u<<16; pads(); initial_a=a; initial_b=b; initial_c=c;
    starts=reads=stops=resets=0; bgr_code=1495; signal_code=1365;
}
static void hardware(void)
{
    if ((adc.CR&ADC_CR_CALEN) && fault!=2) { adc.CR=ADC_CR_ADEN; adc.ISR|=ADC_ISR_EOCAL; adc.CALFACT=21; }
    if ((adc.CR&ADC_CR_START) && fault!=3) {
        adc.CR=ADC_CR_ADEN; adc.ISR|=ADC_ISR_EOC|ADC_ISR_EOS;
        adc.DR=adc.CHCFG==ADC_CHCFG_CHN13 ? (fault==5 ? 0 : bgr_code) : (fault==4 ? 4096 : signal_code);
    }
}
static void restored(void)
{
    assert(a.MODE==initial_a.MODE && a.PUPD==initial_a.PUPD && a.ODR==initial_a.ODR);
    assert(b.MODE==initial_b.MODE && b.PUPD==initial_b.PUPD && b.OTYPE==initial_b.OTYPE && b.ODR==initial_b.ODR);
    assert(c.MODE==initial_c.MODE && c.PUPD==initial_c.PUPD && c.ODR==initial_c.ODR);
    assert(rcc.CFG==0x12 && rcc.APB1EN==V_UARTS && !rcc.APB2EN && !pmu.CR1);
    assert(!adc.CR && !adc.CFG2 && !adc.CHCFG);
}
static enum w600_vbat_adc_status run(struct w600_vbat_adc *s,uint32_t first,struct w600_vbat_adc_sample *out)
{
    for (uint32_t step=0;step<900;++step) {
        hardware();
        enum w600_vbat_adc_status status=w600_vbat_adc_poll(s,first+step/8u,out);
        if (status==W600_VBAT_ADC_SAMPLE || status==W600_VBAT_ADC_FAILED) return status;
        assert(!out->valid);
    }
    assert(0); return W600_VBAT_ADC_FAILED;
}
int main(void)
{
    struct w600_vbat_adc s; struct w600_vbat_adc_sample out;
    fault=0;
    for (unsigned wrap=0;wrap<2;++wrap) {
        fresh(&s); uint32_t now=wrap ? UINT32_MAX-3u : 100u;
        assert(w600_vbat_adc_begin(&s,now)==W600_VBAT_ADC_BUSY);
        assert(run(&s,now,&out)==W600_VBAT_ADC_SAMPLE && out.valid && out.millivolts==3300 && out.sequence==1);
        assert(starts==6 && reads==6 && out.raw.valid && out.factory_bgr==1495);
        assert(!s.board.owned && !s.feedback.owns_adc && !s.cleanup_failed);
        assert((uint32_t)(out.completed_at_ms-now)<100 && out.raw.input==W600_FEEDBACK_VBAT); restored();
        assert(w600_vbat_adc_poll(&s,now+20,&out)==W600_VBAT_ADC_IDLE && !out.valid);
        starts=reads=0;
        assert(w600_vbat_adc_begin(&s,now+21)==W600_VBAT_ADC_BUSY);
        assert(run(&s,now+21,&out)==W600_VBAT_ADC_SAMPLE && out.sequence==2); restored(); ++cases;
    }
    for (fault=1;fault<=8;++fault) {
        fresh(&s); enum w600_vbat_adc_status accepted=w600_vbat_adc_begin(&s,100);
        assert(accepted==(fault==8 ? W600_VBAT_ADC_FAILED : W600_VBAT_ADC_BUSY));
        assert(run(&s,100,&out)==W600_VBAT_ADC_FAILED && !out.valid && s.sample.error);
        if (fault==7) assert(s.cleanup_failed && s.board.owned && s.feedback.owns_adc);
        else { restored(); assert(!s.board.owned && !s.feedback.owns_adc); }
        unsigned saved=resets; assert(w600_vbat_adc_begin(&s,120)==W600_VBAT_ADC_FAILED && resets==saved);
        assert(w600_vbat_adc_poll(&s,121,&out)==W600_VBAT_ADC_FAILED); ++cases;
    }
    fault=0;
    for (unsigned which=0;which<5;++which) {
        fresh(&s); factory=0xf0000fffu; bgr_code=990;
        signal_code=(uint32_t[]){99,100,360,361,4095}[which];
        assert(w600_vbat_adc_begin(&s,0)==W600_VBAT_ADC_BUSY);
        enum w600_vbat_adc_status status=run(&s,0,&out);
        bool valid=which==1 || which==2;
        assert(status==(valid ? W600_VBAT_ADC_SAMPLE : W600_VBAT_ADC_FAILED));
        assert(s.sample.millivolts==signal_code*10 && s.sample.valid==valid);
        assert(s.sample.error==(valid ? W600_VBAT_ADC_OK : W600_VBAT_ADC_RANGE)); restored(); ++cases;
    }
    fresh(&s); factory=0;
    assert(w600_vbat_adc_begin(&s,0)==W600_VBAT_ADC_FAILED);
    assert(run(&s,0,&out)==W600_VBAT_ADC_FAILED && !out.valid && s.sample.factory_bgr==0); restored(); ++cases;
    fresh(&s); assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_BUSY);
    w600_vbat_adc_abort(&s); assert(run(&s,100,&out)==W600_VBAT_ADC_FAILED && s.sample.error==W600_VBAT_ADC_ABORTED); restored(); ++cases;
    for (unsigned change=0;change<2;++change) {
        fresh(&s); assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_BUSY);
        uint32_t step=0;
        while (s.phase!=W600_VBAT_ADC_CONVERT) {
            hardware(); assert(w600_vbat_adc_poll(&s,100+step++/8u,&out)==W600_VBAT_ADC_BUSY);
        }
        assert(adc.CR&ADC_CR_START);
        if (change==0) w600_vbat_adc_abort(&s);
        else {
            adc.CFG2|=ADC_CFG2_VDDA_DIV3;
            assert(w600_vbat_adc_poll(&s,100+step/8u,&out)==W600_VBAT_ADC_BUSY);
        }
        assert(w600_vbat_adc_poll(&s,100+step/8u,&out)==W600_VBAT_ADC_FAILED && !out.valid && stops==1);
        assert(s.sample.error==(change ? W600_VBAT_ADC_ACQUIRE : W600_VBAT_ADC_ABORTED)); restored(); ++cases;
    }
    fresh(&s); assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_BUSY);
    assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_FAILED);
    assert(run(&s,100,&out)==W600_VBAT_ADC_FAILED && !starts); restored(); ++cases;
    for (unsigned which=0;which<2;++which) {
        fresh(&s); assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_BUSY);
        assert(w600_vbat_adc_poll(&s,which ? 200 : 99,&out)==W600_VBAT_ADC_BUSY);
        assert(run(&s,200,&out)==W600_VBAT_ADC_FAILED);
        assert(s.sample.error==(which ? W600_VBAT_ADC_TIMEOUT : W600_VBAT_ADC_TIME)); restored(); ++cases;
    }
    fresh(&s); assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_BUSY);
    s.polls=200000; assert(w600_vbat_adc_poll(&s,100,&out)==W600_VBAT_ADC_BUSY);
    assert(run(&s,100,&out)==W600_VBAT_ADC_FAILED && s.sample.error==W600_VBAT_ADC_POLL_LIMIT); restored(); ++cases;
    fresh(&s); rcc.APB1EN|=RCC_APB1EN_TIM4EN;
    assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_FAILED && !resets && !starts && !s.board.owned); ++cases;
    fresh(&s); assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_BUSY);
    rcc.APB1EN|=RCC_APB1EN_TIM4EN;
    assert(run(&s,100,&out)==W600_VBAT_ADC_FAILED && s.cleanup_failed && s.board.owned && s.feedback.owns_adc);
    assert(s.sample.error==W600_VBAT_ADC_CONTEXT); ++cases;
    for (unsigned which=0;which<3;++which) {
        fresh(&s); if (which==0) mask=0; else if (which==1) ipsr=1; else control=1;
        assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_FAILED && !resets && !s.board.owned); ++cases;
    }
    fresh(&s); assert(w600_vbat_adc_poll(&s,0,&out)==W600_VBAT_ADC_IDLE && !out.valid);
    w600_vbat_adc_abort(&s); assert(!s.sample.error);
    assert(w600_vbat_adc_begin(NULL,0)==W600_VBAT_ADC_FAILED && w600_vbat_adc_poll(NULL,0,&out)==W600_VBAT_ADC_FAILED && !out.valid);
    w600_vbat_adc_abort(NULL); ++cases;
    fresh(&s); s.feedback.owns_adc=true;
    assert(w600_vbat_adc_begin(&s,0)==W600_VBAT_ADC_FAILED && s.feedback.owns_adc && !resets);
    assert(w600_vbat_adc_poll(&s,0,&out)==W600_VBAT_ADC_FAILED && s.feedback.owns_adc && s.cleanup_failed && !resets); ++cases;
    fresh(&s); assert(w600_vbat_adc_begin(&s,0)==W600_VBAT_ADC_BUSY);
    uint32_t step=0;
    while (s.phase!=W600_VBAT_ADC_CLEANUP) { hardware(); assert(w600_vbat_adc_poll(&s,step++/8u,0)==W600_VBAT_ADC_BUSY); }
    assert(!s.sample.valid && s.board.owned && s.feedback.owns_adc);
    assert(w600_vbat_adc_poll(&s,step/8u,0)==W600_VBAT_ADC_SAMPLE && s.sample.valid);
    assert(w600_vbat_adc_poll(&s,step/8u,&out)==W600_VBAT_ADC_IDLE && !out.valid); restored(); ++cases;
    for (unsigned change=0;change<2;++change) {
        fresh(&s); assert(w600_vbat_adc_begin(&s,100)==W600_VBAT_ADC_BUSY); step=0;
        while (s.phase!=W600_VBAT_ADC_CLEANUP) {
            hardware(); assert(w600_vbat_adc_poll(&s,100+step++/8u,&out)==W600_VBAT_ADC_BUSY);
        }
        assert(w600_vbat_adc_poll(&s,change ? 200 : 99,&out)==W600_VBAT_ADC_FAILED && !out.valid);
        assert(s.sample.error==(change ? W600_VBAT_ADC_TIMEOUT : W600_VBAT_ADC_TIME)); restored(); ++cases;
    }
    fresh(&s); s.phase=(enum w600_vbat_adc_phase)99;
    assert(w600_vbat_adc_poll(&s,0,&out)==W600_VBAT_ADC_BUSY && !resets);
    assert(w600_vbat_adc_poll(&s,0,&out)==W600_VBAT_ADC_FAILED && s.sample.error==W600_VBAT_ADC_STATE && !resets); ++cases;
    printf("PASS %u VBAT ADC cases: actual board/sampler source, CH14 only, range/raw/factory/time/abort/reuse/cleanup/ownership; no hardware tested\n",cases);
}
