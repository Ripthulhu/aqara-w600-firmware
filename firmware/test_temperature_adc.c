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
static unsigned starts,reads,writes,resets,fault,cases;
static bool clear_ignored;
static uint16_t supplied[5];
static void flags_clear(uint32_t);
static void adc_command(uint32_t);
static uint32_t adc_data(void);
static void t_written(volatile uint32_t *,uint32_t);
static void board_observe(unsigned);
static GPIO_t *a_regs(void) { assert(rcc.IOPEN&1u); return &a; }
static GPIO_t *b_regs(void) { assert(rcc.IOPEN&2u); return &b; }
static GPIO_t *c_regs(void) { assert(rcc.IOPEN&4u); return &c; }
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
#define GPIOA a_regs()
#define GPIOB b_regs()
#define GPIOC c_regs()
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
#define TEMP_ADC_OBSERVE(reg,value) t_written(reg,value)
#define TEMP_ADC_CLEAR_FLAGS(bits) flags_clear(bits)
#define TEMP_ADC_COMMAND(value) adc_command(value)
#define TEMP_ADC_READ_DATA() adc_data()
#include "../startup/w600_feedback_board.c"
#include "../startup/w600_feedback.c"
#include "temperature_adc.c"
static void pads(void) { a.IDR=a.ODR; b.IDR=b.ODR; c.IDR=c.ODR; }
static void flags_clear(uint32_t bits)
{
    assert(bits==T_FLAGS);
    if (!clear_ignored) adc.ISR&=~bits;
}
static void adc_command(uint32_t command)
{
    assert(rcc.APB2EN&RCC_APB2EN_ADCEN);
    if (command==ADC_CR_ADEN) adc.CR=ADC_CR_ADEN;
    else if (command==ADC_CR_CALEN) {
        assert(adc.CR==ADC_CR_ADEN && !adc.CHCFG);
        adc.CR|=ADC_CR_CALEN;
    } else if (command==ADC_CR_START) {
        assert(adc.CR==ADC_CR_ADEN && adc.CHCFG==ADC_CHCFG_CHN0);
        assert(!(b.IDR&T_PIN) && (b.MODE&T_B9)==(1u<<18));
        assert(starts<5); ++starts; adc.CR|=ADC_CR_START;
    } else {
        assert(command==ADC_CR_STOP); adc.CR=ADC_CR_ADEN;
    }
}
static uint32_t adc_data(void)
{
    assert(adc.CR==ADC_CR_ADEN && adc.CHCFG==1);
    assert((adc.ISR&(ADC_ISR_EOC|ADC_ISR_EOS))==(ADC_ISR_EOC|ADC_ISR_EOS));
    ++reads; adc.ISR&=~ADC_ISR_EOC;
    if (fault==7) adc.ISR|=ADC_ISR_OVRN;
    return fault==6 ? 4096u : adc.DR;
}
static void t_written(volatile uint32_t *reg,uint32_t value)
{
    ++writes;
    if (reg==&b.BSR) b.ODR|=value;
    if (reg==&b.BR) b.ODR&=~value;
    if (fault==1 && reg==&b.MODE && (value&T_B9)==(1u<<18)) b.MODE=(value&~T_B9)|T_B9;
    if (fault==12 && reg==&adc.CHCFG && value==1) adc.CHCFG=0;
    if (reg==&a.MODE || reg==&a.PUPD) assert(((value^(reg==&a.MODE ? initial_a.MODE : initial_a.PUPD))&~T_A1)==0);
    if (reg==&b.MODE || reg==&b.PUPD) {
        uint32_t original=reg==&b.MODE ? initial_b.MODE : initial_b.PUPD;
        assert(((value^original)&~(T_B9|PB13_FIELDS))==0);
    }
    assert((a.MODE&0xc0000000u)==0x40000000u && (c.MODE&0x00300000u)==0x00100000u);
    assert((b.MODE&0xf0000000u)==0x50000000u && (b.ODR&0xc000u)==0xc000u);
    pads();
}
static void board_observe(unsigned stage)
{
    if (stage==RESET_ON || stage==CLEAN_RESET_ON) { memset(&adc,0,sizeof adc); ++resets; }
    if (stage==SET_BGR) pmu.SR=PMU_SR_BGR_READY;
    if (stage==CLEAN_BGR && !(pmu.CR1&PMU_CR1_BGR_EN)) pmu.SR=0;
    if (fault==2 && stage==SET_ADC_CLOCK) rcc.APB2EN&=~RCC_APB2EN_ADCEN;
    if (fault==10 && stage==CLEAN_ADC_GATE) rcc.APB2EN|=RCC_APB2EN_ADCEN;
    pads();
}
static void fresh(struct w600_temperature_adc *s,bool idle_output)
{
    memset(s,0,sizeof *s); memset(&rcc,0,sizeof rcc); memset(&pmu,0,sizeof pmu);
    memset(&adc,0,sizeof adc); memset(&a,0,sizeof a); memset(&b,0,sizeof b); memset(&c,0,sizeof c);
    memset(&nvic,0,sizeof nvic); memset(&flash,0,sizeof flash);
    rcc.IOPEN=7; rcc.CFG=0x12; rcc.PLLCFG=0x30260; rcc.AHBEN=RCC_AHBEN_FLASHEN;
    rcc.CSR1=RCC_CSR1_RCHRDY|RCC_CSR1_PLLON|RCC_CSR1_PLLRDY; rcc.APB1EN=T_UARTS;
    flash.ACR=FLASH_ACR_LATENCY_2CLK; mask=1; ipsr=control=0; factory=0xfa2805d7;
    a.MODE=0x7fffffffu; b.MODE=0x5fffffffu; c.MODE=0xffdfffffu;
    b.ODR=0xc000u; b.PUPD=1u<<16;
    if (idle_output) { b.MODE=(b.MODE&~T_B9)|(1u<<18); b.PUPD|=1u<<18; b.ODR|=T_PIN; }
    pads(); initial_a=a; initial_b=b; initial_c=c;
    starts=reads=writes=resets=0; clear_ignored=false;
    for (unsigned i=0;i<5;++i) supplied[i]=2048;
}
static void hardware(void)
{
    if ((adc.CR&ADC_CR_CALEN) && fault!=3) {
        adc.CR=ADC_CR_ADEN; adc.ISR|=ADC_ISR_EOCAL; adc.CALFACT=21;
    }
    if ((adc.CR&ADC_CR_START) && fault!=4) {
        adc.CR=ADC_CR_ADEN; adc.ISR|=ADC_ISR_EOC|ADC_ISR_EOS;
        adc.DR=supplied[starts-1];
        if (fault==5) adc.ISR|=ADC_ISR_OVRN;
    }
}
static void restored(void)
{
    assert(a.MODE==initial_a.MODE && a.PUPD==initial_a.PUPD && a.ODR==initial_a.ODR);
    assert(b.MODE==initial_b.MODE && b.PUPD==initial_b.PUPD && b.OTYPE==initial_b.OTYPE && b.ODR==initial_b.ODR);
    assert(c.MODE==initial_c.MODE && c.PUPD==initial_c.PUPD && c.ODR==initial_c.ODR);
    assert(rcc.CFG==0x12 && rcc.APB1EN==T_UARTS && !rcc.APB2EN);
    assert(!adc.CR && !adc.CHCFG && !adc.CFG1 && !adc.CFG2 && !pmu.CR1);
}
static enum w600_temperature_adc_status run(struct w600_temperature_adc *s,uint32_t first,
                                            struct w600_temperature_adc_sample *out)
{
    for (uint32_t delta=0;delta<110;++delta) {
        hardware();
        enum w600_temperature_adc_status status=w600_temperature_adc_poll(s,first+delta,out);
        if (status==W600_TADC_SAMPLE || status==W600_TADC_FAILED) return status;
        assert(!out->valid);
    }
    assert(0); return W600_TADC_FAILED;
}
int main(void)
{
    struct w600_temperature_adc s; struct w600_temperature_adc_sample out;
    for (unsigned output=0;output<2;++output) for (unsigned wrap=0;wrap<2;++wrap) {
        fault=0; fresh(&s,output!=0); uint32_t now=wrap ? UINT32_MAX-8u : 100u;
        assert(w600_temperature_adc_begin(&s,now,5)==W600_TADC_BUSY);
        assert((b.MODE&T_B9)==(1u<<18) && (s.board.mode&T_B9)==(1u<<18));
        assert(run(&s,now,&out)==W600_TADC_SAMPLE && out.valid && out.sequence==1);
        assert(out.temperature.filtered_adc==2048 && out.temperature.sensor_decic==245 && out.temperature.adjusted_decic==250);
        assert(starts==5 && reads==5 && !s.owned && !s.board.owned && !s.cleanup_failed);
        assert((uint32_t)(out.acquired_at_ms[0]-now)>=5);
        for (unsigned i=1;i<5;++i) assert((uint32_t)(out.acquired_at_ms[i]-out.acquired_at_ms[i-1])>=2);
        restored(); assert(w600_temperature_adc_poll(&s,now+30,&out)==W600_TADC_IDLE && !out.valid);
        starts=reads=0;
        assert(w600_temperature_adc_begin(&s,now+31,-5)==W600_TADC_BUSY);
        assert(run(&s,now+31,&out)==W600_TADC_SAMPLE && out.valid && out.sequence==2);
        assert(out.temperature.adjusted_decic==240); restored(); ++cases;
    }
    for (fault=1;fault<=12;++fault) {
        if (fault==8 || fault==9 || fault==11) continue;
        fresh(&s,false);
        enum w600_temperature_adc_status accepted=w600_temperature_adc_begin(&s,100,0);
        if (fault==1 || fault==2) assert(accepted==W600_TADC_FAILED); else assert(accepted==W600_TADC_BUSY);
        assert(run(&s,100,&out)==W600_TADC_FAILED && !out.valid && s.sample.error);
        if (fault==10) assert(s.cleanup_failed && s.board.owned && s.owned);
        else { restored(); assert(!s.owned && !s.board.owned); }
        unsigned oldstarts=starts; assert(w600_temperature_adc_begin(&s,150,0)==W600_TADC_FAILED && starts==oldstarts);
        ++cases;
    }
    fault=0; fresh(&s,false); factory=0;
    assert(w600_temperature_adc_begin(&s,0,0)==W600_TADC_FAILED); assert(run(&s,0,&out)==W600_TADC_FAILED); restored(); ++cases;
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    w600_temperature_adc_abort(&s); assert(run(&s,100,&out)==W600_TADC_FAILED); restored(); ++cases;
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    assert(w600_temperature_adc_poll(&s,99,&out)==W600_TADC_BUSY); assert(run(&s,100,&out)==W600_TADC_FAILED); restored(); ++cases;
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    assert(w600_temperature_adc_poll(&s,200,&out)==W600_TADC_BUSY); assert(run(&s,200,&out)==W600_TADC_FAILED); restored(); ++cases;
    fresh(&s,false); rcc.APB1EN|=RCC_APB1EN_TIM4EN; unsigned before=writes;
    assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_FAILED && writes==before && !s.owned); ++cases;
    fresh(&s,false); rcc.IOPEN=0;
    assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_FAILED && !writes && !s.owned); ++cases;
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,51)==W600_TADC_FAILED && !writes); ++cases;
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_FAILED);
    assert(run(&s,100,&out)==W600_TADC_FAILED && !starts); restored(); ++cases;
    for (unsigned code=0;code<2;++code) {
        fresh(&s,false); for (unsigned i=0;i<5;++i) supplied[i]=code ? 4095 : 0;
        assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
        assert(run(&s,100,&out)==W600_TADC_FAILED && s.sample.error==W600_TADC_RANGE && !out.valid);
        restored(); ++cases;
    }
    fresh(&s,false); supplied[0]=0; supplied[1]=2047; supplied[2]=2048; supplied[3]=2049; supplied[4]=4095;
    assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    assert(run(&s,100,&out)==W600_TADC_SAMPLE && out.temperature.filtered_adc==2048); restored(); ++cases;
    for (unsigned change=0;change<5;++change) {
        fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
        uint32_t now=100;
        while (s.phase!=W600_TADC_BIAS) { hardware(); assert(w600_temperature_adc_poll(&s,now++,&out)==W600_TADC_BUSY); }
        switch (change) {
        case 0: adc.CFG2^=ADC_CFG2_REF; break;
        case 1: adc.CALFACT^=1; break;
        case 2: adc.SAMPT^=1; break;
        case 3: rcc.CSR1&=~RCC_CSR1_PLLRDY; break;
        default: a.PUPD|=T_A1; break;
        }
        assert(run(&s,now,&out)==W600_TADC_FAILED && !out.valid);
        if (change==3) rcc.CSR1|=RCC_CSR1_PLLRDY;
        restored(); ++cases;
    }
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    uint32_t now=100;
    while (s.phase!=W600_TADC_BIAS) { hardware(); assert(w600_temperature_adc_poll(&s,now++,&out)==W600_TADC_BUSY); }
    clear_ignored=true; adc.ISR|=ADC_ISR_EOC;
    assert(run(&s,now,&out)==W600_TADC_FAILED && !out.valid); restored(); ++cases;
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    for (unsigned n=0;n<100005 && s.phase!=W600_TADC_FAULT;++n) {
        (void)w600_temperature_adc_poll(&s,100,&out);
    }
    assert(s.phase==W600_TADC_FAULT && !out.valid && !s.owned && !s.board.owned); restored(); ++cases;
    fresh(&s,false); assert(w600_temperature_adc_begin(&s,100,0)==W600_TADC_BUSY);
    now=100;
    while (s.phase!=W600_TADC_BIAS) { hardware(); assert(w600_temperature_adc_poll(&s,now++,&out)==W600_TADC_BUSY); }
    assert(!(b.ODR&T_PIN)); rcc.APB1EN|=RCC_APB1EN_TIM4EN;
    assert(run(&s,now,&out)==W600_TADC_FAILED && !out.valid);
    assert(s.owned && s.board.owned && s.cleanup_failed && (b.ODR&T_PIN)); ++cases;
    fault=0; fresh(&s,false); now=100;
    assert(w600_temperature_adc_begin(&s,now,0)==W600_TADC_BUSY);
    while (s.phase!=W600_TADC_BIAS) { hardware(); assert(w600_temperature_adc_poll(&s,now++,&out)==W600_TADC_BUSY); }
    uint32_t biased=s.phase_ms;
    assert(w600_temperature_adc_poll(&s,biased+1,&out)==W600_TADC_BUSY && !starts);
    assert(w600_temperature_adc_poll(&s,biased+2,&out)==W600_TADC_BUSY && starts==1);
    w600_temperature_adc_abort(&s); assert(run(&s,biased+2,&out)==W600_TADC_FAILED); restored(); ++cases;
    printf("PASS %u temperature ADC cases: actual board/calibration source, CH0-only five reads, bias/gaps, repeated/wrapped samples, cleanup order, flags/raw/time/context/ownership faults; no hardware tested\n",cases);
}
