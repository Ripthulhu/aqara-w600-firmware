#include <assert.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"

static RCC_t rcc;
static FLASH_t flash;
static GPIO_t gpioa, gpiob, gpioc;
static SysTick_Type systick;
static NVIC_Type nvic;
static uint32_t primask, ipsr, control, factory;
static void observe(int stage);
#undef RCC
#undef FLASH
#undef GPIOA
#undef GPIOB
#undef GPIOC
#undef SysTick
#undef NVIC
#define RCC (&rcc)
#define FLASH (&flash)
#define GPIOA (&gpioa)
#define GPIOB (&gpiob)
#define GPIOC (&gpioc)
#define SysTick (&systick)
#define NVIC (&nvic)
#define __get_PRIMASK() (primask)
#define __get_IPSR() (ipsr)
#define __get_CONTROL() (control)
#define __DSB() ((void)0)
#define __ISB() ((void)0)
#define STARTUP_FACTORY_RCH (factory)
#define STARTUP_OBSERVE(stage) observe(stage)
#include "ciu32_startup.c"

static int fault, stuck_enable;
static unsigned counts[128], last_stage;
static uint32_t initial_mode[3], initial_flash, initial_pll_fields;

static void apply_gpio(GPIO_t *gpio)
{
    gpio->ODR = (gpio->ODR | (gpio->BSR & 0xffffu)) &
        ~(gpio->BR | (gpio->BSR >> 16));
    gpio->BSR = gpio->BR = 0;
    uint32_t idr = gpio->IDR;
    for (unsigned bit = 0; bit < 16; ++bit)
        if (((gpio->MODE >> (2 * bit)) & 3u) == 1u)
            idr = (idr & ~(1u << bit)) | (gpio->ODR & (1u << bit));
    *(volatile uint32_t *)&gpio->IDR = idr;
}

static void observe(int stage)
{
    assert(stage >= 0 && stage < 128);
    ++counts[stage];
    last_stage = (unsigned)stage;
    apply_gpio(&gpioa); apply_gpio(&gpiob); apply_gpio(&gpioc);
    if (stage == LATCHED) {
        assert(gpioa.MODE == initial_mode[0] && gpiob.MODE == initial_mode[1] &&
               gpioc.MODE == initial_mode[2]);
        assert((gpiob.ODR & MOTOR_PINS) == MOTOR_PINS);
        assert(!(gpioa.ODR & (1u << 15)) && !(gpioc.ODR & (1u << 10)));
        assert(flash.ACR == initial_flash);
        return;
    }
    if (stage == OUTPUTS_SET) {
        assert(flash.ACR == initial_flash);
        if (fault == CIU32_STARTUP_MOTOR_OFF_FAILED)
            *(volatile uint32_t *)&gpiob.IDR = 0;
        return;
    }
    assert(motor_is_off());
    assert(factory == 0xffb70048u);
    if (stage == CIU32_STARTUP_FLASH_TIMEOUT) {
        if (fault == stage) flash.ACR &= ~FLASH_ACR_LATENCY;
    } else if (stage == CIU32_STARTUP_RCH_TIMEOUT) {
        if (fault != stage) rcc.CSR1 |= RCC_CSR1_RCHRDY;
        else rcc.CSR1 &= ~RCC_CSR1_RCHRDY;
    } else if (stage == CIU32_STARTUP_RCH_SWITCH_TIMEOUT) {
        assert(rcc.CSR1 & RCC_CSR1_RCHRDY);
        rcc.CFG = (rcc.CFG & ~RCC_CFG_SYSWS) |
            (fault == stage ? RCC_CFG_SYSWS_PLL : RCC_CFG_SYSWS_RCHSYS);
    } else if (stage == CIU32_STARTUP_PLL_STOP_TIMEOUT) {
        assert((rcc.CFG & RCC_CFG_SYSWS) == RCC_CFG_SYSWS_RCHSYS);
        assert(!(rcc.PLLCFG & RCC_PLLCFG_PLLEN) && !(rcc.CSR1 & RCC_CSR1_PLLON));
        assert((rcc.PLLCFG & ~RCC_PLLCFG_PLLEN) == initial_pll_fields);
        if (fault == stage) rcc.CSR1 |= RCC_CSR1_PLLRDY;
        else rcc.CSR1 &= ~RCC_CSR1_PLLRDY;
        if (stuck_enable == 1) rcc.PLLCFG |= RCC_PLLCFG_PLLEN;
        if (stuck_enable == 2) rcc.CSR1 |= RCC_CSR1_PLLON;
    } else if (stage == CONFIGURED) {
        assert(!(rcc.CSR1 & (RCC_CSR1_PLLON | RCC_CSR1_PLLRDY)));
        if (fault == CIU32_STARTUP_CONFIG_FAILED) rcc.PLLCFG ^= 8u;
    } else if (stage == CIU32_STARTUP_PLL_TIMEOUT) {
        assert(rcc.CSR1 & RCC_CSR1_PLLON);
        if (fault == stage) rcc.CSR1 &= ~RCC_CSR1_PLLRDY;
        else rcc.CSR1 |= RCC_CSR1_PLLRDY;
    } else if (stage == CIU32_STARTUP_PLL_OUTPUT_FAILED) {
        if (fault == stage) rcc.PLLCFG &= ~RCC_PLLCFG_PLLEN;
    } else if (stage == CIU32_STARTUP_PLL_SWITCH_TIMEOUT) {
        assert((flash.ACR & FLASH_ACR_LATENCY) == FLASH_ACR_LATENCY_2CLK);
        assert(rcc.PLLCFG & RCC_PLLCFG_PLLEN);
        rcc.CFG = (rcc.CFG & ~RCC_CFG_SYSWS) |
            (fault == stage ? RCC_CFG_SYSWS_RCHSYS : RCC_CFG_SYSWS_PLL);
    } else if (stage == CIU32_STARTUP_VERIFY_FAILED && fault == stage) {
        rcc.CFG |= RCC_CFG_HPRE_2;
    }
}

static void reset_model(int running_pll)
{
    memset(&rcc, 0, sizeof rcc); memset(&flash, 0, sizeof flash);
    memset(&gpioa, 0, sizeof gpioa); memset(&gpiob, 0, sizeof gpiob);
    memset(&gpioc, 0, sizeof gpioc); memset(&nvic, 0, sizeof nvic);
    memset(&systick, 0, sizeof systick); memset(counts, 0, sizeof counts);
    primask = 1; ipsr = control = 0; factory = 0xffb70048u; rcc.RCHCAL = 72;
    gpioa.MODE = gpiob.MODE = gpioc.MODE = 0xffffffffu;
    gpioa.ODR = gpiob.ODR = gpioc.ODR = 0x5555u;
    gpioa.OTYPE = gpiob.OTYPE = gpioc.OTYPE = 0xffffu;
    gpioa.PUPD = gpiob.PUPD = gpioc.PUPD = 0xffffffffu;
    initial_mode[0] = gpioa.MODE; initial_mode[1] = gpiob.MODE;
    initial_mode[2] = gpioc.MODE;
    rcc.CFG = running_pll ? 0x12u : 0;
    rcc.PLLCFG = running_pll ? 0x30260u : 0;
    initial_pll_fields = rcc.PLLCFG & ~RCC_PLLCFG_PLLEN;
    rcc.CSR1 = running_pll ? RCC_CSR1_RCHON | RCC_CSR1_RCHRDY |
        RCC_CSR1_PLLON | RCC_CSR1_PLLRDY : 0;
    flash.ACR = initial_flash = running_pll ? 2 : 0;
    fault = -1; stuck_enable = 0; last_stage = 0;
}

int main(void)
{
    for (int pll = 0; pll < 3; ++pll) {
        reset_model(pll);
        if (pll == 2) {
            rcc.CFG = RCC_CFG_SYSW_HXTAL | RCC_CFG_SYSWS_HXTAL;
            rcc.PLLCFG = RCC_PLLCFG_PLLSRC_HXTAL | (8u << RCC_PLLCFG_PLLM_POS) |
                (4u << RCC_PLLCFG_PLLN_POS) | RCC_PLLCFG_PLLDIV_4 | RCC_PLLCFG_PLLEN;
        }
        rcc.APB2EN = RCC_APB2EN_DBGEN;
        rcc.CFG |= RCC_CFG_HPRE_8 | RCC_CFG_P1PRE_4 | RCC_CFG_P2PRE_16;
        rcc.CSR1 |= RCC_CSR1_RCHDIV_8;
        rcc.PLLCFG |= 0x80000000u;
        initial_pll_fields = rcc.PLLCFG & ~RCC_PLLCFG_PLLEN;
        flash.ACR |= 0x100u; initial_flash = flash.ACR;
        assert(ciu32_startup_48mhz() == CIU32_STARTUP_OK);
        assert(rcc.CFG == 0x12u && rcc.PLLCFG == 0x80030260u);
        assert(flash.ACR == 0x102u && rcc.RCHCAL == 72 && motor_is_off());
        assert(primask == 1 && nvic.ISER[0] == 0 && systick.CTRL == 0);
    }
    for (int i = 0; i < 9; ++i) {
        reset_model(1);
        if (i == 0) primask = 0;
        if (i == 1) nvic.ISER[0] = 1;
        if (i == 2) systick.CTRL = 1;
        if (i == 3) rcc.APB1EN = 1;
        if (i == 4) rcc.APB2EN = 1;
        if (i == 5) rcc.AHBEN = RCC_AHBEN_DMAEN;
        if (i == 6) ipsr = 16;
        if (i == 7) control = 1;
        if (i == 8) control = 2;
        RCC_t before = rcc; FLASH_t before_flash = flash; GPIO_t before_gpio = gpiob;
        assert(ciu32_startup_48mhz() == CIU32_STARTUP_BUSY);
        assert(!memcmp(&before, &rcc, sizeof rcc));
        assert(!memcmp(&before_flash, &flash, sizeof flash));
        assert(!memcmp(&before_gpio, &gpiob, sizeof gpiob) && counts[LATCHED] == 0);
    }
    for (int i = 0; i < 3; ++i) {
        reset_model(0);
        if (i == 0) factory ^= 0x10000u;
        if (i == 1) factory = 0xff7f0080u;
        if (i == 2) rcc.RCHCAL = 71;
        assert(ciu32_startup_48mhz() == CIU32_STARTUP_TRIM_INVALID);
        assert(motor_is_off() && flash.ACR == 0 && rcc.CSR1 == 0);
    }
    for (int enable = 1; enable < 3; ++enable) {
        reset_model(1); stuck_enable = enable;
        rcc.CSR1 |= RCC_CSR1_RCHDIV_8;
        rcc.PLLCFG ^= 8u;
        initial_pll_fields = rcc.PLLCFG & ~RCC_PLLCFG_PLLEN;
        const uint32_t before_div = rcc.CSR1 & RCC_CSR1_RCHDIV;
        assert(ciu32_startup_48mhz() == CIU32_STARTUP_CONFIG_FAILED);
        assert(counts[CONFIGURED] == 0 && motor_is_off());
        assert((rcc.PLLCFG & ~RCC_PLLCFG_PLLEN) == initial_pll_fields);
        assert((rcc.CSR1 & RCC_CSR1_RCHDIV) == before_div);
    }
    const int errors[] = {CIU32_STARTUP_MOTOR_OFF_FAILED, CIU32_STARTUP_FLASH_TIMEOUT,
        CIU32_STARTUP_RCH_TIMEOUT, CIU32_STARTUP_RCH_SWITCH_TIMEOUT,
        CIU32_STARTUP_PLL_STOP_TIMEOUT, CIU32_STARTUP_CONFIG_FAILED,
        CIU32_STARTUP_PLL_TIMEOUT, CIU32_STARTUP_PLL_OUTPUT_FAILED,
        CIU32_STARTUP_PLL_SWITCH_TIMEOUT, CIU32_STARTUP_VERIFY_FAILED};
    for (unsigned i = 0; i < sizeof errors / sizeof errors[0]; ++i) {
        reset_model(1); fault = errors[i];
        assert((int)ciu32_startup_48mhz() == fault);
        assert(rcc.RCHCAL == 72 && factory == 0xffb70048u);
        if (fault != CIU32_STARTUP_MOTOR_OFF_FAILED) assert(motor_is_off());
        if (fault == CIU32_STARTUP_FLASH_TIMEOUT || fault == CIU32_STARTUP_RCH_TIMEOUT ||
            fault == CIU32_STARTUP_RCH_SWITCH_TIMEOUT || fault == CIU32_STARTUP_PLL_STOP_TIMEOUT ||
            fault == CIU32_STARTUP_PLL_TIMEOUT || fault == CIU32_STARTUP_PLL_SWITCH_TIMEOUT)
            assert(counts[fault] == POLL_LIMIT && last_stage == (unsigned)fault);
    }
    puts("Startup: 27 scenarios passed (3 success, 9 busy, 3 trim, 2 stuck enables, 10 failures)");
    return 0;
}
