#include <stdint.h>
#include "ciu32l071.h"
#include "ciu32_startup.h"

#ifndef STARTUP_OBSERVE
#define STARTUP_OBSERVE(stage) ((void)(stage))
#endif
#ifndef STARTUP_FACTORY_RCH
#define STARTUP_FACTORY_RCH (*(volatile const uint32_t *)RCH_CAL)
#endif

enum { POLL_LIMIT = 100000, MOTOR_PINS = 0xc000u,
       LATCHED = 100, OUTPUTS_SET, CONFIGURED };

static void hold_off(void)
{
    GPIOC->BR = 1u << 10;
    GPIOA->BR = 1u << 15;
    GPIOB->BSR = MOTOR_PINS;
}

static int motor_is_off(void)
{
    return (GPIOB->MODE & 0xf0000000u) == 0x50000000u &&
           (GPIOC->MODE & (3u << 20)) == (1u << 20) &&
           (GPIOA->MODE & (3u << 30)) == (1u << 30) &&
           (GPIOB->OTYPE & MOTOR_PINS) == 0 &&
           (GPIOC->OTYPE & (1u << 10)) == 0 &&
           (GPIOA->OTYPE & (1u << 15)) == 0 &&
           (GPIOB->ODR & MOTOR_PINS) == MOTOR_PINS &&
           (GPIOB->IDR & MOTOR_PINS) == MOTOR_PINS &&
           (GPIOC->ODR & (1u << 10)) == 0 &&
           (GPIOC->IDR & (1u << 10)) == 0 &&
           (GPIOA->ODR & (1u << 15)) == 0 &&
           (GPIOA->IDR & (1u << 15)) == 0;
}

static int wait_field(volatile const uint32_t *reg, uint32_t mask,
                      uint32_t expected, enum ciu32_startup_result stage)
{
    for (uint32_t i = 0; i < POLL_LIMIT; ++i) {
        hold_off();
        STARTUP_OBSERVE(stage);
        if ((*reg & mask) == expected) return 1;
    }
    return 0;
}

enum ciu32_startup_result ciu32_startup_48mhz(void)
{
    const uint32_t pll_mask = RCC_PLLCFG_PLLSRC | RCC_PLLCFG_PLLM |
        RCC_PLLCFG_PLLN | RCC_PLLCFG_PLLDIV | RCC_PLLCFG_PLLEN;
    const uint32_t pll_value = (12u << RCC_PLLCFG_PLLM_POS) |
        (2u << RCC_PLLCFG_PLLN_POS) | RCC_PLLCFG_PLLDIV_2;
    const uint32_t bus_mask = RCC_CFG_HPRE | RCC_CFG_P1PRE | RCC_CFG_P2PRE;

    if (!__get_PRIMASK() || __get_IPSR() != 0 || __get_CONTROL() != 0 ||
        NVIC->ISER[0] != 0 ||
        (SysTick->CTRL & SysTick_CTRL_ENABLE_Msk) || RCC->APB1EN != 0 ||
        (RCC->APB2EN & ~RCC_APB2EN_DBGEN) || (RCC->AHBEN & RCC_AHBEN_DMAEN))
        return CIU32_STARTUP_BUSY;

    RCC->IOPEN |= RCC_IOPEN_GPIOAEN | RCC_IOPEN_GPIOBEN | RCC_IOPEN_GPIOCEN;
    (void)RCC->IOPEN;
    __DSB();
    hold_off();
    STARTUP_OBSERVE(LATCHED);
    GPIOC->OTYPE &= ~(1u << 10);
    GPIOA->OTYPE &= ~(1u << 15);
    GPIOB->OTYPE &= ~MOTOR_PINS;
    GPIOC->PUPD &= ~(3u << 20);
    GPIOA->PUPD &= ~(3u << 30);
    GPIOB->PUPD &= ~0xf0000000u;
    GPIOC->MODE = (GPIOC->MODE & ~(3u << 20)) | (1u << 20);
    GPIOA->MODE = (GPIOA->MODE & ~(3u << 30)) | (1u << 30);
    GPIOB->MODE = (GPIOB->MODE & ~0xf0000000u) | 0x50000000u;
    __DSB();
    STARTUP_OBSERVE(OUTPUTS_SET);
    if (!motor_is_off()) return CIU32_STARTUP_MOTOR_OFF_FAILED;

    const uint32_t trim = STARTUP_FACTORY_RCH;
    const uint32_t low = trim & 0xffffu;
    if ((trim >> 16) != (low ^ 0xffffu) || low > 0x7fu ||
        (RCC->RCHCAL & RCC_RCHCAL_RCH_CAL) != low)
        return CIU32_STARTUP_TRIM_INVALID;

    FLASH->ACR = (FLASH->ACR & ~FLASH_ACR_LATENCY) | FLASH_ACR_LATENCY_2CLK;
    if (!wait_field(&FLASH->ACR, FLASH_ACR_LATENCY, FLASH_ACR_LATENCY_2CLK,
                    CIU32_STARTUP_FLASH_TIMEOUT)) return CIU32_STARTUP_FLASH_TIMEOUT;

    RCC->CSR1 |= RCC_CSR1_RCHON;
    if (!wait_field(&RCC->CSR1, RCC_CSR1_RCHRDY, RCC_CSR1_RCHRDY,
                    CIU32_STARTUP_RCH_TIMEOUT)) return CIU32_STARTUP_RCH_TIMEOUT;
    RCC->CFG &= ~RCC_CFG_SYSW;
    if (!wait_field(&RCC->CFG, RCC_CFG_SYSWS, RCC_CFG_SYSWS_RCHSYS,
                    CIU32_STARTUP_RCH_SWITCH_TIMEOUT)) return CIU32_STARTUP_RCH_SWITCH_TIMEOUT;

    RCC->PLLCFG &= ~RCC_PLLCFG_PLLEN;
    RCC->CSR1 &= ~RCC_CSR1_PLLON;
    if (!wait_field(&RCC->CSR1, RCC_CSR1_PLLRDY, 0,
                    CIU32_STARTUP_PLL_STOP_TIMEOUT)) return CIU32_STARTUP_PLL_STOP_TIMEOUT;
    if ((RCC->PLLCFG & RCC_PLLCFG_PLLEN) || (RCC->CSR1 & RCC_CSR1_PLLON))
        return CIU32_STARTUP_CONFIG_FAILED;
    RCC->CSR1 &= ~RCC_CSR1_RCHDIV;
    RCC->PLLCFG = (RCC->PLLCFG & ~pll_mask) | pll_value;
    RCC->CFG &= ~bus_mask;
    STARTUP_OBSERVE(CONFIGURED);
    if ((RCC->CSR1 & (RCC_CSR1_PLLON | RCC_CSR1_RCHDIV)) ||
        (RCC->PLLCFG & pll_mask) != pll_value || (RCC->CFG & bus_mask))
        return CIU32_STARTUP_CONFIG_FAILED;

    RCC->CSR1 |= RCC_CSR1_PLLON;
    if (!wait_field(&RCC->CSR1, RCC_CSR1_PLLRDY, RCC_CSR1_PLLRDY,
                    CIU32_STARTUP_PLL_TIMEOUT)) return CIU32_STARTUP_PLL_TIMEOUT;
    RCC->PLLCFG |= RCC_PLLCFG_PLLEN;
    __DSB();
    STARTUP_OBSERVE(CIU32_STARTUP_PLL_OUTPUT_FAILED);
    if (!(RCC->PLLCFG & RCC_PLLCFG_PLLEN)) return CIU32_STARTUP_PLL_OUTPUT_FAILED;
    RCC->CFG = (RCC->CFG & ~RCC_CFG_SYSW) | RCC_CFG_SYSW_PLL;
    if (!wait_field(&RCC->CFG, RCC_CFG_SYSWS, RCC_CFG_SYSWS_PLL,
                    CIU32_STARTUP_PLL_SWITCH_TIMEOUT)) return CIU32_STARTUP_PLL_SWITCH_TIMEOUT;
    __DSB();
    __ISB();
    STARTUP_OBSERVE(CIU32_STARTUP_VERIFY_FAILED);
    if ((RCC->CFG & (RCC_CFG_SYSW | RCC_CFG_SYSWS | bus_mask)) !=
            (RCC_CFG_SYSW_PLL | RCC_CFG_SYSWS_PLL) ||
        (RCC->PLLCFG & pll_mask) != (pll_value | RCC_PLLCFG_PLLEN) ||
        (RCC->CSR1 & (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY |
                       RCC_CSR1_RCHDIV)) !=
            (RCC_CSR1_RCHRDY | RCC_CSR1_PLLON | RCC_CSR1_PLLRDY) ||
        (FLASH->ACR & FLASH_ACR_LATENCY) != FLASH_ACR_LATENCY_2CLK ||
        !motor_is_off()) return CIU32_STARTUP_VERIFY_FAILED;
    return CIU32_STARTUP_OK;
}
