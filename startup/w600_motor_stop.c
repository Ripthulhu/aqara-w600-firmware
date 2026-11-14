#include "ciu32l071.h"
#include "w600_motor_stop.h"
#ifndef W600_MOTOR_STOP_READ
#define W600_MOTOR_STOP_READ(reg) (reg)
#endif
#ifndef W600_MOTOR_STOP_WRITE
#define W600_MOTOR_STOP_WRITE(reg, value) do { (reg) = (value); __DSB(); } while (0)
#endif
#define STOP_READ(reg) W600_MOTOR_STOP_READ(reg)
#define GPIO_WRITE(reg, value) do { \
    if (!stop_clocks()) return 0; \
    W600_MOTOR_STOP_WRITE(reg, value); \
} while (0)

static int stop_clocks(void)
{
    return (STOP_READ(RCC->IOPEN) & 7u) == 7u;
}
static int stop_selectors(void)
{
    return stop_clocks() &&
        (STOP_READ(GPIOC->MODE) & 0x00300000u) == 0x00100000u &&
        (STOP_READ(GPIOA->MODE) & 0xc0000000u) == 0x40000000u &&
        !(STOP_READ(GPIOC->OTYPE) & 0x400u) &&
        !(STOP_READ(GPIOA->OTYPE) & 0x8000u) &&
        !(STOP_READ(GPIOC->PUPD) & 0x00300000u) &&
        !(STOP_READ(GPIOA->PUPD) & 0xc0000000u) &&
        !(STOP_READ(GPIOC->ODR) & 0x400u) &&
        !(STOP_READ(GPIOA->ODR) & 0x8000u) &&
        !(STOP_READ(GPIOC->IDR) & 0x400u) &&
        !(STOP_READ(GPIOA->IDR) & 0x8000u);
}
int w600_motor_stop(void)
{
    if (!stop_clocks()) {
        W600_MOTOR_STOP_WRITE(RCC->IOPEN, STOP_READ(RCC->IOPEN) | 7u);
        if (!stop_clocks()) return 0;
    }
    GPIO_WRITE(GPIOC->BR, 0x400u);
    GPIO_WRITE(GPIOA->BR, 0x8000u);
    if (!stop_clocks() || (STOP_READ(GPIOC->ODR) & 0x400u) ||
        (STOP_READ(GPIOA->ODR) & 0x8000u)) return 0;
    GPIO_WRITE(GPIOC->OTYPE, STOP_READ(GPIOC->OTYPE) & ~0x400u);
    GPIO_WRITE(GPIOA->OTYPE, STOP_READ(GPIOA->OTYPE) & ~0x8000u);
    GPIO_WRITE(GPIOC->PUPD, STOP_READ(GPIOC->PUPD) & ~0x00300000u);
    GPIO_WRITE(GPIOA->PUPD, STOP_READ(GPIOA->PUPD) & ~0xc0000000u);
    if (!stop_clocks() || (STOP_READ(GPIOC->OTYPE) & 0x400u) ||
        (STOP_READ(GPIOA->OTYPE) & 0x8000u) ||
        (STOP_READ(GPIOC->PUPD) & 0x00300000u) ||
        (STOP_READ(GPIOA->PUPD) & 0xc0000000u)) return 0;
    GPIO_WRITE(GPIOC->MODE, (STOP_READ(GPIOC->MODE) & ~0x00300000u) | 0x00100000u);
    GPIO_WRITE(GPIOA->MODE, (STOP_READ(GPIOA->MODE) & ~0xc0000000u) | 0x40000000u);
    if (!stop_selectors()) return 0;
    GPIO_WRITE(GPIOB->BSR, 0xc000u);
    if (!stop_clocks() || (STOP_READ(GPIOB->ODR) & 0xc000u) != 0xc000u) return 0;
    GPIO_WRITE(GPIOB->OTYPE, STOP_READ(GPIOB->OTYPE) & ~0xc000u);
    GPIO_WRITE(GPIOB->PUPD, STOP_READ(GPIOB->PUPD) & ~0xf0000000u);
    if (!stop_clocks() || (STOP_READ(GPIOB->OTYPE) & 0xc000u) ||
        (STOP_READ(GPIOB->PUPD) & 0xf0000000u)) return 0;
    GPIO_WRITE(GPIOB->MODE, (STOP_READ(GPIOB->MODE) & ~0xf0000000u) | 0x50000000u);
    return stop_selectors() &&
        (STOP_READ(GPIOB->MODE) & 0xf0000000u) == 0x50000000u &&
        !(STOP_READ(GPIOB->OTYPE) & 0xc000u) &&
        !(STOP_READ(GPIOB->PUPD) & 0xf0000000u) &&
        (STOP_READ(GPIOB->ODR) & 0xc000u) == 0xc000u &&
        (STOP_READ(GPIOB->IDR) & 0xc000u) == 0xc000u;
}
