#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "ciu32l071.h"

static RCC_t rcc;
static GPIO_t a, b, c;
static unsigned cases, writes, pb_writes, drop_gate_at;
static volatile uint32_t *ignore;
static uint32_t stuck_a, stuck_b, stuck_c;
static int af_low;
static uint32_t read_reg(volatile uint32_t *reg);
static void write_reg(volatile uint32_t *reg, uint32_t value);
#undef RCC
#undef GPIOA
#undef GPIOB
#undef GPIOC
#define RCC (&rcc)
#define GPIOA (&a)
#define GPIOB (&b)
#define GPIOC (&c)
#define W600_MOTOR_STOP_READ(reg) read_reg(&(reg))
#define W600_MOTOR_STOP_WRITE(reg, value) write_reg(&(reg), (uint32_t)(value))
#include "w600_motor_stop.c"

static void pads(void)
{
    a.IDR = (a.IDR & ~0x8000u) | (a.ODR & 0x8000u) | stuck_a;
    c.IDR = (c.IDR & ~0x400u) | (c.ODR & 0x400u) | stuck_c;
    b.IDR = (b.IDR & ~0xc000u) | (b.ODR & 0xc000u);
    if (af_low && (b.MODE & 0xf0000000u) != 0x50000000u)
        b.IDR &= ~0x4000u;
    b.IDR &= ~stuck_b;
}
static int belongs(volatile uint32_t *reg, GPIO_t *gpio)
{
    uintptr_t p = (uintptr_t)reg, base = (uintptr_t)gpio;
    return p >= base && p < base + sizeof *gpio;
}
static uint32_t read_reg(volatile uint32_t *reg)
{
    if (reg != &rcc.IOPEN) {
        assert((rcc.IOPEN & 7u) == 7u);
        assert(belongs(reg, &a) || belongs(reg, &b) || belongs(reg, &c));
        assert(reg != &a.BR && reg != &b.BR && reg != &c.BR);
        assert(reg != &a.BSR && reg != &b.BSR && reg != &c.BSR);
    }
    return *reg;
}
static void write_reg(volatile uint32_t *reg, uint32_t value)
{
    ++writes;
    if (reg != &rcc.IOPEN) assert((rcc.IOPEN & 7u) == 7u);
    assert(reg == &rcc.IOPEN || reg == &a.BR || reg == &c.BR ||
        reg == &a.MODE || reg == &c.MODE || reg == &a.OTYPE ||
        reg == &c.OTYPE || reg == &a.PUPD || reg == &c.PUPD ||
        reg == &b.BSR || reg == &b.OTYPE || reg == &b.PUPD || reg == &b.MODE);
    if (reg == &b.BSR || reg == &b.OTYPE || reg == &b.PUPD || reg == &b.MODE) {
        ++pb_writes;
        assert((a.MODE & 0xc0000000u) == 0x40000000u);
        assert((c.MODE & 0x00300000u) == 0x00100000u);
        assert(!(a.OTYPE & 0x8000u) && !(c.OTYPE & 0x400u));
        assert(!(a.PUPD & 0xc0000000u) && !(c.PUPD & 0x00300000u));
        assert(!(a.ODR & 0x8000u) && !(c.ODR & 0x400u));
        assert(!(a.IDR & 0x8000u) && !(c.IDR & 0x400u));
    }
    if (reg == &a.MODE) assert(!(a.ODR & 0x8000u) && !(a.PUPD & 0xc0000000u));
    if (reg == &c.MODE) assert(!(c.ODR & 0x400u) && !(c.PUPD & 0x00300000u));
    if (reg == &b.MODE) {
        assert((b.ODR & 0xc000u) == 0xc000u);
        assert(!(b.OTYPE & 0xc000u));
        assert(!(b.PUPD & 0xf0000000u));
    }
    if (reg != ignore) {
        if (reg == &a.BR) a.ODR &= ~value;
        else if (reg == &c.BR) c.ODR &= ~value;
        else if (reg == &b.BSR) b.ODR |= value;
        else *reg = value;
    }
    pads();
    if (writes == drop_gate_at) rcc.IOPEN &= ~4u;
}
static void setup(int early)
{
    memset(&rcc, 0xa5, sizeof rcc);
    memset(&a, 0x96, sizeof a); memset(&b, 0x69, sizeof b); memset(&c, 0x5a, sizeof c);
    rcc.IOPEN = early ? 0x18u : 0x1fu;
    a.MODE = (a.MODE & ~0xc0000000u) | 0x80000000u;
    c.MODE = (c.MODE & ~0x00300000u) | 0x00200000u;
    b.MODE = (b.MODE & ~0xf0000000u) | 0x60000000u;
    a.OTYPE |= 0x8000u; c.OTYPE |= 0x400u; b.OTYPE |= 0xc000u;
    a.PUPD |= 0xc0000000u; c.PUPD |= 0x00300000u; b.PUPD |= 0xf0000000u;
    a.ODR |= 0x8000u; c.ODR |= 0x400u; b.ODR &= ~0xc000u;
    writes = pb_writes = drop_gate_at = 0; ignore = NULL;
    stuck_a = stuck_b = stuck_c = 0; af_low = 1; pads();
}
static void preserved(const RCC_t *old_rcc, const GPIO_t *old_a,
    const GPIO_t *old_b, const GPIO_t *old_c)
{
    RCC_t rr = rcc; GPIO_t aa = a, bb = b, cc = c;
    rr.IOPEN = old_rcc->IOPEN;
    aa.MODE = old_a->MODE; aa.OTYPE = old_a->OTYPE; aa.PUPD = old_a->PUPD; aa.ODR = old_a->ODR; aa.IDR = old_a->IDR;
    bb.MODE = old_b->MODE; bb.OTYPE = old_b->OTYPE; bb.PUPD = old_b->PUPD; bb.ODR = old_b->ODR; bb.IDR = old_b->IDR;
    cc.MODE = old_c->MODE; cc.OTYPE = old_c->OTYPE; cc.PUPD = old_c->PUPD; cc.ODR = old_c->ODR; cc.IDR = old_c->IDR;
    assert(!memcmp(&rr, old_rcc, sizeof rr));
    assert(!memcmp(&aa, old_a, sizeof aa)); assert(!memcmp(&bb, old_b, sizeof bb));
    assert(!memcmp(&cc, old_c, sizeof cc));
    assert((a.MODE & ~0xc0000000u) == (old_a->MODE & ~0xc0000000u));
    assert((c.MODE & ~0x00300000u) == (old_c->MODE & ~0x00300000u));
    assert((b.MODE & ~0xf0000000u) == (old_b->MODE & ~0xf0000000u));
    assert((a.OTYPE & ~0x8000u) == (old_a->OTYPE & ~0x8000u));
    assert((c.OTYPE & ~0x400u) == (old_c->OTYPE & ~0x400u));
    assert((b.OTYPE & ~0xc000u) == (old_b->OTYPE & ~0xc000u));
    assert((a.PUPD & ~0xc0000000u) == (old_a->PUPD & ~0xc0000000u));
    assert((c.PUPD & ~0x00300000u) == (old_c->PUPD & ~0x00300000u));
    assert((b.PUPD & ~0xf0000000u) == (old_b->PUPD & ~0xf0000000u));
    assert((a.ODR & ~0x8000u) == (old_a->ODR & ~0x8000u));
    assert((c.ODR & ~0x400u) == (old_c->ODR & ~0x400u));
    assert((b.ODR & ~0xc000u) == (old_b->ODR & ~0xc000u));
}
int main(void)
{
    /* Identical source has no CPU-context reads: Thread, IRQ and early fault use it. */
    for (unsigned context = 0; context < 3; ++context) {
        setup(context == 2);
        RCC_t rr = rcc; GPIO_t aa = a, bb = b, cc = c;
        assert(w600_motor_stop()); assert(pb_writes == 4);
        assert((rcc.IOPEN & 7u) == 7u);
        preserved(&rr, &aa, &bb, &cc);
        assert(w600_motor_stop()); preserved(&rr, &aa, &bb, &cc);
        ++cases;
    }
    volatile uint32_t *ignored[] = {&a.BR, &c.BR, &a.OTYPE, &c.OTYPE,
        &a.PUPD, &c.PUPD, &a.MODE, &c.MODE, &b.BSR, &b.OTYPE, &b.PUPD, &b.MODE};
    for (unsigned n = 0; n < sizeof ignored / sizeof ignored[0]; ++n) {
        setup(0); ignore = ignored[n];
        assert(!w600_motor_stop());
        if (n < 8) assert(pb_writes == 0);
        if (n < 11) assert((b.MODE & 0xf0000000u) == 0x60000000u);
        ++cases;
    }
    for (unsigned n = 0; n < 3; ++n) {
        setup(0);
        if (n == 0) stuck_a = 0x8000u;
        if (n == 1) stuck_c = 0x400u;
        if (n == 2) stuck_b = 0x4000u;
        pads(); assert(!w600_motor_stop());
        if (n < 2) assert(pb_writes == 0);
        ++cases;
    }
    setup(1); ignore = &rcc.IOPEN;
    assert(!w600_motor_stop() && writes == 1 && pb_writes == 0); ++cases;
    /* Lost GPIO gate after each store is detected before another gated access. */
    setup(0); assert(w600_motor_stop()); unsigned normal_writes = writes;
    for (unsigned n = 1; n <= normal_writes; ++n) {
        setup(0); drop_gate_at = n; assert(!w600_motor_stop()); ++cases;
    }
    printf("PASS %u actual-source GPIO motor-stop cases; no hardware validation\n", cases);
}
