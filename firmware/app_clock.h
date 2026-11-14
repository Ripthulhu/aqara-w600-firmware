#ifndef W600_APP_CLOCK_H
#define W600_APP_CLOCK_H
#include <stdbool.h>
#include <stdint.h>
struct w600_app_time { uint32_t ms, cycles; };
/* Thread, IRQs masked for less than1ms; the application's48MHz/1ms SysTick must remain running.
 * cycles wraps every ~89s: use unsigned differences for short local windows. */
bool w600_app_clock(const volatile uint32_t *ticks, struct w600_app_time *out);
/* Thread, privileged, IRQs masked; caller must first check that all work is idle.
 * Keeps SysTick/UART/clocks running. Returns with the same interrupt mask. */
bool w600_app_idle(void);
/* After STOP: normal clocks restored, wake IRQs released, SysTick stopped.
 * Advances from the pre-sleep timestamp and restarts the1ms tick. Rounds up
 * by less than1ms so deadline/temperature ages cannot move backwards. */
bool w600_app_resume(volatile uint32_t *ticks,const struct w600_app_time *before,
                     uint32_t elapsed_slow_ticks,uint32_t slow_hz);
#endif
