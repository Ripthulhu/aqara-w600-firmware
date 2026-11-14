#ifndef W600_SLEEP_CLOCK_H
#define W600_SLEEP_CLOCK_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum w600_sleep_clock_phase {
    W600_SC_OFF, W600_SC_RCL_WAIT, W600_SC_ENABLE_WAIT,
    W600_SC_COUNTER_WAIT, W600_SC_MEASURE, W600_SC_DONE, W600_SC_FAILED, W600_SC_ARMED
};
enum w600_sleep_clock_error {
    W600_SC_OK, W600_SC_CONTEXT, W600_SC_FACTORY, W600_SC_TIME,
    W600_SC_HARDWARE, W600_SC_TIMEOUT, W600_SC_RATE, W600_SC_RELEASE
};
struct w600_sleep_clock {
    uint32_t phase,error,owned,cleanup_failed,phase_ms,last_ms;
    uint32_t start_cycles,start_count,first_hz,hz,windows,sample_cycles,sample_count;
    uint32_t arming;
    const volatile uint32_t *ticks;
};
_Static_assert(offsetof(struct w600_sleep_clock,owned)==8 &&
    offsetof(struct w600_sleep_clock,hz)==36 && offsetof(struct w600_sleep_clock,windows)==40,
    "Slow-clock observation layout");
/* Privileged Thread, IRQs masked; caller excludes motor/ADC/FLASH/display work.
 * Borrows unused RCL (including retained-on after reset) and idle LPTIM2.
 * Rejects RTC/watchdog/other clock users. No GPIO, interrupt, motor or PMU changes.
 * Nonblocking: two200ms rate measurements against the existing48MHz clock. */
bool w600_sleep_clock_begin(struct w600_sleep_clock *,const volatile uint32_t *ticks);
void w600_sleep_clock_poll(struct w600_sleep_clock *);
/* After a successful rate check, asynchronously arm a continuous half-second
 * wake timer. No interrupt is enabled here; its owner must configure wake-up. */
bool w600_sleep_clock_arm(struct w600_sleep_clock *,const volatile uint32_t *ticks);
/* Release before another hardware owner runs. Success leaves the original
 * clock selection/gates; errors are latched and never retried automatically. */
bool w600_sleep_clock_cancel(struct w600_sleep_clock *);
#endif
