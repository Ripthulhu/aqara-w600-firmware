#ifndef W600_STOP_MODE_H
#define W600_STOP_MODE_H
#include "sleep_clock.h"
enum w600_stop_result { W600_STOP_SKIPPED, W600_STOP_WOKE, W600_STOP_FAILED };
struct w600_stop_report {
    uint32_t entered,woke,error,wake_lines,elapsed;
    uint32_t radio_wakes,button_wakes,timer_wakes,last_radio_at;
};
/* Privileged Thread, IRQs masked, every other hardware owner idle. Borrows
 * wake IRQs only while asleep; leaves the armed slow timer for caller release. */
enum w600_stop_result w600_stop_mode(struct w600_sleep_clock *,volatile uint32_t *,
                                     struct w600_stop_report *);
#endif
