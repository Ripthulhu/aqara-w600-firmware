#ifndef W600_BUTTONS_H
#define W600_BUTTONS_H
#include <stdbool.h>
#include <stdint.h>

enum {
    W600_BUTTON_PRIMARY=1, W600_BUTTON_DECREASE=2, W600_BUTTON_INCREASE=4,
    W600_BUTTON_WAKE=8, W600_BUTTON_PAIR=16, W600_BUTTON_LOCK=32
};
struct w600_button {
    uint32_t changed_at, pressed_at;
    bool raw, stable, released;
    uint8_t fired;
};
struct w600_buttons {
    struct w600_button input[3];
    uint32_t last_at;
    bool initialized, clock_error;
};

/* Zero/init once. pressed_mask uses the three active-low input meanings above.
 * Returns WAKE, DECREASE, INCREASE, LOCK (3s) or PAIR (10s) events.
 * Dial falling edges count immediately, with a 30ms minimum event interval and
 * 5ms stable release before rearming; no auto-repeat. Main button debounce30ms.
 * Poll every1ms to catch short dial pulses. Held-at-boot inputs must release first.
 * Clock rollback/half-range latches an error; never initializes pairing itself. */
void w600_buttons_init(struct w600_buttons *);
uint8_t w600_buttons_poll(struct w600_buttons *,uint32_t now,uint8_t pressed_mask);
#endif
