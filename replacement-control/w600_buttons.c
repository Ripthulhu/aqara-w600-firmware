#include "w600_buttons.h"

void w600_buttons_init(struct w600_buttons *buttons)
{
    if (buttons) *buttons=(struct w600_buttons){0};
}
uint8_t w600_buttons_poll(struct w600_buttons *buttons,uint32_t now,uint8_t pressed_mask)
{
    if (!buttons || buttons->clock_error || (pressed_mask&~7u)) return 0;
    if (!buttons->initialized) {
        buttons->initialized=true; buttons->last_at=now;
        for (unsigned i=0;i<3;++i) {
            bool pressed=(pressed_mask&(1u<<i))!=0;
            buttons->input[i]=(struct w600_button){.changed_at=now,.raw=pressed,.stable=pressed};
        }
        return 0;
    }
    if ((uint32_t)(now-buttons->last_at)>=UINT32_C(0x80000000)) {
        buttons->clock_error=true; return 0;
    }
    buttons->last_at=now;
    uint8_t events=0;
    for (unsigned i=0;i<3;++i) {
        struct w600_button *button=&buttons->input[i];
        bool pressed=(pressed_mask&(1u<<i))!=0;
        bool changed=button->raw!=pressed;
        if (i && !button->raw && (uint32_t)(now-button->changed_at)>=5u)
            button->released=true;
        if (changed) { button->raw=pressed; button->changed_at=now; }
        if (i) {
            /* Dial contacts pulse briefly; debounce the interval, not pulse width. */
            if (changed && pressed) {
                if (button->released && (!button->fired ||
                    (uint32_t)(now-button->pressed_at)>=30u)) {
                    events|=(uint8_t)(W600_BUTTON_WAKE|(1u<<i));
                    button->pressed_at=now; button->fired=1;
                }
                button->released=false;
            }
            continue;
        }
        if (button->stable!=button->raw && (uint32_t)(now-button->changed_at)>=30u) {
            button->stable=button->raw; button->fired=false;
            if (button->stable) {
                button->pressed_at=now;
                if (button->released) events|=(uint8_t)(W600_BUTTON_WAKE|(i ? 1u<<i : 0));
            }
        }
        if (!button->stable && (uint32_t)(now-button->changed_at)>=30u) button->released=true;
        if (!i && button->stable && button->raw && button->released) {
            uint32_t held=now-button->pressed_at;
            if (button->fired<2 && held>=10000u) {
                button->fired=2; events|=W600_BUTTON_PAIR;
            } else if (!button->fired && held>=3000u) {
                button->fired=1; events|=W600_BUTTON_LOCK;
            }
        }
    }
    /* Opposite inputs together have no unambiguous setpoint direction. */
    if ((pressed_mask&6u)==6u) events&=(uint8_t)~6u;
    return events;
}
