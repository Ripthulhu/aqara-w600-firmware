#include "w600_buttons.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void released(struct w600_buttons *buttons,uint32_t at)
{
    w600_buttons_init(buttons);
    assert(!w600_buttons_poll(buttons,at,0));
    assert(!w600_buttons_poll(buttons,at+30u,0));
}
static uint8_t press(struct w600_buttons *buttons,uint32_t at,uint8_t mask)
{
    uint8_t events=w600_buttons_poll(buttons,at,mask);
    return (uint8_t)(events|w600_buttons_poll(buttons,at+30u,mask));
}
int main(void)
{
    struct w600_buttons buttons;
    for (uint8_t mask=1;mask<=7;++mask) {
        w600_buttons_init(&buttons);
        assert(!w600_buttons_poll(&buttons,0,mask));
        assert(!w600_buttons_poll(&buttons,30000,mask));
        assert(!w600_buttons_poll(&buttons,30001,0));
        assert(!w600_buttons_poll(&buttons,30031,0));
        uint8_t expected=(uint8_t)(W600_BUTTON_WAKE|(mask&6u));
        if ((mask&6u)==6u) expected=W600_BUTTON_WAKE;
        assert(press(&buttons,30100,mask)==expected);
    }
    released(&buttons,0);
    assert(press(&buttons,100,1)==W600_BUTTON_WAKE);
    assert(!w600_buttons_poll(&buttons,3129,1));
    assert(w600_buttons_poll(&buttons,3130,1)==W600_BUTTON_LOCK);
    assert(!w600_buttons_poll(&buttons,5000,1));
    assert(!w600_buttons_poll(&buttons,10129,1));
    assert(w600_buttons_poll(&buttons,10130,1)==W600_BUTTON_PAIR);
    assert(!w600_buttons_poll(&buttons,30000,1));
    assert(!w600_buttons_poll(&buttons,30001,0));
    assert(!w600_buttons_poll(&buttons,30031,0));
    assert(press(&buttons,30100,1)==W600_BUTTON_WAKE);
    assert(w600_buttons_poll(&buttons,40130,1)==W600_BUTTON_PAIR);

    for (uint8_t mask=2;mask<=4;mask*=2) {
        released(&buttons,0);
        assert(press(&buttons,100,mask)==(W600_BUTTON_WAKE|mask));
        assert(!w600_buttons_poll(&buttons,20000,mask));
        assert(!w600_buttons_poll(&buttons,20001,0));
        assert(!w600_buttons_poll(&buttons,20031,0));
        assert(press(&buttons,20100,mask)==(W600_BUTTON_WAKE|mask));
    }
    for (uint8_t mask=2;mask<=4;mask*=2) for (unsigned width=1;width<=15;width+=width==1 ? 4u : 10u) {
        released(&buttons,0);
        for (uint32_t at=100;at<=190;at+=30) {
            assert(w600_buttons_poll(&buttons,at,mask)==(W600_BUTTON_WAKE|mask));
            assert(!w600_buttons_poll(&buttons,at+width,0));
        }
    }
    released(&buttons,0);
    assert(w600_buttons_poll(&buttons,100,2)==(W600_BUTTON_WAKE|W600_BUTTON_DECREASE));
    assert(!w600_buttons_poll(&buttons,101,0));
    assert(!w600_buttons_poll(&buttons,102,2)); /* Contact bounce. */
    assert(!w600_buttons_poll(&buttons,103,0));
    assert(!w600_buttons_poll(&buttons,108,2)); /* Rearmed, but inside30ms. */
    assert(!w600_buttons_poll(&buttons,109,0));
    assert(!w600_buttons_poll(&buttons,113,2)); /* Less than5ms released. */
    assert(!w600_buttons_poll(&buttons,114,0));
    assert(w600_buttons_poll(&buttons,130,2)==(W600_BUTTON_WAKE|W600_BUTTON_DECREASE));
    assert(!w600_buttons_poll(&buttons,131,0));
    assert(!w600_buttons_poll(&buttons,159,2)); /* 29ms since accepted pulse. */
    assert(!w600_buttons_poll(&buttons,160,0));
    assert(!w600_buttons_poll(&buttons,164,2)); /* Refractory ended, release still too short. */
    assert(!w600_buttons_poll(&buttons,165,0));
    assert(w600_buttons_poll(&buttons,170,2)==(W600_BUTTON_WAKE|W600_BUTTON_DECREASE));

    released(&buttons,0);
    assert(press(&buttons,100,6)==W600_BUTTON_WAKE);
    assert(!w600_buttons_poll(&buttons,200,2));
    assert(!w600_buttons_poll(&buttons,230,2));
    assert(!w600_buttons_poll(&buttons,300,0));
    assert(!w600_buttons_poll(&buttons,330,0));
    assert(press(&buttons,400,4)==(W600_BUTTON_WAKE|W600_BUTTON_INCREASE));

    for (uint8_t mask=2;mask<=4;mask*=2) {
        w600_buttons_init(&buttons);
        assert(!w600_buttons_poll(&buttons,0,mask));
        assert(!w600_buttons_poll(&buttons,10000,mask));
        assert(!w600_buttons_poll(&buttons,10001,0));
        assert(!w600_buttons_poll(&buttons,10005,mask));
        assert(!w600_buttons_poll(&buttons,10006,0));
        assert(w600_buttons_poll(&buttons,10011,mask)==(W600_BUTTON_WAKE|mask));
    }

    released(&buttons,0);
    assert(press(&buttons,100,1)==W600_BUTTON_WAKE);
    assert(!w600_buttons_poll(&buttons,10129,0));
    assert(!w600_buttons_poll(&buttons,10130,0));

    released(&buttons,UINT32_MAX-200u);
    assert(press(&buttons,UINT32_MAX-100u,1)==W600_BUTTON_WAKE);
    assert(w600_buttons_poll(&buttons,2929,1)==W600_BUTTON_LOCK);
    assert(!w600_buttons_poll(&buttons,9928,1));
    assert(w600_buttons_poll(&buttons,9929,1)==W600_BUTTON_PAIR);

    released(&buttons,UINT32_MAX-200u);
    assert(w600_buttons_poll(&buttons,UINT32_MAX-10u,4)==(W600_BUTTON_WAKE|W600_BUTTON_INCREASE));
    assert(!w600_buttons_poll(&buttons,UINT32_MAX-9u,0));
    assert(!w600_buttons_poll(&buttons,18,4)); /* 29ms across wrap. */
    assert(!w600_buttons_poll(&buttons,19,0));
    assert(w600_buttons_poll(&buttons,24,4)==(W600_BUTTON_WAKE|W600_BUTTON_INCREASE));

    released(&buttons,1000);
    assert(!w600_buttons_poll(&buttons,1029,1) && buttons.clock_error);
    assert(!w600_buttons_poll(&buttons,20000,1));
    released(&buttons,0);
    assert(!w600_buttons_poll(&buttons,UINT32_C(0x8000001e),0) && buttons.clock_error);
    released(&buttons,0);
    struct w600_buttons saved=buttons;
    assert(!w600_buttons_poll(&buttons,100,8) && !memcmp(&buttons,&saved,sizeof(buttons)));
    assert(!w600_buttons_poll(NULL,100,0));
    w600_buttons_init(NULL);
    puts("PASS buttons: boot-held masks, main30ms debounce/holds, dial1/5/15ms pulses, 30ms refractory/5ms release, bounce, opposite inputs, wrap/rollback, invalid mask/NULL; no hardware actions");
}
