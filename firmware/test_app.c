#define W600_APP_HOST_TEST 1
#include "app.c"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned calls;
static unsigned target_calls;
static unsigned control_calls;
static uint32_t called_at;
static int accepted;
bool w600_radio_request_pairing(struct w600_radio *r,uint32_t now)
{
    assert(r==&radio); ++calls; called_at=now;
    return accepted!=0;
}
bool w600_radio_report_target(struct w600_radio *r)
{ assert(r==&radio); ++target_calls; return true; }
bool w600_radio_report_control(struct w600_radio *r)
{ assert(r==&radio); ++control_calls; return true; }
static void clear(void)
{
    memset(&button,0,sizeof(button));
    memset(&radio,0,sizeof(radio));
    memset((void *)&app_report,0,sizeof(app_report));
    calls=0; target_calls=0; control_calls=0; called_at=0; accepted=1;
    app_ticks=0;
#ifdef W600_DISPLAY_BENCH
    display_activity_at=0; display_awake=false;
#endif
}
static void release(uint32_t now)
{
    button_poll(now,0); button_poll(now+30,0);
}
static void press(uint32_t now)
{
    button_poll(now,1); button_poll(now+30,1);
}
int main(void)
{
    _Static_assert(sizeof(struct w600_endpoint)==36,"Keep radio endpoint layout");
    clear(); release(0); press(100); button_poll(3130,1);
    assert(radio.endpoint.child_locked && control_calls==1);
    radio.endpoint.control_enabled=true; radio.endpoint.known|=W600_KNOWN_TARGET;
    radio.endpoint.target_raw=2000;
    release(3200); button_poll(3300,4); button_poll(3330,4);
    assert(radio.endpoint.target_raw==2000 && !target_calls);
    release(3400); press(3500); button_poll(6530,1);
    assert(!radio.endpoint.child_locked && control_calls==2);
    button_poll(13530,1); assert(!radio.endpoint.child_locked && calls==1 && control_calls==3);
    puts("PASS child lock: 3s toggle, locked dial still wakes but cannot change target, pairing clears lock");
    for(unsigned input=2;input<=4;input+=2) {
        for(unsigned raw=500;raw<=3000;++raw) {
            clear(); radio.endpoint=(struct w600_endpoint){.control_enabled=true,
                .known=W600_KNOWN_TARGET|W600_KNOWN_EXTERNAL_USE|W600_KNOWN_EXTERNAL_TEMP,
                .target_raw=(uint16_t)raw,.external_use=1,.external_temp_raw=1950,.mode_known=true};
            release(0); button_poll(100,(int)input); button_poll(130,(int)input);
            unsigned target=input==2 ? (raw<=550 ? 500 : raw-50) : (raw>=2950 ? 3000 : raw+50);
            assert(radio.endpoint.target_raw==target && target_calls==(target!=raw));
            assert(radio.endpoint.control_updates==(target!=raw) && !calls);
            assert(!radio.endpoint.mode && radio.endpoint.mode_known && radio.endpoint.external_use==1 &&
                   radio.endpoint.external_temp_raw==1950 && !radio.endpoint.external_updates);
            button_poll(5000,(int)input);
            assert(radio.endpoint.target_raw==target && target_calls==(target!=raw));
        }
    }
    clear(); release(0); button_poll(100,4); button_poll(130,4);
    assert(!(radio.endpoint.known&W600_KNOWN_TARGET) && !target_calls);
    clear(); radio.endpoint=(struct w600_endpoint){.control_enabled=true,.known=W600_KNOWN_TARGET,.target_raw=2000};
    release(0); button_poll(100,6); button_poll(130,6); button_poll(500,4);
    assert(radio.endpoint.target_raw==2000 && !target_calls && !calls);
    release(501); button_poll(600,4); button_poll(630,4);
    assert(radio.endpoint.target_raw==2050 && target_calls==1);
    puts("PASS local buttons: all valid targets, 0.5C limits, held/conflicting inputs, unknown target, Off/source ownership");
#ifdef W600_DISPLAY_BENCH
    struct w600_endpoint endpoint={0};
    uint8_t frame[4]={1,2,3,4}, expected[4]={0};
    target_frame(&endpoint,frame); assert(!memcmp(frame,expected,4));
    endpoint.known=W600_KNOWN_TARGET;
    for (uint32_t raw=0;raw<=9990;++raw) {
        endpoint.target_raw=(uint16_t)raw;
        memset(expected,0,4); assert(w600_display_encode(raw/10u,false,expected));
        target_frame(&endpoint,frame); assert(!memcmp(frame,expected,4));
    }
    for (uint32_t raw=9991;raw<=UINT16_MAX;++raw) {
        endpoint.target_raw=(uint16_t)raw; memset(expected,0,4);
        target_frame(&endpoint,frame); assert(!memcmp(frame,expected,4));
    }
    puts("PASS display target: unknown blank, all65536 raw targets, no invented position/status");
    for (unsigned lock=0;lock<2;++lock) for (unsigned fault=0;fault<256;++fault) {
        endpoint=(struct w600_endpoint){.child_locked=lock!=0,.control_error=(uint8_t)fault};
        target_frame(&endpoint,frame);
        unsigned bits=(lock ? 0x10u : 0) |
            (fault>=W600_CONTROL_TEMPERATURE && fault<=W600_CONTROL_DRIVER ? 4u : 0);
        assert(frame[0]==0 && frame[1]==0 && frame[2]==bits && frame[3]==0);
        endpoint.calibration_error=1; target_frame(&endpoint,frame);
        assert(frame[2]==(bits|4u));
    }
    clear(); radio.endpoint.known=W600_KNOWN_TARGET; radio.endpoint.target_raw=2000;
    struct w600_endpoint unchanged=radio.endpoint;
    assert(!display_should_show(0,0) && !display_should_show(100,0));
    assert(display_should_show(10101,W600_BUTTON_WAKE));
    assert(display_should_show(20100,0) && !display_should_show(20101,0));
    assert(!memcmp(&unchanged,&radio.endpoint,sizeof unchanged) && !calls && !target_calls);
    for (unsigned input=1;input<=4;input*=2) {
        clear(); release(0);
        assert(!display_should_show(100,button_poll(100,(int)input)));
        assert(display_should_show(130,button_poll(130,(int)input)));
        assert(display_should_show(10129,button_poll(10129,(int)input)));
        assert(!display_should_show(10130,button_poll(10130,(int)input)));
    }
    clear(); radio.endpoint=unchanged;
    radio.endpoint.target_raw=2150;
    assert(!display_should_show(300000,0));
    radio.endpoint.calibration_busy=true;
    assert(!display_should_show(1000000,0));
    assert(display_should_show(1000001,W600_BUTTON_WAKE));
    radio.endpoint.target_raw=2200;
    assert(display_should_show(1010000,0) && !display_should_show(1010001,0));
    clear(); button_poll(0,7); button_poll(30000,7);
    assert(!display_should_show(30000,button_poll(40000,7)));
    clear(); release(0); button_poll(100,1); button_poll(115,0);
    assert(!display_should_show(200,button_poll(200,0)));
    clear(); assert(display_should_show(UINT32_MAX-5000u,W600_BUTTON_WAKE));
    assert(display_should_show(4998,0) && !display_should_show(4999,0));
    assert(!display_should_show(UINT32_MAX-5000u,0));
    assert(!(radio.endpoint.known&W600_KNOWN_TARGET) && !calls && !target_calls);
    puts("PASS screen: physical input only, dark boot, held boot/glitches ignored, remote target/calibration silent, 10s timeout/wrap");
#endif
    clear(); button_poll(0,1); button_poll(30000,1);
    assert(!calls); /* A held button at boot must not erase pairing. */
    release(30001); press(30100); button_poll(40129,1); assert(!calls);
    button_poll(40130,1); assert(calls==1 && called_at==40130);
    button_poll(80000,1); assert(calls==1);
    assert(app_report.pairing_attempts==1 && app_report.pairing_request_accepted==1);
    release(80001); press(80100); button_poll(90130,1); assert(calls==2);

    clear(); release(0); press(100); release(500); button_poll(20000,0);
    assert(!calls); /* Short presses do not request pairing. */
    button_poll(20100,1); button_poll(20101,0); button_poll(20102,1);
    button_poll(20132,1); button_poll(30131,1); assert(!calls);
    button_poll(30132,1); assert(calls==1);

    clear(); accepted=0; release(0); press(100); button_poll(10130,1);
    button_poll(30000,1); assert(calls==1 && app_report.pairing_request_accepted==0);

    clear(); release(0); press(100); button_poll(10129,0); button_poll(10130,0);
    assert(!calls); /* A release already seen at the deadline cancels it. */

    clear(); release(UINT32_MAX-200); press(UINT32_MAX-100);
    button_poll(9928,1); assert(!calls);
    button_poll(9929,1); assert(calls==1); /* Millisecond rollover. */
    puts("PASS button: held-at-boot, debounce, deadline, one-shot, release, rejection, wrap");
}
