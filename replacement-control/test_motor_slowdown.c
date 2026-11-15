#include "motor_slowdown.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void high(struct motor_slowdown *s,uint32_t *at,uint32_t mv)
{ for(unsigned i=0;i<22;i++,*at+=200) assert(!motor_slowdown_observe(s,*at,mv)); }
static void window(struct motor_seat *s,uint32_t *at,uint32_t mv,uint32_t expected)
{
    for(unsigned i=0;i<10;++i,*at+=200) {
        if(expected) (void)motor_seat_return_observe(s,*at,mv,expected);
        else (void)motor_seat_observe(s,*at,mv);
    }
}
static void plateau_tests(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct motor_seat s={0};uint32_t at=wrap ? UINT32_MAX-10000u : 0;
        for(unsigned i=0;i<10;++i) window(&s,&at,1390,0);
        for(unsigned i=0;i<20;++i) window(&s,&at,945,0);
        assert(!motor_seat_loaded(&s) && !s.candidate);
        window(&s,&at,1042,0); /* Saved failure's75% entry boundary is1042.5mV. */
        for(unsigned i=1;i<25;++i) {
            window(&s,&at,i%2 ? 1049 : 1041,0);
            assert(motor_seat_loaded(&s)==(i==24));
        }
        assert(s.plateau_mv==1042 && !s.candidate); /* No invented valley recovery. */
        window(&s,&at,1092,0);assert(motor_seat_loaded(&s)); /* Exactly50mV. */
        window(&s,&at,1093,0);assert(!motor_seat_loaded(&s));
        for(unsigned i=0;i<30;++i) window(&s,&at,1093,0);
        assert(!motor_seat_loaded(&s)); /* An exited band cannot restart above75%. */
        motor_seat_reset(&s);at=0;
        for(unsigned i=0;i<10;++i) window(&s,&at,1300,0);
        for(unsigned i=0;i<100;++i) window(&s,&at,1000,0);
        assert(!motor_seat_loaded(&s)); /* Mere spring contact never enters the band. */
        motor_seat_reset(&s);at=0;
        for(unsigned i=0;i<10;++i) window(&s,&at,1390,0);
        window(&s,&at,1040,0);
        for(unsigned i=0;i<10;++i) window(&s,&at,1045+5*i,0);
        assert(!motor_seat_loaded(&s) && s.plateau_mv==1040);
        window(&s,&at,1095,0);assert(!s.plateau_count); /* Fixed anchor, not moving average. */
        motor_seat_reset(&s);at=0;
        for(unsigned i=0;i<10;++i) window(&s,&at,1390,0);
        for(unsigned i=0;i<24;++i) window(&s,&at,1042,0);
        at+=400;window(&s,&at,1042,0);
        assert(!motor_seat_loaded(&s) && !s.baseline_mv); /* Missing samples erase proof. */
    }
    for(unsigned wrong=0;wrong<3;++wrong) {
        struct motor_seat s={0};uint32_t at=0;
        for(unsigned i=0;i<10;++i) window(&s,&at,1390,1000);
        for(unsigned i=0;i<30;++i) window(&s,&at,wrong==1 ? 1101 : wrong==2 ? 899 : 1000,1000);
        assert(motor_seat_loaded(&s)==!wrong); /* Hysteresis never weakens return matching. */
    }
    struct motor_seat s={0};uint32_t at=0;
    for(unsigned i=0;i<10;++i) window(&s,&at,1390,945);
    for(unsigned i=0;i<30;++i) window(&s,&at,1042,945);
    assert(!motor_seat_loaded(&s)); /* Within100mV but outside10% still fails. */
    puts("PASS fixed-anchor load band: threshold chatter, contact rejection, drift, gaps and return matching");
}
static void seat_tests(void)
{
    _Static_assert(sizeof(struct motor_seat)==32,"Keep motor-state layout unchanged");
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct motor_seat s={0};uint32_t at=wrap ? UINT32_MAX-3000u : 0;
        for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,1300));
        for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,1000));
        assert(s.below_count==2 && !s.candidate); /* Spring load alone is not closed. */
        for(unsigned i=0;i<20;i++,at+=200) assert(!motor_seat_observe(&s,at,800));
        for(unsigned i=0;i<19;i++,at+=200) assert(!motor_seat_observe(&s,at,1000));
        assert(motor_seat_observe(&s,at,1000) && s.candidate);
        assert(!motor_seat_observe(&s,at,1000) && !s.have_sample); /* Duplicate breaks proof. */
        for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,1000));
        assert(!s.candidate);
        assert(!motor_seat_observe(&s,at+1000,800) && !s.baseline_mv);
        assert(!motor_seat_observe(&s,at+1200,4001) && !s.have_sample);
    }
    struct motor_seat s={0};uint32_t at=0;
    for(unsigned i=0;i<1000;i++,at+=200) {
        uint32_t mv=i%10==0 ? 3500 : 1000;
        assert(!motor_seat_observe(&s,at,mv)); /* Spikes cannot inflate running baseline. */
    }
    assert(s.baseline_mv==1000);
    motor_seat_reset(&s);at=0;
    for(unsigned i=0;i<400;i++,at+=200) {
        uint32_t mv=i<100 ? 1300 : i<105 ? 800 : 1300;
        assert(!motor_seat_observe(&s,at,mv)); /* One100ms dip is insufficient. */
    }
    motor_seat_reset(&s);at=0;
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,1300));
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,900));
    for(unsigned i=0;i<10;i++,at+=200) assert(!motor_seat_observe(&s,at,1050));
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,900));
    motor_seat_reset(&s);at=0;
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,1385));
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,918));
    for(unsigned i=0;i<19;i++,at+=200) assert(!motor_seat_observe(&s,at,1047));
    assert(motor_seat_observe(&s,at,1047)); /* Mounted trace recovers13%, not15%. */
    motor_seat_reset(&s);at=0;
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,3800));
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,3000));
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,3320));
    for(unsigned wrap=0;wrap<2;++wrap) {
        motor_seat_reset(&s);at=wrap ? UINT32_MAX-10000u : 0;
        for(unsigned i=0;i<100;i++,at+=200) assert(!motor_seat_observe(&s,at,1300));
        for(unsigned i=0;i<249;i++,at+=200) {
            assert(!motor_seat_observe(&s,at,900));assert(!motor_seat_loaded(&s));
        }
        assert(!motor_seat_observe(&s,at,900));assert(motor_seat_loaded(&s));at+=200;
        assert(!motor_seat_observe(&s,at+500,900));assert(!motor_seat_loaded(&s));
        motor_seat_reset(&s);at=0;
        for(unsigned i=0;i<100;i++,at+=200) motor_seat_observe(&s,at,1300);
        for(unsigned i=0;i<500;i++,at+=200) {
            motor_seat_observe(&s,at,1050);assert(!motor_seat_loaded(&s));
        }
        for(unsigned i=0;i<1000;i++,at+=200) {
            motor_seat_observe(&s,at,1000-i/2);assert(!motor_seat_loaded(&s));
        }
    }
    puts("PASS closing seat: sustained valley/recovery, plateaus, short dips/recovery, spikes, gaps, duplicates, invalid and wrap");
}
int main(int argc,char **argv)
{
    struct motor_slowdown s={0};uint32_t at=0;
    if(argc==2 && !strcmp(argv[1],"seat-replay")) {
        struct motor_seat seat={0};uint32_t mv;bool reported=false;
        while(scanf("%u %u",&at,&mv)==2)
            if(motor_seat_observe(&seat,at,mv) && !reported) {
                printf("seat_at=%u baseline_mv=%u trough_mv=%u sample_mv=%u\n",at,seat.baseline_mv,seat.trough_mv,mv);reported=true;
            }
        if(!reported) puts("no_seat");
        return 0;
    }
    if(argc==2 && !strcmp(argv[1],"replay")) {
        uint32_t mv;bool reported=false;
        while(scanf("%u %u",&at,&mv)==2)
            if(motor_slowdown_observe(&s,at,mv) && !reported) {
                printf("candidate_at=%u baseline_mv=%u sample_mv=%u\n",at,s.baseline_mv,mv);reported=true;
            }
        if(!reported) puts("no_candidate");
        return 0;
    }
    high(&s,&at,1300);
    for(unsigned i=0;i<10;i++,at+=200) assert(!motor_slowdown_observe(&s,at,1000));
    assert(motor_slowdown_observe(&s,at,1000));
    assert(s.baseline_mv==1300 && s.candidate);
    motor_slowdown_reset(&s);at=0;
    /* A loaded plateau below the old1.1V gate can establish a baseline, not an endpoint. */
    for(unsigned i=0;i<300;i++,at+=200) assert(!motor_slowdown_observe(&s,at,1000));
    assert(s.baseline_mv==1000 && !s.candidate);
    motor_slowdown_reset(&s);at=0;high(&s,&at,800);
    assert(s.baseline_mv==800);
    for(unsigned i=0;i<10;i++,at+=200) assert(!motor_slowdown_observe(&s,at,650));
    assert(motor_slowdown_observe(&s,at,650));
    motor_slowdown_reset(&s);at=0;
    for(unsigned i=0;i<300;i++,at+=200) assert(!motor_slowdown_observe(&s,at,599));
    assert(!s.baseline_mv && !s.candidate);
    motor_slowdown_reset(&s);at=0;high(&s,&at,1300);
    for(unsigned i=0;i<100;i++,at+=200) assert(!motor_slowdown_observe(&s,at,1200));
    /* Transient dip, startup spike, gap, duplicate/backward and invalid input. */
    motor_slowdown_reset(&s);at=0;high(&s,&at,1300);
    for(unsigned i=0;i<5;i++,at+=200) assert(!motor_slowdown_observe(&s,at,900));
    assert(!motor_slowdown_observe(&s,at,1300));at+=200;
    assert(!motor_slowdown_observe(&s,at+400,900));assert(!s.baseline_mv);
    assert(!motor_slowdown_observe(&s,at+400,900));
    assert(!motor_slowdown_observe(&s,at+400,900));assert(!s.have_sample);
    high(&s,&at,1300);assert(!motor_slowdown_observe(&s,at-1000,900));assert(!s.baseline_mv);
    motor_slowdown_reset(&s);at=0;
    assert(!motor_slowdown_observe(&s,at,3000));at+=200;
    for(unsigned i=0;i<20;i++,at+=200) assert(!motor_slowdown_observe(&s,at,1000));
    assert(s.baseline_mv==1000 && !s.candidate);
    for(unsigned i=0;i<100;i++,at+=200)
        assert(!motor_slowdown_observe(&s,at,i%11==0 ? 3000 : 1000));
    assert(s.baseline_mv==1000 && !s.candidate);
    assert(!motor_slowdown_observe(&s,at,4001));assert(!s.have_sample);
    motor_slowdown_reset(&s);at=UINT32_MAX-5000;high(&s,&at,1300);
    for(unsigned i=0;i<10;i++,at+=200) assert(!motor_slowdown_observe(&s,at,1000));
    assert(motor_slowdown_observe(&s,at,1000));
    puts("PASS provisional slowdown: sustained drop, steady signals, transients, gaps, invalid input, wrap");
    seat_tests();
    plateau_tests();
}
