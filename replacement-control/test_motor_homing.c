#include "motor_homing.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

/* Synthetic test units only: not proposed W600 thresholds or safe travel times. */
static const struct motor_homing_policy policy={1000,100,100,20,40,10,100,10,10,3,3};
struct fixture { struct motor_homing h; unsigned stops; bool motor_on; };
static void stop(void *context)
{
    struct fixture *f=context;
    ++f->stops; f->motor_on=false;
}
static void init(struct fixture *f)
{
    *f=(struct fixture){.motor_on=true};
    assert(motor_homing_init(&f->h,stop,f));
    assert(!f->motor_on && f->stops==1 && f->h.state==MOTOR_HOMING_IDLE);
}
static void start(struct fixture *f,uint32_t now,const struct motor_homing_policy *p)
{
    assert(motor_homing_start(&f->h,now,p));
    assert(!f->motor_on && f->h.guard.armed); /* arm does not drive */
    f->motor_on=true;
}
static bool sample(struct fixture *f,uint32_t now,uint32_t at,uint32_t value)
{
    if (!motor_homing_foreground_progress(&f->h,now)) return false;
    return motor_homing_feedback(&f->h,now,at,value,true);
}
static void qualify_motion(struct fixture *f,uint32_t t)
{
    assert(sample(f,t,t,40)); assert(sample(f,t+5,t+5,50));
    assert(sample(f,t+10,t+10,100));
    assert(f->h.state==MOTOR_HOMING_SEEK_ENDPOINT);
}
static void qualify_endpoint(struct fixture *f,uint32_t t)
{
    assert(sample(f,t,t,10)); assert(sample(f,t+5,t+5,0));
    assert(f->h.state==MOTOR_HOMING_SEEK_ENDPOINT);
    assert(sample(f,t+10,t+10,10));
    assert(f->h.state==MOTOR_HOMING_ENDPOINT_OBSERVED && !f->h.guard.armed && !f->motor_on);
    assert(f->h.error==MOTOR_HOMING_OK && f->h.guard.error==MOTOR_GUARD_OK);
}
static void happy_path_and_wrap(void)
{
    const uint32_t starts[]={0,UINT32_MAX-12,UINT32_C(0x80000000)};
    for (unsigned i=0;i<sizeof starts/sizeof *starts;++i) {
        struct fixture f; init(&f); start(&f,starts[i],&policy);
        qualify_motion(&f,starts[i]); qualify_endpoint(&f,starts[i]+15);
        assert(f.stops==2);
        assert(!sample(&f,starts[i]+26,starts[i]+26,0));
        assert(!motor_homing_start(&f.h,starts[i]+30,&policy));
        assert(f.h.state==MOTOR_HOMING_ENDPOINT_OBSERVED);
        assert(motor_homing_reset(&f.h,true));
        assert(f.h.state==MOTOR_HOMING_IDLE && !f.h.have_sample);
    }
}
static void counts_and_dwell_are_separate(void)
{
    struct fixture f; init(&f); start(&f,0,&policy);
    assert(sample(&f,0,0,50)); assert(sample(&f,1,1,50)); assert(sample(&f,2,2,50));
    assert(f.h.motion_count==3 && f.h.state==MOTOR_HOMING_SEEK_MOTION);
    assert(sample(&f,10,10,50));
    assert(f.h.state==MOTOR_HOMING_SEEK_ENDPOINT);
    assert(sample(&f,11,11,0)); assert(sample(&f,12,12,0)); assert(sample(&f,13,13,0));
    assert(f.h.endpoint_count==3 && f.motor_on); /* count met, dwell not met */
    assert(sample(&f,21,21,0)); assert(!f.motor_on);

    init(&f); start(&f,0,&policy);
    assert(sample(&f,0,0,50)); assert(sample(&f,20,20,50));
    assert(f.h.state==MOTOR_HOMING_SEEK_MOTION); /* dwell met, count not met */
    assert(sample(&f,21,21,50));
    assert(f.h.state==MOTOR_HOMING_SEEK_ENDPOINT);
    assert(sample(&f,22,22,0)); assert(sample(&f,42,42,0));
    assert(f.motor_on && f.h.endpoint_count==2);
    assert(sample(&f,43,43,0)); assert(!f.motor_on);
}
static void qualified_motion_survives_slowdown_but_low_streaks_do_not(void)
{
    struct fixture f; init(&f); start(&f,0,&policy); qualify_motion(&f,0);
    assert(sample(&f,15,15,0));
    assert(sample(&f,20,20,20)); /* ambiguous interrupts only the low streak */
    assert(f.h.state==MOTOR_HOMING_SEEK_ENDPOINT && f.h.endpoint_count==0);
    assert(sample(&f,25,25,0)); assert(sample(&f,30,30,0));
    assert(f.motor_on && f.h.endpoint_count==2);
    assert(sample(&f,35,35,50)); /* high also breaks the low streak */
    assert(f.h.state==MOTOR_HOMING_SEEK_ENDPOINT && f.h.endpoint_count==0);
    qualify_endpoint(&f,40);

    init(&f); start(&f,0,&policy); qualify_motion(&f,0);
    assert(sample(&f,15,15,20));
    assert(f.h.state==MOTOR_HOMING_SEEK_ENDPOINT);
    qualify_endpoint(&f,20);

    init(&f); start(&f,0,&policy);
    assert(sample(&f,0,0,50)); assert(sample(&f,5,5,20));
    assert(f.h.state==MOTOR_HOMING_SEEK_MOTION && f.h.motion_count==0);
    assert(sample(&f,10,10,0)); assert(sample(&f,20,20,0));
    assert(f.motor_on); /* Startup low cannot use an incomplete motion streak. */
}
static void no_motion_and_slipping_like_signal_time_out(void)
{
    for (unsigned kind=0;kind<2;++kind) {
        struct fixture f; init(&f); start(&f,0,&policy);
        for (uint32_t t=0;t<1000;t+=10) assert(sample(&f,t,t,kind ? 50 : 0));
        assert(f.motor_on && f.h.guard.move_deadline==1000);
        assert(!sample(&f,1000,1000,0));
        assert(!f.motor_on && f.h.state==MOTOR_HOMING_FAILED);
        assert(f.h.guard.error==MOTOR_GUARD_MOVE_TIMEOUT);
    }
}
static void stale_invalid_and_missing_feedback(void)
{
    const unsigned kinds=6;
    for (unsigned kind=0;kind<kinds;++kind) {
        struct fixture f; init(&f); start(&f,100,&policy);
        assert(sample(&f,100,100,50));
        uint32_t deadline=f.h.guard.adc_deadline;
        bool valid=kind!=5;
        uint32_t at=kind==0 ? 100 : kind==1 ? 99 : kind==2 ? 121 : 110;
        uint32_t now=kind==3 ? 130 : 110;
        assert(!motor_homing_feedback(&f.h,now,at,kind==4 ? 101 : 50,valid));
        assert(!f.motor_on && f.h.state==MOTOR_HOMING_FAILED);
        assert(f.h.guard.adc_deadline==deadline); /* rejected data never renews */
        assert(f.h.error==(kind<4 ? MOTOR_HOMING_STALE_FEEDBACK : MOTOR_HOMING_INVALID_FEEDBACK));
    }
    struct fixture f; init(&f); start(&f,0,&policy);
    assert(motor_homing_foreground_progress(&f.h,90));
    motor_homing_tick(&f.h,100);
    assert(!f.motor_on && f.h.guard.error==MOTOR_GUARD_ADC_TIMEOUT);

    init(&f); start(&f,0,&policy);
    assert(motor_homing_feedback(&f.h,0,0,50,true));
    assert(motor_homing_feedback(&f.h,90,90,50,true));
    motor_homing_tick(&f.h,100);
    assert(!f.motor_on && f.h.guard.error==MOTOR_GUARD_FOREGROUND_TIMEOUT);

    init(&f); start(&f,100,&policy);
    assert(sample(&f,100,100,50)); assert(sample(&f,110,110,50));
    uint32_t deadline=f.h.guard.adc_deadline;
    assert(!motor_homing_feedback(&f.h,120,105,50,true)); /* after arm, but reordered */
    assert(f.h.error==MOTOR_HOMING_STALE_FEEDBACK && f.h.guard.adc_deadline==deadline);
}
static void acquisition_age_deadline_and_expired_calls(void)
{
    struct fixture f; struct motor_homing_policy p=policy;
    p.max_sample_age_ticks=p.feedback_lease_ticks;
    init(&f); start(&f,0,&p);
    assert(sample(&f,99,0,50));
    assert(f.h.guard.adc_deadline==199); /* receipt lease alone would be too late */
    assert(!motor_homing_feedback(&f.h,100,100,0,true));
    assert(!f.motor_on && f.h.error==MOTOR_HOMING_STALE_FEEDBACK);

    const uint32_t starts[]={0,UINT32_MAX-49};
    for (unsigned i=0;i<2;++i) {
        init(&f); start(&f,starts[i],&policy);
        assert(sample(&f,starts[i],starts[i],50));
        assert(!motor_homing_foreground_progress(&f.h,starts[i]+100));
        assert(!motor_homing_feedback(&f.h,starts[i]+100,starts[i]+100,0,true));
        assert(!f.h.guard.armed && !f.motor_on && f.h.state==MOTOR_HOMING_FAILED);
    }
    init(&f); start(&f,100,&policy);
    assert(!motor_homing_feedback(&f.h,100,99,50,true)); /* predates this attempt */

    p=policy; p.max_move_ticks=40; p.foreground_lease_ticks=40; p.feedback_lease_ticks=40;
    init(&f); start(&f,0,&p); qualify_motion(&f,0);
    assert(sample(&f,30,30,0)); assert(sample(&f,35,35,0));
    assert(!motor_homing_feedback(&f.h,40,40,0,true)); /* would meet count+dwell, but expired */
    assert(f.h.state==MOTOR_HOMING_FAILED && f.h.guard.error==MOTOR_GUARD_MOVE_TIMEOUT);
}
static void bad_policy_rearm_and_error_latches(void)
{
    for (unsigned i=0;i<13;++i) {
        struct fixture f; struct motor_homing_policy p=policy; init(&f);
        switch(i) {
        case 0:p.max_move_ticks=0;break;
        case 1:p.max_move_ticks=UINT32_C(0x80000000);break;
        case 2:p.foreground_lease_ticks=0;break;
        case 3:p.feedback_lease_ticks=1001;break;
        case 4:p.max_sample_age_ticks=101;break;
        case 5:p.max_sample_age_ticks=0;break;
        case 6:p.motion_samples=0;break;
        case 7:p.endpoint_samples=0;break;
        case 8:p.motion_dwell_ticks=0;break;
        case 9:p.endpoint_dwell_ticks=0;break;
        case 10:p.endpoint_dwell_ticks=990;break;
        case 11:p.endpoint_max=40;break;
        default:p.signal_max=39;break;
        }
        assert(!motor_homing_start(&f.h,0,&p));
        assert(!f.motor_on && f.h.error==MOTOR_HOMING_BAD_POLICY);
    }
    struct fixture f; init(&f);
    assert(!motor_homing_start(&f.h,0,NULL));
    assert(motor_homing_reset(&f.h,true)); start(&f,0,&policy);
    assert(!motor_homing_start(&f.h,10,NULL));
    assert(!f.motor_on && f.h.guard.error==MOTOR_GUARD_REARM);
    assert(f.h.guard.move_deadline==1000);
    assert(motor_homing_reset(&f.h,true)); start(&f,20,&policy);
    motor_homing_abort(&f.h);
    assert(!f.motor_on && f.h.error==MOTOR_HOMING_ABORTED);
    assert(!motor_homing_start(&f.h,21,&policy));
    assert(!motor_homing_reset(&f.h,false));
    assert(f.h.error==MOTOR_HOMING_ABORTED);
    assert(motor_homing_reset(&f.h,true)); start(&f,30,&policy);
    assert(!motor_homing_reset(&f.h,true));
    assert(!f.motor_on && f.h.state==MOTOR_HOMING_FAILED);
    assert(motor_homing_reset(&f.h,true));
    struct motor_homing no_callback;
    assert(!motor_homing_init(&no_callback,NULL,NULL));
    assert(!motor_homing_start(&no_callback,0,&policy));
}
static void short_return_then_low_signature(void)
{
    /* Reduced V13 trace signature; the second variant extends motion synthetically. */
    const struct motor_homing_policy p={45000,100,100,10,1100,100,4000,200,20,11,2};
    for(unsigned extended=0;extended<2;++extended) {
        struct fixture f; init(&f); start(&f,0,&p);
        for(unsigned sequence=2;sequence<=47;++sequence) {
            uint32_t at=1+(sequence-2)*20;
            uint32_t value=sequence==2 ? 1 : sequence==3 ? 816 : sequence==4 ? 1059 :
                sequence<=11 || (extended && sequence<=15) ? 1200 :
                sequence==46 ? 2 : sequence==47 ? 90 : 900;
            assert(sample(&f,at,at,value));
        }
        if(extended) assert(f.h.state==MOTOR_HOMING_ENDPOINT_OBSERVED && !f.motor_on);
        else {
            assert(f.h.state==MOTOR_HOMING_SEEK_MOTION && !f.h.endpoint_count && f.motor_on);
            motor_homing_abort(&f.h); assert(!f.motor_on);
        }
    }
}
int main(void)
{
    short_return_then_low_signature();
    happy_path_and_wrap(); counts_and_dwell_are_separate();
    qualified_motion_survives_slowdown_but_low_streaks_do_not(); no_motion_and_slipping_like_signal_time_out();
    stale_invalid_and_missing_feedback(); acquisition_age_deadline_and_expired_calls();
    bad_policy_rearm_and_error_latches();
    puts("bounded homing controller assertions passed; no physical calibration claim");
    return 0;
}
