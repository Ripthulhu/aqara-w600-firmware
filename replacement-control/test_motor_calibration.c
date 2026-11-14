#include "motor_calibration.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

/* Synthetic units only; these are NOT W600 thresholds, travel or stop timings. */
static const struct motor_calibration_policy policy={
    .homing={{300,40,40,10,40,10,100,2,2,2,2},{300,40,40,10,40,10,100,2,2,2,2}},
    .whole_attempt_ticks=2000,.probe_on_ticks=12,
    .min_travel_ticks=10,.max_travel_ticks=120,.repeat_tolerance_ticks=4,
    .stop_confirm_ticks=30,.min_stop_ticks=3
};
struct fixture { struct motor_calibration c; unsigned stops,starts; bool motor_on; };
static unsigned cases;
static void stop(void *context)
{ struct fixture *f=context; ++f->stops; f->motor_on=false; }
static void init(struct fixture *f,uint32_t now,const struct motor_calibration_policy *p)
{
    *f=(struct fixture){.motor_on=true};
    assert(motor_calibration_init(&f->c,stop,f));
    assert(!f->motor_on && f->stops==1 && f->c.state==MOTOR_CAL_IDLE);
    assert(motor_calibration_start(&f->c,now,p,true));
    assert(!f->motor_on); /* The sequencer NEVER starts the motor. */
}
static void drive(struct fixture *f)
{
    assert(f->c.state==MOTOR_CAL_ACTIVE && f->c.homing.guard.armed);
    assert(f->c.action.direction && f->c.action.token && f->c.action.on_budget_ticks);
    assert(!f->motor_on); f->motor_on=true; ++f->starts;
}
static bool sample(struct fixture *f,uint32_t offset,uint32_t value,uint32_t on)
{
    uint32_t now=f->c.phase_started_at+offset, token=f->c.action.token;
    if(!motor_calibration_foreground_progress(&f->c,token,now,on)) return false;
    if(f->c.state!=MOTOR_CAL_ACTIVE) return f->c.state==MOTOR_CAL_WAIT_STOP;
    return motor_calibration_feedback(&f->c,token,now,now,value,true,on,on);
}
static void motion_probe(struct fixture *f)
{
    drive(f);
    uint32_t token=f->c.action.token,start=f->c.phase_started_at;
    uint32_t attempt=f->c.attempt_started_at,deadline=f->c.homing.guard.move_deadline;
    unsigned stops=f->stops;
    for(uint32_t at=1;at<=f->c.policy.probe_on_ticks;++at)
        assert(sample(f,at,f->c.homing.policy.motion_min+5,at));
    assert(f->c.state==MOTOR_CAL_ACTIVE && f->c.phase==MOTOR_CAL_HOME && f->motor_on);
    assert(f->c.action.token==token && f->c.phase_started_at==start &&
           f->c.attempt_started_at==attempt && f->stops==stops &&
           f->c.homing.guard.move_deadline==deadline && f->c.homing.started_at==start);
    assert(f->c.action.on_budget_ticks==f->c.homing.policy.max_move_ticks);
}
static void low_probe(struct fixture *f)
{
    drive(f);
    for(uint32_t at=1;at<=policy.probe_on_ticks;++at) assert(sample(f,at,0,at));
    assert(f->c.state==MOTOR_CAL_WAIT_STOP && !f->motor_on && f->c.intentional_abort);
}
static bool confirm(struct fixture *f,bool physical)
{
    if(!f->c.have_stop_proof) {
        uint32_t first=f->c.stop_requested_at+f->c.policy.min_stop_ticks;
        assert(!motor_calibration_confirm_stop(&f->c,f->c.action.token,first,f->c.on_ticks,true,physical));
        if(f->c.state==MOTOR_CAL_FAILED) return false;
        assert(f->c.state==MOTOR_CAL_WAIT_STOP && !f->motor_on);
    }
    uint32_t now=f->c.stop_observed_at+f->c.policy.min_stop_ticks;
    return motor_calibration_confirm_stop(&f->c,f->c.action.token,now,f->c.on_ticks,true,physical);
}
static void seek_endpoint_scaled(struct fixture *f,uint32_t travel,uint32_t dwell,uint32_t scale)
{
    assert(f->c.phase==MOTOR_CAL_HOME || f->c.phase==MOTOR_CAL_MEASURE);
    if(!f->motor_on) {
        drive(f); assert(sample(f,scale,50,1)); assert(sample(f,3*scale,50,3));
    }
    for(uint32_t at=f->c.on_ticks+7;at<travel;at+=10) assert(sample(f,at*scale,50,at));
    assert(sample(f,travel*scale,0,travel));
    assert(f->c.have_first_low && f->c.first_low_on==travel);
    assert(sample(f,(travel+dwell)*scale,0,travel+dwell));
    assert(f->c.state==MOTOR_CAL_WAIT_STOP && !f->motor_on && !f->c.intentional_abort);
}
static void seek_endpoint(struct fixture *f,uint32_t travel,uint32_t dwell)
{ seek_endpoint_scaled(f,travel,dwell,1); }
static void prepare_measured(struct fixture *f,uint32_t now,bool fallback,
                             const struct motor_calibration_policy *p)
{
    init(f,now,p);
    if(fallback) { low_probe(f); assert(confirm(f,false)); assert(f->c.phase==MOTOR_CAL_PROBE_OPEN); }
    motion_probe(f);
    assert(f->c.phase==MOTOR_CAL_HOME && f->c.direction==(fallback ? MOTOR_CAL_OPEN : MOTOR_CAL_CLOSE));
    seek_endpoint(f,20,p->homing[f->c.direction-1u].endpoint_dwell_ticks); /* Initial partial HOME is not learned. */
    assert(confirm(f,true)); assert(f->c.phase==MOTOR_CAL_MEASURE && !f->motor_on);
}
static void finish_four(struct fixture *f,uint32_t dwell)
{
    for(unsigned leg=0;leg<4;++leg) {
        uint32_t travel=(f->c.direction==MOTOR_CAL_OPEN ? 50u : 40u)+(leg>=2 ? 2u : 0u);
        seek_endpoint(f,travel,dwell);
        assert(confirm(f,true));
        if(leg<3) assert(f->c.state==MOTOR_CAL_ACTIVE && !f->motor_on);
    }
    assert(f->c.state==MOTOR_CAL_CANDIDATE && f->c.error==MOTOR_CAL_OK && !f->motor_on);
    assert(f->c.candidate.opening_ticks==51 && f->c.candidate.closing_ticks==41);
    assert(!f->c.action.direction && !f->c.homing.guard.armed);
}
static void flows_and_wrap(void)
{
    for(unsigned fallback=0;fallback<2;++fallback) {
        for(unsigned wrap=0;wrap<2;++wrap) {
            struct fixture f; prepare_measured(&f,wrap ? UINT32_MAX-5u : 0,fallback!=0,&policy);
            uint32_t attempt=f.c.attempt_started_at, deadline=attempt+policy.whole_attempt_ticks;
            finish_four(&f,2);
            assert(f.c.attempt_started_at==attempt && attempt+f.c.policy.whole_attempt_ticks==deadline);
            assert(f.c.candidate.final_endpoint_permille==(fallback ? 1000 : 0));
            assert(f.starts==(fallback ? 6u : 5u));
            assert(f.c.samples[0][0]==50 && f.c.samples[0][1]==52);
            assert(f.c.samples[1][0]==40 && f.c.samples[1][1]==42);
            assert(f.c.final_on_samples[0][0]==52 && f.c.final_on_samples[1][1]==44);
            ++cases;
        }
    }
}
static void first_low_not_confirmation_energy(void)
{
    struct motor_calibration_policy p=policy;
    p.homing[0].endpoint_dwell_ticks=20; p.homing[1].endpoint_dwell_ticks=20;
    struct fixture f; prepare_measured(&f,0,false,&p); finish_four(&f,20);
    assert(f.c.final_on_samples[0][0]==70 && f.c.samples[0][0]==50);
    assert(f.c.final_on_samples[1][1]==62 && f.c.samples[1][1]==42); ++cases;

    prepare_measured(&f,0,false,&policy); drive(&f);
    assert(sample(&f,1,50,1)); assert(sample(&f,3,50,3));
    for(uint32_t at=10;at<40;at+=10) assert(sample(&f,at,50,at));
    assert(sample(&f,40,0,40)); assert(f.c.first_low_on==40);
    assert(sample(&f,41,20,41)); assert(!f.c.have_first_low && !f.c.first_low_on);
    assert(sample(&f,45,50,45)); assert(sample(&f,47,50,47));
    assert(sample(&f,80,0,80)); assert(sample(&f,82,0,82));
    assert(confirm(&f,true));
    assert(f.c.samples[0][0]==80); ++cases;

    /* Acquisition and receipt counters differ, but remain time-consistent. */
    prepare_measured(&f,0,false,&policy); drive(&f);
    assert(sample(&f,1,50,1)); assert(sample(&f,3,50,3));
    for(uint32_t at=10;at<40;at+=10) assert(sample(&f,at,50,at));
    uint32_t start=f.c.phase_started_at, token=f.c.action.token;
    assert(motor_calibration_foreground_progress(&f.c,token,start+43,43));
    assert(motor_calibration_feedback(&f.c,token,start+43,start+40,0,true,40,43));
    assert(motor_calibration_foreground_progress(&f.c,token,start+45,45));
    assert(motor_calibration_feedback(&f.c,token,start+45,start+42,0,true,42,45));
    assert(f.c.first_low_on==40 && f.c.on_ticks==45 && confirm(&f,true));
    assert(f.c.samples[0][0]==40 && f.c.final_on_samples[0][0]==45); ++cases;

    /* A longer initial partial home is still not one of the four learned legs. */
    init(&f,0,&policy); motion_probe(&f);
    seek_endpoint(&f,80,2); assert(confirm(&f,true)); finish_four(&f,2); ++cases;
}
static void off_gaps_are_not_travel(void)
{
    struct fixture f; prepare_measured(&f,0,false,&policy);
    uint32_t start=f.c.phase_started_at, on_total=0;
    for(unsigned leg=0;leg<4;++leg) {
        uint32_t travel=(f.c.direction==MOTOR_CAL_OPEN ? 50u : 40u)+(leg>=2 ? 2u : 0u);
        seek_endpoint_scaled(&f,travel,2,2);
        assert(f.c.stop_requested_at-f.c.phase_started_at==2*f.c.on_ticks);
        on_total+=f.c.on_ticks;
        assert(confirm(&f,true));
    }
    assert(f.c.state==MOTOR_CAL_CANDIDATE && f.c.candidate.opening_ticks==51 &&
           f.c.candidate.closing_ticks==41);
    assert(f.c.final_on_samples[0][0]==52 && f.c.final_on_samples[1][1]==44);
    assert(f.c.last_now-start==2*on_total+8*policy.min_stop_ticks);
    assert(!f.motor_on); ++cases;
}
static void asymmetric_endpoint(struct fixture *f,uint32_t travel)
{
    const struct motor_homing_policy *h=&f->c.homing.policy;
    uint32_t high=h->motion_min+5, low=h->endpoint_max;
    if(!f->motor_on) {
        drive(f);
        for(uint32_t i=0;i<h->motion_samples;++i) assert(sample(f,1+2*i,high,1+2*i));
    }
    assert(f->c.homing.state==MOTOR_HOMING_SEEK_ENDPOINT);
    for(uint32_t at=f->c.on_ticks+7;at<travel;at+=10) assert(sample(f,at,high,at));
    for(uint32_t i=0;i<h->endpoint_samples;++i) assert(sample(f,travel+3*i,low,travel+3*i));
    assert(f->c.state==MOTOR_CAL_WAIT_STOP && f->c.first_low_on==travel && !f->motor_on);
}
static void direction_specific_policies(void)
{
    struct motor_calibration_policy p=policy;
    p.homing[0]=(struct motor_homing_policy){250,35,35,10,70,15,120,4,6,3,3};
    p.homing[1]=(struct motor_homing_policy){300,40,40,10,35,7,90,2,3,2,2};
    struct fixture f; init(&f,0,&p);
    assert(f.c.homing.policy.motion_min==35); motion_probe(&f);
    assert(f.c.direction==MOTOR_CAL_CLOSE && f.c.homing.policy.motion_min==35 &&
           f.c.homing.policy.endpoint_samples==2 && f.c.action.on_budget_ticks==300);
    asymmetric_endpoint(&f,20); assert(confirm(&f,true));
    for(unsigned leg=0;leg<4;++leg) {
        bool opening=f.c.direction==MOTOR_CAL_OPEN;
        assert(f.c.homing.policy.motion_min==(opening ? 70u : 35u));
        assert(f.c.homing.policy.endpoint_max==(opening ? 15u : 7u));
        assert(f.c.action.on_budget_ticks==(opening ? 250u : 300u));
        asymmetric_endpoint(&f,opening ? 50u : 40u); assert(confirm(&f,true));
    }
    assert(f.c.state==MOTOR_CAL_CANDIDATE && f.c.candidate.opening_ticks==50 &&
           f.c.candidate.closing_ticks==40); ++cases;
}
static void probe_failures_are_bounded(void)
{
    struct fixture f; init(&f,0,&policy); low_probe(&f);
    assert(confirm(&f,false)); low_probe(&f);
    assert(!confirm(&f,false));
    assert(f.c.error==MOTOR_CAL_NO_MOTION && f.c.state==MOTOR_CAL_FAILED && f.starts==2);
    assert(!f.c.candidate.opening_ticks && !f.c.candidate.closing_ticks);
    assert(!motor_calibration_start(&f.c,40,&policy,true)); ++cases;
    init(&f,0,&policy); drive(&f);
    assert(sample(&f,1,20,1));
    assert(sample(&f,12,20,12));
    assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on && !f.c.error);
    assert(!f.c.have_first_low && !f.c.measured_legs);
    uint32_t deadline=f.c.attempt_started_at+f.c.policy.whole_attempt_ticks;
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,13,12,false,false));
    assert(f.c.state==MOTOR_CAL_WAIT_STOP && f.starts==1);
    assert(confirm(&f,false) && f.c.phase==MOTOR_CAL_PROBE_OPEN);
    assert(f.c.attempt_started_at+f.c.policy.whole_attempt_ticks==deadline);
    drive(&f); assert(sample(&f,1,20,1));
    assert(!sample(&f,12,20,12));
    assert(f.c.error==MOTOR_CAL_PROBE_SIGNAL && f.c.phase==MOTOR_CAL_PROBE_OPEN && f.starts==2);
    assert(!f.motor_on && !f.c.candidate.opening_ticks && !f.c.candidate.closing_ticks); ++cases;
    init(&f,0,&policy); drive(&f);
    assert(sample(&f,1,50,1)); assert(sample(&f,3,20,3)); assert(sample(&f,12,20,12));
    assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on && !f.c.have_first_low);
    assert(confirm(&f,false)); motion_probe(&f);
    assert(f.c.phase==MOTOR_CAL_HOME && f.c.direction==MOTOR_CAL_OPEN && !f.c.measured_legs);
    seek_endpoint(&f,20,2); assert(confirm(&f,true)); finish_four(&f,2);
    assert(f.c.candidate.final_endpoint_permille==1000 && f.starts==6); ++cases;
    init(&f,0,&policy); drive(&f); assert(sample(&f,1,0,1));
    assert(!sample(&f,13,0,13));
    assert(f.c.error==MOTOR_CAL_ENERGY_LIMIT && f.c.phase==MOTOR_CAL_PROBE_CLOSE); ++cases;
}
static void stop_confirmation_contract(void)
{
    struct fixture f; init(&f,0,&policy); low_probe(&f);
    uint32_t stopped=f.c.stop_requested_at,on=f.c.on_ticks;
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped,on,true,false));
    assert(f.c.have_stop_proof && f.c.stop_observed_at==stopped && f.c.state==MOTOR_CAL_WAIT_STOP);
    struct motor_calibration before=f.c;
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped+2,on,true,false));
    assert(!memcmp(&before,&f.c,sizeof before));
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped+3,on,false,false));
    assert(!memcmp(&before,&f.c,sizeof before));
    assert(confirm(&f,false) && f.c.phase==MOTOR_CAL_PROBE_OPEN && !f.motor_on); ++cases;
    motion_probe(&f);
    seek_endpoint(&f,20,2);
    assert(!confirm(&f,false) && f.c.error==MOTOR_CAL_REFERENCE_UNQUALIFIED && !f.motor_on); ++cases;
    init(&f,0,&policy); low_probe(&f);
    uint32_t stop_at=f.c.stop_requested_at;
    motor_calibration_tick(&f.c,stop_at+policy.stop_confirm_ticks);
    assert(f.c.error==MOTOR_CAL_STOP_TIMEOUT && f.c.state==MOTOR_CAL_FAILED);
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stop_at+31,f.c.on_ticks,true,true)); ++cases;
    init(&f,0,&policy); low_probe(&f);
    stop_at=f.c.stop_requested_at;
    uint32_t late=stop_at+policy.stop_confirm_ticks-1;
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,late,f.c.on_ticks,true,false));
    assert(f.c.have_stop_proof && f.c.stop_requested_at==stop_at);
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,late+policy.min_stop_ticks,f.c.on_ticks,true,false));
    assert(f.c.error==MOTOR_CAL_STOP_TIMEOUT); ++cases;
    struct motor_calibration_policy p=policy; p.probe_slack_ticks=3;
    init(&f,0,&p); low_probe(&f);
    stopped=f.c.stop_requested_at;on=f.c.on_ticks;
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped+7,on+2,true,false));
    assert(f.c.stopped_on_ticks==on+2 && !f.motor_on && f.c.state==MOTOR_CAL_WAIT_STOP);
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped+9,on+3,true,false));
    assert(f.c.error==MOTOR_CAL_COUNTER); ++cases;
    init(&f,0,&p); low_probe(&f);
    stopped=f.c.stop_requested_at;on=f.c.on_ticks;
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped+7,on+2,true,false));
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped+9,on+2,true,false));
    assert(motor_calibration_confirm_stop(&f.c,f.c.action.token,stopped+10,on+2,true,false));
    assert(f.c.phase==MOTOR_CAL_PROBE_OPEN && !f.motor_on); ++cases;
    init(&f,0,&policy); low_probe(&f);
    f.c.homing.error=MOTOR_HOMING_INVALID_FEEDBACK;
    assert(!confirm(&f,false) && f.c.error==MOTOR_CAL_HOMING); ++cases;
}
static void invalid_feedback_and_counts(void)
{
    for(unsigned kind=0;kind<6;++kind) {
        struct fixture f; init(&f,0,&policy); drive(&f); assert(sample(&f,1,50,1));
        uint32_t now=kind==1 ? 12u : 2u, measured=kind==1 ? 2u : now;
        uint32_t current=now, at_sample=kind==2 ? current+1 : kind==3 ? 0u : measured;
        if(kind==4) { measured=1; at_sample=2; }
        if(kind==5) measured=1; /* Duplicate acquisition timestamp. */
        assert(!motor_calibration_feedback(&f.c,f.c.action.token,now,measured,50,kind!=0,at_sample,current));
        assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && f.starts==1);
        assert(f.c.phase==MOTOR_CAL_PROBE_CLOSE && !f.c.candidate.opening_ticks); ++cases;
    }
    struct fixture f; init(&f,0,&policy); drive(&f); assert(sample(&f,1,0,1));
    assert(!motor_calibration_foreground_progress(&f.c,f.c.action.token,2,0));
    assert(f.c.error==MOTOR_CAL_COUNTER); ++cases;
    init(&f,0,&policy); drive(&f);
    assert(!motor_calibration_foreground_progress(&f.c,f.c.action.token,1,2));
    assert(f.c.error==MOTOR_CAL_COUNTER); ++cases;
    init(&f,0,&policy); drive(&f);
    assert(motor_calibration_foreground_progress(&f.c,f.c.action.token,10,1));
    assert(!motor_calibration_foreground_progress(&f.c,f.c.action.token,11,10));
    assert(f.c.error==MOTOR_CAL_COUNTER); ++cases;
    init(&f,0,&policy); motion_probe(&f);
    uint32_t start=f.c.phase_started_at;
    assert(!motor_calibration_feedback(&f.c,f.c.action.token,start+20,start+15,0,true,10,20));
    assert(f.c.error==MOTOR_CAL_COUNTER); ++cases;
}
static void stalls_deadlines_and_no_resurrection(void)
{
    struct fixture f;
    for(unsigned kind=0;kind<2;++kind) {
        init(&f,0,&policy); drive(&f);
        for(uint32_t t=1;t<40;t+=10) {
            if(kind==0) assert(motor_calibration_feedback(&f.c,f.c.action.token,t,t,0,true,0,0));
            else assert(motor_calibration_foreground_progress(&f.c,f.c.action.token,t,0));
        }
        motor_calibration_tick(&f.c,40);
        assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && f.c.error==MOTOR_CAL_HOMING);
        assert(f.c.phase==MOTOR_CAL_PROBE_CLOSE); ++cases;
    }
    struct motor_calibration_policy p=policy; p.whole_attempt_ticks=policy.probe_on_ticks+60;
    prepare_measured(&f,0,false,&p);
    seek_endpoint(&f,20,2); assert(confirm(&f,true));
    uint32_t phase_token=f.c.action.token;
    drive(&f); assert(sample(&f,1,50,1)); assert(sample(&f,3,50,3));
    motor_calibration_tick(&f.c,p.whole_attempt_ticks);
    assert(f.c.error==MOTOR_CAL_ATTEMPT_TIMEOUT && !f.motor_on);
    assert(!motor_calibration_foreground_progress(&f.c,phase_token,p.whole_attempt_ticks,4));
    assert(!f.c.candidate.opening_ticks); ++cases;
    init(&f,0,&policy); motion_probe(&f);
    for(uint32_t at=f.c.on_ticks+1;at<300;at+=10) assert(sample(&f,at,50,at));
    motor_calibration_tick(&f.c,f.c.phase_started_at+300);
    assert(f.c.state==MOTOR_CAL_FAILED && f.c.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT && !f.motor_on); ++cases;
    init(&f,100,&policy); drive(&f); assert(sample(&f,1,0,1));
    motor_calibration_tick(&f.c,100);
    assert(f.c.error==MOTOR_CAL_CLOCK && !f.motor_on); ++cases;
}
static void stale_tokens_reset_and_ranges(void)
{
    struct fixture f; init(&f,0,&policy); low_probe(&f);
    uint32_t old=f.c.action.token; assert(confirm(&f,false));
    struct motor_calibration before=f.c;
    assert(!motor_calibration_feedback(&f.c,old,99999,99999,0,true,0,0));
    assert(!motor_calibration_foreground_progress(&f.c,old,99999,0));
    assert(!motor_calibration_confirm_stop(&f.c,old,99999,0,true,true));
    assert(!motor_calibration_start(&f.c,99999,&policy,true));
    assert(!motor_calibration_reset(&f.c,true));
    assert(!memcmp(&before,&f.c,sizeof before)); ++cases;
    motor_calibration_abort(&f.c); assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on);
    uint32_t token=f.c.last_token;
    assert(!motor_calibration_reset(&f.c,false));
    assert(motor_calibration_reset(&f.c,true) && f.c.last_token==token);
    assert(motor_calibration_start(&f.c,100,&policy,true) && f.c.action.token==token+1); ++cases;
    prepare_measured(&f,0,false,&policy); seek_endpoint(&f,5,2);
    assert(!confirm(&f,true) && f.c.error==MOTOR_CAL_TRAVEL_RANGE); ++cases;
    prepare_measured(&f,0,false,&policy); seek_endpoint(&f,121,2);
    assert(!confirm(&f,true) && f.c.error==MOTOR_CAL_TRAVEL_RANGE); ++cases;
    prepare_measured(&f,0,false,&policy);
    seek_endpoint(&f,40,2); assert(confirm(&f,true));
    seek_endpoint(&f,50,2); assert(confirm(&f,true));
    seek_endpoint(&f,45,2); assert(!confirm(&f,true));
    assert(f.c.error==MOTOR_CAL_REPEATABILITY && !f.c.candidate.opening_ticks); ++cases;
    init(&f,0,&policy); low_probe(&f);
    assert(!motor_calibration_confirm_stop(&f.c,f.c.action.token,f.c.stop_requested_at+3,f.c.on_ticks-1,true,false));
    assert(f.c.error==MOTOR_CAL_COUNTER); ++cases;
}
static void bad_policy_and_token_exhaustion(void)
{
    for(unsigned kind=0;kind<8;++kind) {
        struct fixture f={0}; struct motor_calibration_policy p=policy;
        assert(motor_calibration_init(&f.c,stop,&f));
        if(kind==0) p.whole_attempt_ticks=0;
        if(kind==1) p.whole_attempt_ticks=UINT32_C(0x80000000);
        if(kind==2) p.min_stop_ticks=0;
        if(kind==3) p.min_stop_ticks=p.stop_confirm_ticks;
        if(kind==4) p.repeat_tolerance_ticks=0;
        if(kind==5) p.min_travel_ticks=p.max_travel_ticks+1;
        if(kind==6) p.homing[0].motion_samples=0; /* Reject invalid OPEN before first CLOSE action. */
        if(kind==7) p.homing[1].motion_samples=0;
        if(kind>=6) {
            unsigned stops=f.stops;
            assert(!motor_homing_policy_valid(&p.homing[kind-6]));
            assert(f.stops==stops && !f.c.homing.guard.armed);
        }
        assert(!motor_calibration_start(&f.c,0,&p,true));
        assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && !f.c.action.direction); ++cases;
    }
    struct fixture f={0}; assert(motor_calibration_init(&f.c,stop,&f));
    assert(!motor_calibration_start(&f.c,0,&policy,false) && f.c.state==MOTOR_CAL_IDLE); ++cases;
    f.c.last_token=UINT32_MAX;
    assert(!motor_calibration_start(&f.c,0,&policy,true) && f.c.error==MOTOR_CAL_TOKEN_EXHAUSTED);
    assert(motor_calibration_reset(&f.c,true) && f.c.last_token==UINT32_MAX); ++cases;
    f.c.last_token=UINT32_MAX-1u;
    assert(motor_calibration_start(&f.c,100,&policy,true)); low_probe(&f);
    assert(!confirm(&f,false) && f.c.error==MOTOR_CAL_TOKEN_EXHAUSTED && !f.motor_on); ++cases;
    assert(!motor_calibration_init(NULL,stop,&f));
    assert(!motor_calibration_init(&f.c,NULL,&f) && f.c.state==MOTOR_CAL_FAILED); ++cases;
}
static void qualified_probe_uses_its_budget(void)
{
    struct fixture f; init(&f,0,&policy); drive(&f);
    assert(sample(&f,1,50,1)); assert(sample(&f,3,50,3));
    assert(f.c.homing.state==MOTOR_HOMING_SEEK_ENDPOINT && f.c.state==MOTOR_CAL_ACTIVE && f.motor_on);
    for(uint32_t at=4;at<=policy.probe_on_ticks;++at) assert(sample(&f,at,50,at));
    assert(f.c.state==MOTOR_CAL_ACTIVE && f.c.on_ticks==policy.probe_on_ticks && f.motor_on);
    assert(f.c.phase==MOTOR_CAL_HOME && f.c.action.token==1 && f.starts==1); ++cases;
    init(&f,0,&policy); drive(&f);
    assert(sample(&f,1,50,1)); assert(sample(&f,3,50,3));
    assert(sample(&f,4,0,4)); assert(sample(&f,6,0,6));
    assert(f.c.state==MOTOR_CAL_WAIT_STOP && f.c.on_ticks==6 && !f.motor_on);
    assert(!f.c.intentional_abort && f.c.homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED);
    assert(f.c.phase==MOTOR_CAL_HOME && !f.c.measured_legs);
    assert(confirm(&f,true) && f.c.phase==MOTOR_CAL_MEASURE && !f.c.measured_legs); ++cases;
}
static void continuous_home_preserves_guards(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f; init(&f,wrap ? UINT32_MAX-5u : 0,&policy); motion_probe(&f);
        uint32_t start=f.c.phase_started_at,token=f.c.action.token;
        for(uint32_t at=13;at<300;at+=10) assert(sample(&f,at,20,at));
        assert(f.c.state==MOTOR_CAL_ACTIVE && f.c.homing.state==MOTOR_HOMING_SEEK_ENDPOINT);
        assert(!f.c.measured_legs && !f.c.have_first_low && f.c.action.token==token && f.starts==1);
        motor_calibration_tick(&f.c,start+300);
        assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on &&
               f.c.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT);
        assert(f.c.action.token==token && !f.c.action.direction && !f.c.candidate.opening_ticks);
        ++cases;
    }
    struct fixture f; init(&f,0,&policy); motion_probe(&f);
    motor_calibration_tick(&f.c,f.c.phase_started_at+52);
    assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && f.starts==1); ++cases;
    init(&f,0,&policy); motion_probe(&f); motor_calibration_abort(&f.c);
    assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && f.starts==1); ++cases;
}
int main(void)
{
    continuous_home_preserves_guards();
    qualified_probe_uses_its_budget();
    struct motor_calibration_policy p=policy;
    p.probe_slack_ticks=3;
    struct fixture f; init(&f,0,&p); drive(&f);
    assert(f.c.action.on_budget_ticks==15);
    assert(sample(&f,10,0,10)); assert(sample(&f,14,0,14));
    assert(f.c.state==MOTOR_CAL_WAIT_STOP && f.c.on_ticks==14 && !f.motor_on);
    assert(confirm(&f,false)); ++cases;
    init(&f,0,&p); drive(&f); assert(sample(&f,1,0,1));
    assert(motor_calibration_foreground_progress(&f.c,f.c.action.token,14,14));
    assert(f.c.state==MOTOR_CAL_ACTIVE && f.motor_on); /* Wait for next fresh ADC, within budget. */
    assert(motor_calibration_feedback(&f.c,f.c.action.token,14,14,0,true,14,14));
    assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on); ++cases;
    init(&f,0,&p); drive(&f); assert(sample(&f,10,0,10));
    assert(!sample(&f,16,0,16) && f.c.error==MOTOR_CAL_ENERGY_LIMIT); ++cases;
    p.probe_slack_ticks=UINT32_MAX;
    f=(struct fixture){0}; assert(motor_calibration_init(&f.c,stop,&f));
    assert(!motor_calibration_start(&f.c,0,&p,true) && f.c.error==MOTOR_CAL_BAD_POLICY); ++cases;
    flows_and_wrap(); first_low_not_confirmation_energy(); off_gaps_are_not_travel();
    direction_specific_policies(); probe_failures_are_bounded();
    stop_confirmation_contract(); invalid_feedback_and_counts(); stalls_deadlines_and_no_resurrection();
    stale_tokens_reset_and_ranges(); bad_policy_and_token_exhaustion();
    printf("motor calibration: %u cases PASS; candidates only, no hardware or position reference\n",cases);
    return 0;
}
