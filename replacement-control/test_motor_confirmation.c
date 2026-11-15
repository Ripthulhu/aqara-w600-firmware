#include "motor_calibration.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

/* Synthetic timing/counters, production voltage bands. No physical reference proof. */
static const struct motor_calibration_policy policy={
    .homing={{450000,1000,1000,100,1100,100,4000,2000,200,11,2},
             {450000,1000,1000,100,1100,100,4000,2000,200,11,2}},
    .whole_attempt_ticks=3600000,.probe_on_ticks=15000,.probe_slack_ticks=300,
    .min_travel_ticks=20000,.max_travel_ticks=449000,.repeat_tolerance_ticks=50000,
    .stop_confirm_ticks=10000,.min_stop_ticks=1100
};
static const struct motor_calibration_confirmation confirmation={210,18000,20000,48000,50000,5000};
struct fixture {struct motor_calibration c;bool motor_on;unsigned starts,stops;};
static unsigned cases;
static void stop(void *arg) {struct fixture *f=arg;f->motor_on=false;++f->stops;}
static void init(struct fixture *f,uint32_t at)
{
    *f=(struct fixture){0};assert(motor_calibration_init(&f->c,stop,f));
    f->c.confirmation=confirmation;
    assert(motor_calibration_start(&f->c,at,&policy,true));
}
static uint32_t on_at(uint32_t at) {return at-(at/200u)*15u;}
static void drive(struct fixture *f)
{
    assert(!f->motor_on && f->c.state==MOTOR_CAL_ACTIVE && f->c.homing.guard.armed);
    ++f->starts;f->motor_on=true;
}
static bool feed(struct fixture *f,uint32_t at,uint32_t value)
{
    uint32_t now=f->c.phase_started_at+at,token=f->c.action.token,on=on_at(at);
    if (!motor_calibration_foreground_progress(&f->c,token,now,on)) return false;
    if (f->c.state!=MOTOR_CAL_ACTIVE) return f->c.state==MOTOR_CAL_WAIT_STOP;
    return motor_calibration_feedback(&f->c,token,now,now,value,true,on,on);
}
static void high_until(struct fixture *f,uint32_t until)
{
    if (!f->motor_on) drive(f);
    for (uint32_t at=200;at<=until;at+=200) assert(feed(f,at,1300));
}
static void candidate(struct fixture *f,uint32_t at)
{
    high_until(f,at-600);
    assert(feed(f,at-400,900));assert(feed(f,at-200,400));assert(feed(f,at,1));
    assert(f->c.state==MOTOR_CAL_WAIT_STOP && !f->motor_on && f->c.have_first_low);
    assert(f->c.first_low_on==on_at(at) && f->c.intermediate_count==2);
}
static bool confirm(struct fixture *f,bool qualified)
{
    uint32_t token=f->c.action.token,on=f->c.on_ticks;
    uint32_t first=f->c.stop_requested_at+10;
    assert(!motor_calibration_confirm_stop(&f->c,token,first,on,true,qualified));
    if (f->c.state==MOTOR_CAL_FAILED) return false;
    assert(f->c.have_stop_proof && !f->motor_on);
    return motor_calibration_confirm_stop(&f->c,token,
        first+motor_calibration_stop_dwell(&f->c),on,true,qualified);
}
static void backoff(struct fixture *f)
{
    assert(f->c.phase==MOTOR_CAL_CONFIRM_BACKOFF && f->c.homing.policy.max_move_ticks==20000);
    high_until(f,18000);
    assert(f->c.state==MOTOR_CAL_WAIT_STOP && !f->motor_on);
    assert(confirm(f,false) && f->c.phase==MOTOR_CAL_CONFIRM_RETURN);
    assert(f->c.homing.policy.max_move_ticks==50000);
}
static void confirmed(struct fixture *f,uint32_t travel)
{
    uint32_t token=f->c.action.token,attempt=f->c.attempt_started_at;
    uint8_t direction=f->c.direction;
    candidate(f,travel);assert(confirm(f,true));
    assert(f->c.original_token==token && f->c.original_first_low==on_at(travel));
    assert(f->c.direction!=direction);
    backoff(f);assert(f->c.direction==direction);
    candidate(f,19000);assert(confirm(f,true));
    assert(f->c.attempt_started_at==attempt && f->c.confirmed_return_on==on_at(19000));
    assert(f->c.confirmed_return_mv==1 && f->c.confirmed_intermediate_count==2);
}
static void complete_flows(void)
{
    /* Spring resistance while closing is not proof that the valve is shut. */
    {
        struct fixture f;init(&f,0);high_until(&f,4000);
        for(uint32_t at=4200;at<=6200;at+=200) assert(feed(&f,at,1000));
        assert(f.c.state==MOTOR_CAL_ACTIVE && f.motor_on && !f.c.confirmed_candidates);
        assert(f.c.closing_load_seen);
        assert(feed(&f,6400,400));assert(feed(&f,6600,1));
        assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on);
        assert(f.c.first_low_on==on_at(6600));assert(confirm(&f,true));
        backoff(&f);high_until(&f,10000);
        for(uint32_t at=10200;at<=12200;at+=200) assert(feed(&f,at,1000));
        assert(f.c.state==MOTOR_CAL_ACTIVE && f.motor_on);
        assert(feed(&f,12400,400));assert(feed(&f,12600,1));
        assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on);
        assert(confirm(&f,true) && f.c.confirmed_candidates==1 && !f.c.measured_legs);
        assert(f.c.phase==MOTOR_CAL_MEASURE && f.c.confirmed_return_mv==1);++cases;
    }
    for (unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f,wrap ? UINT32_MAX-10000u : 0);
        confirmed(&f,40000);assert(!f.c.measured_legs && f.c.phase==MOTOR_CAL_MEASURE);
        for (unsigned leg=0;leg<4;++leg) {
            uint32_t token=f.c.action.token;unsigned direction=f.c.direction;
            uint32_t travel=direction==1 ? 50000 : 60000;
            confirmed(&f,travel);
            assert(f.c.sample_tokens[direction-1][leg/2]==token);
            assert(f.c.samples[direction-1][leg/2]==on_at(travel));
            assert(f.c.final_on_samples[direction-1][leg/2]==on_at(travel));
        }
        assert(f.c.state==MOTOR_CAL_CANDIDATE && f.c.measured_legs==4 && f.starts==15);
        assert(f.c.confirmed_candidates==5 && f.c.last_token==15 && !f.motor_on);
        assert(f.c.candidate.opening_ticks==on_at(50000) && f.c.candidate.closing_ticks==on_at(60000));
        assert(f.c.candidate.final_endpoint_permille==0);++cases;
    }
    struct fixture f;init(&f,0);candidate(&f,5000);
    assert(f.c.phase==MOTOR_CAL_HOME && !f.c.measured_legs);assert(confirm(&f,true));
    backoff(&f);candidate(&f,5000);assert(confirm(&f,true));
    assert(!f.c.measured_legs && f.c.phase==MOTOR_CAL_MEASURE);++cases;
}
static void uncertain_lows_stop(void)
{
    for (unsigned kind=0;kind<4;++kind) {
        struct fixture f;init(&f,0);high_until(&f,3000);
        if (kind==1) assert(feed(&f,3200,400));
        if (kind>=2) {assert(feed(&f,3200,900));assert(feed(&f,kind==2 ? 3300 : 3600,400));}
        uint32_t at=kind==0 ? 3200 : kind==1 ? 3400 : kind==2 ? 3400 : 3800;
        assert(!feed(&f,at,1));
        assert(f.c.state==MOTOR_CAL_FAILED && f.c.error==MOTOR_CAL_REFERENCE_UNQUALIFIED && !f.motor_on);
        unsigned starts=f.starts;assert(!motor_calibration_feedback(&f.c,f.c.action.token,
            at+200,at+200,0,true,on_at(at),on_at(at)));
        assert(f.starts==starts && !f.c.measured_legs && !f.c.confirmed_candidates);++cases;
    }
    /* Ripple inside the intermediate band is not an automatic rejection. */
    struct fixture f;init(&f,0);high_until(&f,3000);
    assert(feed(&f,3200,700));assert(feed(&f,3400,900));assert(feed(&f,3600,400));
    assert(feed(&f,3800,1) && f.c.state==MOTOR_CAL_WAIT_STOP);++cases;
    for (uint32_t gap=201;gap<=210;gap+=9) {
        init(&f,0);high_until(&f,3000);
        assert(feed(&f,3000+gap,900));assert(feed(&f,3000+2*gap,400));
        assert(feed(&f,3000+3*gap,1) && f.c.state==MOTOR_CAL_WAIT_STOP);++cases;
    }
}
static void confirmation_failures(void)
{
    for (unsigned kind=0;kind<6;++kind) {
        struct fixture f;init(&f,0);candidate(&f,40000);assert(confirm(&f,true));
        if (kind==0) {
            high_until(&f,3000);assert(feed(&f,3200,900));assert(feed(&f,3400,400));
            assert(!feed(&f,3600,1));
        }
        if (kind==1) {
            drive(&f);
            for (unsigned at=200;at<18000;at+=200) assert(feed(&f,at,900));
            assert(!feed(&f,18000,900) && f.c.error==MOTOR_CAL_NO_MOTION);
        } else if (kind>=2) {
            backoff(&f);
            if (kind==2) {high_until(&f,3000);assert(!feed(&f,3200,1));}
            if (kind==3) {high_until(&f,47800);assert(!feed(&f,48000,1300));}
            if (kind==4) {
                high_until(&f,3000);
                assert(!motor_calibration_feedback(&f.c,f.c.action.token,f.c.phase_started_at+3200,
                    f.c.phase_started_at+3200,900,false,on_at(3200),on_at(3200)));
            }
            if (kind==5) {
                high_until(&f,47000);assert(feed(&f,47200,900));assert(feed(&f,47400,700));
                assert(feed(&f,47600,500));assert(feed(&f,47800,300));assert(!feed(&f,48000,1));
            }
        }
        assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && !f.c.measured_legs && !f.c.confirmed_candidates);
        unsigned starts=f.starts;motor_calibration_tick(&f.c,f.c.last_now+100000);
        assert(f.starts==starts && !motor_calibration_start(&f.c,0,&policy,true));++cases;
    }
}
static void stop_faults_and_tokens(void)
{
    for (unsigned kind=0;kind<5;++kind) {
        struct fixture f;init(&f,0);candidate(&f,40000);
        uint32_t token=f.c.action.token,at=f.c.stop_requested_at,on=f.c.on_ticks;
        if (kind==0) {
            assert(!motor_calibration_confirm_stop(&f.c,token,at+10,on,true,false));
        } else {
            assert(!motor_calibration_confirm_stop(&f.c,token,at+10,on,true,true));
            if (kind==1) assert(!motor_calibration_confirm_stop(&f.c,token,at+5010,on+1,true,true));
            if (kind==2) motor_calibration_tick(&f.c,at+policy.stop_confirm_ticks);
            if (kind==3) motor_calibration_tick(&f.c,at);
            if (kind==4) {f.c.policy.whole_attempt_ticks=at+100;motor_calibration_tick(&f.c,at+100);}
        }
        assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && f.starts==1);++cases;
    }
    struct fixture f;init(&f,0);candidate(&f,40000);uint32_t old=f.c.action.token;
    assert(confirm(&f,true));struct motor_calibration saved=f.c;
    assert(!motor_calibration_feedback(&f.c,old,999999,999999,0,true,0,0));
    assert(!motor_calibration_confirm_stop(&f.c,old,999999,0,true,true));
    assert(!memcmp(&saved,&f.c,sizeof saved));++cases;
    drive(&f);motor_calibration_tick(&f.c,f.c.phase_started_at+1000);
    assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && f.c.homing.guard.error);++cases;
    uint32_t token=f.c.last_token;
    assert(motor_calibration_reset(&f.c,true));
    assert(f.c.state==MOTOR_CAL_IDLE && f.c.last_token==token && !f.c.confirmed_candidates);
    assert(!memcmp(&f.c.confirmation,&confirmation,sizeof confirmation));
    assert(motor_calibration_start(&f.c,100000,&policy,true) && f.c.action.token==token+1);
    assert(!f.motor_on);++cases;
}
static void replay(const char *path,uint32_t expected_at,uint32_t expected_mv)
{
    for (unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f,wrap ? UINT32_MAX-10000u : 0);drive(&f);
        FILE *file=fopen(path,"rb");assert(file);
        uint32_t pair[2],last=0;unsigned count=0;
        while (fread(pair,sizeof pair,1,file)==1) {
            assert(pair[0]>last && pair[0]<450000 && pair[1]<=4000);last=pair[0];++count;
            assert(feed(&f,pair[0],pair[1]));
            if (f.c.state!=MOTOR_CAL_ACTIVE) break;
        }
        assert(!ferror(file));fclose(file);
        if (expected_at) {
            assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on && last==expected_at);
            assert(f.c.last_signal==expected_mv && f.c.phase==MOTOR_CAL_HOME);
            assert(!f.c.measured_legs && !f.c.confirmed_candidates && !f.c.candidate.opening_ticks);
            assert(confirm(&f,true) && f.c.phase==MOTOR_CAL_CONFIRM_BACKOFF);
            assert(f.starts==1 && !f.motor_on); /* Replay never runs synthetic confirmation legs. */
        } else {
            assert(count==2249 && f.c.state==MOTOR_CAL_ACTIVE);
            motor_calibration_tick(&f.c,f.c.phase_started_at+450000);
            assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && !f.c.confirmed_candidates);
            assert(f.c.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT);
        }
        printf("PASS measured signal replay: %u samples, wrap%u, candidate_at%u mv%u; no reference\n",
               count,wrap,expected_at,expected_mv);
    }
}
static void mounting_retraction(void)
{
    for (unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f={0};assert(motor_calibration_init(&f.c,stop,&f));
        f.c.confirmation=confirmation;
        uint32_t at=wrap ? UINT32_MAX-10000u : 0;
        assert(motor_calibration_retract_start(&f.c,at,&policy,true));
        assert(f.c.direction==MOTOR_CAL_OPEN && f.c.phase==MOTOR_CAL_PROBE_OPEN);
        assert(f.c.policy.whole_attempt_ticks==600000u && !f.motor_on);
        assert(!motor_calibration_retract_start(&f.c,at+1,&policy,true));
        confirmed(&f,40000);
        assert(f.c.state==MOTOR_CAL_RETRACTED && !f.motor_on && f.starts==3);
        assert(f.c.confirmed_candidates==1 && !f.c.measured_legs);
        assert(!f.c.candidate.opening_ticks && !f.c.candidate.closing_ticks);
        assert(f.c.action.direction==MOTOR_CAL_NONE && !f.c.action.on_budget_ticks);
        motor_calibration_tick(&f.c,f.c.last_now+1000000u);
        assert(f.c.state==MOTOR_CAL_RETRACTED && !f.motor_on && f.starts==3);
        assert(!motor_calibration_retract_start(&f.c,f.c.last_now,&policy,true));
        assert(motor_calibration_reset(&f.c,true) && !f.c.retract_only);
        ++cases;
    }
    struct fixture f={0};assert(motor_calibration_init(&f.c,stop,&f));
    f.c.confirmation=confirmation;
    assert(!motor_calibration_retract_start(&f.c,0,&policy,false));
    assert(motor_calibration_retract_start(&f.c,0,&policy,true));
    drive(&f);motor_calibration_tick(&f.c,600000u);
    assert(f.c.state==MOTOR_CAL_FAILED && f.c.error==MOTOR_CAL_ATTEMPT_TIMEOUT && !f.motor_on);
    ++cases;
}
static void closing_load_is_not_closed(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f,wrap ? UINT32_MAX-10000u : 0);high_until(&f,4000);
        for(uint32_t at=4200;at<=6200;at+=200) assert(feed(&f,at,1000));
        assert(f.c.state==MOTOR_CAL_ACTIVE && f.motor_on && f.c.closing_load_seen);
        assert(f.c.closing_load_at==f.c.phase_started_at+6200);
        for(uint32_t at=6400;at<450000;at+=200) assert(feed(&f,at,1000));
        assert(!feed(&f,450000,1000));
        assert(f.c.state==MOTOR_CAL_FAILED && f.c.error==MOTOR_CAL_HOMING);
        assert(!f.motor_on && !f.c.confirmed_candidates && !f.c.measured_legs);
        assert(motor_calibration_reset(&f.c,true) && !f.c.closing_load_seen);
        init(&f,wrap ? UINT32_MAX-10000u : 0);high_until(&f,4000);
        for(uint32_t at=4200;at<=6200;at+=200) assert(feed(&f,at,1000));
        uint32_t now=f.c.last_now;
        for(unsigned i=1;i<=450;++i) motor_calibration_tick(&f.c,now+i*1000u);
        assert(!f.motor_on && f.c.state==MOTOR_CAL_FAILED && !f.c.confirmed_candidates);
        cases+=2;
    }
}
static void closing_valley_is_not_closed(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f,wrap ? UINT32_MAX-10000u : 0);high_until(&f,4000);
        uint32_t at=4200;
        for(;at<=10000;at+=200) assert(feed(&f,at,800));
        for(;f.c.state==MOTOR_CAL_ACTIVE && at<18000;at+=200) assert(feed(&f,at,1000));
        assert(f.c.state==MOTOR_CAL_ACTIVE && f.motor_on && f.c.seat.candidate);
        for(;at<450000;at+=200) assert(feed(&f,at,1000));
        assert(!feed(&f,450000,1000));
        assert(f.c.state==MOTOR_CAL_FAILED && !f.motor_on && !f.c.confirmed_candidates);
        ++cases;
    }
}
static void early_valley_then_confirmed_load(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f,wrap ? UINT32_MAX-10000u : 0);
        f.c.confirmation.return_ticks=100000;f.c.confirmation.return_limit_ticks=102000;
        drive(&f);uint32_t at=200;
        for(;at<=100000;at+=200) assert(feed(&f,at,1365));
        for(;at<=208000;at+=200) assert(feed(&f,at,678));
        for(;at<=214000;at+=200) assert(feed(&f,at,849));
        assert(f.c.seat.candidate && f.c.state==MOTOR_CAL_ACTIVE && f.motor_on);
        assert((uint32_t)(f.c.last_now-f.c.closing_load_at)<200000u);
        for(;f.c.state==MOTOR_CAL_ACTIVE && at<400000;at+=200) assert(feed(&f,at,849));
        assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on);
        assert((uint32_t)(f.c.stop_requested_at-f.c.closing_load_at)>=200000u);
        assert(confirm(&f,true) && f.c.original_load && !f.c.original_seat && !f.c.confirmed_candidates);
        high_until(&f,18000);assert(f.c.state==MOTOR_CAL_WAIT_STOP);
        assert(confirm(&f,false) && f.c.phase==MOTOR_CAL_CONFIRM_RETURN);
        high_until(&f,20000);
        for(at=20200;f.c.state==MOTOR_CAL_ACTIVE && at<100000;at+=200) assert(feed(&f,at,849));
        assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on);
        assert(confirm(&f,true) && f.c.confirmed_candidates==1 && f.c.phase==MOTOR_CAL_MEASURE);
        ++cases;
    }
}
static void sustained_load_confirmation(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) for(unsigned wrong=0;wrong<3;++wrong) {
        struct fixture f={0};uint32_t epoch=wrap ? UINT32_MAX-10000u : 0;
        assert(motor_calibration_init(&f.c,stop,&f));
        f.c.confirmation=confirmation;f.c.confirmation.return_ticks=100000;
        f.c.confirmation.return_limit_ticks=102000;
        assert(motor_calibration_start(&f.c,epoch,&policy,true));
        high_until(&f,20000);uint32_t at=20200;
        for(;f.c.state==MOTOR_CAL_ACTIVE && at<300000;at+=200) assert(feed(&f,at,940));
        assert(at>=220000 && f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on);
        assert(!f.c.seat.candidate && motor_seat_loaded(&f.c.seat));
        assert(confirm(&f,true) && f.c.original_load && f.c.original_load_mv==940);
        assert(!f.c.confirmed_candidates && !f.c.measured_legs);
        high_until(&f,18000);assert(f.c.state==MOTOR_CAL_WAIT_STOP);
        assert(confirm(&f,false) && f.c.phase==MOTOR_CAL_CONFIRM_RETURN);
        drive(&f);
        for(at=200;at<=20000;at+=200) assert(feed(&f,at,1200));
        if(wrong==2) {
            assert(feed(&f,20200,900));assert(feed(&f,20400,400));assert(feed(&f,20600,1));
            assert(f.c.state==MOTOR_CAL_WAIT_STOP && !confirm(&f,true));
        } else {
            for(at=20200;f.c.state==MOTOR_CAL_ACTIVE && at<100000;at+=200)
                assert(feed(&f,at,wrong ? 800 : 940));
            if(wrong) assert(!feed(&f,100000,800));
            else {
                assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on);
                assert(confirm(&f,true) && f.c.confirmed_candidates==1 && !f.c.measured_legs);
                assert(f.c.phase==MOTOR_CAL_MEASURE && f.c.confirmed_return_mv==940);
            }
        }
        if(wrong) assert(f.c.state==MOTOR_CAL_FAILED && !f.c.confirmed_candidates && !f.motor_on);
        ++cases;
    }
}
static void same_closing_signal(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f,wrap ? UINT32_MAX-10000u : 0);
        f.c.phase=MOTOR_CAL_MEASURE;f.c.measured_close_kind=2;
        high_until(&f,4000);uint32_t at=4200;
        for(;at<=10000;at+=200) assert(feed(&f,at,800));
        for(;at<=18000;at+=200) assert(feed(&f,at,940));
        assert(f.c.seat.candidate && f.c.state==MOTOR_CAL_ACTIVE && f.motor_on);
        for(;f.c.state==MOTOR_CAL_ACTIVE && at<300000;at+=200) assert(feed(&f,at,940));
        assert(at>=200000 && f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on);
        assert(confirm(&f,true) && f.c.original_load && !f.c.original_seat);
        assert(!f.c.confirmed_candidates && !f.c.measured_legs);
        init(&f,0);f.c.phase=MOTOR_CAL_MEASURE;f.c.measured_close_kind=1;
        high_until(&f,20000);
        for(at=20200;at<450000;at+=200) assert(feed(&f,at,940));
        assert(!feed(&f,450000,940) && f.c.state==MOTOR_CAL_FAILED && !f.motor_on);
        assert(!f.c.confirmed_candidates && !f.c.measured_legs);
        cases+=2;
    }
}
static void threshold_chatter_confirmation(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f,wrap ? UINT32_MAX-10000u : 0);drive(&f);
        uint32_t at=200;
        for(;at<=20000;at+=200) assert(feed(&f,at,1390));
        for(;at<=60000;at+=200) assert(feed(&f,at,945));
        for(;at<=62000;at+=200) assert(feed(&f,at,1042));
        for(;f.c.state==MOTOR_CAL_ACTIVE && at<300000;at+=200)
            assert(feed(&f,at,(at/2000u)%2 ? 1049 : 1041));
        assert(f.c.state==MOTOR_CAL_WAIT_STOP && !f.motor_on && !f.c.seat.candidate);
        assert((uint32_t)(f.c.stop_requested_at-f.c.closing_load_at)>=200000u);
        assert(confirm(&f,true) && f.c.original_load && !f.c.confirmed_candidates);
        assert(f.c.phase==MOTOR_CAL_CONFIRM_BACKOFF && !f.c.measured_legs);
        ++cases;
    }
}
int main(int argc,char **argv)
{
    if (argc==4) {replay(argv[1],(uint32_t)strtoul(argv[2],NULL,10),(uint32_t)strtoul(argv[3],NULL,10));return 0;}
    assert(argc==1);
    complete_flows();uncertain_lows_stop();confirmation_failures();stop_faults_and_tokens();mounting_retraction();
    closing_load_is_not_closed();
    closing_valley_is_not_closed();
    sustained_load_confirmation();
    same_closing_signal();
    threshold_chatter_confirmation();
    early_valley_then_confirmed_load();
    printf("motor confirmation: %u cases PASS; bounded electrical confirmation, not physical acceptance\n",cases);
    return 0;
}
