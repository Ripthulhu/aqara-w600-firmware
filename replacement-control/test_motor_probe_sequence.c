#include "motor_probe_sequence.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

/* Synthetic scheduling limits, NOT motor travel or sensing thresholds. */
static const struct motor_probe_limits limits={1000,100,100,20,10,2};
static const uint32_t factory=UINT32_C(0xfa2805d7);
static unsigned cases;
static struct motor_probe_sequence s;
static struct motor_probe_action action;
static struct w600_feedback_sample raw(uint32_t at,uint32_t sequence)
{
    struct w600_feedback_sample p={.oldest_at_ms=at,.completed_at_ms=at+2,
        .sequence=sequence,.error=W600_FEEDBACK_OK,.valid=true};
    for(unsigned i=0;i<3;++i) p.pair[i]=(struct w600_feedback_pair){1495,0,at+i};
    return p;
}
static struct tim4_motor_pulse done(unsigned direction)
{
    return (struct tim4_motor_pulse){.status=TIM4_PULSE_DONE,.direction=direction,
        .channel=direction==1 ? 2u : 1u,.seen_low=1,.low_count=200,.settle_count=50};
}
static void begin_at(uint32_t now,unsigned direction,const struct motor_probe_limits *p)
{
    memset(&s,0xa5,sizeof s); motor_probe_sequence_init(&s);
    assert(motor_probe_sequence_begin(&s,now,1,direction,p,true));
    assert(s.state==MOTOR_PROBE_BASELINE && !s.cleanup_pending);
    assert(motor_probe_sequence_take(&s,now,&action));
    assert(action.kind==MOTOR_PROBE_START_SAMPLE && action.key.step==0);
}
static uint32_t baseline(uint32_t start,unsigned direction)
{
    begin_at(start,direction,&limits);
    struct w600_feedback_sample p=raw(start,1);
    assert(motor_probe_sequence_sample(&s,&action.key,start+2,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p));
    assert(s.state==MOTOR_PROBE_OBSERVED && s.raw.pair[0].signal==0);
    assert(!s.completed_pulses && !s.nominal_low_ticks_100khz);
    return start+2;
}
static void pulse_start(uint32_t at)
{
    struct motor_probe_key k=s.action.key;
    assert(motor_probe_sequence_next(&s,&k,at,false));
    assert(motor_probe_sequence_take(&s,at,&action));
    assert(action.kind==MOTOR_PROBE_START_PULSE && action.key.step==s.completed_pulses+1);
}
static uint32_t step(uint32_t at)
{
    pulse_start(at);
    struct tim4_motor_pulse p=done(action.key.direction);
    assert(motor_probe_sequence_pulse(&s,&action.key,at+21,&p,at+20,at+21));
    assert(s.state==MOTOR_PROBE_SAMPLE && s.nominal_low_ticks_100khz==1900*s.completed_pulses);
    assert(motor_probe_sequence_take(&s,at+22,&action));
    assert(action.kind==MOTOR_PROBE_START_SAMPLE);
    struct w600_feedback_sample sample=raw(at+22,s.raw.sequence+1);
    assert(motor_probe_sequence_sample(&s,&action.key,at+24,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&sample));
    return at+24;
}
static void cleanup(void)
{
    assert(s.cleanup_pending);
    assert(motor_probe_sequence_take(&s,s.last_now_ms,&action));
    assert(action.kind==MOTOR_PROBE_STOP);
    assert(!motor_probe_sequence_adc_released(&s,action.key.attempt,s.last_now_ms,true));
    assert(motor_probe_sequence_pulse_released(&s,action.key.attempt,s.last_now_ms,true));
    assert(motor_probe_sequence_adc_released(&s,action.key.attempt,s.last_now_ms,true));
    assert(!s.cleanup_pending);
}
static void happy_wrap_and_raw_only(void)
{
    const uint32_t starts[]={0,UINT32_MAX-10,UINT32_C(0x80000000)};
    for(unsigned d=1;d<=2;++d) for(unsigned n=0;n<3;++n) {
        uint32_t now=baseline(starts[n],d);
        now=step(now+1); now=step(now+1);
        struct motor_probe_key k=s.action.key;
        assert(motor_probe_sequence_next(&s,&k,now+1,true));
        assert(s.state==MOTOR_PROBE_STOPPING && s.error==MOTOR_PROBE_OK);
        assert(s.nominal_low_ticks_100khz==3800 && s.completed_pulses==2);
        cleanup(); assert(s.state==MOTOR_PROBE_COMPLETE_RAW && s.error==MOTOR_PROBE_OK);
        assert(!motor_probe_sequence_take(&s,now+2,&action));
        assert(!motor_probe_sequence_reset(&s,false));
        assert(motor_probe_sequence_reset(&s,true) && s.last_attempt==1);
        ++cases;
    }
    /* Off/communications gaps are wall time, never extra energized ticks. */
    uint32_t now=baseline(0,1);
    assert(motor_probe_sequence_progress(&s,1,90));
    now=step(91); assert(s.nominal_low_ticks_100khz==1900 && now==115);
    assert(motor_probe_sequence_progress(&s,1,180));
    now=step(181); assert(s.nominal_low_ticks_100khz==3800 && now==205); ++cases;
    now=baseline(0,1); struct motor_probe_key k=s.action.key;
    assert(motor_probe_sequence_next(&s,&k,now,true)); cleanup();
    assert(s.state==MOTOR_PROBE_COMPLETE_RAW && !s.completed_pulses); ++cases;
    /* All six fast conversions may legitimately share one millisecond tick. */
    begin_at(0,1,&limits); struct w600_feedback_sample p=raw(0,1);
    p.pair[1].acquired_at_ms=p.pair[2].acquired_at_ms=p.completed_at_ms=0;
    assert(motor_probe_sequence_sample(&s,&action.key,0,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p)); ++cases;
}
static void key_phase_and_once_only(void)
{
    begin_at(0,1,&limits);
    struct motor_probe_sequence before=s;
    struct w600_feedback_sample p=raw(0,1);
    for(unsigned n=0;n<3;++n) {
        struct motor_probe_key k=action.key;
        if(n==0) ++k.attempt;
        if(n==1) ++k.step;
        if(n==2) ++k.direction;
        assert(!motor_probe_sequence_sample(&s,&k,99999,
            MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p));
        assert(!memcmp(&s,&before,sizeof s)); ++cases;
    }
    assert(!motor_probe_sequence_take(&s,0,&action));
    assert(!motor_probe_sequence_progress(&s,2,99999));
    assert(!memcmp(&s,&before,sizeof s));
    assert(motor_probe_sequence_sample(&s,&action.key,2,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p));
    before=s;
    assert(!motor_probe_sequence_sample(&s,&action.key,2,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p));
    assert(!memcmp(&s,&before,sizeof s)); ++cases;
    struct motor_probe_key old=action.key;
    pulse_start(3); before=s;
    struct tim4_motor_pulse pulse=done(1);
    assert(!motor_probe_sequence_pulse(&s,&old,99999,&pulse,23,24));
    assert(!motor_probe_sequence_sample(&s,&action.key,99999,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p));
    assert(!memcmp(&s,&before,sizeof s)); ++cases;
    assert(motor_probe_sequence_pulse(&s,&action.key,24,&pulse,23,24));
    before=s;
    assert(!motor_probe_sequence_pulse(&s,&action.key,24,&pulse,23,24));
    p=raw(24,2);
    assert(!motor_probe_sequence_sample(&s,&action.key,26,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p)); /* SAMPLE not taken. */
    assert(!memcmp(&s,&before,sizeof s)); ++cases;
}
static void pulse_bad_evidence(void)
{
    for(unsigned n=0;n<19;++n) {
        (void)baseline(0,1); pulse_start(3);
        struct tim4_motor_pulse p=done(1);
        uint32_t now=24,off=23,settled=24;
        switch(n) {
        case 0:p.status=TIM4_PULSE_FAILED;break; case 1:p.status=TIM4_PULSE_SETTLING;break;
        case 2:p.error=TIM4_PULSE_TIMER;break; case 3:p.cleanup_error=1;break;
        case 4:p.owned=1;break; case 5:p.direction=2;break; case 6:p.channel=1;break;
        case 7:p.seen_low=0;break; case 8:p.seen_low=2;break; case 9:p.low_count=99;break;
        case 10:p.low_count=2000;break; case 11:p.settle_count=100;break;
        case 12:off=22;break; case 13:off=UINT32_MAX;break; case 14:settled=off;break;
        case 15:settled=25;break; case 16:off=24;settled=23;break;
        case 17:now=103;break; default:break;
        }
        assert(!motor_probe_sequence_pulse(&s,&action.key,now,n==18?NULL:&p,off,settled));
        assert(s.state==MOTOR_PROBE_FAILED && s.error && s.cleanup_pending);
        assert(!s.nominal_low_ticks_100khz && !s.completed_pulses);
        enum motor_probe_error first=s.error;
        motor_probe_sequence_abort(&s); assert(s.error==first);
        cleanup(); assert(s.state==MOTOR_PROBE_FAILED && s.error==first); ++cases;
    }
}
static void sample_bad_evidence(void)
{
    for(unsigned n=0;n<17;++n) {
        begin_at(100,1,&limits);
        struct w600_feedback_sample p=raw(100,1);
        uint32_t word=factory,now=102;
        enum motor_probe_source source=MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW;
        switch(n) {
        case 0:p.valid=false;break; case 1:p.error=W600_FEEDBACK_RAW;break;
        case 2:p.sequence=0;break; case 3:p.pair[0].bgr=0;break;
        case 4:p.pair[1].bgr=4096;break; case 5:p.pair[2].signal=4096;break;
        case 6:word^=0x10000;break; case 7:word=0xffff0000;break;
        case 8:p.oldest_at_ms=101;break; case 9:p.pair[1].acquired_at_ms=99;break;
        case 10:p.completed_at_ms=103;break; case 11:p=raw(99,1);break;
        case 12:p.pair[2].acquired_at_ms=103;break; case 13:now=110;break;
        case 14:source=(enum motor_probe_source)2;break;
        case 15:p.pair[2].acquired_at_ms=100;break; default:break;
        }
        assert(!motor_probe_sequence_sample(&s,&action.key,now,source,word,n==16?NULL:&p));
        assert(s.state==MOTOR_PROBE_FAILED && s.error && !s.have_sample);
        cleanup(); ++cases;
    }
    for(unsigned n=0;n<3;++n) {
        (void)baseline(0,1); pulse_start(3); struct tim4_motor_pulse p=done(1);
        assert(motor_probe_sequence_pulse(&s,&action.key,24,&p,23,24));
        assert(motor_probe_sequence_take(&s,25,&action));
        struct w600_feedback_sample sample=raw(25,2);
        if(n==0) sample.sequence=1;
        if(n==1) sample=raw(1,2);
        assert(!motor_probe_sequence_sample(&s,&action.key,27,
            MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,n==2?0xfa2705d8u:factory,&sample));
        assert(s.error==MOTOR_PROBE_SAMPLE_ERROR && s.nominal_low_ticks_100khz==1900);
        cleanup(); ++cases;
    }
}
static void deadlines_limits_and_cleanup(void)
{
    const uint32_t starts[]={0,UINT32_MAX-9};
    for(unsigned n=0;n<2;++n) {
        struct motor_probe_limits p=limits;
        p.total_ms=p.foreground_ms=p.acquisition_ms=p.pulse_ms=30;
        begin_at(starts[n],1,&p);
        motor_probe_sequence_tick(&s,starts[n]+29);
        assert(s.state==MOTOR_PROBE_BASELINE);
        motor_probe_sequence_tick(&s,starts[n]+30);
        assert(s.error==MOTOR_PROBE_TOTAL_TIMEOUT); cleanup(); ++cases;
        begin_at(starts[n],1,&limits);
        assert(motor_probe_sequence_progress(&s,1,starts[n]+19));
        motor_probe_sequence_tick(&s,starts[n]+20);
        assert(s.error==MOTOR_PROBE_OPERATION_TIMEOUT); cleanup(); ++cases;
    }
    struct motor_probe_limits p=limits;
    p.foreground_ms=10; begin_at(0,1,&p);
    motor_probe_sequence_tick(&s,10);
    assert(s.error==MOTOR_PROBE_FOREGROUND_TIMEOUT); cleanup(); ++cases;
    begin_at(0,1,&limits);
    assert(motor_probe_sequence_progress(&s,1,19));
    motor_probe_sequence_tick(&s,20);
    assert(s.error==MOTOR_PROBE_OPERATION_TIMEOUT); cleanup(); ++cases;
    (void)baseline(0,1); pulse_start(3);
    assert(motor_probe_sequence_progress(&s,1,90));
    assert(motor_probe_sequence_progress(&s,1,102));
    struct tim4_motor_pulse pulse=done(1);
    assert(!motor_probe_sequence_pulse(&s,&action.key,103,&pulse,23,24));
    assert(s.error==MOTOR_PROBE_OPERATION_TIMEOUT && !s.nominal_low_ticks_100khz);
    cleanup(); ++cases;
    p=limits;p.total_ms=50;p.foreground_ms=40;p.pulse_ms=40;
    begin_at(0,1,&p); struct w600_feedback_sample sample=raw(0,1);
    assert(motor_probe_sequence_sample(&s,&action.key,2,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&sample));
    assert(motor_probe_sequence_progress(&s,1,39));
    assert(s.foreground_deadline_ms==50);
    struct motor_probe_key k=s.action.key;
    assert(!motor_probe_sequence_next(&s,&k,50,true));
    assert(s.error==MOTOR_PROBE_TOTAL_TIMEOUT); cleanup(); ++cases;
    begin_at(100,1,&limits); motor_probe_sequence_tick(&s,101);
    motor_probe_sequence_tick(&s,100); assert(s.error==MOTOR_PROBE_CLOCK); cleanup(); ++cases;
    uint32_t now=baseline(0,1); now=step(now+1); now=step(now+1); k=s.action.key;
    assert(!motor_probe_sequence_next(&s,&k,now+1,false));
    assert(s.error==MOTOR_PROBE_PULSE_LIMIT && s.completed_pulses==2);
    cleanup(); ++cases;
    /* End while queued/driving/settling/acquiring never fabricates a full pulse. */
    for(unsigned phase=0;phase<4;++phase) {
        (void)baseline(0,1);
        if(phase) pulse_start(3);
        if(phase==2) motor_probe_sequence_tick(&s,23);
        if(phase==3) {
            assert(motor_probe_sequence_pulse(&s,&action.key,24,&pulse,23,24));
            assert(motor_probe_sequence_take(&s,25,&action));
        }
        motor_probe_sequence_abort(&s);
        assert(s.state==MOTOR_PROBE_FAILED && s.error==MOTOR_PROBE_ABORTED);
        assert(s.completed_pulses==(phase==3 ? 1u : 0u));
        assert(!motor_probe_sequence_reset(&s,true));
        cleanup(); assert(s.state==MOTOR_PROBE_FAILED); ++cases;
    }
    now=baseline(0,1); k=s.action.key;
    assert(motor_probe_sequence_next(&s,&k,now,true));
    assert(!motor_probe_sequence_pulse_released(&s,1,now,true)); /* STOP not taken. */
    assert(motor_probe_sequence_take(&s,now,&action));
    assert(!motor_probe_sequence_pulse_released(&s,2,now,true));
    assert(!motor_probe_sequence_pulse_released(&s,1,now,false));
    assert(s.error==MOTOR_PROBE_CLEANUP_ERROR && s.cleanup_pending);
    assert(!motor_probe_sequence_adc_released(&s,1,now,true));
    assert(motor_probe_sequence_pulse_released(&s,1,now,true));
    assert(!motor_probe_sequence_adc_released(&s,1,now,false) && s.cleanup_pending);
    motor_probe_sequence_abort(&s);
    assert(s.error==MOTOR_PROBE_CLEANUP_ERROR && s.pulse_released && s.taken);
    assert(motor_probe_sequence_adc_released(&s,1,now,true));
    assert(s.state==MOTOR_PROBE_FAILED && s.error==MOTOR_PROBE_CLEANUP_ERROR); ++cases;
    /* Cleanup completion also checks time: no final receipt revives expiry. */
    for(unsigned late=0;late<2;++late) {
        now=baseline(0,1); k=s.action.key;
        assert(motor_probe_sequence_next(&s,&k,now,true));
        assert(motor_probe_sequence_take(&s,now,&action));
        if(late) assert(motor_probe_sequence_pulse_released(&s,1,now,true));
        assert(motor_probe_sequence_pulse_released(&s,1,1000,true));
        assert(motor_probe_sequence_adc_released(&s,1,1000,true));
        assert(s.state==MOTOR_PROBE_FAILED && s.error==MOTOR_PROBE_TOTAL_TIMEOUT &&
            !s.cleanup_pending); ++cases;
    }
}
static void invalid_begin_and_token_floor(void)
{
    for(unsigned n=0;n<15;++n) {
        motor_probe_sequence_init(&s); struct motor_probe_limits p=limits;
        switch(n) {
        case 0:p.total_ms=0;break; case 1:p.total_ms=UINT32_C(0x80000000);break;
        case 2:p.foreground_ms=0;break; case 3:p.foreground_ms=1001;break;
        case 4:p.pulse_ms=0;break; case 5:p.acquisition_ms=0;break;
        case 6:p.max_sample_age_ms=0;break; case 7:p.max_sample_age_ms=21;break;
        case 8:p.max_pulses=0;break; case 9:p.max_pulses=UINT32_MAX;break;
        case 11:p.pulse_ms=1001;break; case 12:p.acquisition_ms=1001;break;
        default:break;
        }
        unsigned direction=n==13 ? 0u : n==14 ? 3u : 1u;
        assert(!motor_probe_sequence_begin(&s,0,1,direction,n==10?NULL:&p,true));
        assert(s.error==MOTOR_PROBE_LIMITS); cleanup(); ++cases;
    }
    motor_probe_sequence_init(&s);
    assert(!motor_probe_sequence_begin(&s,0,1,1,&limits,false) && s.state==MOTOR_PROBE_IDLE);
    assert(motor_probe_sequence_begin(&s,0,UINT32_MAX,2,&limits,true));
    assert(motor_probe_sequence_take(&s,0,&action));
    struct w600_feedback_sample p=raw(0,1);
    assert(motor_probe_sequence_sample(&s,&action.key,2,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p));
    struct motor_probe_key old=action.key;
    assert(motor_probe_sequence_next(&s,&old,2,true)); cleanup();
    assert(motor_probe_sequence_reset(&s,true) && s.last_attempt==UINT32_MAX);
    assert(!motor_probe_sequence_begin(&s,3,UINT32_MAX,2,&limits,true));
    assert(s.error==MOTOR_PROBE_TOKEN); cleanup(); ++cases;
    (void)baseline(0,1); old=s.action.key; motor_probe_sequence_abort(&s); cleanup();
    assert(motor_probe_sequence_reset(&s,true));
    assert(motor_probe_sequence_begin(&s,100,2,2,&limits,true));
    struct motor_probe_sequence before=s;
    assert(!motor_probe_sequence_sample(&s,&old,99999,
        MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW,factory,&p));
    assert(!motor_probe_sequence_pulse_released(&s,1,99999,true));
    assert(!memcmp(&s,&before,sizeof s)); ++cases;
}
int main(void)
{
    happy_wrap_and_raw_only(); key_phase_and_once_only(); pulse_bad_evidence();
    sample_bad_evidence(); deadlines_limits_and_cleanup(); invalid_begin_and_token_floor();
    printf("PASS %u actual-source bounded raw probe cases; no hardware/calibration claim\n",cases);
}
