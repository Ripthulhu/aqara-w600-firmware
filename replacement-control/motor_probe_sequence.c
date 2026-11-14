#include "motor_probe_sequence.h"
#include <limits.h>

#define HALF UINT32_C(0x80000000)
static bool live(const struct motor_probe_sequence *s)
{ return s->state>=MOTOR_PROBE_BASELINE && s->state<=MOTOR_PROBE_STOPPING; }
static bool reached(uint32_t now,uint32_t at)
{ return (uint32_t)(now-at)<HALF; }
static bool key_ok(const struct motor_probe_sequence *s,const struct motor_probe_key *k)
{
    return k && k->attempt==s->action.key.attempt && k->step==s->action.key.step &&
        k->direction==s->action.key.direction;
}
static void stop_request(struct motor_probe_sequence *s)
{
    s->action.kind=MOTOR_PROBE_STOP; s->taken=false;
    s->cleanup_pending=true; s->pulse_released=false;
}
static void fail(struct motor_probe_sequence *s,enum motor_probe_error error)
{
    if(s->error==MOTOR_PROBE_OK) s->error=error;
    if(s->state!=MOTOR_PROBE_FAILED && s->state!=MOTOR_PROBE_STOPPING) stop_request(s);
    s->state=MOTOR_PROBE_FAILED;
}
static bool limits_ok(const struct motor_probe_limits *p)
{
    return p && p->total_ms && p->total_ms<HALF &&
        p->foreground_ms && p->foreground_ms<=p->total_ms &&
        p->pulse_ms && p->pulse_ms<=p->total_ms &&
        p->acquisition_ms && p->acquisition_ms<=p->total_ms &&
        p->max_sample_age_ms && p->max_sample_age_ms<=p->acquisition_ms &&
        p->max_pulses && p->max_pulses<=UINT32_MAX/MOTOR_PROBE_LOW_TICKS_100KHZ_PER_PULSE;
}
static void request(struct motor_probe_sequence *s,uint32_t now,bool pulse)
{
    s->state=pulse ? MOTOR_PROBE_PULSE :
        (s->completed_pulses ? MOTOR_PROBE_SAMPLE : MOTOR_PROBE_BASELINE);
    s->action.kind=pulse ? MOTOR_PROBE_START_PULSE : MOTOR_PROBE_START_SAMPLE;
    s->taken=false; s->operation_at_ms=now;
    s->operation_deadline_ms=now+(pulse ? s->limits.pulse_ms : s->limits.acquisition_ms);
}
void motor_probe_sequence_init(struct motor_probe_sequence *s)
{ *s=(struct motor_probe_sequence){0}; }
bool motor_probe_sequence_begin(struct motor_probe_sequence *s,uint32_t now,
    uint32_t attempt,unsigned direction,const struct motor_probe_limits *p,bool stopped)
{
    if(s->state!=MOTOR_PROBE_IDLE || s->cleanup_pending || !stopped) return false;
    if(!attempt || attempt<=s->last_attempt) { fail(s,MOTOR_PROBE_TOKEN); return false; }
    s->last_attempt=attempt;
    s->action.key=(struct motor_probe_key){attempt,0,direction};
    if(!limits_ok(p) || (direction!=1 && direction!=2)) {
        fail(s,MOTOR_PROBE_LIMITS); return false;
    }
    s->limits=*p; s->started_at_ms=s->last_now_ms=now;
    s->foreground_deadline_ms=now+p->foreground_ms;
    request(s,now,false); return true;
}
void motor_probe_sequence_tick(struct motor_probe_sequence *s,uint32_t now)
{
    if(!live(s)) return;
    if((uint32_t)(now-s->last_now_ms)>=HALF) { fail(s,MOTOR_PROBE_CLOCK); return; }
    s->last_now_ms=now;
    if(reached(now,s->started_at_ms+s->limits.total_ms)) fail(s,MOTOR_PROBE_TOTAL_TIMEOUT);
    else if(reached(now,s->foreground_deadline_ms)) fail(s,MOTOR_PROBE_FOREGROUND_TIMEOUT);
    else if((s->state==MOTOR_PROBE_PULSE || s->state==MOTOR_PROBE_BASELINE ||
             s->state==MOTOR_PROBE_SAMPLE) && reached(now,s->operation_deadline_ms))
        fail(s,MOTOR_PROBE_OPERATION_TIMEOUT);
}
bool motor_probe_sequence_progress(struct motor_probe_sequence *s,uint32_t attempt,uint32_t now)
{
    if(attempt!=s->action.key.attempt || !live(s)) return false;
    motor_probe_sequence_tick(s,now);
    if(!live(s)) return false;
    uint32_t remaining=s->started_at_ms+s->limits.total_ms-now;
    uint32_t lease=s->limits.foreground_ms;
    s->foreground_deadline_ms=now+(lease<remaining ? lease : remaining);
    return true;
}
bool motor_probe_sequence_take(struct motor_probe_sequence *s,uint32_t now,
    struct motor_probe_action *out)
{
    motor_probe_sequence_tick(s,now);
    if(!out || s->action.kind==MOTOR_PROBE_NO_ACTION) return false;
    *out=s->action; s->action.kind=MOTOR_PROBE_NO_ACTION;
    s->taken=true; s->taken_at_ms=now; return true;
}
bool motor_probe_sequence_pulse(struct motor_probe_sequence *s,const struct motor_probe_key *k,
    uint32_t now,const struct tim4_motor_pulse *p,uint32_t off_at,uint32_t settled_at)
{
    if(!key_ok(s,k) || s->state!=MOTOR_PROBE_PULSE || !s->taken) return false;
    motor_probe_sequence_tick(s,now);
    if(s->state!=MOTOR_PROBE_PULSE) return false;
    if(!p || p->status!=TIM4_PULSE_DONE || p->error || p->cleanup_error || p->owned ||
       p->direction!=k->direction || p->channel!=(k->direction==1 ? 2u : 1u) ||
       p->seen_low!=1 || p->low_count<100 || p->low_count>1999 || p->settle_count>99 ||
       (uint32_t)(off_at-s->taken_at_ms)<20 ||
       (uint32_t)(off_at-s->taken_at_ms)>=HALF ||
       (uint32_t)(settled_at-off_at)<1 || (uint32_t)(settled_at-off_at)>=HALF ||
       (uint32_t)(now-settled_at)>=HALF) {
        fail(s,MOTOR_PROBE_PULSE_ERROR); return false;
    }
    ++s->completed_pulses;
    s->nominal_low_ticks_100khz+=MOTOR_PROBE_LOW_TICKS_100KHZ_PER_PULSE;
    request(s,now,false); return true;
}
static bool sample_ok(const struct motor_probe_sequence *s,uint32_t now,
    uint32_t factory,const struct w600_feedback_sample *p)
{
    uint32_t code=factory&0xffffu;
    if(!p || !p->valid || p->error!=W600_FEEDBACK_OK || !p->sequence ||
       !code || code>4095 || (((factory>>16)^code)&0xffffu)!=0xffffu ||
       (s->have_sample && (factory!=s->factory_word || p->sequence<=s->raw.sequence ||
        !((uint32_t)(p->oldest_at_ms-s->raw.completed_at_ms)) ||
        (uint32_t)(p->oldest_at_ms-s->raw.completed_at_ms)>=HALF)) ||
       p->oldest_at_ms!=p->pair[0].acquired_at_ms ||
       (uint32_t)(p->oldest_at_ms-s->taken_at_ms)>=HALF ||
       (uint32_t)(p->completed_at_ms-p->oldest_at_ms)>=HALF ||
       (uint32_t)(now-p->completed_at_ms)>=HALF) return false;
    for(unsigned i=0;i<3;++i) {
        if(!p->pair[i].bgr || p->pair[i].bgr>4095 || p->pair[i].signal>4095 ||
           (uint32_t)(p->pair[i].acquired_at_ms-p->oldest_at_ms)>
               (uint32_t)(p->completed_at_ms-p->oldest_at_ms) ||
           (i && (uint32_t)(p->pair[i].acquired_at_ms-p->pair[i-1].acquired_at_ms)>=HALF))
            return false;
    }
    return true;
}
bool motor_probe_sequence_sample(struct motor_probe_sequence *s,const struct motor_probe_key *k,
    uint32_t now,enum motor_probe_source source,uint32_t factory,
    const struct w600_feedback_sample *p)
{
    if(!key_ok(s,k) || (s->state!=MOTOR_PROBE_BASELINE && s->state!=MOTOR_PROBE_SAMPLE) ||
       !s->taken) return false;
    motor_probe_sequence_tick(s,now);
    if(s->state!=MOTOR_PROBE_BASELINE && s->state!=MOTOR_PROBE_SAMPLE) return false;
    if(source!=MOTOR_PROBE_RAW_GPIO_HH_SELECTORS_LOW || !sample_ok(s,now,factory,p)) {
        fail(s,MOTOR_PROBE_SAMPLE_ERROR); return false;
    }
    if((uint32_t)(now-p->oldest_at_ms)>=s->limits.max_sample_age_ms) {
        fail(s,MOTOR_PROBE_SAMPLE_STALE); return false;
    }
    s->raw=*p; s->factory_word=factory; s->have_sample=true;
    s->state=MOTOR_PROBE_OBSERVED; s->taken=false; return true;
}
bool motor_probe_sequence_next(struct motor_probe_sequence *s,const struct motor_probe_key *k,
    uint32_t now,bool finish)
{
    if(!key_ok(s,k) || s->state!=MOTOR_PROBE_OBSERVED) return false;
    motor_probe_sequence_tick(s,now);
    if(s->state!=MOTOR_PROBE_OBSERVED) return false;
    if(finish) { stop_request(s); s->state=MOTOR_PROBE_STOPPING; return true; }
    if(s->completed_pulses>=s->limits.max_pulses) {
        fail(s,MOTOR_PROBE_PULSE_LIMIT); return false;
    }
    s->action.key.step=s->completed_pulses+1u;
    request(s,now,true); return true;
}
void motor_probe_sequence_abort(struct motor_probe_sequence *s)
{
    if(s->state!=MOTOR_PROBE_COMPLETE_RAW && s->state!=MOTOR_PROBE_FAILED)
        fail(s,MOTOR_PROBE_ABORTED);
}
bool motor_probe_sequence_pulse_released(struct motor_probe_sequence *s,uint32_t attempt,
    uint32_t now,bool released)
{
    if(attempt!=s->action.key.attempt) return false;
    motor_probe_sequence_tick(s,now);
    if(!s->cleanup_pending || !s->taken ||
       (s->state!=MOTOR_PROBE_STOPPING && s->state!=MOTOR_PROBE_FAILED)) return false;
    if(!released) { fail(s,MOTOR_PROBE_CLEANUP_ERROR); return false; }
    s->pulse_released=true; return true;
}
bool motor_probe_sequence_adc_released(struct motor_probe_sequence *s,uint32_t attempt,
    uint32_t now,bool released)
{
    if(attempt!=s->action.key.attempt) return false;
    motor_probe_sequence_tick(s,now);
    if(!s->cleanup_pending || !s->pulse_released) return false;
    if(!released) { fail(s,MOTOR_PROBE_CLEANUP_ERROR); return false; }
    s->cleanup_pending=false;
    if(s->state==MOTOR_PROBE_STOPPING) s->state=MOTOR_PROBE_COMPLETE_RAW;
    return true;
}
bool motor_probe_sequence_reset(struct motor_probe_sequence *s,bool stopped)
{
    if(!stopped || s->cleanup_pending || (s->state!=MOTOR_PROBE_IDLE &&
       s->state!=MOTOR_PROBE_COMPLETE_RAW && s->state!=MOTOR_PROBE_FAILED)) return false;
    uint32_t attempt=s->last_attempt;
    *s=(struct motor_probe_sequence){.last_attempt=attempt}; return true;
}
