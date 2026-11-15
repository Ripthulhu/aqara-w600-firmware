#include "motor_calibration.h"
#include <limits.h>
#include <stddef.h>

#define HALF UINT32_C(0x80000000)
enum { STOP_PROBE_LOW=1, STOP_ENDPOINT=3, STOP_PROBE_INCONCLUSIVE,
       STOP_CANDIDATE, STOP_CONFIRM_BACKOFF, STOP_CONFIRM_RETURN,
       STOP_LOAD_CANDIDATE=10, STOP_LOAD_RETURN };

static bool live(const struct motor_calibration *c)
{ return c->state==MOTOR_CAL_ACTIVE || c->state==MOTOR_CAL_WAIT_STOP; }
static bool probe(const struct motor_calibration *c)
{ return c->phase==MOTOR_CAL_PROBE_CLOSE || c->phase==MOTOR_CAL_PROBE_OPEN; }
static uint8_t opposite(uint8_t direction)
{ return direction==MOTOR_CAL_OPEN ? MOTOR_CAL_CLOSE : MOTOR_CAL_OPEN; }
static bool reached(uint32_t now,uint32_t deadline)
{ return (uint32_t)(now-deadline)<HALF; }
static void fail(struct motor_calibration *c,enum motor_calibration_error error)
{
    if(c->error==MOTOR_CAL_OK) c->error=error;
    motor_homing_abort(&c->homing);
    c->action.direction=MOTOR_CAL_NONE; c->action.on_budget_ticks=0;
    c->state=MOTOR_CAL_FAILED;
}
static bool time_ok(struct motor_calibration *c,uint32_t now)
{
    if((uint32_t)(now-c->last_now)>=HALF) { fail(c,MOTOR_CAL_CLOCK); return false; }
    if(reached(now,c->attempt_started_at+c->policy.whole_attempt_ticks)) {
        fail(c,MOTOR_CAL_ATTEMPT_TIMEOUT); return false;
    }
    /* Spring contact is not an end stop; the independent homing deadline bounds seating. */
    if(c->state==MOTOR_CAL_WAIT_STOP &&
       reached(now,c->stop_requested_at+c->policy.stop_confirm_ticks)) {
        fail(c,MOTOR_CAL_STOP_TIMEOUT); return false;
    }
    return true;
}
static bool expected_stop(const struct motor_calibration *c)
{
    if(c->intentional_abort)
        return c->homing.state==MOTOR_HOMING_FAILED && c->homing.error==MOTOR_HOMING_ABORTED &&
               c->homing.guard.error==MOTOR_GUARD_EXTERNAL_FAULT && !c->homing.guard.armed;
    return c->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED &&
           c->homing.error==MOTOR_HOMING_OK && c->homing.guard.error==MOTOR_GUARD_OK &&
           !c->homing.guard.armed;
}
void motor_calibration_tick(struct motor_calibration *c,uint32_t now)
{
    if(!c || !live(c) || !time_ok(c,now)) return;
    c->last_now=now;
    motor_homing_tick(&c->homing,now);
    if((c->state==MOTOR_CAL_ACTIVE && c->homing.error!=MOTOR_HOMING_OK) ||
       (c->state==MOTOR_CAL_WAIT_STOP && !expected_stop(c))) fail(c,MOTOR_CAL_HOMING);
}
static bool policy_ok(const struct motor_calibration_policy *p)
{
    return p && p->whole_attempt_ticks && p->whole_attempt_ticks<HALF &&
        p->probe_on_ticks && p->probe_on_ticks<=p->max_travel_ticks &&
        p->probe_slack_ticks<=p->max_travel_ticks-p->probe_on_ticks &&
        p->min_travel_ticks && p->min_travel_ticks<=p->max_travel_ticks &&
        p->max_travel_ticks<HALF &&
        motor_homing_policy_valid(&p->homing[0]) && motor_homing_policy_valid(&p->homing[1]) &&
        p->max_travel_ticks<=p->homing[0].max_move_ticks &&
        p->max_travel_ticks<=p->homing[1].max_move_ticks &&
        p->repeat_tolerance_ticks && p->repeat_tolerance_ticks<=p->max_travel_ticks &&
        p->stop_confirm_ticks && p->stop_confirm_ticks<HALF &&
        p->min_stop_ticks && p->min_stop_ticks<p->stop_confirm_ticks;
}
static bool confirmation_ok(const struct motor_calibration *c,
                            const struct motor_calibration_policy *p)
{
    const struct motor_calibration_confirmation *q=&c->confirmation;
    if (!q->max_sample_gap_ticks)
        return !(q->backoff_ticks || q->backoff_limit_ticks || q->return_ticks ||
                 q->return_limit_ticks || q->rest_ticks);
    if (!q->backoff_ticks || q->backoff_ticks>=q->backoff_limit_ticks ||
        !q->return_ticks || q->return_ticks>=q->return_limit_ticks ||
        q->rest_ticks<p->min_stop_ticks || q->rest_ticks>=p->stop_confirm_ticks) return false;
    for (unsigned i=0;i<2;++i) {
        const struct motor_homing_policy *h=&p->homing[i];
        uint32_t shortest=q->backoff_limit_ticks<q->return_limit_ticks ?
            q->backoff_limit_ticks : q->return_limit_ticks;
        if (q->max_sample_gap_ticks>=h->feedback_lease_ticks ||
            q->backoff_limit_ticks>h->max_move_ticks || q->return_limit_ticks>h->max_move_ticks ||
            shortest<h->foreground_lease_ticks || shortest<h->feedback_lease_ticks ||
            shortest<=h->motion_dwell_ticks ||
            shortest-h->motion_dwell_ticks<=h->endpoint_dwell_ticks)
            return false;
    }
    return true;
}
static bool begin(struct motor_calibration *c,enum motor_calibration_phase phase,
                  uint8_t direction,uint32_t now)
{
    if(c->last_token==UINT32_MAX) { fail(c,MOTOR_CAL_TOKEN_EXHAUSTED); return false; }
    c->homing.policy=c->policy.homing[direction-1u];
    if (phase==MOTOR_CAL_CONFIRM_BACKOFF) c->homing.policy.max_move_ticks=c->confirmation.backoff_limit_ticks;
    if (phase==MOTOR_CAL_CONFIRM_RETURN) c->homing.policy.max_move_ticks=c->confirmation.return_limit_ticks;
    if(!motor_homing_start(&c->homing,now,&c->homing.policy)) {
        fail(c,MOTOR_CAL_HOMING); return false;
    }
    c->phase=phase; c->direction=direction; c->phase_started_at=now;
    c->on_ticks=0; c->last_on_at=now; c->last_signal=0; c->intentional_abort=false; c->stop_reason=0;
    c->last_sample_on=0; c->first_low_on=0; c->have_sample_on=false; c->have_first_low=false;
    c->stop_observed_at=0; c->stopped_on_ticks=0; c->have_stop_proof=false;
    c->intermediate_count=0; c->pattern_high=false; c->pattern_have_sample=false;
    c->intermediate_started_at=0; c->pattern_last_at=0;
    motor_slowdown_reset(&c->slowdown);
    motor_seat_reset(&c->seat);
    c->action=(struct motor_calibration_action){++c->last_token,
        phase==MOTOR_CAL_PROBE_CLOSE || phase==MOTOR_CAL_PROBE_OPEN ?
            c->policy.probe_on_ticks+c->policy.probe_slack_ticks : c->homing.policy.max_move_ticks,direction};
    c->state=MOTOR_CAL_ACTIVE;
    c->closing_load_seen=false; c->closing_load_at=0;
    return true;
}
bool motor_calibration_init(struct motor_calibration *c,void (*stop)(void *),void *context)
{
    if(!c) return false;
    *c=(struct motor_calibration){0};
    if(!motor_homing_init(&c->homing,stop,context)) {
        c->state=MOTOR_CAL_FAILED; c->error=MOTOR_CAL_BAD_STATE; return false;
    }
    return true;
}
static bool start(struct motor_calibration *c,uint32_t now,
                  const struct motor_calibration_policy *policy,bool proven_stopped,bool retract)
{
    if(!c || c->state!=MOTOR_CAL_IDLE || c->error!=MOTOR_CAL_OK || !proven_stopped) return false;
    if(!policy_ok(policy) || !confirmation_ok(c,policy)) { fail(c,MOTOR_CAL_BAD_POLICY); return false; }
    if(retract && !c->confirmation.max_sample_gap_ticks) { fail(c,MOTOR_CAL_BAD_POLICY); return false; }
    c->policy=*policy; c->attempt_started_at=now; c->last_now=now;
    c->retract_only=retract;
    if(retract) c->policy.whole_attempt_ticks=600000u; /* Fixed60s, including confirmation/rest. */
    return begin(c,retract ? MOTOR_CAL_PROBE_OPEN : MOTOR_CAL_PROBE_CLOSE,
                 retract ? MOTOR_CAL_OPEN : MOTOR_CAL_CLOSE,now);
}
bool motor_calibration_start(struct motor_calibration *c,uint32_t now,
                             const struct motor_calibration_policy *policy,bool proven_stopped)
{ return start(c,now,policy,proven_stopped,false); }
bool motor_calibration_retract_start(struct motor_calibration *c,uint32_t now,
                             const struct motor_calibration_policy *policy,bool proven_stopped)
{ return start(c,now,policy,proven_stopped,true); }
static bool count_ok(struct motor_calibration *c,uint32_t now,uint32_t on)
{
    uint32_t limit=probe(c) ? c->policy.probe_on_ticks+c->policy.probe_slack_ticks : c->homing.policy.max_move_ticks;
    if(on<c->on_ticks || on>(uint32_t)(now-c->phase_started_at) ||
       on-c->on_ticks>(uint32_t)(now-c->last_on_at)) {
        fail(c,MOTOR_CAL_COUNTER); return false;
    }
    if(on>limit) { fail(c,MOTOR_CAL_ENERGY_LIMIT); return false; }
    c->on_ticks=on; c->last_on_at=now;
    return true;
}
static void request_stop(struct motor_calibration *c,uint32_t now,uint8_t reason)
{
    c->intentional_abort=c->homing.state==MOTOR_HOMING_SEEK_MOTION ||
                         c->homing.state==MOTOR_HOMING_SEEK_ENDPOINT;
    motor_homing_abort(&c->homing);
    c->stop_requested_at=now; c->stop_reason=reason; c->state=MOTOR_CAL_WAIT_STOP;
    c->action.direction=MOTOR_CAL_NONE; c->action.on_budget_ticks=0;
    if(!expected_stop(c)) fail(c,MOTOR_CAL_HOMING);
}
static bool phase_result(struct motor_calibration *c,uint32_t now)
{
    if (c->phase==MOTOR_CAL_CONFIRM_BACKOFF) {
        if (c->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED) {
            fail(c,MOTOR_CAL_REFERENCE_UNQUALIFIED); return false;
        }
        if ((uint32_t)(now-c->phase_started_at)>=c->confirmation.backoff_ticks) {
            if (c->homing.state!=MOTOR_HOMING_SEEK_ENDPOINT) { fail(c,MOTOR_CAL_NO_MOTION); return false; }
            request_stop(c,now,STOP_CONFIRM_BACKOFF);
        }
        return c->state!=MOTOR_CAL_FAILED;
    }
    if (c->phase==MOTOR_CAL_CONFIRM_RETURN &&
        ((uint32_t)(now-c->phase_started_at)>=c->confirmation.return_ticks ||
         c->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED)) {
        fail(c,MOTOR_CAL_REFERENCE_UNQUALIFIED); return false;
    }
    if(probe(c) && (c->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED ||
       (c->homing.state==MOTOR_HOMING_SEEK_ENDPOINT && c->on_ticks>=c->policy.probe_on_ticks))) {
        /* Keep the qualified move: reversing here can return straight into the starting end. */
        c->phase=MOTOR_CAL_HOME;
        c->action.on_budget_ticks=c->homing.policy.max_move_ticks;
    }
    if(c->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED) {
        if(!c->have_first_low) { fail(c,MOTOR_CAL_COUNTER); return false; }
        request_stop(c,now,STOP_ENDPOINT);
    }
    else if(probe(c) && c->on_ticks>=c->policy.probe_on_ticks) {
        bool fresh=c->homing.have_sample &&
            (uint32_t)(now-c->homing.last_sample_at)<c->homing.policy.max_sample_age_ticks;
        /* Target can fall between ADC windows; the explicit hard budget still applies. */
        if(!fresh) return true;
        if(!c->homing.motion_count && c->last_signal<=c->homing.policy.endpoint_max)
            request_stop(c,now,STOP_PROBE_LOW);
        else if(c->phase==MOTOR_CAL_PROBE_CLOSE)
            request_stop(c,now,STOP_PROBE_INCONCLUSIVE);
        else fail(c,MOTOR_CAL_PROBE_SIGNAL);
    } else if(!probe(c) && c->on_ticks==c->homing.policy.max_move_ticks)
        fail(c,MOTOR_CAL_ENERGY_LIMIT);
    return c->state!=MOTOR_CAL_FAILED;
}
static bool candidate_feedback(struct motor_calibration *c,uint32_t now,uint32_t at,uint32_t value)
{
    if (!c->confirmation.max_sample_gap_ticks) return false;
    if (c->phase==MOTOR_CAL_CONFIRM_RETURN &&
        (uint32_t)(now-c->phase_started_at)>=c->confirmation.return_ticks) {
        fail(c,MOTOR_CAL_REFERENCE_UNQUALIFIED); return true;
    }
    bool continuous=c->pattern_have_sample &&
        (uint32_t)(at-c->pattern_last_at)<=c->confirmation.max_sample_gap_ticks;
    if (!continuous) { c->pattern_high=false; c->intermediate_count=0; }
    bool low=c->homing.state==MOTOR_HOMING_SEEK_ENDPOINT && value<=c->homing.policy.endpoint_max;
    bool loaded=motor_slowdown_observe(&c->slowdown,at,value) &&
                c->homing.state==MOTOR_HOMING_SEEK_ENDPOINT;
    bool returning=c->phase==MOTOR_CAL_CONFIRM_RETURN;
    if(returning && c->original_load)
        (void)motor_seat_return_observe(&c->seat,at,value,c->original_load_mv);
    else (void)motor_seat_observe(&c->seat,at,value);
    bool plateau=motor_seat_loaded(&c->seat) && c->direction==MOTOR_CAL_CLOSE &&
                 c->homing.state==MOTOR_HOMING_SEEK_ENDPOINT;
    if(returning) {
        uint32_t mv=c->seat.plateau_mv,old=c->original_load_mv;
        uint32_t delta=mv>old ? mv-old : old-mv;
        plateau=plateau && c->original_load && delta<=100u && delta*100u<=old*10u;
    } else {
        /* Initial spring resistance is not closure; require twenty seconds of load first. */
        plateau=plateau && c->closing_load_seen && (uint32_t)(now-c->closing_load_at)>=200000u;
    }
    bool measuring_close=c->phase==MOTOR_CAL_MEASURE && c->direction==MOTOR_CAL_CLOSE;
    if(measuring_close && c->measured_close_kind) {
        plateau=plateau && c->measured_close_kind==2;
    }
    /* A spring-load valley/recovery is not a closed valve; confirm a stable load instead. */
    if(plateau && c->phase!=MOTOR_CAL_CONFIRM_BACKOFF) {
        c->have_first_low=true;c->first_low_on=c->last_sample_on;
        if(probe(c)) {c->phase=MOTOR_CAL_HOME;c->action.on_budget_ticks=c->homing.policy.max_move_ticks;}
        request_stop(c,now,returning ? STOP_LOAD_RETURN : STOP_LOAD_CANDIDATE);
        return true;
    }
    if(loaded && !low) {
        if(c->phase==MOTOR_CAL_CONFIRM_BACKOFF) {
            fail(c,MOTOR_CAL_REFERENCE_UNQUALIFIED);return true;
        }
        if(c->direction==MOTOR_CAL_CLOSE) {
            if(!c->closing_load_seen) {c->closing_load_seen=true;c->closing_load_at=now;}
        } else {
            c->have_first_low=true;c->first_low_on=c->last_sample_on;
            if(probe(c)) {c->phase=MOTOR_CAL_HOME;c->action.on_budget_ticks=c->homing.policy.max_move_ticks;}
            request_stop(c,now,c->phase==MOTOR_CAL_CONFIRM_RETURN ? STOP_CONFIRM_RETURN : STOP_CANDIDATE);
            return true;
        }
    }
    if (low) {
        bool pattern=c->pattern_high && c->intermediate_count>=2 && continuous &&
            (uint32_t)(c->pattern_last_at-c->intermediate_started_at)>=c->homing.policy.endpoint_dwell_ticks;
        /* A qualified low always stops calibration. Missing history cannot be
         * repaired with post-stop zeros or by driving on for a later dip. */
        if (!pattern || c->phase==MOTOR_CAL_CONFIRM_BACKOFF) {
            fail(c,MOTOR_CAL_REFERENCE_UNQUALIFIED); return true;
        }
        if (probe(c)) { c->phase=MOTOR_CAL_HOME; c->action.on_budget_ticks=c->homing.policy.max_move_ticks; }
        request_stop(c,now,c->phase==MOTOR_CAL_CONFIRM_RETURN ? STOP_CONFIRM_RETURN : STOP_CANDIDATE);
        return true;
    }
    if (c->homing.state!=MOTOR_HOMING_SEEK_ENDPOINT) {
        c->pattern_high=false; c->intermediate_count=0;
    } else if (value>=c->homing.policy.motion_min) {
        c->pattern_high=true; c->intermediate_count=0;
    } else if (value>c->homing.policy.endpoint_max && c->pattern_high) {
        if (!c->intermediate_count) c->intermediate_started_at=at;
        if (c->intermediate_count<UINT16_MAX) ++c->intermediate_count;
    }
    c->pattern_last_at=at; c->pattern_have_sample=true;
    return false;
}
bool motor_calibration_foreground_progress(struct motor_calibration *c,uint32_t token,
                                           uint32_t now,uint32_t on)
{
    if(!c || c->state!=MOTOR_CAL_ACTIVE || token!=c->action.token) return false;
    motor_calibration_tick(c,now);
    if(c->state!=MOTOR_CAL_ACTIVE || !count_ok(c,now,on)) return false;
    if(!motor_homing_foreground_progress(&c->homing,now)) { fail(c,MOTOR_CAL_HOMING); return false; }
    return phase_result(c,now);
}
bool motor_calibration_feedback(struct motor_calibration *c,uint32_t token,uint32_t now,
                                uint32_t measured_at,uint32_t value,bool valid,
                                uint32_t sample_on,uint32_t on)
{
    if(!c || c->state!=MOTOR_CAL_ACTIVE || token!=c->action.token) return false;
    motor_calibration_tick(c,now);
    if(c->state!=MOTOR_CAL_ACTIVE || !count_ok(c,now,on)) return false;
    if(sample_on>on || sample_on>(uint32_t)(measured_at-c->phase_started_at) ||
       on-sample_on>(uint32_t)(now-measured_at) ||
       (c->have_sample_on && (sample_on<c->last_sample_on ||
        sample_on-c->last_sample_on>(uint32_t)(measured_at-c->homing.last_sample_at)))) {
        fail(c,MOTOR_CAL_COUNTER); return false;
    }
    uint16_t previous_count=c->homing.endpoint_count;
    if(!motor_homing_feedback(&c->homing,now,measured_at,value,valid)) {
        fail(c,MOTOR_CAL_HOMING); return false;
    }
    c->last_signal=value;
    c->have_sample_on=true; c->last_sample_on=sample_on;
    if(!c->homing.endpoint_count) { c->have_first_low=false; c->first_low_on=0; }
    else if(!previous_count) { c->have_first_low=true; c->first_low_on=sample_on; }
    if (candidate_feedback(c,now,measured_at,value)) return c->state!=MOTOR_CAL_FAILED;
    return phase_result(c,now);
}
uint32_t motor_calibration_stop_dwell(const struct motor_calibration *c)
{
    return c->stop_reason==STOP_CANDIDATE || c->stop_reason==STOP_CONFIRM_BACKOFF ?
        c->confirmation.rest_ticks : c->stop_reason==STOP_LOAD_CANDIDATE ?
        c->confirmation.rest_ticks : c->policy.min_stop_ticks;
}
bool motor_calibration_confirm_stop(struct motor_calibration *c,uint32_t token,
                                    uint32_t now,uint32_t final_on,bool proven_stopped,
                                    bool qualified_endpoint)
{
    if(!c || c->state!=MOTOR_CAL_WAIT_STOP || token!=c->action.token) return false;
    if(!time_ok(c,now)) return false;
    if(!proven_stopped) return false;
    if(!expected_stop(c)) { fail(c,MOTOR_CAL_HOMING); return false; }
    if(!probe(c) && c->phase!=MOTOR_CAL_CONFIRM_BACKOFF && !qualified_endpoint) {
        fail(c,MOTOR_CAL_REFERENCE_UNQUALIFIED); return false;
    }
    if(!c->have_stop_proof) {
        if(!count_ok(c,now,final_on)) return false;
        c->last_now=now; c->stop_observed_at=now; c->stopped_on_ticks=final_on;
        c->have_stop_proof=true;
        return false;
    }
    if(final_on!=c->stopped_on_ticks) { fail(c,MOTOR_CAL_COUNTER); return false; }
    if((uint32_t)(now-c->stop_observed_at)<motor_calibration_stop_dwell(c)) return false;
    motor_calibration_tick(c,now);
    if(c->state!=MOTOR_CAL_WAIT_STOP || !count_ok(c,now,final_on)) return false;
    if (c->stop_reason==STOP_CANDIDATE || c->stop_reason==STOP_LOAD_CANDIDATE) {
        c->original_seat=false;
        c->original_load=c->stop_reason==STOP_LOAD_CANDIDATE;
        c->original_load_mv=c->original_load ? c->seat.plateau_mv : 0;
        c->original_phase=c->phase; c->original_direction=c->direction;
        c->original_token=token; c->original_first_low=c->first_low_on; c->original_final_on=final_on;
        c->original_intermediate_count=c->intermediate_count;
        if (!motor_homing_reset(&c->homing,true)) { fail(c,MOTOR_CAL_HOMING); return false; }
        return begin(c,MOTOR_CAL_CONFIRM_BACKOFF,opposite(c->direction),now);
    }
    if (c->phase==MOTOR_CAL_CONFIRM_BACKOFF) {
        if (c->stop_reason!=STOP_CONFIRM_BACKOFF) { fail(c,MOTOR_CAL_BAD_STATE); return false; }
        if (!motor_homing_reset(&c->homing,true)) { fail(c,MOTOR_CAL_HOMING); return false; }
        return begin(c,MOTOR_CAL_CONFIRM_RETURN,c->original_direction,now);
    }
    /* Credit only the original full leg. Confirmation keeps its new token,
     * guard and latest stop proof; no old live sequencer state is restored. */
    enum motor_calibration_phase completed_phase=c->phase;
    uint32_t travel=c->first_low_on, travel_final=final_on, travel_token=token;
    if (c->phase==MOTOR_CAL_CONFIRM_RETURN) {
        if (c->stop_reason!=(c->original_load ? STOP_LOAD_RETURN : STOP_CONFIRM_RETURN) || c->original_direction!=c->direction ||
            (c->original_phase!=MOTOR_CAL_HOME && c->original_phase!=MOTOR_CAL_MEASURE)) {
            fail(c,MOTOR_CAL_BAD_STATE); return false;
        }
        completed_phase=c->original_phase; travel=c->original_first_low;
        travel_final=c->original_final_on; travel_token=c->original_token;
        ++c->confirmed_candidates; c->confirmed_return_on=c->first_low_on;
        c->confirmed_return_mv=c->last_signal; c->confirmed_intermediate_count=c->intermediate_count;
    }
    if(c->retract_only) {
        if(completed_phase!=MOTOR_CAL_HOME || c->direction!=MOTOR_CAL_OPEN ||
           c->phase!=MOTOR_CAL_CONFIRM_RETURN || c->confirmed_candidates!=1) {
            fail(c,MOTOR_CAL_REFERENCE_UNQUALIFIED); return false;
        }
        c->action.direction=MOTOR_CAL_NONE; c->action.on_budget_ticks=0;
        c->state=MOTOR_CAL_RETRACTED;
        return true;
    }
    uint8_t direction=opposite(c->direction);
    enum motor_calibration_phase next=MOTOR_CAL_HOME;
    if(probe(c) && (c->stop_reason==STOP_PROBE_LOW || c->stop_reason==STOP_PROBE_INCONCLUSIVE)) {
        if(c->phase==MOTOR_CAL_PROBE_OPEN) { fail(c,MOTOR_CAL_NO_MOTION); return false; }
        next=MOTOR_CAL_PROBE_OPEN;
    } else if(completed_phase==MOTOR_CAL_HOME) next=MOTOR_CAL_MEASURE;
    else if(completed_phase==MOTOR_CAL_MEASURE) {
        if(!c->have_first_low || travel<c->policy.min_travel_ticks ||
           travel>c->policy.max_travel_ticks) { fail(c,MOTOR_CAL_TRAVEL_RANGE); return false; }
        unsigned index=c->direction-1u, repeat=c->measured_legs/2u;
        if(c->direction==MOTOR_CAL_CLOSE) {
            uint8_t kind=c->original_load ? 2u : 3u;
            if(c->measured_close_kind && kind!=c->measured_close_kind) {
                fail(c,MOTOR_CAL_REPEATABILITY); return false;
            }
            c->measured_close_kind=kind;
        }
        c->samples[index][repeat]=travel; c->final_on_samples[index][repeat]=travel_final;
        c->sample_tokens[index][repeat]=travel_token;
        if(repeat) {
            uint32_t first=c->samples[index][0];
            uint32_t difference=first>travel ? first-travel : travel-first;
            if(difference>c->policy.repeat_tolerance_ticks) { fail(c,MOTOR_CAL_REPEATABILITY); return false; }
        }
        if(++c->measured_legs==4) {
            c->candidate.opening_ticks=(uint32_t)(((uint64_t)c->samples[0][0]+c->samples[0][1])/2u);
            c->candidate.closing_ticks=(uint32_t)(((uint64_t)c->samples[1][0]+c->samples[1][1])/2u);
            c->candidate.final_endpoint_permille=c->direction==MOTOR_CAL_OPEN ? 1000 : 0;
            c->state=MOTOR_CAL_CANDIDATE;
            return true;
        }
        next=MOTOR_CAL_MEASURE;
    }
    if(!motor_homing_reset(&c->homing,true)) { fail(c,MOTOR_CAL_HOMING); return false; }
    return begin(c,next,direction,now);
}
void motor_calibration_abort(struct motor_calibration *c)
{ if(c) fail(c,MOTOR_CAL_ABORTED); }
bool motor_calibration_reset(struct motor_calibration *c,bool proven_stopped)
{
    if(!c || live(c) || !proven_stopped || c->homing.guard.armed) return false;
    if(!motor_homing_reset(&c->homing,true)) return false;
    uint32_t token=c->last_token;
    /* Homing is already reset. Preserve it and the configured confirmation
     * policy in place instead of copying either large object onto the stack. */
    size_t first=offsetof(struct motor_calibration,policy);
    for (size_t i=first;i<offsetof(struct motor_calibration,confirmation);++i) ((uint8_t *)c)[i]=0;
    first=offsetof(struct motor_calibration,intermediate_started_at);
    for (size_t i=first;i<sizeof(*c);++i) ((uint8_t *)c)[i]=0;
    c->last_token=token;
    return true;
}
