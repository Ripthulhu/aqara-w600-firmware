#include "motor_slowdown.h"
#include "motor_homing.h"

void motor_slowdown_reset(struct motor_slowdown *s)
{ if(s) *s=(struct motor_slowdown){0}; }

bool motor_slowdown_observe(struct motor_slowdown *s,uint32_t at,uint32_t mv)
{
    if(!s) return false;
    if(s->candidate) return true;
    /* Missing/duplicate/backward samples cannot bridge a slowdown dwell. */
    if(mv>4000 || (s->have_sample && (!(uint32_t)(at-s->last_at) ||
                                     (uint32_t)(at-s->last_at)>210))) {
        motor_slowdown_reset(s);
        return false;
    }
    s->last_at=at;s->have_sample=true;
    if(mv>=MOTOR_HOMING_MOTION_MIN_MV) {
        if(!s->high_count) {s->high_at=at;s->high_min_mv=mv;}
        if(mv<s->high_min_mv) s->high_min_mv=mv;
        ++s->high_count;
        if(s->high_count>=11 && (uint32_t)(at-s->high_at)>=2000) {
            /* A startup spike must not inflate the sustained running baseline. */
            if(s->high_min_mv>s->baseline_mv) s->baseline_mv=s->high_min_mv;
            s->high_count=0;s->high_min_mv=0;
        }
    } else { s->high_count=0;s->high_min_mv=0; }
    /* A sustained relative drop also catches slipping with nonzero BEMF. */
    if(s->baseline_mv && mv*100u<=s->baseline_mv*85u) {
        if(!s->low) {s->low=true;s->low_at=at;}
        if((uint32_t)(at-s->low_at)>=2000) s->candidate=true;
    } else s->low=false;
    return s->candidate;
}

void motor_seat_reset(struct motor_seat *s)
{ if(s) *s=(struct motor_seat){0}; }

static bool seat_observe(struct motor_seat *s,uint32_t at,uint32_t mv,uint32_t expected_mv)
{
    if(!s) return false;
    if(mv>4000 || (s->have_sample && (!(uint32_t)(at-s->last_at) ||
                                    (uint32_t)(at-s->last_at)>210))) {
        motor_seat_reset(s);return false;
    }
    s->last_at=at;s->have_sample=true;
    if(!s->count) {s->window_at=at;s->window_min_mv=mv;}
    if(mv<s->window_min_mv) s->window_min_mv=mv;
    s->sum+=mv;
    if(++s->count<10 || (uint32_t)(at-s->window_at)<1800) return s->candidate;
    uint32_t mean=s->sum/s->count;
    if(s->window_min_mv>=MOTOR_HOMING_MOTION_MIN_MV && s->window_min_mv>s->baseline_mv)
        s->baseline_mv=s->window_min_mv;
    s->count=0;s->sum=0;
    if(!s->baseline_mv) return false;
    uint32_t difference=mean>expected_mv ? mean-expected_mv : expected_mv-mean;
    uint32_t delta=mean>s->plateau_mv ? mean-s->plateau_mv : s->plateau_mv-mean;
    /* Keep a qualified load band across threshold noise, without following drift. */
    bool load=expected_mv ? difference<=100u && difference*100u<=expected_mv*10u :
                           mean*100u<=s->baseline_mv*75u || (s->plateau_count && delta<=50u);
    if(load) {
        if(!s->plateau_count || delta>50u) {s->plateau_mv=mean;s->plateau_count=1;}
        else if(s->plateau_count<25u) ++s->plateau_count;
    } else s->plateau_count=0;
    bool loaded=mean*100u<=s->baseline_mv*85u;
    if(s->below_count<2) {
        if(!loaded) {s->below_count=0;s->trough_mv=0;return false;}
        if(!s->below_count || mean<s->trough_mv) s->trough_mv=mean;
        ++s->below_count;return false;
    }
    if(mean<s->trough_mv) {s->trough_mv=mean;s->recovery_count=0;}
    /* Window means reject commutation spikes; one dip or a loaded plateau is insufficient. */
    if(mean>=s->trough_mv+120u && mean*100u>=s->trough_mv*112u) {
        if(++s->recovery_count>=2) s->candidate=true;
    } else s->recovery_count=0;
    return s->candidate;
}

bool motor_seat_loaded(const struct motor_seat *s)
{ return s && s->have_sample && s->plateau_count>=25u; }

bool motor_seat_observe(struct motor_seat *s,uint32_t at,uint32_t mv)
{ return seat_observe(s,at,mv,0); }

bool motor_seat_return_observe(struct motor_seat *s,uint32_t at,uint32_t mv,uint32_t expected_mv)
{ return expected_mv>=100u && expected_mv<=4000u && seat_observe(s,at,mv,expected_mv); }
