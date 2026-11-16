#include "valve_calibration.h"
#include <limits.h>
#ifndef W600_VALVE_CALIBRATION_TEST
#include "ciu32l071.h"
void W600_UART3_4_Handler(void);
#endif

/*100us units; movement qualification matches the driver and load detector. */
static const struct motor_calibration_policy policy={
    .homing={{450000,1000,1000,100,MOTOR_HOMING_MOTION_MIN_MV,100,4000,2000,200,11,2},
             {450000,1000,1000,100,MOTOR_HOMING_MOTION_MIN_MV,100,4000,2000,200,11,2}},
    /* A qualified probe continues as HOME without stopping or reversing. */
    .whole_attempt_ticks=3600000,.probe_on_ticks=15000,.probe_slack_ticks=300,
    .min_travel_ticks=20000,.max_travel_ticks=449000,.repeat_tolerance_ticks=50000,
    .stop_confirm_ticks=10000,.min_stop_ticks=1100
};
/* Existing voltage bands;20ms acquisitions plus1ms timestamp quantization.
 * One3.2s/10.2s-capped confirmation pair per candidate, never a retry loop. */
static const struct motor_calibration_confirmation confirmation={210,30000,32000,100000,102000,5000};
static bool live(const struct w600_valve_calibration *c)
{ return c->sequence.state==MOTOR_CAL_ACTIVE || c->sequence.state==MOTOR_CAL_WAIT_STOP; }
static bool stopped(const struct w600_motor_service *m)
{ return m->stop_confirmed && !m->owned && !m->board.owned && !m->cleanup_failed; }
static void calibration_stop(void *context)
{
    struct w600_valve_calibration *c=context;
    struct w600_motor_service *m=c->motor;
    if (m->phase>=W600_MOTOR_PREPARING && m->phase<=W600_MOTOR_RESTORING) {
#ifndef W600_VALVE_CALIBRATION_TEST
        if (__get_IPSR()) { w600_motor_service_abort(m); return; }
        /* Drain RX without allowing SysTick into the unfinished stop transition. */
        W600_UART3_4_Handler();
#endif
        (void)w600_motor_service_finish(m);
    }
}
static void publish(struct w600_valve_calibration *c)
{
    struct w600_endpoint *e=c->endpoint;
    e->calibration_busy=live(c) || c->calibration_preparing;
    e->calibration_error=(uint8_t)c->sequence.error;
    e->calibrated=c->calibration_applied;
    e->opening_known=c->position.valid;
    if (e->opening_known) e->opening_permille=(uint16_t)(c->position.position_ppm/1000u);
    if (c->motor->error || c->motor->cleanup_failed) e->control_enabled=false;
}
static void movement_fail(struct w600_valve_calibration *c,uint8_t error)
{
    if (!c->motion_error) c->motion_error=error;
    if (c->motor->phase>=W600_MOTOR_PREPARING && c->motor->phase<=W600_MOTOR_RESTORING)
        w600_motor_service_abort(c->motor);
    motor_position_invalidate(&c->position);
    c->calibration_applied=false; c->moving=false;
    c->endpoint->moving=false;
}
static uint16_t requested(struct w600_valve_calibration *c,uint32_t now)
{
    struct w600_endpoint *e=c->endpoint;
    e->control_error=c->motion_error;
    if (!e->mode) return 0;
    if (e->mode==2) return 1000;
    const unsigned required=W600_KNOWN_TARGET|W600_KNOWN_EXTERNAL_USE|W600_KNOWN_EXTERNAL_TEMP;
    /* Each room has its own sensor; a source change must never authorize local regulation. */
    bool available=(e->known&required)==required && e->external_use && c->external_seen &&
        (uint32_t)(now-c->external_at)<12u*60u*10000u; /* 100 us ticks; two HA heartbeats plus slack. */
    if (!available) { e->control_error=W600_CONTROL_TEMPERATURE; return 0; }
    int32_t actual=(int16_t)e->external_temp_raw;
    int32_t error=(int32_t)e->target_raw-actual;
    if (error<=0) return 0;
    /* Direct selected-sensor control:1C band, minimum20% while below target.
     * shortcut: initial proportional law; tune flow/comfort on mounted valves. */
    return error>=100 ? 1000 : error<20 ? 200 : (uint16_t)(error*10);
}
/* Keep the ordinary-move arithmetic frame off calibration's deeper stop path. */
static void __attribute__((noinline)) movement_poll(struct w600_valve_calibration *c,uint32_t now,bool sensor_busy)
{
    struct w600_motor_service *m=c->motor;
    struct w600_endpoint *e=c->endpoint;
    uint16_t target=requested(c,now);
    bool changed=c->control_updates!=e->control_updates;
    e->requested_opening=target;
    if (c->motion_error) { e->control_error=c->motion_error; return; }
    if (c->moving) {
        struct w600_motor_time t;
        if (!w600_motor_service_time(m,&t)) { movement_fail(c,W600_CONTROL_DRIVER); return; }
        uint32_t token=c->position.token;
        if (m->samples!=c->move_samples) {
            c->move_samples=m->samples;
            if (m->samples>1 && m->last_mv>=MOTOR_HOMING_MOTION_MIN_MV) c->saw_motion=true;
        }
        if (m->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED) {
            bool expected=(c->move_target==0 && m->direction==2) ||
                          (c->move_target==1000 && m->direction==1);
            if (!expected) { movement_fail(c,W600_CONTROL_ENDPOINT); return; }
            if (m->phase!=W600_MOTOR_DONE) return;
            motor_position_invalidate(&c->position);
            if (!motor_position_reference(&c->position,c->position.opening_ticks,
                    c->position.closing_ticks,c->move_target)) {
                movement_fail(c,W600_CONTROL_POSITION); return;
            }
        } else {
            uint32_t accounted=t.on_lower;
            uint32_t full=c->position.direction==MOTOR_POSITION_OPENING ?
                c->position.opening_ticks : c->position.closing_ticks;
            /* The3ms sensing lead plus polling/quantization can cross a full-stroke budget.
             * Clamp only the timed endpoint estimate within5ms; raw drive time/guards stay intact. */
            if (accounted>full && c->move_budget==full &&
                (c->move_target==0 || c->move_target==1000) && accounted-full<=50u) accounted=full;
#ifndef W600_VALVE_CALIBRATION_TEST
            /* Position math is foreground-only; keep UART reception alive during64-bit division. */
            uint32_t position_mask=__get_PRIMASK(); __enable_irq();
#endif
            bool observed=motor_position_observe(&c->position,token,accounted,m->phase==W600_MOTOR_DONE);
#ifndef W600_VALVE_CALIBRATION_TEST
            __set_PRIMASK(position_mask);
#endif
            if (!observed) {
                movement_fail(c,W600_CONTROL_POSITION); return;
            }
            uint16_t difference=target>c->move_target ? target-c->move_target : c->move_target-target;
            bool interrupted=target!=c->move_target && (changed || !target || difference>=50);
            if (target==c->move_target) c->control_updates=e->control_updates;
            if (!c->move_stopping && (t.on_lower>=c->move_budget || interrupted) &&
                m->phase>=W600_MOTOR_PREPARING && m->phase<=W600_MOTOR_RESTORING) {
                c->move_stopping=true;
                c->interrupted_move=interrupted;
                if (!w600_motor_service_finish(m)) { movement_fail(c,W600_CONTROL_DRIVER); return; }
            }
            if (m->phase!=W600_MOTOR_DONE) return;
            if (!c->saw_motion && !(c->interrupted_move && t.on_lower<200u)) {
                movement_fail(c,W600_CONTROL_NO_MOTION); return;
            }
        }
        c->moving=false; e->moving=false; c->have_move=true; c->last_move_at=now;
        return;
    }
    if (!e->mode_known || live(c) || e->calibration_pending) return;
    if (!c->calibration_applied || !c->position.valid) {
        e->control_error=W600_CONTROL_NEEDS_CALIBRATION; return;
    }
    if (sensor_busy) return;
    uint16_t current=(uint16_t)(c->position.position_ppm/1000u);
    uint16_t difference=target>current ? target-current : current-target;
    if (!difference) { c->control_updates=e->control_updates; return; }
    if (!changed && target && target!=1000 && difference<50) return;
    if (!changed && target && e->mode!=2 && c->have_move &&
        (uint32_t)(now-c->last_move_at)<600000u) return;
    if (m->phase==W600_MOTOR_DONE && !w600_motor_service_rearm(m)) return;
    if (m->phase!=W600_MOTOR_IDLE) return;
    if (c->position.token==UINT32_MAX) { movement_fail(c,W600_CONTROL_TOKEN); return; }
    struct motor_position_plan plan;
    if (!motor_position_plan(&c->position,c->position.token+1u,target,&plan)) {
        movement_fail(c,W600_CONTROL_POSITION); return;
    }
    if (!plan.direction) return;
    c->move_budget=plan.required_ticks<200u ? 200u : plan.required_ticks;
    c->move_target=target; c->move_stopping=false; c->interrupted_move=false;
    c->saw_motion=false; c->move_samples=0;
    c->moving=true; e->moving=true;
    c->control_updates=e->control_updates;
    if (!w600_motor_service_begin(m,plan.direction)) movement_fail(c,W600_CONTROL_DRIVER);
}
bool w600_valve_calibration_init(struct w600_valve_calibration *c,
                                struct w600_motor_service *m,struct w600_endpoint *e)
{
    if (!c || !m || !e || m->phase!=W600_MOTOR_IDLE || m->error || !stopped(m)) return false;
    *c=(struct w600_valve_calibration){.motor=m,.endpoint=e};
    motor_position_init(&c->position);
    if (!motor_calibration_init(&c->sequence,calibration_stop,c)) return false;
    c->sequence.confirmation=confirmation;
    /* MCU owns the reported mode; never wait for an unchanged radio Off write. */
    e->mode=0; e->mode_known=true;
    e->control_enabled=true;
    publish(c);
    return true;
}
void w600_valve_calibration_tick(struct w600_valve_calibration *c,uint32_t now)
{
    if (c && c->motor) {
        /* SysTick is millisecond-quantized; foreground timestamps include fractions. */
        if ((uint32_t)(c->sequence.last_now-now)<10u) now=c->sequence.last_now;
        motor_calibration_tick(&c->sequence,now);
    }
}
void w600_valve_calibration_feedback(struct w600_valve_calibration *c)
{
    struct w600_motor_service *m=c->motor;
    if (c->sequence.state!=MOTOR_CAL_ACTIVE || c->started_token!=c->sequence.action.token ||
        m->samples==c->samples) return;
    if (c->endpoint->cancel_calibration || m->error || m->cleanup_failed ||
        m->samples!=c->samples+1u || m->direction!=c->sequence.action.direction ||
        !m->sample.valid || m->sample.error || m->sample.sequence!=m->samples) {
        motor_calibration_abort(&c->sequence); return;
    }
    struct w600_motor_time t;
    if (!w600_motor_service_time(m,&t)) { motor_calibration_abort(&c->sequence); return; }
    c->samples=m->samples;
    /* The first batch is stationary; only pre-stop moving samples can qualify. */
    if (m->samples>1)
        (void)motor_calibration_feedback(&c->sequence,c->sequence.action.token,t.now_ticks,
            m->sample_at_ticks,m->last_mv,true,t.sample_on_lower,t.on_lower);
    if (c->sequence.state==MOTOR_CAL_WAIT_STOP) {
        c->endpoint_qualified=!m->error && !m->homing.error && !m->homing.guard.error &&
            (m->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED ||
             (c->sequence.confirmation.max_sample_gap_ticks && m->requested_stop &&
              m->homing.state==MOTOR_HOMING_SEEK_ENDPOINT &&
              (m->last_mv<=policy.homing[0].endpoint_max ||
               (m->direction==1 && c->sequence.slowdown.candidate) ||
               (m->direction==2 && motor_seat_loaded(&c->sequence.seat)))));
    }
}
void w600_valve_calibration_poll(struct w600_valve_calibration *c,uint32_t now,bool sensor_busy)
{
    struct w600_motor_service *m=c->motor;
    struct w600_endpoint *e=c->endpoint;
    if (live(c) && c->sequence.retract_only && !c->calibration_preparing && (!e->mode_known || e->mode!=0))
        motor_calibration_abort(&c->sequence);
    /* Receipt stays fresh during calibration too, not when later first selected. */
    if ((e->known&W600_KNOWN_EXTERNAL_TEMP) &&
        (!c->external_seen || c->external_updates!=e->external_updates)) {
        c->external_seen=true; c->external_updates=e->external_updates; c->external_at=now;
    }
    /* Only a new sensor record may restore availability after expiry. */
    if (c->external_seen && (uint32_t)(now-c->external_at)>=12u*60u*10000u)
        e->known&=~W600_KNOWN_EXTERNAL_TEMP;
    if (e->cancel_calibration) {
        e->cancel_calibration=false; e->calibration_pending=false;
        c->calibration_preparing=false;
        if (live(c)) motor_calibration_abort(&c->sequence);
        motor_position_invalidate(&c->position);
        c->calibration_applied=false;
    }
    if (m->error || m->cleanup_failed) {
        c->calibration_preparing=false;
        if (live(c)) motor_calibration_abort(&c->sequence);
        e->calibration_pending=false;
        motor_position_invalidate(&c->position);
        c->calibration_applied=false; c->moving=false; e->moving=false;
        e->control_error=W600_CONTROL_DRIVER;
        publish(c); return;
    }
    if (c->calibration_preparing && c->sequence.state==MOTOR_CAL_FAILED)
        c->calibration_preparing=false;
    if (c->calibration_preparing && c->sequence.state==MOTOR_CAL_RETRACTED &&
        !sensor_busy && stopped(m) && m->phase==W600_MOTOR_IDLE) {
        uint32_t began=c->sequence.attempt_started_at;
        c->calibration_preparing=false;
        if (!motor_calibration_reset(&c->sequence,true) ||
            !motor_calibration_start(&c->sequence,now,&policy,true))
            motor_calibration_abort(&c->sequence);
        else c->sequence.attempt_started_at=began; /* Preparation shares the six-minute limit. */
        c->started_token=0;
    }
    if (e->calibration_pending && !sensor_busy && stopped(m) && !live(c)) {
        if (m->phase==W600_MOTOR_DONE && !w600_motor_service_rearm(m)) return;
        if (m->phase!=W600_MOTOR_IDLE) return;
        e->calibration_pending=false;
        motor_position_invalidate(&c->position);
        c->calibration_applied=false; c->motion_error=0; e->control_error=0;
        if (c->sequence.state!=MOTOR_CAL_IDLE && !motor_calibration_reset(&c->sequence,true)) {
            e->control_enabled=false; publish(c); return;
        }
        c->started_token=0;
        c->calibration_preparing=c->sequence.confirmation.max_sample_gap_ticks!=0;
        if (c->calibration_preparing)
            (void)motor_calibration_retract_start(&c->sequence,now,&policy,true);
        else (void)motor_calibration_start(&c->sequence,now,&policy,true);
    }
    if (!live(c)) { movement_poll(c,now,sensor_busy); publish(c); return; }
    motor_calibration_tick(&c->sequence,now);
    if (c->sequence.state==MOTOR_CAL_ACTIVE) {
        uint32_t token=c->sequence.action.token;
        if (token!=c->started_token) {
            if (sensor_busy || m->phase!=W600_MOTOR_IDLE) {
                motor_calibration_abort(&c->sequence);
            } else {
                c->started_token=token; c->samples=0; c->final_on=0;
                c->endpoint_qualified=false;
                if (!w600_motor_service_begin_bounded(m,c->sequence.action.direction,
                        c->sequence.homing.policy.max_move_ticks/10u))
                    motor_calibration_abort(&c->sequence);
            }
        }
        struct w600_motor_time t;
        if (c->sequence.state==MOTOR_CAL_ACTIVE && w600_motor_service_time(m,&t)) {
            if (c->sequence.state==MOTOR_CAL_ACTIVE)
                (void)motor_calibration_foreground_progress(&c->sequence,token,t.now_ticks,t.on_lower);
            if (c->sequence.state==MOTOR_CAL_WAIT_STOP) {
                /* Immediate feedback may already have saved candidate proof. */
                if (m->homing.state==MOTOR_HOMING_ENDPOINT_OBSERVED &&
                    !m->homing.error && !m->homing.guard.error) c->endpoint_qualified=true;
            } else if (m->phase==W600_MOTOR_DONE || m->phase==W600_MOTOR_CLEANUP) {
                /* Driver stopped without the sequencer qualifying this action. */
                motor_calibration_abort(&c->sequence);
            }
        }
    }
    if (c->sequence.state==MOTOR_CAL_WAIT_STOP && m->phase==W600_MOTOR_DONE && stopped(m)) {
        struct w600_motor_time t;
        if (!w600_motor_service_time(m,&t)) motor_calibration_abort(&c->sequence);
        else {
            c->final_on=t.on_lower;
            bool ready=c->sequence.have_stop_proof &&
                (uint32_t)(t.now_ticks-c->sequence.stop_observed_at)>=motor_calibration_stop_dwell(&c->sequence);
            /* Rearm before advancing: a new action's leases start immediately. */
            if (!ready || w600_motor_service_rearm(m)) {
#ifndef W600_VALVE_CALIBRATION_TEST
                W600_UART3_4_Handler();
#endif
                (void)motor_calibration_confirm_stop(&c->sequence,c->sequence.action.token,
                    t.now_ticks,c->final_on,true,c->endpoint_qualified);
            }
        }
    }
    if (c->sequence.state==MOTOR_CAL_CANDIDATE && !c->calibration_applied) {
        const struct motor_calibration_candidate *v=&c->sequence.candidate;
        c->calibration_applied=motor_position_reference(&c->position,v->opening_ticks,v->closing_ticks,
                                      v->final_endpoint_permille);
        if (!c->calibration_applied)
            motor_calibration_abort(&c->sequence);
    }
    publish(c);
}

void w600_valve_mounting_poll(struct w600_valve_calibration *c,uint32_t now,
                             bool sensor_busy,volatile struct w600_mount_report *r)
{
    struct w600_endpoint *e=c->endpoint;
    struct w600_motor_service *m=c->motor;
    uint32_t command=r->command;
    if (command) {
        r->command=0;
        bool returning=r->used==1 && r->state==W600_MOUNT_SAMPLED &&
            m->direction==1 && m->requested_stop && command==W600_MOUNT_CLOSE_SAMPLE_COMMAND;
        if (!r->used || returning) {
            ++r->used; r->state=W600_MOUNT_REJECTED; r->error=0;
            if (command==W600_MOUNT_RETRACT_COMMAND && !sensor_busy && e->control_enabled &&
                e->mode_known && e->mode==0 && !e->calibration_pending && !e->cancel_calibration &&
                !live(c) && !c->moving && !c->position.active && !c->calibration_applied &&
                !c->position.valid && c->sequence.state==MOTOR_CAL_IDLE && stopped(m) &&
                m->phase==W600_MOTOR_IDLE && !m->error &&
                motor_calibration_retract_start(&c->sequence,now,&policy,true)) {
                c->started_token=0; r->state=W600_MOUNT_ACTIVE;
                publish(c);
            }
            if ((command==W600_MOUNT_CLOSE_SAMPLE_COMMAND || command==W600_MOUNT_OPEN_SAMPLE_COMMAND) &&
                !sensor_busy && e->control_enabled &&
                e->mode_known && e->mode==0 && !e->calibration_pending && !e->cancel_calibration &&
                !live(c) && !c->moving && !c->position.active && !c->calibration_applied &&
                !c->position.valid && c->sequence.state==MOTOR_CAL_IDLE && stopped(m) &&
                (m->phase==W600_MOTOR_IDLE || (returning && m->phase==W600_MOTOR_DONE)) && !m->error) {
                /* The return needs a second explicit request after a clean stop/rest. */
                if (returning && !w600_motor_service_rearm(m)) return;
                if (w600_motor_service_begin_bounded(m,command==W600_MOUNT_OPEN_SAMPLE_COMMAND ? 1 : 2,returning ? 10200 : 5200)) {
                    r->started_at=now; r->state=W600_MOUNT_SAMPLING; e->moving=true;
                } else r->error=m->error;
            }
        }
    }
    if (r->state==W600_MOUNT_ACTIVE) {
        if (c->sequence.state==MOTOR_CAL_RETRACTED) r->state=W600_MOUNT_RETRACTED;
        else if (!live(c)) r->state=W600_MOUNT_FAILED;
        r->error=c->sequence.error;
    }
    if (r->state==W600_MOUNT_SAMPLING) {
        bool active=m->phase>=W600_MOTOR_PREPARING && m->phase<=W600_MOTOR_RESTORING;
        if (active && (sensor_busy || !e->control_enabled || !e->mode_known || e->mode!=0 ||
                       e->cancel_calibration || e->calibration_pending || m->error))
            w600_motor_service_abort(m);
        else if (active && (uint32_t)(now-r->started_at)>=(r->used==2 ? 100000u : 50000u))
            (void)w600_motor_service_finish(m);
        if ((m->phase==W600_MOTOR_DONE || m->phase==W600_MOTOR_FAILED) && stopped(m)) {
            r->error=m->error; r->state=m->error ? W600_MOUNT_FAILED : W600_MOUNT_SAMPLED;
            e->moving=false;
        } else if (m->cleanup_failed) {r->error=m->error;r->state=W600_MOUNT_FAILED;}
    }
}
