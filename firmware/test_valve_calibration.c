#define W600_VALVE_CALIBRATION_TEST 1
#include "valve_calibration.c"
#include <assert.h>
#include <stdio.h>

/* Driver-boundary simulation: tests production controller/endpoint/sequencer,
 * not MMIO timing, RF delivery, gearbox force or the physical end-stop signal. */
static uint32_t now, on, next_sample, drive_start;
static unsigned starts, finishes;
static bool no_feedback;
static bool still_motor;
static uint32_t travel_ticks;
static uint32_t startup_ticks;
static uint32_t running_mv;
static bool patterned;
static bool closed_start;
static unsigned caps[32],directions[32];
static void driver_stop(void *arg)
{
    struct w600_motor_service *m=arg;
    m->stop_confirmed=1; m->phase=W600_MOTOR_CLEANUP;
}
bool w600_motor_service_begin(struct w600_motor_service *m,unsigned direction)
{ return w600_motor_service_begin_bounded(m,direction,45000); }
bool w600_motor_service_begin_bounded(struct w600_motor_service *m,unsigned direction,uint32_t limit_ms)
{
    assert(m->phase==W600_MOTOR_IDLE && !m->error);
    assert(direction==1 || direction==2);
    struct motor_homing_policy h=policy.homing[direction-1];h.max_move_ticks=limit_ms*10u;
    assert(motor_homing_start(&m->homing,now,&h));
    assert(starts<32);caps[starts]=limit_ms;directions[starts]=direction;m->limit_ms=limit_ms;
    on=0; drive_start=now; next_sample=now+200; ++starts;
    m->direction=direction; m->phase=W600_MOTOR_DRIVING;
    m->samples=0; m->requested_stop=0; m->time_valid=true; m->stop_confirmed=0;
    return true;
}
bool w600_motor_service_finish(struct w600_motor_service *m)
{
    assert(m->phase>=W600_MOTOR_PREPARING && m->phase<=W600_MOTOR_RESTORING);
    motor_guard_disarm(&m->homing.guard);
    m->requested_stop=1; ++finishes;
    return true;
}
void w600_motor_service_abort(struct w600_motor_service *m)
{ driver_stop(m); m->error=W600_MOTOR_ABORTED; m->phase=W600_MOTOR_FAILED; }
bool w600_motor_service_rearm(struct w600_motor_service *m)
{
    if (m->phase!=W600_MOTOR_DONE || m->error || now-m->completed_ms<1010) return false;
    assert(motor_homing_reset(&m->homing,true));
    m->phase=W600_MOTOR_IDLE; m->time_valid=false; m->samples=0;
    return true;
}
bool w600_motor_service_time(const struct w600_motor_service *m,struct w600_motor_time *t)
{
    if (!m->time_valid || m->error) { *t=(struct w600_motor_time){0}; return false; }
    *t=(struct w600_motor_time){now,on,on,m->sample_on_lower,m->sample_on_upper};
    return true;
}
struct fixture {
    struct w600_valve_calibration control;
    struct w600_motor_service motor;
    struct w600_endpoint endpoint;
};
static void init(struct fixture *f)
{
    *f=(struct fixture){0}; now=on=starts=finishes=0; no_feedback=false;
    still_motor=false; travel_ticks=25000; startup_ticks=0; running_mv=1500;
    patterned=false; closed_start=false;
    assert(motor_homing_init(&f->motor.homing,driver_stop,&f->motor));
    f->motor.phase=W600_MOTOR_IDLE;
    assert(w600_valve_calibration_init(&f->control,&f->motor,&f->endpoint));
    assert(f->control.sequence.confirmation.max_sample_gap_ticks==210);
    /* Retain legacy2-low controller regressions explicitly; new tests below
     * enable the production confirmation configuration. Ordinary moves use2-low. */
    f->control.sequence.confirmation=(struct motor_calibration_confirmation){0};
    assert(f->endpoint.control_enabled && !f->endpoint.calibrated && !starts);
    assert(!f->endpoint.mode && f->endpoint.mode_known && !f->endpoint.known);
}
static uint8_t command(struct fixture *f,uint8_t seq,uint8_t id,uint8_t value)
{
    struct w600_frame in={.kind=2,.sequence=seq,.length=4,.payload={7,id,0,value}},out;
    assert(w600_endpoint_process(&f->endpoint,&in,&out));
    assert(out.length==3 && out.kind==3 && out.sequence==seq);
    return out.payload[0];
}
static void step(struct fixture *f,bool poll)
{
    now+=10;
    struct w600_motor_service *m=&f->motor;
    if (m->phase==W600_MOTOR_CLEANUP) {
        m->phase=W600_MOTOR_DONE; m->completed_ms=now;
    } else if (m->phase==W600_MOTOR_DRIVING) {
        on+=10;
        motor_homing_tick(&m->homing,now);
        if (m->homing.error) { m->error=W600_MOTOR_FEEDBACK; driver_stop(m); }
        if (poll && !m->error && m->phase==W600_MOTOR_DRIVING) {
            assert(motor_homing_foreground_progress(&m->homing,now));
            if (!no_feedback && (uint32_t)(now-next_sample)<UINT32_C(0x80000000)) {
                next_sample+=200; ++m->samples;
                m->sample_at_ticks=now; m->sample_on_lower=m->sample_on_upper=on;
                m->last_mv=!still_motor && now-drive_start<travel_ticks ?
                    (now-drive_start<startup_ticks ? MOTOR_HOMING_MOTION_MIN_MV-200 : running_mv) : 0;
                if (patterned) {
                    uint32_t goal=f->control.sequence.phase==MOTOR_CAL_CONFIRM_RETURN ? 19000 : travel_ticks;
                    uint32_t elapsed=now-drive_start;
                    m->last_mv=f->control.sequence.phase==MOTOR_CAL_CONFIRM_BACKOFF ? 1300 :
                        elapsed>=goal ? 1 : elapsed>=goal-200 ? 300 : elapsed>=goal-400 ? 500 : 1300;
                }
                if (closed_start && f->control.sequence.phase==MOTOR_CAL_PROBE_CLOSE) m->last_mv=0;
                m->sample.valid=true;m->sample.error=0;m->sample.sequence=m->samples;
                if (m->samples>1)
                    assert(motor_homing_feedback(&m->homing,now,now,m->last_mv,true));
            }
        }
    }
    if (poll) w600_valve_calibration_feedback(&f->control);
    w600_valve_calibration_tick(&f->control,now);
    if (poll) w600_valve_calibration_poll(&f->control,now,false);
    if (f->control.calibration_preparing && f->control.sequence.state==MOTOR_CAL_RETRACTED)
        closed_start=false; /* The simulated preparation physically reached the open end. */
}
static void finish_calibration(struct fixture *f)
{
    for (unsigned i=0;i<25000 && !f->endpoint.calibrated && !f->control.sequence.error;++i) step(f,true);
    assert(f->control.sequence.state==MOTOR_CAL_CANDIDATE);
    assert(f->endpoint.calibrated && f->endpoint.opening_known && !f->endpoint.calibration_busy);
    assert(f->endpoint.opening_permille==0 && starts==5);
    assert(f->control.position.opening_ticks==25000 && f->control.position.closing_ticks==25000);
    assert(f->motor.stop_confirmed && f->motor.phase==W600_MOTOR_IDLE);
}
static void reference(struct fixture *f,uint16_t at)
{
    init(f); travel_ticks=1000000;
    assert(motor_position_reference(&f->control.position,40000,20000,at));
    f->control.calibration_applied=true;
    publish(&f->control); /* Publish the seeded fixture without starting normal control. */
    assert(f->endpoint.calibrated && !starts);
}
static void word(struct fixture *f,uint8_t id,uint16_t value)
{
    struct w600_frame in={.kind=2,.sequence=(uint8_t)(f->endpoint.last_request_sequence+1),
        .length=5,.payload={7,id,4,(uint8_t)(value>>8),(uint8_t)value}},out;
    assert(w600_endpoint_process(&f->endpoint,&in,&out) && out.payload[0]==0);
}
static void settled(struct fixture *f)
{
    for(unsigned i=0;i<5000;++i) {
        step(f,true);
        if (f->control.motion_error || (!f->control.moving && f->control.have_move &&
            f->endpoint.opening_known && f->endpoint.opening_permille==f->endpoint.requested_opening)) return;
    }
    assert(!"normal move did not stop");
}
static void startup_off(void)
{
    struct fixture f;init(&f);
    for (unsigned i=0;i<4;++i) {
        w600_valve_calibration_poll(&f.control,now,(i&1)==0);
        assert(f.endpoint.control_error==W600_CONTROL_NEEDS_CALIBRATION && !starts);
    }
    assert(command(&f,1,9,1)==0);
    w600_valve_calibration_poll(&f.control,now,false);
    assert(starts==1 && f.endpoint.calibration_busy && !f.endpoint.control_error);
}
static void unknown_sensor_selection(void)
{
    struct fixture f;reference(&f,0);
    word(&f,0,2200);
    assert(w600_endpoint_local_temperature(&f.endpoint,100,true));
    assert(command(&f,60,3,1)==0);
    step(&f,true);
    assert(!starts && !f.endpoint.requested_opening &&
           f.endpoint.control_error==W600_CONTROL_TEMPERATURE);
    struct w600_frame select={.kind=2,.sequence=61,.length=4,.payload={7,4,1,0}},reply;
    assert(w600_endpoint_process(&f.endpoint,&select,&reply) && !reply.payload[0]);
    step(&f,true);
    assert(!starts && !f.endpoint.requested_opening &&
           f.endpoint.control_error==W600_CONTROL_TEMPERATURE);
    select.sequence=62;select.payload[3]=1;
    assert(w600_endpoint_process(&f.endpoint,&select,&reply) && !reply.payload[0]);
    step(&f,true);
    assert(!starts && f.endpoint.control_error==W600_CONTROL_TEMPERATURE);
    word(&f,5,2100);settled(&f);
    assert(f.endpoint.opening_permille==1000 && !f.endpoint.control_error);
    select.sequence=63;select.payload[3]=0;
    assert(w600_endpoint_process(&f.endpoint,&select,&reply) && !reply.payload[0]);
    settled(&f);
    assert(!f.endpoint.opening_permille && !f.endpoint.requested_opening &&
           f.endpoint.control_error==W600_CONTROL_TEMPERATURE && f.endpoint.calibrated);
    unsigned old=starts;
    assert(w600_endpoint_local_temperature(&f.endpoint,50,true));
    for(unsigned i=0;i<10;++i) step(&f,true);
    assert(starts==old && f.endpoint.control_error==W600_CONTROL_TEMPERATURE);
    word(&f,5,2100);settled(&f);
    assert(f.endpoint.opening_permille==1000 && !f.endpoint.control_error);
}
static void normal_controls(void)
{
    struct fixture f; reference(&f,1000);
    word(&f,0,2200); word(&f,5,2100);
    assert(!f.endpoint.mode && !starts); /* Targets/temperatures never authorize opening. */
    assert(command(&f,30,3,0)==0); settled(&f);
    assert(f.endpoint.opening_permille==0 && f.endpoint.calibrated && !f.control.motion_error);
    assert(command(&f,31,3,1)==0); settled(&f);
    assert(f.endpoint.opening_permille==1000 && !f.control.motion_error && starts==2);
    word(&f,5,2150); word(&f,0,2201); settled(&f);
    assert(f.endpoint.requested_opening==510 && f.endpoint.opening_permille==510 && starts==3);
    unsigned old=starts;
    assert(command(&f,32,3,1)==0);
    for(unsigned i=0;i<10;++i) step(&f,true);
    assert(starts==old); /* Duplicate desired state, no repeated motor work. */
    word(&f,0,2202); settled(&f);
    assert(f.endpoint.opening_permille==520 && starts==4); /* Deliberate small target change. */
    assert(f.control.position.opening_ticks==40000 && f.control.position.closing_ticks==20000);

    reference(&f,0); word(&f,0,2200); word(&f,5,2100); assert(command(&f,33,3,1)==0);
    step(&f,true); assert(f.endpoint.moving);
    for(unsigned i=0;i<100;++i) step(&f,true);
    assert(command(&f,34,3,0)==0); settled(&f);
    for(unsigned i=0;i<150;++i) step(&f,true);
    settled(&f);
    assert(!f.endpoint.mode && f.endpoint.opening_permille==0 && !f.control.motion_error);
    old=starts; word(&f,5,1500); word(&f,0,2500);
    for(unsigned i=0;i<100;++i) step(&f,true);
    assert(starts==old && !f.endpoint.requested_opening); /* Off cannot be undone by sensor/target refresh. */

    reference(&f,1000); word(&f,0,2200); word(&f,5,2100); assert(command(&f,35,3,1)==0);
    step(&f,true); assert(!f.endpoint.moving);
    assert(w600_endpoint_local_temperature(&f.endpoint,100,true));
    now=f.control.external_at+12u*60u*10000u-1u; /* Controller clock is in 100 us ticks. */
    w600_valve_calibration_poll(&f.control,now,false);
    assert(!f.endpoint.control_error && f.endpoint.requested_opening==1000 && !f.endpoint.moving);
    ++now;
    w600_valve_calibration_poll(&f.control,now,false);
    assert(f.endpoint.control_error==W600_CONTROL_TEMPERATURE && !f.endpoint.requested_opening);
    assert(f.endpoint.moving); /* Stale external input closes, never falls back to hot/cold local. */
    settled(&f); assert(f.endpoint.opening_permille==0);
    word(&f,5,2100); step(&f,true);
    assert(f.endpoint.control_error==0 && f.endpoint.requested_opening==1000);

    reference(&f,0); word(&f,0,2200); word(&f,5,2100); assert(command(&f,36,3,1)==0);
    still_motor=true; settled(&f);
    assert(f.control.motion_error==W600_CONTROL_NO_MOTION && !f.endpoint.calibrated && !f.endpoint.opening_known);
    old=starts; for(unsigned i=0;i<1000;++i) step(&f,true);
    assert(starts==old); /* No blind retry after mechanical failure. */

    reference(&f,1000); word(&f,0,2200); word(&f,5,2300); assert(command(&f,40,3,1)==0);
    settled(&f); assert(!f.endpoint.opening_permille && !f.control.motion_error);
    reference(&f,0);
    struct w600_frame force={.kind=2,.sequence=41,.length=4,.payload={7,0x1d,1,2}},reply;
    assert(w600_endpoint_process(&f.endpoint,&force,&reply) && !reply.payload[0]);
    settled(&f); assert(f.endpoint.opening_permille==1000 && !f.endpoint.known);
    reference(&f,0); word(&f,5,2000); assert(command(&f,42,3,1)==0);
    for(unsigned i=0;i<8;++i) {
        word(&f,0,(i&1) ? 2020 : 2050); settled(&f);
        uint16_t target=(i&1) ? 200 : 500;
        assert(f.endpoint.opening_permille>=target-1 && f.endpoint.opening_permille<=target+1);
        assert(!f.control.motion_error && f.endpoint.calibrated);
    }
    assert(f.control.position.opening_ticks==40000 && f.control.position.closing_ticks==20000);
    reference(&f,0); f.control.position.token=UINT32_MAX;
    word(&f,0,2200); word(&f,5,2100); assert(command(&f,43,3,1)==0);
    step(&f,true); assert(f.control.motion_error==W600_CONTROL_TOKEN && !starts);

    reference(&f,0); word(&f,0,2200); word(&f,5,2150); assert(command(&f,37,3,1)==0);
    travel_ticks=4000; settled(&f);
    assert(f.control.motion_error==W600_CONTROL_ENDPOINT && !f.endpoint.calibrated);
    reference(&f,0); word(&f,0,2200); word(&f,5,2100); assert(command(&f,38,3,1)==0);
    step(&f,true); assert(command(&f,39,3,0)==0); settled(&f);
    assert(!f.control.motion_error); /* Immediate Off is not a no-motion fault. */
}
static void stale_temperature_wrap(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) for(unsigned off=0;off<2;++off) {
        struct fixture f;reference(&f,0);
        now=wrap ? UINT32_MAX-100u : 10u;
        word(&f,0,2200);word(&f,5,2100);
        if(!off) assert(command(&f,70,3,1)==0);
        w600_valve_calibration_poll(&f.control,now,true);
        uint32_t received_at=f.control.external_at;
        now=received_at+12u*60u*10000u-1u;
        w600_valve_calibration_poll(&f.control,now,true);
        assert(f.endpoint.requested_opening==(off ? 0 : 1000) && !starts);
        ++now;
        w600_valve_calibration_poll(&f.control,now,true);
        assert(!f.endpoint.requested_opening && !starts);
        /* A full timer wrap must not revive the cached sensor value. */
        now=received_at;
        if(off) assert(command(&f,71,3,1)==0);
        w600_valve_calibration_poll(&f.control,now,true);
        assert(!f.endpoint.requested_opening && !starts &&
               f.endpoint.control_error==W600_CONTROL_TEMPERATURE);
        word(&f,0,2300);
        w600_valve_calibration_poll(&f.control,now+10u,true);
        assert(!f.endpoint.requested_opening && !starts);
        word(&f,5,2100); /* Even an unchanged value is a new sensor receipt. */
        w600_valve_calibration_poll(&f.control,now+20u,true);
        assert(f.endpoint.requested_opening==1000 && !f.endpoint.control_error && !starts);
        assert(f.control.external_at==now+20u);
    }
    puts("PASS external temperature: expiry survives full timer wrap, Off and timestamp wrap; only new sensor receipt restores control");
}
static void loaded_feedback(void)
{
    struct fixture f;
    for(unsigned direction=0;direction<2;++direction) {
        reference(&f,direction ? 0 : 1000);running_mv=600;
        if(direction) {
            word(&f,0,2200);word(&f,5,2100);
            assert(command(&f,1,3,1)==0);
        }
        settled(&f);
        assert(f.endpoint.calibrated && !f.control.motion_error && f.control.saw_motion);
        assert(f.endpoint.opening_permille==(direction ? 1000 : 0));
    }
    reference(&f,1000);running_mv=599;settled(&f);
    assert(f.control.motion_error==W600_CONTROL_NO_MOTION && !f.endpoint.calibrated);

    reference(&f,1000);
    for(unsigned i=0;i<2200;++i) {
        running_mv=i<600 ? 1300 : i<1000 ? 800 : 1000;
        step(&f,true);
        if(i==1500) assert(f.control.moving && f.endpoint.opening_permille>0 && !f.motor.stop_confirmed);
        if(f.control.have_move && !f.control.moving) break;
    }
    assert(now>=20000u && f.motor.stop_confirmed);
    assert(f.endpoint.calibrated && f.endpoint.opening_known && !f.endpoint.opening_permille);
    assert(!f.control.motion_error && f.control.position.valid);

    for(unsigned wrap=0;wrap<2;++wrap) {
        init(&f);f.control.sequence.confirmation=confirmation;patterned=true;
        if(wrap) now=UINT32_MAX-10000u;
        assert(command(&f,1,9,1)==0);
        step(&f,true);
        for(unsigned i=0;i<15000 && f.control.calibration_preparing && !f.control.sequence.error;++i) step(&f,true);
        assert(starts==4 && f.control.sequence.direction==MOTOR_CAL_CLOSE && !f.endpoint.calibrated);
        patterned=false;running_mv=800;travel_ticks=1000000;
        uint32_t began=now;
        for(unsigned i=0;i<46000 && !f.control.sequence.error;++i) {
            if((uint32_t)(now-began)>=10000u) running_mv=650;
            step(&f,true);
        }
        assert(f.control.sequence.closing_load_seen && f.control.sequence.slowdown.candidate);
        assert(f.control.sequence.error==MOTOR_CAL_HOMING);
        assert(f.motor.stop_confirmed && starts==4 && caps[3]==45000);
        assert((uint32_t)(now-began)<=450010u); /* One1ms dispatch before the driver starts. */
        assert(!f.endpoint.calibrated && !f.endpoint.opening_known && !f.control.position.valid);
    }
    puts("loaded feedback:600mV ordinary moves both ways,599mV rejection,closing load bounded by45s without reference, clock wrap PASS");
}
static void sustained_load_driver_boundary(void)
{
    struct fixture f;init(&f);f.control.sequence.confirmation=confirmation;
    travel_ticks=1000000;running_mv=1300;
    assert(motor_calibration_start(&f.control.sequence,now,&policy,true));
    for(unsigned i=0;i<45000 && !f.control.sequence.error && !f.control.sequence.original_load;++i) {
        if(now>=20000u) running_mv=940;
        step(&f,true);
    }
    assert(f.control.sequence.original_load && f.control.sequence.original_load_mv==940);
    assert(f.control.endpoint_qualified && !f.control.sequence.error);
    assert(f.control.sequence.phase==MOTOR_CAL_CONFIRM_BACKOFF);
    assert(!f.endpoint.calibrated && !f.endpoint.opening_known && !f.control.position.valid);
    assert(now>220000u && now<450000u);
    puts("PASS sustained-load driver boundary: requested stop qualified, backoff required, no early reference");
}
static void slow_start_probe(void)
{
    struct fixture f; init(&f); startup_ticks=2200;
    assert(command(&f,1,9,1)==0);
    for (unsigned i=0;i<3000;++i) {
        step(&f,true);
        if (f.control.sequence.state==MOTOR_CAL_FAILED ||
            f.control.sequence.phase!=MOTOR_CAL_PROBE_CLOSE) break;
    }
    assert(!f.control.sequence.error && f.control.sequence.phase==MOTOR_CAL_HOME);
    step(&f,true);
    assert(starts==1 && f.motor.direction==2 && !f.endpoint.calibrated);
    init(&f); startup_ticks=UINT32_MAX;
    assert(command(&f,1,9,1)==0);
    for (unsigned i=0;i<5000;++i) step(&f,true);
    assert(f.control.sequence.error==MOTOR_CAL_PROBE_SIGNAL && f.motor.stop_confirmed);
    assert(starts==2 && f.control.sequence.phase==MOTOR_CAL_PROBE_OPEN && !f.endpoint.calibrated);
    assert(!f.control.calibration_applied && !f.control.position.valid);
}
static void bounded_confirmation(void)
{
    struct fixture f;init(&f);f.control.sequence.confirmation=confirmation;patterned=true;
    assert(command(&f,1,9,1)==0);
    bool saw_wait=false;
    for (unsigned i=0;i<60000 && !f.endpoint.calibrated && !f.control.sequence.error;++i) {
        step(&f,true);
        if (f.control.sequence.state==MOTOR_CAL_WAIT_STOP) {
            saw_wait=true;
            assert(f.motor.stop_confirmed && !f.endpoint.calibrated && !f.endpoint.opening_known);
            unsigned count=f.control.samples;
            w600_valve_calibration_feedback(&f.control);
            assert(f.control.samples==count); /* Immediate hook and ordinary poll consume once. */
        }
    }
    assert(saw_wait && f.endpoint.calibrated && f.control.sequence.confirmed_candidates==5);
    assert(f.control.sequence.state==MOTOR_CAL_CANDIDATE && f.control.sequence.measured_legs==4 && starts==18);
    assert(f.control.position.opening_ticks==25000 && f.control.position.closing_ticks==25000);
    for (unsigned i=0;i<18;++i) {
        assert(caps[i]==(i%3==0 ? 45000 : i%3==1 ? 3200 : 10200));
        if (i%3==1) assert(directions[i]!=directions[i-1]);
        if (i%3==2) assert(directions[i]==directions[i-2]);
    }
    patterned=false;travel_ticks=1000000;
    for (unsigned i=0;i<5000;++i) step(&f,true);
    assert(starts==18 && !f.endpoint.mode && f.endpoint.mode_known &&
           !f.endpoint.opening_permille && f.endpoint.calibrated && !f.control.motion_error);
    /* Closed starts first confirm opening, then use the ordinary calibration. */
    init(&f);f.control.sequence.confirmation=confirmation;patterned=true;closed_start=true;
    assert(command(&f,1,9,1)==0);
    for (unsigned i=0;i<60000 && !f.endpoint.calibrated && !f.control.sequence.error;++i) step(&f,true);
    assert(f.endpoint.calibrated && !f.endpoint.opening_permille && starts==18);
    patterned=false;closed_start=false;travel_ticks=1000000;
    for(unsigned i=0;i<5000;++i) step(&f,true);
    assert(starts==18 && !f.endpoint.mode && f.endpoint.mode_known &&
           !f.endpoint.opening_permille && f.endpoint.calibrated && !f.control.motion_error);
    for (unsigned at_rest=0;at_rest<2;++at_rest) {
        init(&f);f.control.sequence.confirmation=confirmation;patterned=true;
        assert(command(&f,1,9,1)==0);
        for (unsigned i=0;i<10000;++i) {
            step(&f,true);
            if (f.control.sequence.state==MOTOR_CAL_WAIT_STOP && f.motor.phase==W600_MOTOR_DONE &&
                (at_rest ? f.control.sequence.phase==MOTOR_CAL_CONFIRM_BACKOFF :
                           f.control.sequence.phase==MOTOR_CAL_HOME)) break;
        }
        assert(f.control.sequence.state==MOTOR_CAL_WAIT_STOP && f.motor.phase==W600_MOTOR_DONE);
        unsigned before=starts;f.endpoint.cancel_calibration=true;
        for (unsigned i=0;i<700;++i) step(&f,true);
        assert(starts==before && f.control.sequence.state==MOTOR_CAL_FAILED && !f.endpoint.calibrated);
    }
    init(&f);f.control.sequence.confirmation=confirmation;
    assert(command(&f,1,9,1)==0);
    for (unsigned i=0;i<4000 && !f.control.sequence.error;++i) step(&f,true);
    assert(f.control.sequence.error==MOTOR_CAL_REFERENCE_UNQUALIFIED && f.motor.stop_confirmed);
    assert(starts==1 && !f.endpoint.calibrated && !f.endpoint.opening_known);
}
static void mounting_command(void)
{
    struct fixture f;init(&f);f.control.sequence.confirmation=confirmation;patterned=true;
    volatile struct w600_mount_report r={.command=W600_MOUNT_RETRACT_COMMAND};
    w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(!r.command && r.used==1 && r.state==W600_MOUNT_ACTIVE && !starts);
    assert(f.control.sequence.direction==MOTOR_CAL_OPEN && !f.endpoint.mode);
    step(&f,true);
    assert(starts==1 && directions[0]==1 && f.endpoint.calibration_busy);
    r.command=W600_MOUNT_RETRACT_COMMAND;
    w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(!r.command && r.state==W600_MOUNT_ACTIVE && starts==1);
    for(unsigned i=0;i<15000 && r.state==W600_MOUNT_ACTIVE;++i) {
        step(&f,true);w600_valve_mounting_poll(&f.control,now,false,&r);
    }
    assert(r.state==W600_MOUNT_RETRACTED && !r.error && starts==3);
    assert(directions[0]==1 && directions[1]==2 && directions[2]==1);
    assert(caps[0]==45000 && caps[1]==3200 && caps[2]==10200);
    assert(!f.endpoint.calibrated && !f.endpoint.opening_known && !f.endpoint.calibration_busy);
    assert(!f.control.position.valid && !f.control.calibration_applied && f.motor.stop_confirmed);
    for(unsigned i=0;i<2000;++i) step(&f,true);
    r.command=W600_MOUNT_RETRACT_COMMAND;w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(starts==3 && r.state==W600_MOUNT_RETRACTED && !r.command);

    for(unsigned bad=0;bad<9;++bad) {
        init(&f);f.control.sequence.confirmation=confirmation;
        r=(struct w600_mount_report){.command=W600_MOUNT_RETRACT_COMMAND};
        switch(bad) {
        case 0:f.endpoint.mode=1;break;
        case 1:f.endpoint.mode_known=false;break;
        case 2:f.endpoint.calibration_pending=true;break;
        case 3:f.endpoint.control_enabled=false;break;
        case 4:f.control.calibration_applied=true;break;
        case 5:f.control.position.valid=true;break;
        case 6:f.motor.error=W600_MOTOR_ADC;break;
        case 7:r.command=1;break;
        default:break;
        }
        w600_valve_mounting_poll(&f.control,now,bad==8,&r);
        assert(r.used==1 && r.state==W600_MOUNT_REJECTED && !starts && !r.command);
        f.endpoint.mode=0;f.endpoint.mode_known=true;f.endpoint.calibration_pending=false;
        f.endpoint.control_enabled=true;f.control.calibration_applied=false;
        f.control.position.valid=false;f.motor.error=0;
        r.command=W600_MOUNT_RETRACT_COMMAND;w600_valve_mounting_poll(&f.control,now,false,&r);
        assert(r.state==W600_MOUNT_REJECTED && !starts);
    }
    for(unsigned cancel=0;cancel<2;++cancel) {
        init(&f);f.control.sequence.confirmation=confirmation;
        r=(struct w600_mount_report){.command=W600_MOUNT_RETRACT_COMMAND};
        w600_valve_mounting_poll(&f.control,now,false,&r);step(&f,true);
        if(cancel) f.endpoint.mode=1;
        else assert(command(&f,1,3,0)==0);
        for(unsigned i=0;i<5;++i) {step(&f,true);w600_valve_mounting_poll(&f.control,now,false,&r);}
        assert(r.state==W600_MOUNT_FAILED && r.error && starts==1 && f.motor.stop_confirmed);
        assert(!f.endpoint.calibrated && !f.endpoint.opening_known);
    }
    init(&f);f.control.sequence.confirmation=confirmation;still_motor=true;
    r=(struct w600_mount_report){.command=W600_MOUNT_RETRACT_COMMAND};
    w600_valve_mounting_poll(&f.control,now,false,&r);
    for(unsigned i=0;i<3000 && r.state==W600_MOUNT_ACTIVE;++i) {
        step(&f,true);w600_valve_mounting_poll(&f.control,now,false,&r);
    }
    /* No rotation is not proof of a retracted stop. */
    assert(r.state==W600_MOUNT_FAILED && r.error==MOTOR_CAL_REFERENCE_UNQUALIFIED);
    assert(starts==1 && now<30000 && f.motor.stop_confirmed && !f.endpoint.calibrated);
    puts("mounting command: inward probe/HOME/confirmation, early no-motion stop, no calibration, guards, duplicates, rejection and cancellation PASS");
}
static void calibration_preparation_limits(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;reference(&f,0);f.control.sequence.confirmation=confirmation;patterned=true;travel_ticks=25000;
        if(wrap) now=UINT32_MAX-10000u;
        assert(command(&f,1,3,1)==0 && command(&f,2,9,1)==0);
        step(&f,true);uint32_t began=f.control.sequence.attempt_started_at;
        for(unsigned i=0;i<15000 && f.control.sequence.state!=MOTOR_CAL_RETRACTED;++i) step(&f,true);
        assert(f.control.calibration_preparing && f.endpoint.calibration_busy && starts==3);
        assert(!f.endpoint.calibrated && !f.endpoint.opening_known && !f.control.position.valid);
        assert(directions[0]==1 && directions[1]==2 && directions[2]==1);
        assert(f.control.sequence.attempt_started_at==began && f.control.sequence.confirmed_candidates==1);
        assert(command(&f,3,9,1)==3); /* A second request cannot restart preparation. */
        now=began+3600000u;
        w600_valve_calibration_poll(&f.control,now,false);
        assert(starts==3 && f.control.sequence.error==MOTOR_CAL_ATTEMPT_TIMEOUT);
        assert(!f.control.calibration_preparing && !f.endpoint.calibrated && f.motor.stop_confirmed);
    }
    struct fixture f;init(&f);f.control.sequence.confirmation=confirmation;patterned=true;
    assert(command(&f,1,9,1)==0);step(&f,true);uint32_t began=f.control.sequence.attempt_started_at;
    for(unsigned i=0;i<15000 && f.control.calibration_preparing;++i) step(&f,true);
    assert(starts==4 && !f.control.calibration_preparing && f.endpoint.calibration_busy);
    assert(f.control.sequence.attempt_started_at==began && f.control.sequence.policy.whole_attempt_ticks==3600000u);
    assert(!f.endpoint.calibrated && !f.endpoint.opening_known && !f.control.sequence.confirmed_candidates);
    puts("closed-start preparation: confirmed opening, no reference, Heat ownership, duplicate refusal, shared whole limit and wrap PASS");
}
static void closing_sample(void)
{
    struct fixture f;init(&f);travel_ticks=1000000;
    volatile struct w600_mount_report r={.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND};
    w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(r.state==W600_MOUNT_SAMPLING && starts==1 && caps[0]==5200 && directions[0]==2);
    assert(f.endpoint.moving && !f.endpoint.calibrated && !f.endpoint.opening_known);
    assert(command(&f,1,9,1)==3);
    r.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND;w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(starts==1 && !r.command && r.started_at==0);
    for(unsigned i=0;i<5100 && r.state==W600_MOUNT_SAMPLING;++i) {
        step(&f,true);w600_valve_mounting_poll(&f.control,now,false,&r);
    }
    assert(r.state==W600_MOUNT_SAMPLED && !r.error && finishes==1 && starts==1);
    assert(now>=50000 && now<50100 && f.motor.stop_confirmed && !f.endpoint.moving);
    assert(!f.control.position.valid && !f.control.calibration_applied && !f.endpoint.calibrated);
    r.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND;w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(starts==1 && r.state==W600_MOUNT_SAMPLED);
    init(&f);now=UINT32_MAX-30000u;
    r=(struct w600_mount_report){.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND};
    w600_valve_mounting_poll(&f.control,now,false,&r);
    uint32_t began=now;now=began+49999u;w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(!finishes && r.state==W600_MOUNT_SAMPLING);
    now=began+50000u;w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(finishes==1 && f.motor.stop_confirmed);
    for(unsigned bad=0;bad<7;++bad) {
        init(&f);r=(struct w600_mount_report){.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND};
        switch(bad) {
        case 0:f.endpoint.mode=1;break;
        case 1:f.endpoint.mode_known=false;break;
        case 2:f.endpoint.calibration_pending=true;break;
        case 3:f.control.calibration_applied=true;break;
        case 4:f.control.position.valid=true;break;
        case 5:f.motor.error=W600_MOTOR_ADC;break;
        default:break;
        }
        w600_valve_mounting_poll(&f.control,now,bad==6,&r);
        assert(!starts && r.used && r.state==W600_MOUNT_REJECTED);
    }
    init(&f);r=(struct w600_mount_report){.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND};
    w600_valve_mounting_poll(&f.control,now,false,&r);f.endpoint.mode=1;
    w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(f.motor.stop_confirmed && r.state==W600_MOUNT_FAILED && r.error);
    assert(!f.endpoint.calibrated && !f.control.position.valid);
    init(&f);r=(struct w600_mount_report){.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND};
    w600_valve_mounting_poll(&f.control,now,false,&r);
    for(unsigned i=0;i<120;++i) step(&f,false);
    w600_valve_mounting_poll(&f.control,now,false,&r);
    assert(f.motor.stop_confirmed && f.motor.error && r.state==W600_MOUNT_FAILED && starts==1);
    assert(!f.control.position.valid && !f.endpoint.calibrated);
    puts("closing sample: one5s step/5.2s driver cap, duplicate/refusal, wrap, mode cancellation, no calibration PASS");
}
static void backoff_return_sample(void)
{
    for(unsigned wrap=0;wrap<2;++wrap) {
        struct fixture f;init(&f);travel_ticks=1000000;
        if(wrap) now=UINT32_MAX-30000u;
        volatile struct w600_mount_report r={.command=W600_MOUNT_OPEN_SAMPLE_COMMAND};
        w600_valve_mounting_poll(&f.control,now,false,&r);
        assert(r.used==1 && r.state==W600_MOUNT_SAMPLING && starts==1 && directions[0]==1 && caps[0]==5200);
        r.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND;w600_valve_mounting_poll(&f.control,now,false,&r);
        assert(starts==1 && r.used==1 && r.state==W600_MOUNT_SAMPLING);
        for(unsigned i=0;i<5100 && r.state==W600_MOUNT_SAMPLING;++i) {
            step(&f,true);w600_valve_mounting_poll(&f.control,now,false,&r);
        }
        assert(r.state==W600_MOUNT_SAMPLED && !r.error && finishes==1 && !f.endpoint.moving);
        uint32_t first=r.started_at;
        for(unsigned i=0;i<200;++i) step(&f,true);
        assert(starts==1); /* No automatic reversal. */
        r.command=W600_MOUNT_OPEN_SAMPLE_COMMAND;w600_valve_mounting_poll(&f.control,now,false,&r);
        assert(starts==1 && r.used==1 && r.state==W600_MOUNT_SAMPLED);
        r.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND;w600_valve_mounting_poll(&f.control,now,false,&r);
        assert(starts==2 && r.used==2 && r.state==W600_MOUNT_SAMPLING && r.started_at!=first);
        assert(directions[1]==2 && caps[1]==10200 && f.endpoint.moving);
        for(unsigned i=0;i<10100 && r.state==W600_MOUNT_SAMPLING;++i) {
            step(&f,true);w600_valve_mounting_poll(&f.control,now,false,&r);
        }
        assert(r.state==W600_MOUNT_SAMPLED && !r.error && finishes==2 && f.motor.stop_confirmed);
        assert(!f.endpoint.moving && !f.endpoint.calibrated && !f.endpoint.opening_known && !f.control.position.valid);
        for(unsigned command_id=0;command_id<2;++command_id) {
            r.command=command_id ? W600_MOUNT_OPEN_SAMPLE_COMMAND : W600_MOUNT_CLOSE_SAMPLE_COMMAND;
            w600_valve_mounting_poll(&f.control,now,false,&r);
            assert(starts==2 && r.used==2 && !r.command);
        }
    }
    for(unsigned bad=0;bad<6;++bad) {
        struct fixture f;init(&f);travel_ticks=1000000;
        volatile struct w600_mount_report r={.command=W600_MOUNT_OPEN_SAMPLE_COMMAND};
        w600_valve_mounting_poll(&f.control,now,false,&r);
        for(unsigned i=0;i<5100 && r.state==W600_MOUNT_SAMPLING;++i) {
            step(&f,true);w600_valve_mounting_poll(&f.control,now,false,&r);
        }
        if(bad!=0) for(unsigned i=0;i<200;++i) step(&f,true);
        if(bad==1) f.endpoint.mode=1;
        if(bad==2) f.control.position.valid=true;
        if(bad==3) f.motor.requested_stop=0;
        if(bad==4) f.endpoint.calibration_pending=true;
        r.command=W600_MOUNT_CLOSE_SAMPLE_COMMAND;
        w600_valve_mounting_poll(&f.control,now,bad==5,&r);
        assert(starts==1 && f.motor.stop_confirmed && !f.endpoint.moving);
        assert(r.state==(bad==3 ? W600_MOUNT_SAMPLED : W600_MOUNT_REJECTED));
    }
    puts("backoff/return samples: explicit5s open then10s close only, no auto reversal/retry/reference, duplicate and third-command refusal, rest/mode/reference/pending/sensor guards, wrap PASS");
}
int main(void)
{
    sustained_load_driver_boundary();
    assert(confirmation.max_sample_gap_ticks==210 && confirmation.backoff_ticks==30000 &&
           confirmation.backoff_limit_ticks==32000 && confirmation.return_ticks==100000 &&
           confirmation.return_limit_ticks==102000 && confirmation.rest_ticks==5000);
    assert(policy.homing[0].max_move_ticks==450000 && policy.homing[1].max_move_ticks==450000 &&
           policy.whole_attempt_ticks==3600000);
    mounting_command();
    calibration_preparation_limits();
    startup_off();
    bounded_confirmation();closing_sample();backoff_return_sample();
    loaded_feedback();
    slow_start_probe();
    unknown_sensor_selection();
    normal_controls();
    stale_temperature_wrap();
    struct fixture clock; init(&clock); assert(command(&clock,1,9,1)==0);
    w600_valve_calibration_poll(&clock.control,now,false);
    now=9; w600_valve_calibration_poll(&clock.control,now,false);
    w600_valve_calibration_tick(&clock.control,0);
    assert(!clock.control.sequence.error && clock.control.sequence.last_now==9);
    w600_valve_calibration_tick(&clock.control,10);
    assert(!clock.control.sequence.error && clock.control.sequence.last_now==10);
    w600_valve_calibration_tick(&clock.control,0);
    assert(clock.control.sequence.error==MOTOR_CAL_CLOCK); /* Whole-ms reversal is still a fault. */
    struct fixture f; init(&f);
    assert(command(&f,1,9,1)==0 && f.endpoint.calibration_pending && !f.endpoint.calibrated);
    assert(command(&f,1,9,1)==0); /* Same transaction, no second action. */
    step(&f,true);
    assert(starts==1 && f.endpoint.calibration_busy && !f.endpoint.calibrated);
    assert(command(&f,2,9,1)==3); /* New request while busy is rejected. */
    assert(command(&f,3,9,0)==0);
    finish_calibration(&f);
    struct w600_frame in={.kind=1,.sequence=4,.length=4,.payload={7,0x0b,7,0x1c}},out;
    assert(w600_endpoint_process(&f.endpoint,&in,&out));
    assert(out.length==11 && out.payload[4]==1 && out.payload[8]==3 && !out.payload[9] && !out.payload[10]);
    assert(command(&f,5,9,1)==0);
    step(&f,true); assert(starts==6 && !f.endpoint.calibrated);
    assert(command(&f,6,3,0)==0);
    for(unsigned i=0;i<3;++i) step(&f,true);
    assert(f.control.sequence.error==MOTOR_CAL_ABORTED && f.motor.stop_confirmed);
    assert(!f.endpoint.calibrated && !f.endpoint.opening_known && !f.endpoint.calibration_busy);
    init(&f); assert(command(&f,7,9,1)==0); step(&f,true);
    no_feedback=true;
    for(unsigned i=0;i<120;++i) step(&f,false);
    assert(f.motor.stop_confirmed && f.control.sequence.state==MOTOR_CAL_FAILED);
    step(&f,true); assert(!f.endpoint.calibrated && !f.endpoint.control_enabled);
    assert(command(&f,8,9,1)==3);
    init(&f); assert(command(&f,9,9,1)==0);
    w600_valve_calibration_poll(&f.control,now,true);
    assert(!starts && f.endpoint.calibration_pending); /* Shared ADC belongs to sensor. */
    w600_valve_calibration_poll(&f.control,now,false); assert(starts==1);
    f.motor.error=W600_MOTOR_ADC; driver_stop(&f.motor);
    w600_valve_calibration_poll(&f.control,now,false);
    assert(!f.endpoint.control_enabled && !f.endpoint.calibration_busy && !f.endpoint.calibrated);
    assert(!w600_valve_calibration_init(NULL,&f.motor,&f.endpoint));
    puts("valve control: calibration, normal/forced opening, Off ownership, asymmetric partial roundtrips, freshness, cancellation, duplicates, token/feedback/endpoint faults PASS");
    return 0;
}
