/* Reuse the driver-boundary fixture; no physical movement or reference proof. */
#define main existing_valve_cases
#include "test_valve_calibration.c"
#undef main

int main(void)
{
    for(unsigned opening=0;opening<2;++opening) {
        const unsigned extras[]={1,5,50,51};
        for(unsigned i=0;i<4;++i) {
            struct fixture f;init(&f);
            assert(motor_position_reference(&f.control.position,388588,387535,opening ? 0 : 1000));
            f.control.calibration_applied=true;
            f.endpoint.mode=opening ? 2 : 0;f.endpoint.mode_known=true;
            movement_poll(&f.control,now,false);assert(starts==1 && f.control.moving);
            uint32_t full=opening ? 388588 : 387535;
            now=full+extras[i];on=now;f.motor.samples=2;f.motor.last_mv=1500;
            movement_poll(&f.control,now,false);
            if(extras[i]>50) {
                assert(f.control.motion_error==W600_CONTROL_POSITION && !f.control.calibration_applied);
                assert(f.motor.stop_confirmed && f.motor.error==W600_MOTOR_ABORTED);
            } else {
                assert(!f.control.motion_error && f.motor.requested_stop && f.motor.stop_confirmed);
                assert(on==full+extras[i]); /* Raw drive time must not be rewritten. */
                f.motor.phase=W600_MOTOR_DONE;
                movement_poll(&f.control,now,false);
                assert(!f.control.moving && f.control.position.valid && f.control.calibration_applied);
                assert(f.control.position.position_ppm==(opening ? MOTOR_POSITION_FULL : 0));
            }
        }
    }
    puts("PASS bounded endpoint accounting: both directions, odd learned times, raw count retained, excessive overrun rejected");
}
