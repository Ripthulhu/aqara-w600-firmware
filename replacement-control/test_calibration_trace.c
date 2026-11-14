#include "motor_calibration.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const struct motor_calibration_policy policy={
    .homing={{450000,1000,1000,100,1100,100,4000,2000,200,11,2},
             {450000,1000,1000,100,1100,100,4000,2000,200,11,2}},
    .whole_attempt_ticks=3600000,.probe_on_ticks=15000,.probe_slack_ticks=300,
    .min_travel_ticks=20000,.max_travel_ticks=449000,.repeat_tolerance_ticks=50000,
    .stop_confirm_ticks=10000,.min_stop_ticks=1100
};
static void stop(void *context) { *(bool *)context=false; }
int main(int argc,char **argv)
{
    assert(argc==3 && (!strcmp(argv[2],"V15") || !strcmp(argv[2],"V16")));
    const unsigned expected_low=!strcmp(argv[2],"V16");
    FILE *file=fopen(argv[1],"rb"); assert(file);
    for(unsigned wrap=0;wrap<2;++wrap) {
        assert(fseek(file,0,SEEK_SET)==0);
        uint32_t epoch=wrap ? UINT32_MAX-10000u : 0;
        struct motor_calibration c; bool motor_on=true;
        assert(motor_calibration_init(&c,stop,&motor_on));
        assert(!motor_on && motor_calibration_start(&c,epoch,&policy,true));
        motor_on=true;
        uint32_t previous=0; unsigned qualified_low=0;
        for(unsigned i=0;i<2250;++i) {
            unsigned char b[8]; assert(fread(b,1,8,file)==8);
            uint32_t at=(uint32_t)b[0]|(uint32_t)b[1]<<8|(uint32_t)b[2]<<16|(uint32_t)b[3]<<24;
            uint32_t value=(uint32_t)b[4]|(uint32_t)b[5]<<8|(uint32_t)b[6]<<16|(uint32_t)b[7]<<24;
            assert(at>previous && at<450000 && value<=4000);
            previous=at;
            if(!i) continue; /* Driver's stationary baseline is not motion feedback. */
            /* Replay measured signal/times; energized counts model 1.5ms/window, not measured energy. */
            uint32_t on=at-15u*i;
            assert(motor_calibration_foreground_progress(&c,c.action.token,epoch+at,on));
            assert(motor_calibration_feedback(&c,c.action.token,epoch+at,epoch+at,value,true,on,on));
            assert(motor_on && c.action.token==1 && !c.measured_legs);
            assert(c.homing.endpoint_count<=expected_low);
            assert(c.have_first_low==(c.homing.endpoint_count!=0));
            if(c.homing.endpoint_count) ++qualified_low;
        }
        assert(fgetc(file)==EOF && !ferror(file));
        assert(qualified_low==expected_low && c.phase==MOTOR_CAL_HOME && c.action.token==1 &&
               c.homing.guard.move_deadline==epoch+450000u);
        motor_calibration_tick(&c,epoch+450000u);
        assert(c.state==MOTOR_CAL_FAILED && !motor_on && !c.candidate.opening_ticks &&
               c.homing.guard.error==MOTOR_GUARD_MOVE_TIMEOUT);
        printf("PASS %s trace replay:2250 readings, same action, qualified low observations=%u, no false endpoint, original45s cutoff; wrap=%u\n",argv[2],qualified_low,wrap);
    }
    assert(fclose(file)==0);
    return 0;
}
