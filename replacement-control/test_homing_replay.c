#include "motor_homing.h"
#include <assert.h>
#include <stdio.h>
#include <stdlib.h>

/* Bench replay candidate only; not mounted-valve calibration or defaults. */
static const struct motor_homing_policy policy={50000,100,100,10,1100,100,4000,200,20,11,2};
static void stop(void *context) { ++*(unsigned *)context; }
static void replay(const char *path,unsigned count,unsigned expected_ms,uint32_t epoch)
{
    FILE *file=fopen(path,"rb"); assert(file);
    struct motor_homing h; unsigned stops=0, observed=0;
    assert(motor_homing_init(&h,stop,&stops));
    assert(motor_homing_start(&h,epoch,&policy));
    for(unsigned i=0;i<count;++i) {
        unsigned char b[4]; assert(fread(b,1,4,file)==4);
        uint32_t bgr=(uint32_t)b[0]|(uint32_t)b[1]<<8;
        uint32_t signal=(uint32_t)b[2]|(uint32_t)b[3]<<8;
        assert(bgr && bgr<=4095 && signal<=4095);
        uint32_t mv=(uint32_t)((uint64_t)signal*1495*3300/(bgr*4095));
        unsigned ms=i ? i*20 : 1;
        bool running=motor_homing_foreground_progress(&h,epoch+ms);
        if(running) assert(motor_homing_feedback(&h,epoch+ms,epoch+ms,mv,true));
        if(h.state==MOTOR_HOMING_ENDPOINT_OBSERVED && !observed) observed=ms;
        if(observed) assert(!h.guard.armed && stops==2 && h.error==MOTOR_HOMING_OK);
    }
    assert(fgetc(file)==EOF && !ferror(file)); assert(fclose(file)==0);
    assert(observed==expected_ms);
    if(!expected_ms) {
        assert(h.guard.armed && stops==1);
        motor_homing_tick(&h,epoch+50000);
        assert(h.state==MOTOR_HOMING_FAILED && stops==2);
    }
    printf("PASS replay %s epoch=%u endpoint_ms=%u latched_off=%u\n",path,epoch,observed,stops==2);
}
int main(int argc,char **argv)
{
    assert(argc==5);
    const unsigned counts[]={1250,1750,2250,1500};
    const unsigned expected[]={24320,30340,42240,0};
    for(unsigned i=0;i<4;++i) {
        replay(argv[i+1],counts[i],expected[i],0);
        replay(argv[i+1],counts[i],expected[i],UINT32_MAX-500);
    }
    return 0;
}
