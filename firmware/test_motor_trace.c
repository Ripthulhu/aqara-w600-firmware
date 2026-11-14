#include "motor_trace.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>

int main(void)
{
    assert(sizeof(w600_motor_trace)==18080);
    w600_motor_trace_record(1,1,20,1000);
    assert(!w600_motor_trace.magic && !w600_motor_trace.count);
    w600_motor_trace_begin(100,2);
    assert(w600_motor_trace.magic==W600_MOTOR_TRACE_MAGIC && w600_motor_trace.version==1);
    assert(w600_motor_trace.capacity==2256 && !w600_motor_trace.invalid);
    for (uint32_t i=1;i<=2250;++i) w600_motor_trace_record(100,i,1000+i*200,1300-i%200);
    assert(w600_motor_trace.count==2250 && !w600_motor_trace.dropped && !w600_motor_trace.invalid);
    for (uint32_t i=0;i<2250;++i) {
        assert(w600_motor_trace.samples[i].acquired_at_ticks==1000+(i+1)*200);
        assert(w600_motor_trace.samples[i].millivolts==1300-(i+1)%200);
    }
    for (uint32_t i=2251;i<=2258;++i) w600_motor_trace_record(100,i,i,UINT32_MAX);
    assert(w600_motor_trace.count==2256 && w600_motor_trace.dropped==2 && !w600_motor_trace.invalid);
    assert(w600_motor_trace.samples[2255].millivolts==UINT32_MAX);

    w600_motor_trace_begin(101,1);
    assert(!w600_motor_trace.count && !w600_motor_trace.dropped && !w600_motor_trace.invalid);
    assert(w600_motor_trace.samples[2255].millivolts==UINT32_MAX);
    w600_motor_trace_record(101,1,UINT32_MAX,UINT32_MAX);
    w600_motor_trace_record(101,2,199,0);
    assert(w600_motor_trace.count==2 && w600_motor_trace.samples[0].acquired_at_ticks==UINT32_MAX);
    assert(w600_motor_trace.samples[1].acquired_at_ticks==199 && !w600_motor_trace.invalid);
    w600_motor_trace_record(100,3,399,0);
    assert(w600_motor_trace.count==2 && w600_motor_trace.invalid==W600_TRACE_BAD_GENERATION);
    w600_motor_trace_record(101,3,399,0);assert(w600_motor_trace.count==2);

    for (uint32_t sequence=0;sequence<=3;++sequence) {
        if (sequence==1) continue;
        w600_motor_trace_begin(0,2);w600_motor_trace_record(0,sequence,0,1);
        assert(!w600_motor_trace.count && w600_motor_trace.invalid==W600_TRACE_BAD_SEQUENCE);
    }
    w600_motor_trace_begin(0,1);w600_motor_trace_record(0,1,0,0);
    w600_motor_trace_record(0,1,0,0);
    assert(w600_motor_trace.count==1 && w600_motor_trace.invalid==W600_TRACE_BAD_SEQUENCE);
    for (uint32_t direction=0;direction<=3;direction+=3) {
        w600_motor_trace_begin(3,direction);w600_motor_trace_record(3,1,1,1);
        assert(!w600_motor_trace.count && w600_motor_trace.invalid==W600_TRACE_BAD_DIRECTION);
    }
    puts("PASS trace: exact18080B budget, full2250-sample movement, bounded overflow, generation reset, untouched old slots, full32-bit values/wrap, sequence/generation faults; no motor effects");
}
