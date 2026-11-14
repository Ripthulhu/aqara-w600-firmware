#include "motor_trace.h"
#include <stdatomic.h>
#include <limits.h>

volatile struct w600_motor_trace w600_motor_trace;
_Static_assert(sizeof(struct w600_motor_trace_sample)==8,"Trace sample layout");
_Static_assert(sizeof(struct w600_motor_trace)==18080,"Trace memory budget");

void w600_motor_trace_begin(uint32_t generation,uint32_t direction)
{
    w600_motor_trace.magic=0;
    w600_motor_trace.version=1;w600_motor_trace.capacity=W600_MOTOR_TRACE_CAPACITY;
    w600_motor_trace.generation=generation;w600_motor_trace.direction=direction;
    w600_motor_trace.count=0;w600_motor_trace.dropped=0;
    w600_motor_trace.invalid=(direction==1 || direction==2) ? 0 : W600_TRACE_BAD_DIRECTION;
    /* Old slots are ignored by count; clearing18KB on each reversal would block. */
    atomic_thread_fence(memory_order_release);
    w600_motor_trace.magic=W600_MOTOR_TRACE_MAGIC;
}
void w600_motor_trace_record(uint32_t generation,uint32_t sequence,
                             uint32_t acquired_at_ticks,uint32_t millivolts)
{
    if (w600_motor_trace.magic!=W600_MOTOR_TRACE_MAGIC || w600_motor_trace.invalid) return;
    if (generation!=w600_motor_trace.generation) {
        w600_motor_trace.invalid=W600_TRACE_BAD_GENERATION;return;
    }
    uint32_t count=w600_motor_trace.count,dropped=w600_motor_trace.dropped;
    if (!sequence || (uint64_t)count+dropped+1u!=sequence) {
        w600_motor_trace.invalid=W600_TRACE_BAD_SEQUENCE;return;
    }
    if (count>=W600_MOTOR_TRACE_CAPACITY) {
        if (dropped<UINT32_MAX) w600_motor_trace.dropped=dropped+1u;
        return;
    }
    w600_motor_trace.samples[count].acquired_at_ticks=acquired_at_ticks;
    w600_motor_trace.samples[count].millivolts=millivolts;
    atomic_thread_fence(memory_order_release);
    w600_motor_trace.count=count+1u;
}
