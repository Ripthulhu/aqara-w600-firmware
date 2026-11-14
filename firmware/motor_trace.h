#ifndef W600_MOTOR_TRACE_H
#define W600_MOTOR_TRACE_H
#include <stdint.h>

enum { W600_MOTOR_TRACE_CAPACITY=2256, W600_MOTOR_TRACE_MAGIC=0x57365452 };
enum { W600_TRACE_BAD_DIRECTION=1, W600_TRACE_BAD_GENERATION=2, W600_TRACE_BAD_SEQUENCE=4 };
struct w600_motor_trace_sample { uint32_t acquired_at_ticks, millivolts; };
struct w600_motor_trace {
    uint32_t magic, version, capacity, generation, direction, count, dropped, invalid;
    struct w600_motor_trace_sample samples[W600_MOTOR_TRACE_CAPACITY];
};
extern volatile struct w600_motor_trace w600_motor_trace;

/* Diagnostic only: no motor callback, allocation, wait or policy change.
 * A new drive replaces the previous trace. Index+1 is the validated sample
 * sequence. Retain full32-bit raw values, including diagnostically invalid ones.
 * Read after a proved stop with unchanged generation/header, not as an atomic
 * live snapshot. 2256 slots cover the driver's <=2250 batches in45seconds.
 * Overflow is counted, never wraps or alters motor control. */
void w600_motor_trace_begin(uint32_t generation,uint32_t direction);
void w600_motor_trace_record(uint32_t generation,uint32_t sequence,
                             uint32_t acquired_at_ticks,uint32_t millivolts);
#endif
