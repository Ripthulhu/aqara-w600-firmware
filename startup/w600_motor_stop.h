#ifndef W600_MOTOR_STOP_H
#define W600_MOTOR_STOP_H

/* Privileged Thread or ISR; serialize GPIO/RCC read-modify-write users.
 * No state or initialization. Enables only GPIO A/B/C clocks if necessary.
 * Proves selector GPIO LOW before replacing PB14/15 AF with GPIO HIGH.
 * Returns 1 only after MODE/OTYPE/no-pull/latch/pad readback; 0 is NOT stopped proof.
 * Never changes a timer: on failure leave any independent one-shot running.
 * This is GPIO evidence, not motor current, motion or electrical validation.
 * Finite operations do not preempt a stalled bus or CPU/clock failure. */
int w600_motor_stop(void);

#endif
