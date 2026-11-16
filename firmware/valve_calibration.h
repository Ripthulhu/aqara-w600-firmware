#ifndef W600_VALVE_CALIBRATION_H
#define W600_VALVE_CALIBRATION_H
#include "motor_service.h"
#include "../replacement-control/motor_calibration.h"
#include "../replacement-control/motor_position.h"
#include "../replacement-control/w600_endpoint.h"

struct w600_valve_calibration {
    struct motor_calibration sequence;
    struct motor_position position;
    struct w600_motor_service *motor;
    struct w600_endpoint *endpoint;
    uint32_t started_token, samples, final_on;
    uint32_t move_budget, move_samples, external_updates, external_at;
    uint32_t control_updates, last_move_at;
    uint16_t move_target;
    uint8_t motion_error;
    bool endpoint_qualified, calibration_applied;
    bool external_seen, moving, move_stopping, interrupted_move, saw_motion, have_move;
    bool calibration_preparing;
};

enum w600_mount_state { W600_MOUNT_IDLE, W600_MOUNT_ACTIVE, W600_MOUNT_RETRACTED,
                       W600_MOUNT_REJECTED, W600_MOUNT_FAILED,
                       W600_MOUNT_SAMPLING, W600_MOUNT_SAMPLED };
enum { W600_MOUNT_RETRACT_COMMAND=0xa7u, W600_MOUNT_CLOSE_SAMPLE_COMMAND=0xc5u,
       W600_MOUNT_OPEN_SAMPLE_COMMAND=0xc6u };
struct w600_mount_report { uint32_t command, used, state, error, started_at; };
/* One retract/close probe or one manually requested open/close pair per boot. */
void w600_valve_mounting_poll(struct w600_valve_calibration *,uint32_t now_ticks,
                             bool sensor_busy,volatile struct w600_mount_report *);

/* Serialized Thread calls, after motor service init/poll. No movement at init.
 * tick runs independently from SysTick; each driver action is consumed once.
 * Bench-qualified endpoint signals still require mounted-load validation. */
bool w600_valve_calibration_init(struct w600_valve_calibration *,
                                struct w600_motor_service *,struct w600_endpoint *);
void w600_valve_calibration_poll(struct w600_valve_calibration *,uint32_t now_ticks,
                                bool sensor_busy);
/* Immediately after driver poll, before restoring drive/urgent-loop continue.
 * Consumes only a new sample; no starts, rearm, radio or position publication. */
void w600_valve_calibration_feedback(struct w600_valve_calibration *);
void w600_valve_calibration_tick(struct w600_valve_calibration *,uint32_t now_ticks);
#endif
