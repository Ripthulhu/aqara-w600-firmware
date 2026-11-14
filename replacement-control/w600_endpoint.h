#ifndef W600_ENDPOINT_H
#define W600_ENDPOINT_H
#include "w600_frame.h"
#ifndef W600_CONTROLLER_REVISION
#define W600_CONTROLLER_REVISION 28u
#endif
#if W600_CONTROLLER_REVISION < 1 || W600_CONTROLLER_REVISION > 65535
#error Invalid controller revision
#endif

enum {
    W600_KNOWN_TARGET = 1,
    W600_KNOWN_EXTERNAL_TEMP = 2,
    W600_KNOWN_EXTERNAL_USE = 4,
    W600_KNOWN_LOCAL_TEMP = 8,
    W600_KNOWN_CHILD_LOCK = 16
};
enum w600_control_error {
    W600_CONTROL_OK, W600_CONTROL_NEEDS_CALIBRATION, W600_CONTROL_TEMPERATURE,
    W600_CONTROL_NO_MOTION, W600_CONTROL_ENDPOINT, W600_CONTROL_POSITION,
    W600_CONTROL_TOKEN, W600_CONTROL_DRIVER
};
struct w600_endpoint {
    uint16_t target_raw, external_temp_raw;
    uint8_t known, external_use;
    uint16_t local_temp_raw;
    uint16_t opening_permille;
    uint8_t calibration_error, last_request_sequence;
    bool control_enabled, calibrated, opening_known;
    bool calibration_pending, calibration_busy, cancel_calibration;
    bool calibration_seen, request_seen;
    uint32_t external_updates, control_updates;
    uint16_t requested_opening;
    uint8_t mode, control_error;
    bool moving, mode_known;
    bool child_locked;
};

void w600_endpoint_init(struct w600_endpoint *state);
/* Shared local/radio target update; does not change mode or temperature source. */
bool w600_endpoint_set_target(struct w600_endpoint *,uint16_t raw);
/* Raw local sensor result, not an external-source selection or heat request.
 * DeciC is checked against the converter's complete offset-adjusted range.
 * Invalid/stale acquisition clears availability; bad arguments leave state alone. */
bool w600_endpoint_local_temperature(struct w600_endpoint *, int16_t decic, bool valid);
/* False leaves state/reply unchanged. No hardware actions or unsolicited reports. */
bool w600_endpoint_process(struct w600_endpoint *state,
                           const struct w600_frame *request,
                           struct w600_frame *reply);
#endif
