#ifndef W600_PAIRING_H
#define W600_PAIRING_H
#include "w600_frame.h"

enum w600_pairing_stage {
    W600_PAIRING_IDLE, W600_PAIRING_PREPARED,
    W600_PAIRING_WINDOW_ENDED, W600_PAIRING_CLOCK_ERROR
};
enum w600_pairing_event {
    W600_PAIRING_ACK_OBSERVED = 1,
    W600_PAIRING_RESPONSE_OBSERVED = 2,
    W600_PAIRING_WINDOW_EXPIRED = 4,
    W600_PAIRING_BAD_CLOCK = 8
};
struct w600_pairing {
    enum w600_pairing_stage stage;
    uint32_t prepared_at, last_now, ack_at, response_at;
    uint8_t sequence, response_sequence, response_kind, response_value, response_status;
    bool ack_seen, response_seen, response_has_status;
};

/* Local bookkeeping only: no transmit, settings, network or motor operation. */
void w600_pairing_init(struct w600_pairing *);
/* Explicit intent only. False leaves both objects unchanged. Prepared != sent.
 * The caller owns fresh nonzero sequences and the actual bounded UART send. */
bool w600_pairing_prepare(struct w600_pairing *, uint8_t sequence, uint32_t now,
                          struct w600_frame *report);
/* Call with a CRC-validated frame or NULL for a clock tick. One monotonic clock.
 * The10s window is local observation policy, NOT a Zigbee join timeout.
 * ACKs/0052 are observations only;0052 cannot be tied to this request. */
unsigned w600_pairing_update(struct w600_pairing *, const struct w600_frame *, uint32_t now);
#endif
