#ifndef W600_RADIO_H
#define W600_RADIO_H
#include "../replacement-control/w600_endpoint.h"
#include "../replacement-control/w600_pairing.h"
#ifdef W600_OTA_RECEIVER
#include "ota_receiver.h"
#endif

enum w600_radio_error {
    W600_RADIO_OK, W600_RADIO_CONTEXT, W600_RADIO_UART,
    W600_RADIO_RX_OVERFLOW, W600_RADIO_CLOCK, W600_RADIO_TX_TIMEOUT,
    W600_RADIO_ENCODE, W600_RADIO_EXPIRED_PARTIAL, W600_RADIO_STOPPED_PARTIAL
};
enum w600_radio_tx_phase {
    W600_RADIO_IDLE, W600_RADIO_SPACING, W600_RADIO_WAKE,
    W600_RADIO_BYTES, W600_RADIO_TC, W600_RADIO_STOPPED
};
enum { W600_RADIO_RING_BYTES=512, W600_RADIO_POLL_BYTES=64,
       W600_RADIO_TX_BUDGET=16, W600_RADIO_STALLED_POLLS=100000 };

/* One owner in privileged Thread mode; IRQ only produces ring bytes/errors.
 * Caller supplies one fresh wrapping monotonic millisecond clock per call.
 * Deadlines are cooperative checks, not hard bounds through IRQ/debug stalls.
 * Fields are observations, not Zigbee membership or physical valve position. */
struct w600_radio {
    struct w600_endpoint endpoint;
    struct w600_pairing pairing;
    struct w600_decoder decoder;
    struct w600_frame request, reply, management;
    uint8_t wire[W600_MAX_FRAME], ring[W600_RADIO_RING_BYTES];
    volatile uint32_t rx_head, rx_tail, last_rx_at, uart_errors, error;
    uint32_t last_now, stalled_polls, tx_phase_at, tx_started_at, last_tx_at;
    uint32_t management_rx_cutoff, rx_frame_count, rx_ignored_count;
    uint32_t tx_reply_count, tx_management_count, tx_byte_count;
    uint32_t partial_reset_count, management_cancel_count, pairing_events;
    uint16_t wire_length, wire_position;
    uint8_t next_sequence;
    enum w600_radio_tx_phase tx_phase;
    bool initialized, reply_pending, management_pending, management_sent;
    bool tx_management, any_tx_completed;
    uint32_t tx_temperature_count;
    bool temperature_pending, tx_temperature;
    uint32_t tx_control_count;
    bool control_pending, tx_control;
    uint32_t tx_target_count;
    bool target_pending, tx_target;
    uint32_t target_change_count, last_target_change_at;
#ifdef W600_OTA_RECEIVER
    struct w600_ota *ota;
    bool reply_ota,tx_ota;
#endif
};

/* After verified w600_board_init, with IRQs masked. Enables UART RX/error
 * sources only. Caller owns UART3_4_IRQn=27 (vector43), NVIC and SysTick. */
bool w600_radio_init(struct w600_radio *, uint32_t now);
void w600_radio_irq(struct w600_radio *, uint32_t now);
/* Bounded, nonblocking foreground work. False is stopped or a latched error. */
bool w600_radio_poll(struct w600_radio *, uint32_t now);
/* Explicit intent only; rejects busy/pending input or an active local window.
 * True means prepared, NOT transmitted. One send attempt; never a retry. */
bool w600_radio_request_pairing(struct w600_radio *, uint32_t now);
/* Queue the current available local reading (kind5/0706), coalescing updates.
 * Replies and explicit pairing go first. True is queued, never RF delivery.
 * Caller owns sensor freshness; clearing LOCAL cancels an unsent report. */
bool w600_radio_report_temperature(struct w600_radio *);
/* Queue confirmed calibration success/fault and known mode, sensor selection and opening.
 * Unreferenced idle sends reserved 071c=ffff for the custom coordinator adapter;
 * pending/busy calibration never emits that reference-loss marker.
 * Coalesces changes; completion means UART transmission, not Zigbee delivery. */
bool w600_radio_report_control(struct w600_radio *);
/* Queue the current target after a local input; replies/pairing still go first. */
bool w600_radio_report_target(struct w600_radio *);
/* Disables UART sources/UE and releases PA11; never writes NVIC or motor pins.
 * Stopping a partially transmitted frame latches STOPPED_PARTIAL. */
void w600_radio_stop(struct w600_radio *);
#endif
