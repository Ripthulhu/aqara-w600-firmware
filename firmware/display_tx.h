#ifndef W600_DISPLAY_TX_H
#define W600_DISPLAY_TX_H
#include <stdbool.h>
#include <stdint.h>

enum w600_display_tx_error {
    W600_DISPLAY_TX_OK, W600_DISPLAY_TX_CONTEXT, W600_DISPLAY_TX_UART,
    W600_DISPLAY_TX_CLOCK, W600_DISPLAY_TX_TIMEOUT
};
enum w600_display_tx_phase {
    W600_DISPLAY_TX_IDLE, W600_DISPLAY_TX_DATA, W600_DISPLAY_TX_DATA_TC,
    W600_DISPLAY_TX_GAP, W600_DISPLAY_TX_CONTROL, W600_DISPLAY_TX_FINAL_TC,
    W600_DISPLAY_TX_FAILED
};
enum { W600_DISPLAY_TX_STALLED_POLLS = 100000 };

/* Zero once before init. One exclusive, serialized privileged Thread owner;
 * no ISR calls or other UART2 writers. Caller owns the idle UART, PB6/power,
 * clocks, and a fresh wrapping monotonic millisecond clock on every call. */
struct w600_display_tx {
    uint32_t last_now, started_at, gap_at, stalled_polls;
    enum w600_display_tx_error error;
    enum w600_display_tx_phase phase;
    uint8_t frame[4], position;
    bool initialized;
};

/* IRQs masked for init only. Prepared UART2: TX-only 19200 8O1, PB6 AF1
 * open-drain/pulldown; UART2 IRQ disabled and not pending. No setup writes. */
bool w600_display_tx_init(struct w600_display_tx *, uint32_t now_ms);
/* Copies four data bytes; NULL/busy/failed rejection does not alter a transfer.
 * True means accepted locally, not transmitted or visible on the display. */
bool w600_display_tx_submit(struct w600_display_tx *, const uint8_t frame[4],
                            uint32_t now_ms);
/* At most one TDR byte per poll: 08+data, data TC, >=5ms, 18 FF, final TC.
 * Whole transfer deadline 50ms, never renewed. Checks are cooperative, not a
 * hard bound through CPU/IRQ/debug stalls. IDLE is wire completion only.
 * First error latches; no retries/queue/peripheral disable on context loss.
 * A partial command may already be on the wire. Recovery belongs to caller. */
bool w600_display_tx_poll(struct w600_display_tx *, uint32_t now_ms);
#endif
