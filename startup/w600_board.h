#ifndef W600_BOARD_H
#define W600_BOARD_H
#include <stdint.h>

enum w600_board_result { W600_BOARD_OK, W600_BOARD_CONTEXT, W600_BOARD_UART_BUSY,
    W600_BOARD_UART_STOP, W600_BOARD_CLOCK_GATE, W600_BOARD_CLOCK,
    W600_BOARD_UART_RESET, W600_BOARD_PIN_CONFIG, W600_BOARD_UART_CONFIG,
    W600_BOARD_UART_READY, W600_BOARD_UART_SETTLE };
struct w600_board_status {
    uint32_t magic, version, size, stage, error, clock_result;
    uint32_t cfg_before, cfg_after, apb1_before, apb2_before, ahb_before;
    uint32_t uart2_before, uart3_before, uart2_isr, uart3_isr;
    uint32_t uart3_cr1, uart3_cr2, uart3_cr3, uart3_brr, final_off;
    uint32_t reserved[12];
};

/* Owns only an already motor-inhibited, masked-IRQ RAM takeover. Not a reset
 * handler: caller supplies vectors/stack and quiesces all DMA channels first.
 * UART2/3 drain conservatively; no resumed stock traffic is promised. */
int w600_board_init(volatile struct w600_board_status *status);
#endif
