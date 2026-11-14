#ifndef W600_DISPLAY_H
#define W600_DISPLAY_H
#include <stdbool.h>
#include <stdint.h>

/* Encode0..999 deciC into caller-owned four display DATA bytes, not UART commands.
 * Preserve byte2 and byte3&E8. Nonzero tenths sets the stock fraction marker;
 * it does not render a separate decimal digit (201 and205 encode identically).
 * No temperature correction, icon interpretation or display/transport state.
 * NULL/out-of-range fails without changing the frame; otherwise four writable
 * bytes are required. */
bool w600_display_encode(uint32_t decic, bool flipped, uint8_t frame[4]);
#endif
