#ifndef W600_DISPLAY_BOARD_H
#define W600_DISPLAY_BOARD_H
#include <stdbool.h>

/* Exclusive reset-lifetime owner; privileged Thread with IRQs masked for both
 * calls. Init after board/HHL power setup, motor normal-off, PA4/PB6 analog,
 * no-pull/PP/AF0, UART2 gated/reset clear and IRQ26 disabled/not pending.
 * One admitted attempt per reset; a rejected context changes nothing.
 * On any admitted init/TX failure call stop; never retry init this reset. */
bool w600_display_board_init(void);
/* Also valid after partial init. Disables/resets/gates owned UART2, releases
 * PB6 to analog/no-pull/PP/AF0 and leaves PA4 actively HIGH PP/pullup.
 * Not a full cold-profile restore or proof the display visibly slept.
 * False means cleanup is not proved; no motor/power/other UART is changed.
 * Finite waits do not bound CPU/bus stalls or guarantee a complete wire byte. */
bool w600_display_board_stop(void);
#endif
