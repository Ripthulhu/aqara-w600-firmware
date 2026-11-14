/* Link/disassembly fixture only. These hooks do NOT implement valve safety. */
#include <stdint.h>
volatile uint8_t fixture_data[7] = {1, 2, 3, 4, 5, 6, 7};
volatile uint8_t fixture_bss[11];
volatile uint8_t fixture_noinit[13] __attribute__((section(".noinit")));

int w600_reset_early(void) { return 1; }
void w600_app_main(void) { fixture_bss[0] = fixture_data[0]; }
void w600_reset_fault(void) { for (;;) __asm volatile ("nop"); }
