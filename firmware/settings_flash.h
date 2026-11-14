#ifndef W600_SETTINGS_FLASH_H
#define W600_SETTINGS_FLASH_H
#include <stdint.h>
#include <stddef.h>

#ifdef W600_NV_TEST
#define W600_NV_RAM
#else
#define W600_NV_RAM __attribute__((section(".nvcode"),noinline))
#endif
enum w600_nv_status {
    W600_NV_OK, W600_NV_REQUEST, W600_NV_CONTEXT, W600_NV_CLOCK,
    W600_NV_MOTOR, W600_NV_VOLTAGE, W600_NV_PROTECTED, W600_NV_CRC,
    W600_NV_CONTROLLER, W600_NV_TIMEOUT, W600_NV_PROGRAM, W600_NV_VERIFY,
    W600_NV_LOCK, W600_NV_BUSY_FATAL
};
struct w600_nv_request {
    const uint32_t *image;
    const volatile uint32_t *ticks;
    uint32_t old_crc,image_crc,pvd_ready_at;
    unsigned slot;
};
struct w600_nv_report {
    uint32_t status,erases,stores,verified,final_sr,final_cr,low_voltage;
};
/* SRAM-only worker, not a live-app adapter. Caller owns private buffers,
 * RAM vectors + every reachable UART/SysTick/fault handler, PVD level4 enabled
 * and settled>=2ms, IRQs enabled, stopped motor/ADC/DMA/display, stable clocks.
 * OnlyEE00/F000 are writable. No retry/reset or other region is supported.
 * A busy timeout enters the RAM fault stop; it must not return to FLASH. */
enum w600_nv_status W600_NV_RAM w600_settings_flash_write(
    const struct w600_nv_request *,struct w600_nv_report *);
/* Same SRAM/context checks; OTA caller owns complete-image authorization and
 * descriptor/marker ordering. Never accepts active code or settings pages. */
enum w600_nv_status W600_NV_RAM w600_ota_flash_page_write(
    const struct w600_nv_request *,struct w600_nv_report *,uint32_t address);
#ifndef W600_NV_TEST
void W600_NV_RAM w600_settings_flash_fault(void) __attribute__((noreturn));
#endif
#endif
