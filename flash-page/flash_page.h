#ifndef CIU_FLASH_PAGE_H
#define CIU_FLASH_PAGE_H
#include <stdint.h>

#define CIU_FLASH_PAGE_BYTES 512u
#define CIU_FLASH_APP_BEGIN 0x08002000u
#define CIU_FLASH_APP_END 0x08010800u
#define CIU_FLASH_MAX_ERASE_TICKS 100u
#define CIU_FLASH_MAX_PROGRAM_TICKS 20u
#define CIU_FLASH_MAX_POLLS 1000000u

enum ciu_flash_status {
    CIU_FLASH_OK, CIU_FLASH_REQUEST, CIU_FLASH_CONTEXT, CIU_FLASH_CLOCK,
    CIU_FLASH_MOTOR, CIU_FLASH_CONTROLLER, CIU_FLASH_PROTECTED,
    CIU_FLASH_OLD_CRC, CIU_FLASH_IMAGE_CRC, CIU_FLASH_TIMEOUT,
    CIU_FLASH_POLL_LIMIT, CIU_FLASH_ERROR, CIU_FLASH_NO_EOP,
    CIU_FLASH_ERASE_VERIFY, CIU_FLASH_WORD_VERIFY, CIU_FLASH_FINAL_CRC,
    CIU_FLASH_LOCK_FAILED
};

struct ciu_flash_request {
    uint32_t address;
    const uint32_t *image; /* Exactly128 SRAM words, stable for the entire call. */
    uint32_t expected_old_crc, image_crc; /* IEEE CRC32, init/final XORFFFFFFFF. */
    uint32_t erase_ticks, program_ticks, poll_limit; /* Explicit nonzero limits. */
};

struct ciu_flash_report {
    uint32_t status, initial_sr, final_sr, final_cr;
    uint32_t erase_issued, word_stores, words_verified, locked;
    uint32_t observed_old_crc, observed_image_crc, observed_final_crc;
};

/* Caller owns an SRAM-only call chain/vector table/stack, stopped motor,
 * disabled DMA/IRQs, stable power/clock and private request/image/report storage.
 * Polled SysTick must already be LOAD47999/CTRL5 at48MHz; this code changes none.
 * Timeout is failure, not rollback. BSY may prevent locking; inspect the report.
 * No retry/reset/watchdog feed is performed. Not a deployable programming runner.
 */
enum ciu_flash_status ciu_flash_replace_page(const struct ciu_flash_request *request,
                                            struct ciu_flash_report *report);
/* Same preflight, no controller writes/unlock/erase/program. SRAM report only. */
enum ciu_flash_status ciu_flash_check_page(const struct ciu_flash_request *request,
                                          struct ciu_flash_report *report);
#endif
