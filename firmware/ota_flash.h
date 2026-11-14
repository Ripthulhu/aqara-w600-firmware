#ifndef W600_OTA_FLASH_H
#define W600_OTA_FLASH_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define W600_OTA_FLASH_STAGE 0x08010800u
#define W600_OTA_FLASH_IMAGE_MAX 0xce00u
#define W600_OTA_FLASH_STAGE_END (W600_OTA_FLASH_STAGE+W600_OTA_FLASH_IMAGE_MAX)

/* Must independently validate the staged image against the update policy.
 * The application policy trusts the paired network and checks identity/SHA256. */
typedef bool (*w600_ota_authorize)(void *,uint32_t stage,uint32_t length,uint32_t revision);
bool w600_ota_flash_init(const volatile uint32_t *ticks,w600_ota_authorize,void *context);
/* One-way ownership transfer of two aligned SRAM pages from settings. Caller
 * permanently stops settings access before lending these buffers to OTA. */
bool w600_ota_flash_set_workspace(uint32_t pages[2][128]);
bool w600_ota_flash_read_page(uint32_t address,uint8_t out[512]);
/* No automatic reboot after an uncertain activation-marker write. */
bool w600_ota_flash_restart_safe(void);
/* Serialized idle foreground only; caller quiesces motor/ADC/display/DMA/radio.
 * Public writes are staging-only. old_crc is CRC32 of the existing 512 bytes. */
bool w600_ota_flash_write_page(uint32_t address,uint32_t old_crc,const uint32_t image[128]);
/* Revalidates stage and authorization, writes both descriptors then marker last.
 * NULL authorization denies commit. Never resets; an armed marker is irreversible
 * through this API and the next reset may install the staged application. */
bool w600_ota_flash_commit(uint32_t length,uint16_t crc,uint32_t revision);
#endif
