#ifndef W600_SETTINGS_APP_H
#define W600_SETTINGS_APP_H
#include "../replacement-control/w600_settings_service.h"
#include "settings_flash.h"

/* Serialized foreground calls; init runs masked, poll runs with IRQs enabled.
 * ready must exclude active/pending motor, ADC, display and radio transfers. */
bool w600_settings_app_init(struct w600_endpoint *,const volatile uint32_t *ticks);
void w600_settings_app_poll(const struct w600_endpoint *,uint32_t now,bool ready);
/* Exclusive until reset: update reception borrows the settings buffers/vectors.
 * No subsequent settings save is allowed, including after a failed update. */
uint32_t (*w600_settings_app_ota_takeover(void))[128];
bool w600_settings_app_ota_replace(uint32_t address,uint32_t old_crc,const uint32_t image[128]);
/* Read-only diagnostic state, not proof of durable storage unless SAVED/LOADED. */
extern struct w600_settings_service w600_settings_state;
extern struct w600_nv_report w600_settings_write_report;
#endif
