#ifndef W600_OTA_AUTH_H
#define W600_OTA_AUTH_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

struct w600_ota_authorization {
    uint32_t length,revision;
    uint8_t digest[32];
};
typedef bool (*w600_ota_stage_reader)(void *,uint32_t address,void *out,size_t length);
/* Single pinned image only. Caller supplies a compile-time target configuration
 * and owns immutable staging flash throughout this serialized foreground call.
 * NULL configuration denies all images; this is not a signature trust store. */
bool w600_ota_authorize_image(const struct w600_ota_authorization *,
                             w600_ota_stage_reader,void *context,uint32_t stage,
                             uint32_t length,uint32_t revision);
#ifdef W600_OTA_AUTH_TEST
bool w600_ota_auth_test_sha256(w600_ota_stage_reader,void *,uint32_t address,
                              uint32_t length,uint8_t digest[32]);
#endif
#endif
