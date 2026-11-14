#ifndef W600_OTA_AUTH_H
#define W600_OTA_AUTH_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { W600_OTA_AUTH_WORDS=56, W600_OTA_IDENTITY_BYTES=64 };
typedef bool (*w600_ota_stage_reader)(void *,uint32_t address,void *out,size_t length);
/* Updates trust the paired Zigbee network, not a signing key. Verify a newer
 * W600 image, retained OTA capability and SHA256 before activation. The digest
 * detects corruption; it is not a signature. Caller owns staging and scratch. */
bool w600_ota_authorize_image(w600_ota_stage_reader,void *context,uint32_t stage,
                             uint32_t length,uint32_t revision,uint32_t current_revision,
                             uint32_t scratch[W600_OTA_AUTH_WORDS]);
#ifdef W600_OTA_AUTH_TEST
bool w600_ota_auth_test_sha256(w600_ota_stage_reader,void *,uint32_t address,
                              uint32_t length,uint8_t digest[32]);
#endif
#endif
