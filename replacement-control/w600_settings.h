#ifndef W600_SETTINGS_H
#define W600_SETTINGS_H
#include "w600_endpoint.h"

enum { W600_SETTINGS_BYTES=32, W600_SETTINGS_COMMIT_OFFSET=28,
       W600_SETTINGS_TARGET=1, W600_SETTINGS_SOURCE=2, W600_SETTINGS_MODE=4 };
struct w600_settings {
    uint32_t generation;
    uint16_t target_raw;
    uint8_t fields, mode, external_use;
};

/* Pure record codec, not a FLASH writer. Generation is nonzero and never wraps.
 * No temperatures, position, calibration or forced-open mode are stored.
 * A future writer must retain the old copy, write/verify bytes0..27, then write
 * the commit word at28 and verify the whole new record before reclaiming old.
 */
bool w600_settings_encode(const struct w600_settings *,uint8_t *,size_t);
bool w600_settings_decode(const uint8_t *,size_t,struct w600_settings *);
/* Return selected copy0/1, or-1 for no valid copy/ambiguous equal generations.
 * Failure leaves output unchanged. A damaged or uncommitted copy is ignored. */
int w600_settings_select(const uint8_t *,const uint8_t *,size_t,struct w600_settings *);
/* Boot only: after controller init, before commands, polling or sensor reads.
 * Refuse an already-used endpoint. Learned position remains unknown; Heat still
 * needs fresh selected-source input and a separately established motor reference.
 */
bool w600_settings_restore(const struct w600_settings *,struct w600_endpoint *);
#endif
