#ifndef W600_SETTINGS_STORE_H
#define W600_SETTINGS_STORE_H
#include "w600_settings.h"

enum { W600_SETTINGS_PAGE_BYTES=512, W600_SETTINGS_RECORD_AT=480 };
enum w600_settings_status {
    W600_SETTINGS_LOADED, W600_SETTINGS_EMPTY, W600_SETTINGS_CORRUPT,
    W600_SETTINGS_AMBIGUOUS, W600_SETTINGS_INVALID, W600_SETTINGS_READ_FAILED,
    W600_SETTINGS_FOREIGN, W600_SETTINGS_EXHAUSTED, W600_SETTINGS_UNCHANGED,
    W600_SETTINGS_SAVED, W600_SETTINGS_WRITE_FAILED, W600_SETTINGS_VERIFY_FAILED
};
struct w600_settings_io {
    void *context;
    bool (*read)(void *,unsigned slot,uint8_t out[W600_SETTINGS_PAGE_BYTES]);
    /* Exact reserved slot only; reject a changed old CRC. Program/verify in
     * ascending word order so the record's commit word at508 is written last.
     * The backend owns SRAM execution, UART service, voltage and motor guards.
     * Failure may have changed the target page. Never retry inside the backend. */
    bool (*replace)(void *,unsigned slot,uint32_t old_crc,
                    const uint8_t image[W600_SETTINGS_PAGE_BYTES]);
};
/* Private, non-reentrant scratch: keep it off the small MCU stack. */
struct w600_settings_workspace { uint32_t pages[2][128]; };

/* No record/default is invented on failure; output is changed only on LOADED. */
enum w600_settings_status w600_settings_load(const struct w600_settings_io *,
    struct w600_settings_workspace *,struct w600_settings *out);
/* generation in desired is ignored. Only settings fields are saved; old valid
 * copy is retained. No erase for unchanged settings, including at UINT32_MAX.
 * SAVED requires readback of both pages; failure is not an acknowledgement. */
enum w600_settings_status w600_settings_save(const struct w600_settings_io *,
    struct w600_settings_workspace *,const struct w600_settings *desired);
#endif
