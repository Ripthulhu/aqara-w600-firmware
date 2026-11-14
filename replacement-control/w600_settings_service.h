#ifndef W600_SETTINGS_SERVICE_H
#define W600_SETTINGS_SERVICE_H
#include "w600_settings_store.h"

struct w600_settings_service {
    struct w600_settings pending;
    uint32_t changed_at, attempts, saves;
    enum w600_settings_status status;
    bool initialized, dirty, blocked;
};
/* Zero once. Boot after controller init and before commands/sensors. Bad storage is
 * read-only until restart; neither defaults nor a mechanical reference is saved. */
bool w600_settings_service_init(struct w600_settings_service *,
    const struct w600_settings_io *,struct w600_settings_workspace *,struct w600_endpoint *);
/* Foreground owner only. Coalesce changes for2s; ready means peripherals and
 * controller are idle. One failed save latches; no automatic erase/retry loop. */
void w600_settings_service_poll(struct w600_settings_service *,
    const struct w600_settings_io *,struct w600_settings_workspace *,
    const struct w600_endpoint *,uint32_t now,bool ready);
#endif
