#include "w600_settings_service.h"

static bool snapshot(const struct w600_endpoint *e,struct w600_settings *s)
{
    if(!e || !e->mode_known || e->mode>1 ||
       ((e->known&W600_KNOWN_TARGET) && (e->target_raw<500 || e->target_raw>3000)) ||
       ((e->known&W600_KNOWN_EXTERNAL_USE) && e->external_use>1))return false;
    *s=(struct w600_settings){.fields=W600_SETTINGS_MODE,.mode=e->mode};
    if(e->known&W600_KNOWN_TARGET){s->fields|=W600_SETTINGS_TARGET;s->target_raw=e->target_raw;}
    if(e->known&W600_KNOWN_EXTERNAL_USE){s->fields|=W600_SETTINGS_SOURCE;s->external_use=e->external_use;}
    return true;
}
static bool same(const struct w600_settings *a,const struct w600_settings *b)
{ return a->fields==b->fields && a->target_raw==b->target_raw &&
         a->mode==b->mode && a->external_use==b->external_use; }
bool w600_settings_service_init(struct w600_settings_service *s,
    const struct w600_settings_io *io,struct w600_settings_workspace *w,struct w600_endpoint *e)
{
    if(!s || s->initialized || !e || !e->control_enabled || !e->mode_known ||
       e->mode || e->known || e->control_updates || e->external_updates || e->request_seen ||
       e->calibration_seen || e->calibrated || e->opening_known || e->moving ||
       e->calibration_busy || e->calibration_pending || e->cancel_calibration ||
       e->control_error || e->calibration_error)return false;
    *s=(struct w600_settings_service){0};
    struct w600_settings loaded;
    s->status=w600_settings_load(io,w,&loaded);
    if(s->status==W600_SETTINGS_LOADED && !w600_settings_restore(&loaded,e))
        s->status=W600_SETTINGS_INVALID;
    s->blocked=s->status!=W600_SETTINGS_LOADED && s->status!=W600_SETTINGS_EMPTY;
    if(!snapshot(e,&s->pending))return false;
    s->initialized=true;
    return true;
}
void w600_settings_service_poll(struct w600_settings_service *s,
    const struct w600_settings_io *io,struct w600_settings_workspace *w,
    const struct w600_endpoint *e,uint32_t now,bool ready)
{
    if(!s || !s->initialized || s->blocked)return;
    struct w600_settings next;
    if(!snapshot(e,&next))return;
    if(!same(&next,&s->pending)) {
        s->pending=next;s->changed_at=now;s->dirty=true;
    }
    if(!s->dirty || !ready || (uint32_t)(now-s->changed_at)<2000u)return;
    ++s->attempts;
    s->status=w600_settings_save(io,w,&s->pending);
    if(s->status==W600_SETTINGS_SAVED || s->status==W600_SETTINGS_UNCHANGED) {
        s->dirty=false;
        if(s->status==W600_SETTINGS_SAVED)++s->saves;
    } else s->blocked=true;
}
