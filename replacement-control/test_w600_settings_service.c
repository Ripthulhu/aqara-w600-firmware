#include "w600_settings_service.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint8_t pages[2][512];
static unsigned writes,reads;
static bool fail_write,fail_read;
static bool read_page(void *c,unsigned slot,uint8_t out[512])
{ assert(c==pages && slot<2);++reads;if(fail_read)return false;memcpy(out,pages[slot],512);return true; }
static bool replace(void *c,unsigned slot,uint32_t old_crc,const uint8_t image[512])
{ (void)old_crc;assert(c==pages && slot<2);++writes;if(fail_write)return false;memcpy(pages[slot],image,512);return true; }
static const struct w600_settings_io io={pages,read_page,replace};
static struct w600_settings_workspace workspace;
static struct w600_settings_service service;
static struct w600_endpoint endpoint;
static void setup(void)
{
    memset(pages,255,sizeof pages);writes=reads=0;fail_write=fail_read=false;
    service=(struct w600_settings_service){0};
    endpoint=(struct w600_endpoint){.control_enabled=true,.mode_known=true};
    assert(w600_settings_service_init(&service,&io,&workspace,&endpoint));
    assert(service.status==W600_SETTINGS_EMPTY && !service.dirty && !service.blocked);
}
static void poll(uint32_t now,bool ready)
{ w600_settings_service_poll(&service,&io,&workspace,&endpoint,now,ready); }
static void change(uint16_t target)
{ endpoint.known|=W600_KNOWN_TARGET;endpoint.target_raw=target; }
int main(void)
{
    setup();poll(100000,true);assert(!writes && !service.dirty);
    change(2100);poll(100,true);assert(service.dirty && !writes);
    change(2200);poll(1500,true);poll(3499,true);assert(!writes);
    poll(3500,false);assert(!writes);poll(5000,true);
    assert(writes==1 && service.saves==1 && service.attempts==1 && !service.dirty);
    endpoint.external_temp_raw=1900;endpoint.local_temp_raw=2000;
    endpoint.known|=W600_KNOWN_EXTERNAL_TEMP|W600_KNOWN_LOCAL_TEMP;
    endpoint.opening_permille=500;endpoint.calibrated=true;endpoint.opening_known=true;
    poll(9000,true);assert(writes==1 && !service.dirty);
    endpoint.mode=2;change(2300);poll(10000,true);poll(13000,true);assert(writes==1);
    endpoint.mode=1;endpoint.external_use=1;endpoint.known|=W600_KNOWN_EXTERNAL_USE;
    poll(14000,true);poll(16000,true);assert(writes==2);
    service=(struct w600_settings_service){0};
    endpoint=(struct w600_endpoint){.control_enabled=true,.mode_known=true};
    assert(w600_settings_service_init(&service,&io,&workspace,&endpoint));
    assert(service.status==W600_SETTINGS_LOADED && endpoint.mode==1 && endpoint.target_raw==2300 &&
        endpoint.external_use==1 && endpoint.known==(W600_KNOWN_TARGET|W600_KNOWN_EXTERNAL_USE) &&
        !endpoint.calibrated && !endpoint.opening_known && !endpoint.moving &&
        !endpoint.external_temp_raw && !endpoint.local_temp_raw);
    poll(100000,true);assert(writes==2);
    assert(!w600_settings_service_init(&service,&io,&workspace,&endpoint));

    setup();change(2000);poll(UINT32_MAX-1000u,true);poll(998,true);assert(!writes);
    poll(999,true);assert(writes==1);
    endpoint.mode=0;poll(1500,true);poll(3500,true);assert(writes==1); /* unchanged */
    change(2100);poll(4000,false);endpoint.mode=2;poll(7000,true);assert(writes==1);
    endpoint.mode=0;poll(7001,true);assert(writes==2);

    setup();fail_write=true;change(2100);poll(0,true);poll(2000,true);
    assert(writes==1 && service.blocked && service.status==W600_SETTINGS_WRITE_FAILED);
    change(2200);poll(3000,true);poll(100000,true);assert(writes==1);
    setup();pages[0][0]=0;
    service=(struct w600_settings_service){0};
    assert(w600_settings_service_init(&service,&io,&workspace,&endpoint));
    assert(service.blocked && service.status==W600_SETTINGS_CORRUPT);
    change(2100);poll(0,true);poll(5000,true);assert(!writes);
    setup();fail_read=true;service=(struct w600_settings_service){0};
    assert(w600_settings_service_init(&service,&io,&workspace,&endpoint));
    assert(service.blocked && service.status==W600_SETTINGS_READ_FAILED);
    poll(9000,true);assert(!writes);
    setup();endpoint.known=W600_KNOWN_TARGET;endpoint.target_raw=100;
    poll(0,true);poll(9000,true);assert(!writes);
    puts("PASS settings service: boot restore, no invented defaults/reference, debounce/wrap, busy/forced deferral, one-shot failure");
}
