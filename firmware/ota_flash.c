#include "ota_flash.h"

#define MARKER 0x08001e00u
#define DESCRIPTOR0 0x0801fc00u
#define DESCRIPTOR1 0x0801fe00u
#define MAGIC 0x67890123u
static w600_ota_authorize authorize;
static void *authorization_context;
static uint32_t (*workspace)[128];
#define page (workspace[0])
#define readback (workspace[1])

static bool stage_page(uint32_t address)
{ return !(address&511u) && address>=W600_OTA_FLASH_STAGE && address<W600_OTA_FLASH_STAGE_END; }
static bool allowed_page(uint32_t address)
{ return stage_page(address) || address==MARKER || address==DESCRIPTOR0 || address==DESCRIPTOR1; }
static uint32_t crc32(const void *data)
{
    const uint8_t *p=data;
    uint32_t crc=UINT32_MAX;
    for(unsigned i=0;i<512;++i) {
        crc^=p[i];
        for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
static uint16_t crc16(uint16_t crc,const uint8_t *p,size_t length)
{
    for(size_t i=0;i<length;++i) {
        crc^=(uint16_t)p[i]<<8;
        for(unsigned bit=0;bit<8;++bit)crc=(uint16_t)((crc<<1)^((crc&0x8000u) ? 0x1021u : 0));
    }
    return crc;
}

#ifdef W600_OTA_FLASH_TEST
bool ota_flash_test_read(uint32_t,uint8_t[512]);
bool ota_flash_test_replace(uint32_t,uint32_t,const uint32_t[128]);
static bool board_init(const volatile uint32_t *ticks) { return ticks!=NULL; }
static bool board_read(uint32_t address,uint8_t out[512]) { return ota_flash_test_read(address,out); }
static bool board_replace(uint32_t address,uint32_t old_crc,const uint32_t image[128])
{ return ota_flash_test_replace(address,old_crc,image); }
#else
#include "ciu32l071.h"
bool w600_settings_app_ota_replace(uint32_t,uint32_t,const uint32_t[128]);
static bool board_init(const volatile uint32_t *ticks) { return ticks!=NULL; }
static bool board_read(uint32_t address,uint8_t out[512])
{
    if(FLASH->SR&0x10000u)return false;
    const volatile uint8_t *p=(const volatile uint8_t *)(uintptr_t)address;
    for(unsigned i=0;i<512;++i)out[i]=p[i];
    return true;
}
static bool board_replace(uint32_t address,uint32_t old_crc,const uint32_t image[128])
{ return w600_settings_app_ota_replace(address,old_crc,image); }
#endif

bool w600_ota_flash_init(const volatile uint32_t *ticks,w600_ota_authorize policy,void *context)
{
    if(!board_init(ticks))return false;
    authorize=policy;authorization_context=context;
    return true;
}
bool w600_ota_flash_set_workspace(uint32_t pages[2][128])
{
    uintptr_t p=(uintptr_t)pages;
    if(!pages || (p&3u) || (workspace && workspace!=pages))return false;
#ifndef W600_OTA_FLASH_TEST
    if(p<0x20000000u || p>0x20006000u-1024u)return false;
#endif
    workspace=pages;
    return true;
}
bool w600_ota_flash_read_page(uint32_t address,uint8_t out[512])
{ return out && allowed_page(address) && board_read(address,out); }
static bool marker_erased(void)
{
    if(!workspace)return false;
    if(!w600_ota_flash_read_page(MARKER,(uint8_t *)readback))return false;
    for(unsigned i=0;i<128;++i)if(readback[i]!=UINT32_MAX)return false;
    return true;
}
bool w600_ota_flash_restart_safe(void)
{ return marker_erased(); }
static bool equal(const uint32_t *a,const uint32_t *b)
{
    for(unsigned i=0;i<128;++i)if(a[i]!=b[i])return false;
    return true;
}
static bool replace(uint32_t address,uint32_t old_crc,const uint32_t image[128])
{
    if(!workspace || !allowed_page(address))return false;
    if(image!=page)for(unsigned i=0;i<128;++i)page[i]=image[i];
    if(!board_replace(address,old_crc,page) ||
       !w600_ota_flash_read_page(address,(uint8_t *)readback))return false;
    return equal(page,readback);
}
bool w600_ota_flash_write_page(uint32_t address,uint32_t old_crc,const uint32_t image[128])
{
    return image && stage_page(address) && marker_erased() && replace(address,old_crc,image);
}
static bool verified_image(uint32_t length,uint16_t expected)
{
    if(length<192 || (length&3u) || length>W600_OTA_FLASH_IMAGE_MAX)return false;
    uint16_t crc=0;
    for(uint32_t offset=0;offset<length;offset+=512) {
        if(!w600_ota_flash_read_page(W600_OTA_FLASH_STAGE+offset,(uint8_t *)readback))return false;
        if(!offset) {
            uint32_t sp=readback[0],reset=readback[1];
            if((sp&7u) || sp<0x20000008u || sp>0x20007000u || !(reset&1u) ||
               (reset&~1u)<0x08002000u || (reset&~1u)>=0x08002000u+length)return false;
        }
        uint32_t count=length-offset;
        if(count>512)count=512;
        crc=crc16(crc,(const uint8_t *)readback,count);
    }
    return crc==expected;
}
bool w600_ota_flash_commit(uint32_t length,uint16_t crc,uint32_t revision)
{
    if(!authorize || !marker_erased() || !verified_image(length,crc) ||
       !authorize(authorization_context,W600_OTA_FLASH_STAGE,length,revision))return false;
    for(unsigned slot=0;slot<2;++slot) {
        uint32_t address=slot ? DESCRIPTOR1 : DESCRIPTOR0;
        if(!w600_ota_flash_read_page(address,(uint8_t *)readback))return false;
        uint32_t old_crc=crc32(readback);
        for(unsigned i=0;i<128;++i)page[i]=UINT32_MAX;
        page[0]=0;page[1]=W600_OTA_FLASH_STAGE;page[2]=length;
        page[3]=(uint32_t)crc|0x00aa0000u;page[4]=0;
        page[4]=(uint32_t)crc16(0,(const uint8_t *)page,18)<<16;
        if(!replace(address,old_crc,page))return false;
    }
    if(!verified_image(length,crc) ||
       !authorize(authorization_context,W600_OTA_FLASH_STAGE,length,revision))return false;
    for(unsigned slot=0;slot<2;++slot)
        if(!w600_ota_flash_read_page(slot ? DESCRIPTOR1 : DESCRIPTOR0,(uint8_t *)readback) ||
           !equal(page,readback))return false;
    if(!marker_erased())return false;
    uint32_t old_crc=crc32(readback);
    for(unsigned i=0;i<128;++i)page[i]=UINT32_MAX;
    page[0]=MAGIC;
    return replace(MARKER,old_crc,page);
}
