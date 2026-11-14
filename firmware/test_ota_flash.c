#include "ota_flash.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static uint32_t flash[32768],writes[4],ticks;
static unsigned count,fail_at,corrupt_at,authorizations,deny_at;
static bool permitted,corrupt_previous;
static uint32_t crc32(const void *data)
{
    const uint8_t *p=data;uint32_t crc=~0u;
    for(unsigned i=0;i<512;++i) {
        crc^=p[i];
        for(unsigned b=0;b<8;++b)crc=crc&1u ? (crc>>1)^0xedb88320u : crc>>1;
    }
    return ~crc;
}
static uint16_t crc16(const void *data,unsigned n)
{
    const uint8_t *p=data;uint16_t crc=0;
    for(unsigned i=0;i<n;++i) {
        crc^=(uint16_t)p[i]<<8;
        for(unsigned b=0;b<8;++b)crc=crc&0x8000u ? (uint16_t)((crc<<1)^0x1021u) : (uint16_t)(crc<<1);
    }
    return crc;
}
static uint32_t *at(uint32_t address)
{ assert(address>=0x08000000u && address<=0x0801fe00u);return flash+(address-0x08000000u)/4; }
bool ota_flash_test_read(uint32_t address,uint8_t out[512])
{ assert(!(address&511u));memcpy(out,at(address),512);return true; }
bool ota_flash_test_replace(uint32_t address,uint32_t old_crc,const uint32_t image[128])
{
    assert(count<4);writes[count++]=address;
    if(count==fail_at || crc32(at(address))!=old_crc)return false;
    memcpy(at(address),image,512);
    if(count==corrupt_at)at(address)[3]^=1;
    if(count==2 && corrupt_previous)at(0x0801fc00)[0]^=1;
    return true;
}
static bool authorization(void *context,uint32_t stage,uint32_t length,uint32_t revision)
{
    assert(context==&ticks && stage==W600_OTA_FLASH_STAGE && length>=192 &&
           length<=W600_OTA_FLASH_IMAGE_MAX && revision==20);
    ++authorizations;
    return permitted && authorizations!=deny_at;
}
static void reset(void)
{
    memset(flash,255,sizeof(flash));memset(writes,0,sizeof(writes));
    count=fail_at=corrupt_at=authorizations=deny_at=0;permitted=true;corrupt_previous=false;
    assert(w600_ota_flash_init(&ticks,authorization,&ticks));
    uint8_t *image=(uint8_t *)at(W600_OTA_FLASH_STAGE);
    for(unsigned i=0;i<768;++i)image[i]=(uint8_t)i;
    at(W600_OTA_FLASH_STAGE)[0]=0x20007000;at(W600_OTA_FLASH_STAGE)[1]=0x08002009;
}
static bool commit(void)
{ return w600_ota_flash_commit(768,crc16(at(W600_OTA_FLASH_STAGE),768),20); }
static void unarmed(void)
{ assert(*at(0x08001e00)==UINT32_MAX); }
int main(void)
{
    static uint32_t workspace[2][128];
    reset();assert(!commit() && !count);
    assert(!w600_ota_flash_set_workspace(NULL));
    assert(w600_ota_flash_set_workspace(workspace));
    assert(w600_ota_flash_restart_safe());
    assert(w600_ota_flash_set_workspace(workspace));
    static uint32_t other_workspace[2][128];
    assert(!w600_ota_flash_set_workspace(other_workspace));
    reset();assert(w600_ota_flash_init(&ticks,NULL,NULL));assert(!commit());assert(!count);unarmed();
    reset();permitted=false;assert(!commit());assert(!count && authorizations==1);unarmed();
    reset();deny_at=2;assert(!commit());assert(count==2 && authorizations==2);unarmed();
    reset();assert(commit());assert(count==3 && authorizations==2);
    assert(writes[0]==0x0801fc00 && writes[1]==0x0801fe00 && writes[2]==0x08001e00);
    assert(*at(0x08001e00)==0x67890123);
    assert(!w600_ota_flash_restart_safe());
    assert(!memcmp(at(0x0801fc00),at(0x0801fe00),512));
    uint32_t *d=at(0x0801fc00);
    assert(d[0]==0 && d[1]==W600_OTA_FLASH_STAGE && d[2]==768 &&
           (d[3]>>16)==0xaa && (d[4]>>16)==crc16(d,18));
    assert(!commit() && count==3);
    for(unsigned failure=1;failure<=3;++failure) {
        reset();fail_at=failure;assert(!commit());assert(count==failure);unarmed();
        reset();corrupt_at=failure;assert(!commit());assert(count==failure);
        if(failure<3)unarmed();
    }
    reset();corrupt_previous=true;assert(!commit() && count==2);unarmed();
    reset();assert(!w600_ota_flash_commit(0,0,20));assert(!w600_ota_flash_commit(188,0,20));
    assert(!w600_ota_flash_commit(193,0,20));assert(!w600_ota_flash_commit(W600_OTA_FLASH_IMAGE_MAX+4,0,20));
    assert(!w600_ota_flash_commit(768,0,20));assert(!count);unarmed();
    reset();at(W600_OTA_FLASH_STAGE)[1]=0x08002008;assert(!commit() && !count);
    reset();at(W600_OTA_FLASH_STAGE)[0]=0x20007008;assert(!commit() && !count);
    reset();at(W600_OTA_FLASH_STAGE)[1]=0x08002301;assert(!commit() && !count);
    uint32_t image[128];memset(image,0x55,sizeof(image));
    reset();uint32_t old_crc=crc32(at(W600_OTA_FLASH_STAGE));
    assert(w600_ota_flash_write_page(W600_OTA_FLASH_STAGE,old_crc,image));assert(count==1);
    assert(!memcmp(at(W600_OTA_FLASH_STAGE),image,512));
    assert(!w600_ota_flash_write_page(W600_OTA_FLASH_STAGE,old_crc,image));
    reset();assert(w600_ota_flash_write_page(W600_OTA_FLASH_STAGE_END-512,crc32(at(W600_OTA_FLASH_STAGE_END-512)),image));
    reset();const uint32_t bad[]={W600_OTA_FLASH_STAGE-512,W600_OTA_FLASH_STAGE+4,W600_OTA_FLASH_STAGE_END,
        0x0800ee00,0x08001e00,0x0801fc00,0x0801fe00,UINT32_MAX};
    for(unsigned i=0;i<sizeof(bad)/sizeof(*bad);++i)
        assert(!w600_ota_flash_write_page(bad[i],0,image));
    assert(!count);
    reset();*at(0x08001e00)=0x67890123;
    assert(!w600_ota_flash_write_page(W600_OTA_FLASH_STAGE,crc32(at(W600_OTA_FLASH_STAGE)),image));assert(!commit() && !count);
    reset();at(0x08001e00)[127]=0;assert(!commit() && !count);
    assert(!w600_ota_flash_restart_safe());
    reset();assert(w600_ota_flash_commit(W600_OTA_FLASH_IMAGE_MAX,
        crc16(at(W600_OTA_FLASH_STAGE),W600_OTA_FLASH_IMAGE_MAX),20));assert(count==3);
    puts("OTA flash backend: bounds, authorization, CRC/vector checks, descriptor and marker ordering PASS");
}
