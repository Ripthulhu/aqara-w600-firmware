/* Real receiver/policy/flash backend on memory-backed flash; not a radio or bootloader emulator. */
#include "ota_receiver.h"
#include "ota_auth.h"
#include "ota_flash.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
static uint32_t flash[32768],work[2][128],ticks,current=46;
static struct w600_ota ota;
static uint8_t image[1024],sequence;
static unsigned writes,marker_writes;
static uint32_t *at(uint32_t a) {assert(a>=0x08000000 && a<0x08020000);return flash+(a-0x08000000)/4;}
static uint32_t crc32(const void *p)
{
    uint32_t c=~0u;const uint8_t *b=p;
    for(unsigned i=0;i<512;++i){c^=b[i];for(unsigned j=0;j<8;++j)c=(c>>1)^((0u-(c&1u))&0xedb88320u);}
    return ~c;
}
static uint16_t crc16(const void *p,unsigned n)
{
    uint16_t c=0;const uint8_t *b=p;
    for(unsigned i=0;i<n;++i){c^=(uint16_t)b[i]<<8;for(unsigned j=0;j<8;++j)c=(uint16_t)((c<<1)^((c&0x8000u)?0x1021u:0));}
    return c;
}
bool ota_flash_test_read(uint32_t a,uint8_t out[512]) {memcpy(out,at(a),512);return true;}
bool ota_flash_test_replace(uint32_t a,uint32_t old,const uint32_t p[128])
{
    assert(crc32(at(a))==old);++writes;
    if(a==0x08001e00){++marker_writes;assert(!memcmp(at(0x0801fc00),at(0x0801fe00),512));}
    memcpy(at(a),p,512);return true;
}
static bool quiet(void *p) {(void)p;return true;}
static bool read_stage(void *p,uint32_t a,void *out,size_t n)
{(void)p;assert(a>=W600_OTA_STAGE && a+n<=W600_OTA_STAGE+W600_OTA_MAX_IMAGE);memcpy(out,(uint8_t *)flash+a-0x08000000,n);return true;}
static bool write_stage(void *p,uint32_t a,const uint32_t data[128])
{(void)p;return w600_ota_flash_write_page(a,crc32(at(a)),data);}
static bool authorize(void *p,uint32_t a,uint32_t n,uint32_t r)
{assert(p==&ota);return w600_ota_authorize_image(read_stage,p,a,n,r,current,ota.page);}
static bool commit(void *p,uint32_t n,uint16_t c,uint32_t r)
{assert(p==&ota);return w600_ota_flash_commit(n,c,r);}
static const struct w600_ota_ops ops={quiet,write_stage,read_stage,authorize,commit};
static bool read_image(void *p,uint32_t a,void *out,size_t n)
{(void)p;assert(a>=W600_OTA_STAGE && a+n<=W600_OTA_STAGE+sizeof image);memcpy(out,image+a-W600_OTA_STAGE,n);return true;}
static void make_image(uint32_t revision)
{
    for(unsigned i=0;i<sizeof image;++i)image[i]=(uint8_t)(i+revision);
    const uint32_t vectors[]={0x20007000,0x080020c1};memcpy(image,vectors,sizeof vectors);
    const uint32_t id[]={0x30303657,0x5549432d,0x4f2d3233,0x00314154,1,revision,sizeof image,1};
    memcpy(image+sizeof image-64,id,sizeof id);
    assert(w600_ota_auth_test_sha256(read_image,NULL,W600_OTA_STAGE,sizeof image-32,image+sizeof image-32));
}
static void reset_receiver(void)
{
    w600_ota_init(&ota,&ops,&ota);sequence=1;
    assert(w600_ota_flash_init(&ticks,authorize,&ota));
    assert(w600_ota_flash_set_workspace(work));
}
static void response(const void *data,unsigned n)
{
    struct w600_frame in={.kind=7,.sequence=ota.tx.sequence,.length=(uint16_t)(n+5),.payload={0x40,1,0,0x0b}};
    in.payload[4]=(uint8_t)n;memcpy(in.payload+5,data,n);
    w600_ota_sent(&ota,++ticks);assert(w600_ota_process(&ota,&in,++ticks,&sequence));
}
static bool transfer(uint32_t revision)
{
    uint8_t original[1024];memcpy(original,at(0x08002000),sizeof original);
    unsigned before_marker=marker_writes;
    reset_receiver();
    struct w600_frame start={.kind=2,.sequence=8,.length=4,.payload={0x40,0,1,1}};
    assert(w600_ota_process(&ota,&start,++ticks,&sequence));
    w600_ota_sent(&ota,++ticks);w600_ota_poll(&ota,++ticks,&sequence);
    uint32_t header[32]={0};header[0]=0x6c756d69;header[2]=128;header[8]=sizeof image;
    header[9]=crc16(image,sizeof image);header[10]=revision;header[31]=crc16(header,124);
    response(header,sizeof header);
    while(ota.state==W600_OTA_BODY){unsigned off=ota.received,n=ota.expected_length;response(image+off,n);}
    assert(!memcmp(original,at(0x08002000),sizeof original));
    assert(*at(0x08001e00)==UINT32_MAX && marker_writes==before_marker);
    if(ota.state==W600_OTA_FAILED){assert(ota.error==W600_OTA_AUTH);return false;}
    assert(ota.state==W600_OTA_FINISH);
    struct w600_frame end={.kind=7,.sequence=ota.tx.sequence,.length=8,.payload={0x40,2,0,7,0,0,0,100}};
    w600_ota_sent(&ota,++ticks);assert(w600_ota_process(&ota,&end,++ticks,&sequence));
    ticks+=99;w600_ota_poll(&ota,ticks,&sequence);assert(!marker_writes || marker_writes==before_marker);
    ++ticks;w600_ota_poll(&ota,ticks,&sequence);
    assert(ota.state==W600_OTA_COMMITTED && marker_writes==before_marker+1);
    assert(!memcmp(original,at(0x08002000),sizeof original));
    assert(*at(0x08001e00)==0x67890123 && at(0x0801fc00)[2]==sizeof image);
    return true;
}
static void simulate_bootloader_install(void)
{
    assert(*at(0x08001e00)==0x67890123);
    memcpy(at(0x08002000),at(W600_OTA_STAGE),sizeof image);
    memset(at(0x08001e00),255,512);
    current=at(0x08002000+sizeof image-64)[5];
}
int main(void)
{
    memset(flash,255,sizeof flash);
    make_image(47);assert(transfer(47));simulate_bootloader_install();assert(current==47);
    assert(!transfer(47)); /* The new installation rejects a same-version replay. */
    make_image(48);image[333]^=1;assert(!transfer(48));image[333]^=1;
    assert(transfer(48));simulate_bootloader_install();assert(current==48);
    make_image(49);image[sizeof image-36]=0;assert(!transfer(49));
    assert(writes>0 && marker_writes==2);
    puts("OTA chain46->47->48 PASS: real receiver/auth/backend, marker-last, corruption/replay/OTA-disabled rejection; simulated bootloader only");
}
