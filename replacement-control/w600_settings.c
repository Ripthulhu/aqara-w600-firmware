#include "w600_settings.h"

static uint32_t le32(const uint8_t *p)
{ return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static void put32(uint8_t *p,uint32_t value)
{ for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(value>>(i*8)); }
static uint32_t crc32(const uint8_t *p,size_t size)
{
    uint32_t crc=UINT32_MAX;
    for(size_t i=0;i<size;++i) {
        crc^=p[i];
        for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
static bool valid(const struct w600_settings *s)
{
    return s && s->generation && !(s->fields&~7u) &&
        ((s->fields&W600_SETTINGS_TARGET) ? s->target_raw>=500 && s->target_raw<=3000 : !s->target_raw) &&
        ((s->fields&W600_SETTINGS_MODE) ? s->mode<=1 : !s->mode) &&
        ((s->fields&W600_SETTINGS_SOURCE) ? s->external_use<=1 : !s->external_use);
}
bool w600_settings_encode(const struct w600_settings *s,uint8_t *out,size_t size)
{
    if (!out || size!=W600_SETTINGS_BYTES || !valid(s))return false;
    uint8_t data[W600_SETTINGS_BYTES]={0};
    put32(data,0x564e3657u); /* W6NV */
    data[4]=1;data[5]=s->fields;data[6]=s->mode;data[7]=s->external_use;
    put32(data+8,s->generation);
    data[12]=(uint8_t)s->target_raw;data[13]=(uint8_t)(s->target_raw>>8);
    put32(data+24,crc32(data,24));
    put32(data+W600_SETTINGS_COMMIT_OFFSET,0x59414b4fu); /* OKAY */
    for(unsigned i=0;i<W600_SETTINGS_BYTES;++i)out[i]=data[i];
    return true;
}
bool w600_settings_decode(const uint8_t *data,size_t size,struct w600_settings *out)
{
    if (!data || !out || size!=W600_SETTINGS_BYTES || le32(data)!=0x564e3657u ||
        data[4]!=1 || le32(data+28)!=0x59414b4fu || le32(data+24)!=crc32(data,24))return false;
    for(unsigned i=14;i<24;++i)if(data[i])return false;
    struct w600_settings s={.generation=le32(data+8),
        .target_raw=(uint16_t)((uint16_t)data[12]|(uint16_t)data[13]<<8),
        .fields=data[5],.mode=data[6],.external_use=data[7]};
    if (!valid(&s))return false;
    *out=s;return true;
}
int w600_settings_select(const uint8_t *a,const uint8_t *b,size_t size,struct w600_settings *out)
{
    if (!out || size!=W600_SETTINGS_BYTES)return -1;
    struct w600_settings x,y;
    bool ax=w600_settings_decode(a,size,&x),by=w600_settings_decode(b,size,&y);
    if (!ax && !by)return -1;
    if (ax && by && x.generation==y.generation &&
        (x.target_raw!=y.target_raw || x.fields!=y.fields || x.mode!=y.mode || x.external_use!=y.external_use))return -1;
    int slot=!ax ? 1 : !by || x.generation>=y.generation ? 0 : 1;
    *out=slot ? y : x;return slot;
}
bool w600_settings_restore(const struct w600_settings *s,struct w600_endpoint *e)
{
    if (!valid(s) || !e || !e->control_enabled || !e->mode_known || e->mode || e->known ||
        e->request_seen || e->calibration_seen || e->calibrated || e->opening_known || e->moving ||
        e->calibration_busy || e->calibration_pending || e->cancel_calibration ||
        e->control_updates || e->external_updates || e->control_error || e->calibration_error)return false;
    struct w600_endpoint next=*e;
    next.target_raw=s->target_raw;next.external_use=s->external_use;
    next.mode=(s->fields&W600_SETTINGS_MODE) ? s->mode : 0;
    next.external_temp_raw=0;next.local_temp_raw=0;
    if (s->fields&W600_SETTINGS_TARGET)next.known|=W600_KNOWN_TARGET;
    if (s->fields&W600_SETTINGS_SOURCE)next.known|=W600_KNOWN_EXTERNAL_USE;
    *e=next;return true;
}
