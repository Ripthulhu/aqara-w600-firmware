#include "w600_settings.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static void equal(const struct w600_settings *a,const struct w600_settings *b)
{
    assert(a->generation==b->generation && a->target_raw==b->target_raw &&
           a->fields==b->fields && a->mode==b->mode && a->external_use==b->external_use);
}
static struct w600_endpoint fresh(void)
{ return (struct w600_endpoint){.control_enabled=true,.mode_known=true}; }
static void rehash(uint8_t *wire)
{
    uint32_t crc=UINT32_MAX;
    for(unsigned i=0;i<24;++i) {
        crc^=wire[i];
        for(unsigned bit=0;bit<8;++bit)crc=crc&1u ? (crc>>1)^0xedb88320u : crc>>1;
    }
    crc=~crc;
    for(unsigned i=0;i<4;++i)wire[24+i]=(uint8_t)(crc>>(i*8));
}
int main(void)
{
    uint8_t wire[32],other[32],bad[32];
    struct w600_settings s={0x01020304,2100,7,1,1},out={0};
    const uint8_t golden[]={0x57,0x36,0x4e,0x56,1,7,1,1,4,3,2,1,0x34,8,
        0,0,0,0,0,0,0,0,0,0,0xaa,0x9f,2,0x7e,0x4f,0x4b,0x41,0x59};
    assert(w600_settings_encode(&s,wire,sizeof(wire)) && !memcmp(wire,golden,32));
    assert(w600_settings_decode(wire,32,&out));equal(&s,&out);
    struct w600_settings before=out;
    memcpy(bad,golden,32);rehash(bad);assert(!memcmp(bad,golden,32));
    for(unsigned i=0;i<11;++i) {
        memcpy(bad,golden,32);
        switch(i) {
        case 0:bad[4]=2;break;case 1:bad[5]=8;break;case 2:bad[6]=2;break;case 3:bad[7]=2;break;
        case 4:memset(bad+8,0,4);break;case 5:bad[12]=0xf3;bad[13]=1;break;
        case 6:bad[12]=0xb9;bad[13]=0xb;break;case 7:bad[14]=1;break;
        case 8:bad[5]&=(uint8_t)~W600_SETTINGS_TARGET;break;
        case 9:bad[5]&=(uint8_t)~W600_SETTINGS_SOURCE;break;
        default:bad[5]&=(uint8_t)~W600_SETTINGS_MODE;break;
        }
        rehash(bad);assert(!w600_settings_decode(bad,32,&out));equal(&out,&before);
    }
    for(unsigned i=0;i<256;++i) {
        memcpy(bad,wire,32);bad[i/8]^=(uint8_t)(1u<<(i%8));
        assert(!w600_settings_decode(bad,32,&out));equal(&out,&before);
    }
    for(unsigned size=0;size<=33;++size)if(size!=32) {
        memset(bad,0xa5,32);memcpy(other,bad,32);
        assert(!w600_settings_encode(&s,bad,size) && !memcmp(bad,other,32));
        assert(!w600_settings_decode(wire,size,&out));equal(&out,&before);
    }
    assert(!w600_settings_encode(NULL,wire,32) && !w600_settings_encode(&s,NULL,32));
    assert(!w600_settings_decode(NULL,32,&out) && !w600_settings_decode(wire,32,NULL));
    for(unsigned fields=0;fields<8;++fields) {
        s=(struct w600_settings){1,(fields&1) ? 2100 : 0,(uint8_t)fields,
            (fields&4) ? 1 : 0,(fields&2) ? 1 : 0};
        assert(w600_settings_encode(&s,wire,32) && w600_settings_decode(wire,32,&out));equal(&s,&out);
    }
    for(unsigned target=500;target<=3000;++target)for(unsigned mode=0;mode<2;++mode)for(unsigned ext=0;ext<2;++ext) {
        s=(struct w600_settings){target,(uint16_t)target,7,(uint8_t)mode,(uint8_t)ext};
        assert(w600_settings_encode(&s,wire,32) && w600_settings_decode(wire,32,&out));equal(&s,&out);
    }
    const struct w600_settings invalid[]={ {0,2100,7,1,1},{1,499,7,1,1},{1,3001,7,1,1},
        {1,2100,8,1,1},{1,2100,7,2,1},{1,2100,7,1,2},{1,2100,0,0,0},
        {1,0,0,1,0},{1,0,0,0,1} };
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) {
        memset(wire,0xa5,32);memcpy(bad,wire,32);
        assert(!w600_settings_encode(&invalid[i],wire,32) && !memcmp(wire,bad,32));
    }
    s=(struct w600_settings){10,2100,7,1,1};assert(w600_settings_encode(&s,wire,32));
    s.generation=11;assert(w600_settings_encode(&s,other,32));
    for(unsigned prefix=0;prefix<=32;++prefix) {
        memset(bad,0xff,32);memcpy(bad,other,prefix);
        assert(w600_settings_select(wire,bad,32,&out)==(prefix==32 ? 1 : 0));
        assert(out.generation==(prefix==32 ? 11u : 10u));
    }
    for(unsigned prefix=0;prefix<=32;++prefix) {
        memcpy(bad,wire,32);memset(bad,0xff,prefix);
        assert(w600_settings_select(bad,other,32,&out)==1 && out.generation==11);
    }
    assert(w600_settings_select(other,other,32,&out)==0);
    s.target_raw=2200;assert(w600_settings_encode(&s,bad,32));before=out;
    assert(w600_settings_select(other,bad,32,&out)==-1);equal(&out,&before);
    memset(bad,0xff,32);assert(w600_settings_select(bad,bad,32,&out)==-1);equal(&out,&before);
    s.generation=UINT32_MAX;assert(w600_settings_encode(&s,other,32));
    assert(w600_settings_select(wire,other,32,&out)==1 && out.generation==UINT32_MAX);
    assert(w600_settings_select(wire,other,31,&out)==-1 && w600_settings_select(wire,other,32,NULL)==-1);
    s=(struct w600_settings){1,2100,7,1,1};struct w600_endpoint e=fresh();
    e.external_temp_raw=1234;e.local_temp_raw=567;
    assert(w600_settings_restore(&s,&e));
    assert(e.target_raw==2100 && e.mode==1 && e.mode_known && e.external_use==1 &&
        e.known==(W600_KNOWN_TARGET|W600_KNOWN_EXTERNAL_USE) && !e.external_updates &&
        !e.external_temp_raw && !e.local_temp_raw &&
        !e.calibrated && !e.opening_known && !e.moving && !e.calibration_pending);
    assert(!w600_settings_restore(&s,&e));
    s=(struct w600_settings){1,0,0,0,0};e=fresh();
    assert(w600_settings_restore(&s,&e) && !e.mode && e.mode_known && !e.known);
    for(unsigned i=0;i<16;++i) {
        e=fresh();
        switch(i) {
        case 0:e.control_enabled=false;break;case 1:e.mode_known=false;break;case 2:e.mode=1;break;
        case 3:e.known=W600_KNOWN_LOCAL_TEMP;break;case 4:e.request_seen=true;break;
        case 5:e.calibration_seen=true;break;case 6:e.calibrated=true;break;case 7:e.opening_known=true;break;
        case 8:e.moving=true;break;case 9:e.calibration_busy=true;break;case 10:e.calibration_pending=true;break;
        case 11:e.cancel_calibration=true;break;case 12:e.control_updates=1;break;
        case 13:e.external_updates=1;break;case 14:e.control_error=1;break;default:e.calibration_error=1;break;
        }
        struct w600_endpoint old=e;assert(!w600_settings_restore(&s,&e) && !memcmp(&e,&old,sizeof(e)));
    }
    for(unsigned i=0;i<sizeof(invalid)/sizeof(invalid[0]);++i) {
        e=fresh();struct w600_endpoint old=e;
        assert(!w600_settings_restore(&invalid[i],&e) && !memcmp(&e,&old,sizeof(e)));
    }
    assert(!w600_settings_restore(NULL,&e) && !w600_settings_restore(&s,NULL));
    puts("PASS settings codec: golden CRC32, all bit flips/lengths,10004 target/mode/source combinations, interrupted copies, ambiguity, generations, fresh-boot-only restore; no FLASH or motor actions");
    return 0;
}
