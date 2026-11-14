#include "w600_settings_store.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

struct disk {
    uint8_t pages[2][512];
    unsigned reads,writes,fail_read,cut,corrupt_readback;
    bool lie,damage_active,fail_after;
};
static uint32_t crc(const uint8_t *p)
{
    uint32_t c=~0u;
    for(unsigned i=0;i<512;++i) {
        c^=p[i];for(unsigned b=0;b<8;++b)c=c&1 ? (c>>1)^0xedb88320u : c>>1;
    }
    return ~c;
}
static bool read_page(void *context,unsigned slot,uint8_t *out)
{
    struct disk *d=context;assert(slot<2);
    if(++d->reads==d->fail_read)return false;
    memcpy(out,d->pages[slot],512);
    if(d->writes && d->corrupt_readback==d->reads)out[500]^=1;
    return true;
}
static bool replace_page(void *context,unsigned slot,uint32_t old_crc,const uint8_t *image)
{
    struct disk *d=context;assert(slot<2 && crc(d->pages[slot])==old_crc);
    ++d->writes;
    if(d->lie)return true;
    /* Byte-level cuts are deliberately harsher than an atomic-word assumption. */
    unsigned step=0;
    for(unsigned i=0;i<512;++i) {
        if(step++==d->cut)return false;
        d->pages[slot][i]=255;
    }
    for(unsigned i=0;i<512;++i) {
        if(step++==d->cut)return false;
        d->pages[slot][i]&=image[i];
    }
    if(d->damage_active)d->pages[slot^1][501]^=1;
    return !d->fail_after;
}
static struct w600_settings_io io(struct disk *d)
{ return (struct w600_settings_io){d,read_page,replace_page}; }
static struct disk empty(void)
{
    struct disk d={.cut=~0u};memset(d.pages,255,sizeof d.pages);return d;
}
static void record(struct disk *d,unsigned slot,struct w600_settings s)
{
    assert(w600_settings_encode(&s,d->pages[slot]+480,32));
}
static bool same(const struct w600_settings *a,const struct w600_settings *b)
{ return a->generation==b->generation && a->target_raw==b->target_raw &&
         a->fields==b->fields && a->mode==b->mode && a->external_use==b->external_use; }

int main(void)
{
    struct w600_settings_workspace w;
    struct disk d=empty();struct w600_settings_io bus=io(&d);
    struct w600_settings out={9,2200,7,1,1},before=out;
    assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_EMPTY && same(&out,&before));
    struct w600_settings first={0,2100,7,1,1},second={0,1600,7,0,1};
    assert(w600_settings_save(&bus,&w,&first)==W600_SETTINGS_SAVED && d.writes==1);
    assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_LOADED);
    first.generation=1;assert(same(&out,&first));
    assert(w600_settings_save(&bus,&w,&first)==W600_SETTINGS_UNCHANGED && d.writes==1);
    assert(w600_settings_save(&bus,&w,&second)==W600_SETTINGS_SAVED && d.writes==2);
    assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_LOADED);
    second.generation=2;assert(same(&out,&second));
    struct disk baseline=d;
    struct w600_settings third={0,2400,7,1,0};
    for(unsigned cut=0;cut<=1024;++cut) {
        d=baseline;d.reads=d.writes=0;d.cut=cut;bus=io(&d);
        enum w600_settings_status s=w600_settings_save(&bus,&w,&third);
        assert(s==(cut<1024 ? W600_SETTINGS_WRITE_FAILED : W600_SETTINGS_SAVED));
        assert(!memcmp(d.pages[1],baseline.pages[1],512));
        assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_LOADED);
        third.generation=3;assert(same(&out,&second) || same(&out,&third));
        if(cut<1024)assert(same(&out,&second));
    }
    for(unsigned fail=1;fail<=4;++fail) {
        d=baseline;d.reads=d.writes=0;d.fail_read=fail;bus=io(&d);
        assert(w600_settings_save(&bus,&w,&third)==
            (fail<=2 ? W600_SETTINGS_READ_FAILED : W600_SETTINGS_VERIFY_FAILED));
        assert(d.writes==(fail<=2 ? 0u : 1u));
    }
    for(unsigned bad=0;bad<4;++bad) {
        d=baseline;d.reads=d.writes=0;bus=io(&d);
        if(bad==0)d.lie=true;
        if(bad==1)d.damage_active=true;
        if(bad>=2)d.corrupt_readback=bad+1;
        assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_VERIFY_FAILED);
    }
    d=baseline;d.reads=d.writes=0;d.fail_after=true;bus=io(&d);
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_WRITE_FAILED);
    assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_LOADED && same(&out,&third));
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_UNCHANGED && d.writes==1);
    for(unsigned cut=0;cut<=1024;++cut) {
        d=empty();d.cut=cut;bus=io(&d);out=before;
        assert(w600_settings_save(&bus,&w,&third)==
            (cut<1024 ? W600_SETTINGS_WRITE_FAILED : W600_SETTINGS_SAVED));
        enum w600_settings_status s=w600_settings_load(&bus,&w,&out);
        if(cut<1024)assert((s==W600_SETTINGS_EMPTY || s==W600_SETTINGS_CORRUPT) && same(&out,&before));
        else assert(s==W600_SETTINGS_LOADED && out.generation==1 && out.target_raw==third.target_raw);
    }
    for(unsigned fail=1;fail<=2;++fail) {
        d=baseline;d.reads=0;d.fail_read=fail;bus=io(&d);out=before;
        assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_READ_FAILED && same(&out,&before));
    }
    d=baseline;d.reads=d.writes=0;bus=io(&d);d.pages[0][0]=0;
    assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_LOADED && same(&out,&second));
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_FOREIGN && !d.writes);
    d=baseline;d.reads=d.writes=0;bus=io(&d);d.pages[0][484]=2;
    assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_LOADED && same(&out,&second));
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_FOREIGN && !d.writes);
    d=empty();bus=io(&d);first.generation=second.generation=5;
    record(&d,0,first);record(&d,1,second);out=before;
    assert(w600_settings_load(&bus,&w,&out)==W600_SETTINGS_AMBIGUOUS && same(&out,&before));
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_AMBIGUOUS && !d.writes);
    d=empty();bus=io(&d);first.generation=UINT32_MAX;record(&d,0,first);
    assert(w600_settings_save(&bus,&w,&first)==W600_SETTINGS_UNCHANGED && !d.writes);
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_EXHAUSTED && !d.writes);
    d=empty();bus=io(&d);d.pages[0][480]=0;
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_CORRUPT && !d.writes);
    d=empty();bus=io(&d);third.mode=2;
    assert(w600_settings_save(&bus,&w,&third)==W600_SETTINGS_INVALID && !d.reads && !d.writes);
    assert(w600_settings_load(NULL,&w,&out)==W600_SETTINGS_INVALID);
    assert(w600_settings_save(&bus,NULL,&third)==W600_SETTINGS_INVALID);
    puts("PASS two-page settings store:2050 interrupted first/replacement writes, preserved active copy, lost acknowledgement, read/write/readback faults, foreign data, ambiguity, generation exhaustion and no-op saves; mock backend only");
}
