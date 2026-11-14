#include "w600_settings_store.h"
#include <string.h>

static uint32_t crc32(const uint8_t *p)
{
    uint32_t crc=UINT32_MAX;
    for(unsigned i=0;i<W600_SETTINGS_PAGE_BYTES;++i) {
        crc^=p[i];
        for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
static bool same(const struct w600_settings *a,const struct w600_settings *b)
{ return a->fields==b->fields && a->target_raw==b->target_raw &&
         a->mode==b->mode && a->external_use==b->external_use; }
static bool prefix(const uint8_t *p)
{
    for(unsigned i=0;i<W600_SETTINGS_RECORD_AT;++i)if(p[i]!=255)return false;
    return true;
}
static bool erased(const uint8_t *p)
{
    for(unsigned i=0;i<W600_SETTINGS_PAGE_BYTES;++i)if(p[i]!=255)return false;
    return true;
}
static bool foreign(const uint8_t *p)
{
    const uint8_t *r=p+W600_SETTINGS_RECORD_AT;
    return !prefix(p) || (r[0]=='W' && r[1]=='6' && r[2]=='N' && r[3]=='V' && r[4]!=1 && r[4]!=255);
}
static bool equal_record(const uint8_t *a,const uint8_t *b)
{
    for(unsigned i=0;i<W600_SETTINGS_BYTES;++i)if(a[i]!=b[i])return false;
    return true;
}
static bool read_pages(const struct w600_settings_io *io,struct w600_settings_workspace *w)
{
    return io->read(io->context,0,(uint8_t *)w->pages[0]) &&
           io->read(io->context,1,(uint8_t *)w->pages[1]);
}
static enum w600_settings_status select_saved(struct w600_settings_workspace *w,
    struct w600_settings *out,int *slot)
{
    const uint8_t *a=(const uint8_t *)w->pages[0],*b=(const uint8_t *)w->pages[1];
    const uint8_t *ra=prefix(a) ? a+W600_SETTINGS_RECORD_AT : NULL;
    const uint8_t *rb=prefix(b) ? b+W600_SETTINGS_RECORD_AT : NULL;
    *slot=w600_settings_select(ra,rb,W600_SETTINGS_BYTES,out);
    if(*slot>=0)return W600_SETTINGS_LOADED;
    struct w600_settings x,y;
    if(w600_settings_decode(ra,W600_SETTINGS_BYTES,&x) &&
       w600_settings_decode(rb,W600_SETTINGS_BYTES,&y))return W600_SETTINGS_AMBIGUOUS;
    return erased(a) && erased(b) ? W600_SETTINGS_EMPTY : W600_SETTINGS_CORRUPT;
}
enum w600_settings_status w600_settings_load(const struct w600_settings_io *io,
    struct w600_settings_workspace *w,struct w600_settings *out)
{
    if(!io || !io->read || !w || !out)return W600_SETTINGS_INVALID;
    if(!read_pages(io,w))return W600_SETTINGS_READ_FAILED;
    int slot;
    return select_saved(w,out,&slot);
}
enum w600_settings_status w600_settings_save(const struct w600_settings_io *io,
    struct w600_settings_workspace *w,const struct w600_settings *desired)
{
    if(!io || !io->read || !io->replace || !w || !desired)return W600_SETTINGS_INVALID;
    struct w600_settings next=*desired,old;
    uint8_t encoded[W600_SETTINGS_BYTES];
    next.generation=1;
    if(!w600_settings_encode(&next,encoded,sizeof encoded))return W600_SETTINGS_INVALID;
    if(!read_pages(io,w))return W600_SETTINGS_READ_FAILED;
    for(unsigned i=0;i<2;++i)if(foreign((const uint8_t *)w->pages[i]))return W600_SETTINGS_FOREIGN;
    int active;
    enum w600_settings_status status=select_saved(w,&old,&active);
    if(status!=W600_SETTINGS_LOADED && status!=W600_SETTINGS_EMPTY)return status;
    uint32_t retained_crc=0;
    if(status==W600_SETTINGS_LOADED) {
        if(same(&next,&old))return W600_SETTINGS_UNCHANGED;
        if(old.generation==UINT32_MAX)return W600_SETTINGS_EXHAUSTED;
        next.generation=old.generation+1;
        retained_crc=crc32((const uint8_t *)w->pages[active]);
    }
    unsigned target=active==0 ? 1u : 0u;
    uint8_t *page=(uint8_t *)w->pages[target];
    uint32_t old_crc=crc32(page);
    if(!w600_settings_encode(&next,encoded,sizeof encoded))return W600_SETTINGS_INVALID;
    memset(page,255,W600_SETTINGS_PAGE_BYTES);
    memcpy(page+W600_SETTINGS_RECORD_AT,encoded,sizeof encoded);
    if(!io->replace(io->context,target,old_crc,page))return W600_SETTINGS_WRITE_FAILED;
    if(!read_pages(io,w))return W600_SETTINGS_VERIFY_FAILED;
    page=(uint8_t *)w->pages[target];
    if(!prefix(page) || !equal_record(page+W600_SETTINGS_RECORD_AT,encoded) ||
       (active>=0 && crc32((const uint8_t *)w->pages[active])!=retained_crc))
        return W600_SETTINGS_VERIFY_FAILED;
    return W600_SETTINGS_SAVED;
}
