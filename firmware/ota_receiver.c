#include "ota_receiver.h"
#include <string.h>

static uint32_t le32(const uint8_t *p)
{ return (uint32_t)p[0]|(uint32_t)p[1]<<8|(uint32_t)p[2]<<16|(uint32_t)p[3]<<24; }
static uint16_t crc16(uint16_t crc,const void *data,size_t length)
{
    const uint8_t *p=data;
    while(length--) {
        crc^=(uint16_t)*p++<<8;
        for(unsigned i=0;i<8;++i)crc=(uint16_t)((crc<<1)^((crc&0x8000u)?0x1021u:0));
    }
    return crc;
}
static uint8_t allocate(uint8_t *next)
{
    uint8_t seq=*next;
    if(!seq)seq=1;
    *next=(uint8_t)(seq+1u);
    if(!*next)*next=1;
    return seq;
}
static void prepare(struct w600_ota *o,uint8_t kind,uint8_t sequence,uint16_t length)
{
    memset(&o->tx,0,sizeof(o->tx));
    o->tx.kind=kind;o->tx.sequence=sequence;o->tx.length=length;
    o->pending=true;o->waiting=false;o->retries=0;
}
static void fail(struct w600_ota *o,enum w600_ota_error error,uint8_t *next)
{
    o->state=W600_OTA_FAILED;o->error=error;
    prepare(o,6,allocate(next),4);
    o->tx.payload[0]=0x40;o->tx.payload[1]=4;o->tx.payload[2]=1;
    o->tx.payload[3]=error==W600_OTA_HEADER_ERROR ? 0x23 :
        error==W600_OTA_BOUNDS ? 0x25 : error==W600_OTA_FORMAT ? 7 : 0x22;
}
static bool clock_ok(struct w600_ota *o,uint32_t now,uint8_t *next)
{
    if((uint32_t)(now-o->last_now)>=0x80000000u) {
        fail(o,W600_OTA_CLOCK,next);return false;
    }
    o->last_now=now;
    if((uint32_t)(now-o->started_at)>=W600_OTA_TOTAL_MS) {
        fail(o,W600_OTA_TIMEOUT,next);return false;
    }
    return true;
}
static void request(struct w600_ota *o,uint32_t offset,uint8_t length,uint8_t *next)
{
    prepare(o,6,allocate(next),11);
    uint8_t *p=o->tx.payload;
    p[0]=0x40;p[1]=1;p[2]=0x0c;p[3]=2;p[4]=7;
    p[5]=(uint8_t)(offset>>24);p[6]=(uint8_t)(offset>>16);
    p[7]=(uint8_t)(offset>>8);p[8]=(uint8_t)offset;p[9]=1;p[10]=length;
    o->expected_length=length;
}
static void request_body(struct w600_ota *o,uint8_t *next)
{
    uint32_t n=o->image_length-o->received;
    if(n>W600_OTA_CHUNK)n=W600_OTA_CHUNK;
    request(o,128u+o->received,(uint8_t)n,next);
}
void w600_ota_init(struct w600_ota *o,const struct w600_ota_ops *ops,void *context)
{
    if(!o)return;
    memset(o,0,sizeof(*o));o->ops=ops;o->context=context;
}
bool w600_ota_active(const struct w600_ota *o)
{ return o && o->state!=W600_OTA_IDLE; }
bool w600_ota_restart_ready(const struct w600_ota *o,uint32_t now)
{
    return o && o->state==W600_OTA_FAILED &&
        ((!o->pending && (uint32_t)(now-o->sent_at)>=100u) ||
         (uint32_t)(now-o->last_now)>=2000u);
}
bool w600_ota_duplicate_start(const struct w600_ota *o,const struct w600_frame *in)
{
    return o && in && !o->pending && o->state>=W600_OTA_START_ACK &&
        o->state<=W600_OTA_DELAY && in->kind==2 && in->sequence==o->start_sequence &&
        in->length==4 && in->payload[0]==0x40 && !in->payload[1] &&
        in->payload[2]<=1 && in->payload[3]==1;
}

static enum w600_ota_error verify(struct w600_ota *o)
{
    uint8_t *p=(uint8_t *)o->page;
    if(!o->ops->quiesce(o->context))return W600_OTA_STOP;
    if(!o->ops->read(o->context,W600_OTA_STAGE,p,8))return W600_OTA_STORAGE;
    uint32_t sp=le32(p),entry=le32(p+4);
    if((sp&7u) || sp<0x20000008u || sp>0x20007000u || !(entry&1u) ||
       entry<0x08002000u || (entry&~1u)>=0x08002000u+o->image_length)
        return W600_OTA_VECTOR;
    uint16_t crc=0;
    for(uint32_t offset=0;offset<o->image_length;) {
        uint32_t n=o->image_length-offset;
        if(n>W600_OTA_PAGE)n=W600_OTA_PAGE;
        if(!o->ops->read(o->context,W600_OTA_STAGE+offset,p,n))return W600_OTA_STORAGE;
        crc=crc16(crc,p,n);offset+=n;
    }
    if(crc!=o->image_crc)return W600_OTA_CRC;
    if(!o->ops->authorize(o->context,W600_OTA_STAGE,o->image_length,o->revision))
        return W600_OTA_AUTH;
    return W600_OTA_OK;
}
static bool append(struct w600_ota *o,const uint8_t *p,uint32_t n)
{
    while(n) {
        uint32_t count=W600_OTA_PAGE-o->page_used;
        if(count>n)count=n;
        memcpy((uint8_t *)o->page+o->page_used,p,count);
        o->page_used=(uint16_t)(o->page_used+count);
        o->received+=count;p+=count;n-=count;
        if(o->page_used==W600_OTA_PAGE || o->received==o->image_length) {
            if(!o->ops->quiesce(o->context) ||
               !o->ops->write_page(o->context,W600_OTA_STAGE+o->page_base,o->page))return false;
            o->page_base+=W600_OTA_PAGE;o->page_used=0;
            memset(o->page,0xff,sizeof(o->page));
        }
    }
    return true;
}
static bool valid_ops(const struct w600_ota_ops *ops)
{ return ops && ops->quiesce && ops->write_page && ops->read && ops->authorize && ops->commit; }

bool w600_ota_process(struct w600_ota *o,const struct w600_frame *in,uint32_t now,
                     uint8_t *next)
{
    if(!o || !in || !next || in->length<2 || in->length>W600_MAX_PAYLOAD)return false;
    const uint8_t *p=in->payload;
    if(p[0]!=0x40)return false;
    if(in->kind==2 && p[1]==0) {
        if(o->state!=W600_OTA_IDLE)return true;
        prepare(o,3,in->sequence,3);
        o->tx.payload[1]=0x40;
        /* Stock0959c tests the value byte; captured radio starts use datatype1. */
        if(in->length!=4 || p[2]>1 || p[3]!=1) {
            o->tx.payload[0]=4;return true;
        }
        if(!valid_ops(o->ops)) {o->tx.payload[0]=1;return true;}
        if(!o->ops->quiesce(o->context)) {
            o->tx.payload[0]=4;o->error=W600_OTA_STOP;return true;
        }
        o->state=W600_OTA_START_ACK;o->error=W600_OTA_OK;o->start_sequence=in->sequence;
        o->started_at=now;o->last_now=now;
        memset(o->page,0xff,sizeof(o->page));
        return true;
    }
    if(in->kind!=7 || (p[1]!=1 && p[1]!=2 && p[1]!=4))return false;
    if(o->state==W600_OTA_IDLE || o->state==W600_OTA_FAILED ||
       o->state==W600_OTA_COMMITTED || !clock_ok(o,now,next))return true;
    if(o->pending || !o->waiting || in->sequence!=o->tx.sequence)return true;
    if(p[1]==4) {fail(o,W600_OTA_RADIO_ABORT,next);return true;}
    if(p[1]==1 && (o->state==W600_OTA_HEADER || o->state==W600_OTA_BODY)) {
        if(in->length!=5u+o->expected_length || p[2] || p[3]!=0x0b ||
           p[4]!=o->expected_length) {fail(o,W600_OTA_FORMAT,next);return true;}
        o->waiting=false;
        const uint8_t *data=p+5;
        if(o->state==W600_OTA_HEADER) {
            if(le32(data)!=0x6c756d69u || le32(data+8)!=128u ||
               le32(data+124)!=(uint32_t)crc16(0,data,124)) {
                fail(o,W600_OTA_HEADER_ERROR,next);return true;
            }
            uint32_t length=le32(data+32),crc=le32(data+36);
            if(length<192u || length>W600_OTA_MAX_IMAGE || (length&3u) || crc>0xffffu) {
                fail(o,W600_OTA_BOUNDS,next);return true;
            }
            o->image_length=length;o->image_crc=(uint16_t)crc;o->revision=le32(data+40);
            o->state=W600_OTA_BODY;request_body(o,next);return true;
        }
        if(o->received>o->image_length || o->expected_length>o->image_length-o->received) {
            fail(o,W600_OTA_BOUNDS,next);return true;
        }
        if(!append(o,data,o->expected_length)) {fail(o,W600_OTA_STORAGE,next);return true;}
        if(o->received<o->image_length) {request_body(o,next);return true;}
        enum w600_ota_error result=verify(o);
        if(result!=W600_OTA_OK) {fail(o,result,next);return true;}
        o->state=W600_OTA_FINISH;prepare(o,6,allocate(next),4);
        o->tx.payload[0]=0x40;o->tx.payload[1]=2;o->tx.payload[2]=1;
        return true;
    }
    if(p[1]==2 && o->state==W600_OTA_FINISH) {
        /* Stock reads BE32 at record+4 and ignores byte3; custom unit is ms. */
        if(in->length!=8 || p[2]) {
            fail(o,W600_OTA_FORMAT,next);return true;
        }
        uint32_t delay=(uint32_t)p[4]<<24|(uint32_t)p[5]<<16|(uint32_t)p[6]<<8|p[7];
        if(delay>W600_OTA_MAX_DELAY_MS) {fail(o,W600_OTA_BOUNDS,next);return true;}
        o->delay_ms=delay<100u ? 100u : delay;o->delay_at=now;
        o->waiting=false;o->state=W600_OTA_DELAY;
    }
    return true;
}

void w600_ota_sent(struct w600_ota *o,uint32_t now)
{
    if(!o || !o->pending)return;
    o->pending=false;o->sent_at=now;
    o->waiting=o->state==W600_OTA_HEADER || o->state==W600_OTA_BODY || o->state==W600_OTA_FINISH;
}
void w600_ota_poll(struct w600_ota *o,uint32_t now,uint8_t *next)
{
    if(!o || !next || o->state==W600_OTA_IDLE || o->state==W600_OTA_FAILED ||
       o->state==W600_OTA_COMMITTED)return;
    if(!clock_ok(o,now,next))return;
    if(o->state==W600_OTA_START_ACK && !o->pending) {
        o->state=W600_OTA_HEADER;request(o,0,128,next);return;
    }
    if(o->state==W600_OTA_DELAY && (uint32_t)(now-o->delay_at)>=o->delay_ms) {
        enum w600_ota_error result=verify(o);
        if(result!=W600_OTA_OK) {fail(o,result,next);return;}
        if(!o->ops->commit(o->context,o->image_length,o->image_crc,o->revision)) {
            fail(o,W600_OTA_COMMIT,next);return;
        }
        o->state=W600_OTA_COMMITTED;return;
    }
    if(!o->waiting || o->pending || (uint32_t)(now-o->sent_at)<W600_OTA_REPLY_MS)return;
    if(o->retries>=3) {fail(o,W600_OTA_TIMEOUT,next);return;}
    ++o->retries;o->waiting=false;o->pending=true;
}
