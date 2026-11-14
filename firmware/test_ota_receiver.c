#include "ota_receiver.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

struct fixture {
    struct w600_ota ota;
    uint8_t stage[W600_OTA_MAX_IMAGE],image[W600_OTA_MAX_IMAGE],header[128];
    uint32_t now;
    uint8_t sequence;
    unsigned writes,reads,authorizations,commits,quiesces;
    unsigned fail_write,fail_read,fail_quiesce;
    bool allow,commit_ok;
};
static struct fixture f;
static unsigned cases;
static uint16_t crc(const void *data,size_t n)
{
    const uint8_t *p=data;uint16_t c=0;
    while(n--) {
        c^=(uint16_t)*p++<<8;
        for(unsigned b=0;b<8;++b)c=(uint16_t)((c<<1)^((c&0x8000u)?0x1021u:0));
    }
    return c;
}
static void le(uint8_t *p,uint32_t value)
{ for(unsigned i=0;i<4;++i)p[i]=(uint8_t)(value>>(i*8)); }
static bool quiesce(void *ctx)
{
    struct fixture *x=ctx;
    ++x->quiesces;return !x->fail_quiesce || x->quiesces!=x->fail_quiesce;
}
static bool write_page(void *ctx,uint32_t address,const uint32_t page[128])
{
    struct fixture *x=ctx;
    assert(address>=W600_OTA_STAGE && !(address&511u));
    assert(address<=W600_OTA_STAGE+W600_OTA_MAX_IMAGE-512u);
    ++x->writes;
    if(x->fail_write==x->writes)return false;
    memcpy(x->stage+address-W600_OTA_STAGE,page,512);return true;
}
static bool read_stage(void *ctx,uint32_t address,void *out,size_t length)
{
    struct fixture *x=ctx;
    assert(address>=W600_OTA_STAGE && length<=W600_OTA_MAX_IMAGE);
    assert(address-W600_OTA_STAGE<=W600_OTA_MAX_IMAGE-length);
    ++x->reads;
    if(x->fail_read==x->reads)return false;
    memcpy(out,x->stage+address-W600_OTA_STAGE,length);return true;
}
static bool authorize(void *ctx,uint32_t address,uint32_t length,uint32_t revision)
{
    struct fixture *x=ctx;
    assert(address==W600_OTA_STAGE && length<=W600_OTA_MAX_IMAGE && revision==29);
    ++x->authorizations;return x->allow;
}
static bool commit(void *ctx,uint32_t length,uint16_t c,uint32_t revision)
{
    struct fixture *x=ctx;
    assert(x->authorizations>=2 && revision==29);
    assert(crc(x->stage,length)==c);
    ++x->commits;return x->commit_ok;
}
static const struct w600_ota_ops ops={quiesce,write_page,read_stage,authorize,commit};
static void setup(uint32_t length)
{
    ++cases;memset(&f,0,sizeof(f));f.sequence=250;f.now=100;
    f.allow=true;f.commit_ok=true;memset(f.stage,0xa5,sizeof(f.stage));
    for(uint32_t i=0;i<length;++i)f.image[i]=(uint8_t)(i*17u+3u);
    le(f.image,0x20007000);le(f.image+4,0x08002081);
    le(f.header,0x6c756d69);le(f.header+8,128);le(f.header+32,length);
    le(f.header+36,crc(f.image,length));le(f.header+40,29);
    le(f.header+124,crc(f.header,124));
    w600_ota_init(&f.ota,&ops,&f);
}
static void poll(uint32_t ms)
{ f.now+=ms;w600_ota_poll(&f.ota,f.now,&f.sequence); }
static void sent(void)
{ assert(f.ota.pending);w600_ota_sent(&f.ota,++f.now); }
static struct w600_frame response(const uint8_t *data,unsigned n)
{
    struct w600_frame in={0};in.kind=7;in.sequence=f.ota.tx.sequence;
    in.length=(uint16_t)(n+5);in.payload[0]=0x40;in.payload[1]=1;
    in.payload[3]=0x0b;in.payload[4]=(uint8_t)n;memcpy(in.payload+5,data,n);
    return in;
}
static void process(struct w600_frame *in)
{ assert(w600_ota_process(&f.ota,in,++f.now,&f.sequence)); }
static void start(void)
{
    struct w600_frame in={.sequence=44,.kind=2,.length=4,.payload={0x40,0,1,1}};
    process(&in);
    assert(w600_ota_active(&f.ota) && f.ota.state==W600_OTA_START_ACK);
    assert(f.ota.pending && f.ota.tx.kind==3 && f.ota.tx.sequence==44 &&
           f.ota.tx.length==3 && f.ota.tx.payload[0]==0 && f.ota.tx.payload[1]==0x40);
    poll(0);assert(f.ota.state==W600_OTA_START_ACK);
    sent();poll(1);
    assert(f.ota.state==W600_OTA_HEADER && f.ota.tx.kind==6 && f.ota.tx.length==11);
    const uint8_t expected[]={0x40,1,0x0c,2,7,0,0,0,0,1,128};
    assert(!memcmp(f.ota.tx.payload,expected,sizeof(expected)));
}
static void header(void)
{
    sent();struct w600_frame in=response(f.header,128);process(&in);
    assert(f.ota.state==W600_OTA_BODY);
}
static void body(void)
{
    while(f.ota.state==W600_OTA_BODY) {
        assert(f.ota.pending && f.ota.expected_length<=224);
        uint32_t offset=f.ota.received;
        unsigned n=f.ota.expected_length;
        sent();struct w600_frame in=response(f.image+offset,n);process(&in);
    }
}
static void finish(unsigned delay)
{
    assert(f.ota.state==W600_OTA_FINISH && f.ota.pending);
    assert(f.ota.tx.length==4 && f.ota.tx.payload[0]==0x40 &&
           f.ota.tx.payload[1]==2 && f.ota.tx.payload[2]==1);
    sent();struct w600_frame in={.kind=7,.length=8,.payload={0x40,2,0,7}};
    in.sequence=f.ota.tx.sequence;
    for(unsigned i=0;i<4;++i)in.payload[4+i]=(uint8_t)(delay>>(24-i*8));
    process(&in);
}
static void failed(enum w600_ota_error error)
{
    assert(f.ota.state==W600_OTA_FAILED && f.ota.error==error);
    assert(w600_ota_active(&f.ota) && f.ota.pending && !f.commits);
    assert(f.ota.tx.kind==6 && f.ota.tx.payload[0]==0x40 && f.ota.tx.payload[1]==4);
}
int main(void)
{
    assert(crc("123456789",9)==0x31c3);
    /* Captured40000102 and40000000 are not starts; stock replies044000. */
    for(unsigned value=0;value<=2;value+=2) {
        setup(1024);
        struct w600_frame in={.sequence=0x25,.kind=2,.length=4,
            .payload={0x40,0,(uint8_t)(value==2), (uint8_t)value}};
        process(&in);
        assert(f.ota.state==W600_OTA_IDLE && f.ota.error==W600_OTA_OK &&
               f.ota.tx.kind==3 && f.ota.tx.sequence==0x25 && f.ota.tx.length==3 &&
               f.ota.tx.payload[0]==4 && f.ota.tx.payload[1]==0x40 && f.ota.tx.payload[2]==0 &&
               !f.quiesces && !f.writes && !f.reads && !f.authorizations && !f.commits);
    }
    for(unsigned type=0;type<=1;++type) {
        setup(1024);
        struct w600_frame in={.sequence=44,.kind=2,.length=4,.payload={0x40,0,(uint8_t)type,1}};
        process(&in);
        assert(f.ota.state==W600_OTA_START_ACK && f.ota.tx.payload[0]==0 && f.quiesces==1 && !f.writes);
        sent();poll(1);
        assert(f.ota.state==W600_OTA_HEADER && f.ota.expected_length==128);
    }
    for(unsigned type=2;type<=255;type+=253) {
        setup(1024);
        struct w600_frame in={.kind=2,.length=4,.payload={0x40,0,(uint8_t)type,1}};
        process(&in);assert(f.ota.state==W600_OTA_IDLE && f.ota.tx.payload[0]==4 && !f.quiesces);
    }
    for(unsigned index=0;index<469;++index) {
        unsigned n=index<465 ? 192+index*4 :
            (unsigned[]){44372,52224,52720,W600_OTA_MAX_IMAGE}[index-465];
        setup(n);start();header();body();finish(0);
        assert(f.ota.state==W600_OTA_DELAY && !f.commits);
        poll(99);assert(!f.commits);poll(1);
        assert(f.ota.state==W600_OTA_COMMITTED && f.commits==1 && f.authorizations==2);
        assert(!memcmp(f.stage,f.image,n));
        for(unsigned i=n;i<((n+511u)&~511u);++i)assert(f.stage[i]==255);
        assert(f.writes==(n+511u)/512u);
        poll(600000);assert(f.commits==1);
    }
    setup(1024);start();sent();
    struct w600_frame in=response(f.header,128),wrong=in;
    ++wrong.sequence;process(&wrong);
    assert(f.ota.state==W600_OTA_HEADER && !f.writes && f.ota.waiting);
    process(&in);assert(f.ota.state==W600_OTA_BODY);
    unsigned before=f.writes;process(&in);assert(f.writes==before);
    sent();in=response(f.image,224);wrong=in;--wrong.length;process(&wrong);
    failed(W600_OTA_FORMAT);

    for(unsigned field=0;field<4;++field) {
        setup(1024);start();sent();
        f.header[(unsigned[]){0,8,124,125}[field]]^=1;
        in=response(f.header,128);process(&in);failed(W600_OTA_HEADER_ERROR);
        assert(!f.writes);
    }
    for(unsigned n=0;n<6;++n) {
        setup(1024);start();sent();
        le(f.header+32,(uint32_t[]){0,128,191,193,W600_OTA_MAX_IMAGE+4,UINT32_MAX}[n]);
        le(f.header+124,crc(f.header,124));in=response(f.header,128);process(&in);
        failed(W600_OTA_BOUNDS);assert(!f.writes);
    }
    setup(1024);start();sent();le(f.header+36,0x10000);
    le(f.header+124,crc(f.header,124));in=response(f.header,128);process(&in);
    failed(W600_OTA_BOUNDS);

    for(unsigned n=0;n<6;++n) {
        setup(1024);
        if(n<3)le(f.image,(uint32_t[]){0x20007004,0x20008000,0}[n]);
        else le(f.image+4,(uint32_t[]){0x08002080,0x08002401,0x08001fff}[n-3]);
        le(f.header+36,crc(f.image,1024));le(f.header+124,crc(f.header,124));
        start();header();body();failed(W600_OTA_VECTOR);assert(!f.authorizations);
    }
    setup(1024);start();header();f.image[700]^=1;body();failed(W600_OTA_CRC);
    setup(1024);f.allow=false;start();header();body();failed(W600_OTA_AUTH);
    setup(1024);f.fail_write=1;start();header();body();failed(W600_OTA_STORAGE);
    setup(1024);f.fail_read=1;start();header();body();failed(W600_OTA_STORAGE);
    setup(1024);f.fail_quiesce=2;start();header();body();failed(W600_OTA_STORAGE);
    setup(1024);f.fail_quiesce=1;
    in=(struct w600_frame){.sequence=5,.kind=2,.length=4,.payload={0x40,0,0,1}};
    process(&in);assert(!w600_ota_active(&f.ota) && f.ota.tx.payload[0]==4 && !f.writes);
    setup(1024);w600_ota_init(&f.ota,NULL,&f);process(&in);
    assert(!w600_ota_active(&f.ota) && f.ota.tx.payload[0]==1 && !f.quiesces);
    for(unsigned n=2;n<=9;++n) {
        setup(1024);in=(struct w600_frame){.kind=2,.length=(uint16_t)n,.payload={0x40,0,0,0}};
        process(&in);assert(!w600_ota_active(&f.ota) && f.ota.tx.payload[0]==4 && !f.quiesces);
    }
    setup(1024);in=(struct w600_frame){.kind=2,.length=4,.payload={0x07,0,0,1}};
    assert(!w600_ota_process(&f.ota,&in,f.now,&f.sequence));
    in.length=257;assert(!w600_ota_process(&f.ota,&in,f.now,&f.sequence));
    assert(!w600_ota_process(NULL,&in,f.now,&f.sequence));

    setup(1024);start();sent();
    uint8_t sequence=f.ota.tx.sequence,next=f.sequence;
    for(unsigned retry=1;retry<=3;++retry) {
        poll(W600_OTA_REPLY_MS-1);assert(!f.ota.pending);
        poll(1);assert(f.ota.pending && f.ota.retries==retry);
        assert(f.ota.tx.sequence==sequence && f.sequence==next);sent();
    }
    poll(W600_OTA_REPLY_MS);failed(W600_OTA_TIMEOUT);
    setup(1024);start();poll(W600_OTA_TOTAL_MS);failed(W600_OTA_TIMEOUT);
    setup(1024);start();--f.now;w600_ota_poll(&f.ota,f.now,&f.sequence);failed(W600_OTA_CLOCK);
    setup(1024);f.now=UINT32_MAX-5;start();header();body();finish(100);
    poll(100);assert(f.ota.state==W600_OTA_COMMITTED);

    setup(1024);start();sent();
    in=(struct w600_frame){.kind=7,.length=4,.payload={0x40,4,0,0}};
    in.sequence=f.ota.tx.sequence;process(&in);failed(W600_OTA_RADIO_ABORT);
    setup(1024);start();sent();in=response(f.header,128);
    in.payload[2]=1;process(&in);failed(W600_OTA_FORMAT);
    setup(1024);start();sent();in=response(f.header,128);
    in.payload[3]=0x0a;process(&in);failed(W600_OTA_FORMAT);
    setup(1024);start();sent();in=response(f.header,128);
    --in.payload[4];process(&in);failed(W600_OTA_FORMAT);

    setup(1024);start();header();body();finish(W600_OTA_MAX_DELAY_MS+1);
    failed(W600_OTA_BOUNDS);
    for(unsigned type=0;type<256;++type) {
        setup(1024);start();header();body();
        assert(f.ota.state==W600_OTA_FINISH && f.authorizations==1 && !f.commits);
        assert(!memcmp(f.stage,f.image,1024));
        sent();
        in=(struct w600_frame){.sequence=(uint8_t)(f.ota.tx.sequence+1u),.kind=7,.length=8,
            .payload={0x40,2,0,(uint8_t)type,0,0,0,100}};
        process(&in);
        assert(f.ota.state==W600_OTA_FINISH && f.ota.waiting && f.authorizations==1 && !f.commits);
        in.sequence=f.ota.tx.sequence;process(&in);
        assert(f.ota.state==W600_OTA_DELAY && f.ota.delay_ms==100 && !f.commits);
        poll(99);assert(!f.commits);poll(1);
        assert(f.ota.state==W600_OTA_COMMITTED && f.commits==1 && f.authorizations==2);
    }
    for(unsigned malformed=0;malformed<4;++malformed) {
        setup(1024);start();header();body();sent();
        in=(struct w600_frame){.sequence=f.ota.tx.sequence,.kind=7,
            .length=(uint16_t)(malformed==0 ? 7 : malformed==1 ? 9 : 8),
            .payload={0x40,2,(uint8_t)(malformed<2 ? 0 : malformed==2 ? 1 : 255),255,0,0,0,100}};
        process(&in);failed(W600_OTA_FORMAT);assert(f.authorizations==1);
    }
    setup(1024);start();header();body();finish(500);
    poll(499);assert(!f.commits);poll(1);assert(f.commits==1);
    setup(1024);start();header();body();finish(100);f.stage[700]^=1;
    poll(100);failed(W600_OTA_CRC);
    setup(1024);start();header();body();finish(100);f.allow=false;
    poll(100);failed(W600_OTA_AUTH);assert(f.authorizations==2);
    setup(1024);start();header();body();finish(100);f.commit_ok=false;
    poll(100);assert(f.ota.state==W600_OTA_FAILED && f.ota.error==W600_OTA_COMMIT && f.commits==1);
    setup(1024);start();header();body();sent();
    in=(struct w600_frame){.kind=7,.length=7,.payload={0x40,2,0,7}};
    in.sequence=f.ota.tx.sequence;process(&in);failed(W600_OTA_FORMAT);
    setup(1024);start();header();
    in=(struct w600_frame){.sequence=44,.kind=2,.length=4,.payload={0x40,0,0,1}};
    struct w600_frame outstanding=f.ota.tx;unsigned quiesces=f.quiesces;
    process(&in);assert(!memcmp(&outstanding,&f.ota.tx,sizeof(outstanding)) && f.quiesces==quiesces);
    body();finish(100);poll(100);assert(f.commits==1);
    assert(!w600_ota_duplicate_start(&f.ota,&in));
    setup(1024);
    in=(struct w600_frame){.sequence=44,.kind=2,.length=4,.payload={0x40,0,0,1}};
    assert(!w600_ota_duplicate_start(&f.ota,&in));
    start();assert(!w600_ota_duplicate_start(&f.ota,&in));
    sent();
    struct w600_ota snapshot=f.ota;
    assert(w600_ota_duplicate_start(&f.ota,&in));
    in.payload[2]=1;assert(w600_ota_duplicate_start(&f.ota,&in));
    in.payload[2]=2;assert(!w600_ota_duplicate_start(&f.ota,&in));in.payload[2]=0;
    assert(!memcmp(&snapshot,&f.ota,sizeof(snapshot)) && f.quiesces==1 && !f.writes);
    ++in.sequence;assert(!w600_ota_duplicate_start(&f.ota,&in));--in.sequence;
    in.payload[3]=0;assert(!w600_ota_duplicate_start(&f.ota,&in));in.payload[3]=1;
    in.length=3;assert(!w600_ota_duplicate_start(&f.ota,&in));in.length=4;
    in.kind=7;assert(!w600_ota_duplicate_start(&f.ota,&in));in.kind=2;
    poll(W600_OTA_REPLY_MS);assert(f.ota.pending && !w600_ota_duplicate_start(&f.ota,&in));
    sent();assert(w600_ota_duplicate_start(&f.ota,&in));
    assert(f.ota.retries==1 && f.quiesces==1 && !f.writes);
    poll(W600_OTA_TOTAL_MS);assert(f.ota.state==W600_OTA_FAILED);
    sent();assert(!w600_ota_duplicate_start(&f.ota,&in));
    assert(!w600_ota_duplicate_start(NULL,&in) && !w600_ota_duplicate_start(&f.ota,NULL));
    w600_ota_poll(NULL,0,&f.sequence);w600_ota_sent(NULL,0);
    assert(!w600_ota_active(NULL));
    assert(!w600_ota_restart_ready(NULL,0));
    for(unsigned fault=0;fault<3;++fault) {
        setup(1024);start();header();
        if(fault==0) { f.allow=false;body();failed(W600_OTA_AUTH); }
        else if(fault==1) {
            sent();in=(struct w600_frame){.sequence=f.ota.tx.sequence,.kind=7,.length=2,.payload={0x40,4}};
            process(&in);failed(W600_OTA_RADIO_ABORT);
        } else {poll(W600_OTA_TOTAL_MS);failed(W600_OTA_TIMEOUT);}
        assert(!w600_ota_restart_ready(&f.ota,f.now));
        assert(!w600_ota_restart_ready(&f.ota,f.ota.last_now+1999));
        assert(w600_ota_restart_ready(&f.ota,f.ota.last_now+2000));
        sent();assert(!w600_ota_restart_ready(&f.ota,f.now+99));
        assert(w600_ota_restart_ready(&f.ota,f.now+100));
    }
    f.ota=(struct w600_ota){.state=W600_OTA_FAILED,.last_now=UINT32_MAX-100,.pending=true};
    assert(!w600_ota_restart_ready(&f.ota,1898));
    assert(w600_ota_restart_ready(&f.ota,1899));
    printf("OTA receiver: %u transfer/fault fixtures passed\n",cases);
    return 0;
}
