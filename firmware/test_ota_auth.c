#include "ota_auth.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

enum { STAGE=0x08010800u, MAX_IMAGE=0xce00u };
static struct {
    uint8_t data[MAX_IMAGE];
    uint32_t length;
    unsigned calls,fail_read;
    bool reenter;
} source;
static unsigned cases;
static uint32_t scratch[W600_OTA_AUTH_WORDS];

static bool read_stage(void *context,uint32_t address,void *out,size_t length)
{
    assert(context==&source && out && length>0 && length<=64);
    assert(address>=STAGE && length<=source.length && address-STAGE<=source.length-length);
    ++source.calls;
    if(source.fail_read==source.calls)return false;
    if(source.reenter && source.calls==1)
        assert(!w600_ota_authorize_image(read_stage,&source,STAGE,source.length,47,46,scratch));
    memcpy(out,source.data+address-STAGE,length);return true;
}
static void digest(uint8_t out[32],const char *hex)
{
    assert(strlen(hex)==64);
    for(unsigned i=0;i<32;++i) {
        unsigned value=0;
        for(unsigned j=0;j<2;++j) {
            unsigned c=(unsigned char)hex[2*i+j];
            assert((c>='0' && c<='9') || (c>='a' && c<='f'));
            value=(value<<4)+(c<='9' ? c-'0' : c-'a'+10);
        }
        out[i]=(uint8_t)value;
    }
}
static void vector(const char *text,const char *expected)
{
    ++cases;source.length=(uint32_t)strlen(text);source.calls=0;source.fail_read=0;
    memcpy(source.data,text,source.length);
    uint8_t calculated[32],known[32];digest(known,expected);
    assert(w600_ota_auth_test_sha256(read_stage,&source,STAGE,source.length,calculated));
    assert(!memcmp(calculated,known,32));
    assert(source.calls==(source.length+63u)/64u);
}
static bool authorize(uint32_t length,uint32_t revision,uint32_t current)
{
    ++cases;source.calls=0;
    return w600_ota_authorize_image(read_stage,&source,STAGE,length,revision,current,scratch);
}
static void seal(uint32_t length,uint32_t revision)
{
    source.length=length;
    for(unsigned i=0;i<length;++i)source.data[i]=(uint8_t)i;
    const uint32_t identity[8]={0x30303657,0x5549432d,0x4f2d3233,0x00314154,1,revision,length,1};
    memcpy(source.data+length-64,identity,32);
    source.calls=0;
    assert(w600_ota_auth_test_sha256(read_stage,&source,STAGE,length-32,source.data+length-32));
}
int main(void)
{
    vector("","e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855");
    vector("abc","ba7816bf8f01cfea414140de5dae2223b00361a396177a9cb410ff61f20015ad");
    vector("abcdbcdecdefdefgefghfghighijhijkijkljklmklmnlmnomnopnopq",
           "248d6a61d20638b8e5c026930c3e6039a33ce45964ff2167f6ecedd419db06c1");
    vector("abcdefghbcdefghicdefghijdefghijkefghijklfghijklmghijklmn"
           "hijklmnoijklmnopjklmnopqklmnopqrlmnopqrsmnopqrstnopqrstu",
           "cf5b16a778af8380036ce59e7b0492370b249b11e8f07a51afac45037afee9d1");
    vector("aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa",
           "ffe054fe7ae0cb6dc65c3af9b61d5209f439851db43d0ba5997337df154668eb");

    seal(256,47);
    assert(authorize(256,47,46) && source.calls==5);
    assert(!authorize(252,47,46) && !source.calls);
    for(unsigned revision=0;revision<=256;++revision) {
        if(revision==47)continue;
        assert(!authorize(256,revision,46));
    }
    assert(!authorize(256,47,47) && !source.calls);
    assert(!authorize(256,47,48) && !source.calls);
    assert(!authorize(256,47,0) && !source.calls);
    assert(!w600_ota_authorize_image(NULL,&source,STAGE,256,47,46,scratch));
    assert(!w600_ota_authorize_image(read_stage,&source,STAGE+4,256,47,46,scratch));
    assert(!w600_ota_authorize_image(read_stage,&source,STAGE,256,47,46,NULL));
    for(unsigned i=0;i<256;++i) {
        source.data[i]^=1;
        assert(!authorize(256,47,46));
        source.data[i]^=1;
    }
    for(unsigned fail=1;fail<=5;++fail) {
        source.fail_read=fail;
        assert(!authorize(256,47,46) && source.calls==fail);
        source.fail_read=0;assert(authorize(256,47,46) && source.calls==5);
    }
    source.reenter=true;
    assert(authorize(256,47,46) && source.calls==5);
    source.reenter=false;
    for(unsigned i=0;i<4;++i) {
        uint32_t length=(uint32_t[]){252,257,MAX_IMAGE+4,UINT32_MAX}[i];
        assert(!authorize(length,47,46) && !source.calls);
    }
    seal(52720,255);assert(authorize(52720,255,254));
    uint8_t sum[32];
    assert(!w600_ota_auth_test_sha256(NULL,&source,STAGE,0,sum));
    assert(!w600_ota_auth_test_sha256(read_stage,&source,STAGE,0,NULL));
    assert(!w600_ota_auth_test_sha256(read_stage,&source,UINT32_MAX,4,sum));
    printf("OTA policy: %u identity/version/integrity/fault fixtures passed\n",cases);
    return 0;
}
