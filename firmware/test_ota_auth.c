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
    const struct w600_ota_authorization *target;
} source;
static unsigned cases;

static bool read_stage(void *context,uint32_t address,void *out,size_t length)
{
    assert(context==&source && out && length>0 && length<=64);
    assert(address>=STAGE && length<=source.length && address-STAGE<=source.length-length);
    ++source.calls;
    if(source.fail_read==source.calls)return false;
    if(source.reenter && source.calls==1)
        assert(!w600_ota_authorize_image(source.target,read_stage,&source,STAGE,
                                         source.length,30));
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
static bool authorize(const struct w600_ota_authorization *target,uint32_t length,uint32_t revision)
{
    ++cases;source.calls=0;
    return w600_ota_authorize_image(target,read_stage,&source,STAGE,length,revision);
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

    source.length=256;
    for(unsigned i=0;i<256;++i)source.data[i]=(uint8_t)i;
    struct w600_ota_authorization target={.length=256,.revision=30};
    digest(target.digest,"40aff2e9d2d8922e47afd4648e6967497158785fbd1da870e7110266bf944880");
    assert(authorize(&target,256,30) && source.calls==4);
    assert(!authorize(NULL,256,30) && !source.calls);
    assert(!authorize(&target,252,30) && !source.calls);
    assert(!authorize(&target,256,29) && !source.calls);
    assert(!w600_ota_authorize_image(&target,NULL,&source,STAGE,256,30));
    assert(!w600_ota_authorize_image(&target,read_stage,&source,STAGE+4,256,30));
    for(unsigned i=0;i<32;++i) {
        target.digest[i]^=1;
        assert(!authorize(&target,256,30) && source.calls==4);
        target.digest[i]^=1;
    }
    source.data[200]^=1;assert(!authorize(&target,256,30));source.data[200]^=1;
    for(unsigned fail=1;fail<=4;++fail) {
        source.fail_read=fail;
        assert(!authorize(&target,256,30) && source.calls==fail);
        source.fail_read=0;assert(authorize(&target,256,30) && source.calls==4);
    }
    source.reenter=true;source.target=&target;
    assert(authorize(&target,256,30) && source.calls==4);
    source.reenter=false;
    struct w600_ota_authorization missing={0};
    assert(!authorize(&missing,0,0) && !source.calls);
    for(unsigned i=0;i<4;++i) {
        uint32_t length=(uint32_t[]){188,193,MAX_IMAGE+4,UINT32_MAX}[i];
        target.length=length;
        assert(!authorize(&target,length,30) && !source.calls);
    }
    source.length=MAX_IMAGE;
    for(unsigned i=0;i<MAX_IMAGE;++i)source.data[i]=(uint8_t)i;
    target.length=MAX_IMAGE;
    assert(w600_ota_auth_test_sha256(read_stage,&source,STAGE,MAX_IMAGE,target.digest));
    assert(authorize(&target,MAX_IMAGE,30) && source.calls==MAX_IMAGE/64);
    assert(!w600_ota_auth_test_sha256(NULL,&source,STAGE,0,target.digest));
    assert(!w600_ota_auth_test_sha256(read_stage,&source,STAGE,0,NULL));
    assert(!w600_ota_auth_test_sha256(read_stage,&source,UINT32_MAX,4,target.digest));
    printf("OTA authorization: %u hash/pin/fault fixtures passed\n",cases);
    return 0;
}
