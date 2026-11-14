#include "ota_auth.h"

/* FIPS180-4 SHA256, bounded here to the controller's 52736-byte image region. */
static const uint32_t k[64]={
    0x428a2f98,0x71374491,0xb5c0fbcf,0xe9b5dba5,0x3956c25b,0x59f111f1,0x923f82a4,0xab1c5ed5,
    0xd807aa98,0x12835b01,0x243185be,0x550c7dc3,0x72be5d74,0x80deb1fe,0x9bdc06a7,0xc19bf174,
    0xe49b69c1,0xefbe4786,0x0fc19dc6,0x240ca1cc,0x2de92c6f,0x4a7484aa,0x5cb0a9dc,0x76f988da,
    0x983e5152,0xa831c66d,0xb00327c8,0xbf597fc7,0xc6e00bf3,0xd5a79147,0x06ca6351,0x14292967,
    0x27b70a85,0x2e1b2138,0x4d2c6dfc,0x53380d13,0x650a7354,0x766a0abb,0x81c2c92e,0x92722c85,
    0xa2bfe8a1,0xa81a664b,0xc24b8b70,0xc76c51a3,0xd192e819,0xd6990624,0xf40e3585,0x106aa070,
    0x19a4c116,0x1e376c08,0x2748774c,0x34b0bcb5,0x391c0cb3,0x4ed8aa4a,0x5b9cca4f,0x682e6ff3,
    0x748f82ee,0x78a5636f,0x84c87814,0x8cc70208,0x90befffa,0xa4506ceb,0xbef9a3f7,0xc67178f2
};
static bool busy;
static const uint32_t magic[4]={0x30303657,0x5549432d,0x4f2d3233,0x00314154};
#ifdef W600_FLASH_APPLICATION
/* The build seals the final length and SHA256; kept last by flash-app.ld. */
const uint32_t w600_ota_identity[16] __attribute__((section(".ota_identity"),used))={
    0x30303657,0x5549432d,0x4f2d3233,0x00314154,
    1,W600_CONTROLLER_REVISION,0,1,0,0,0,0,0,0,0,0
};
#endif

static uint32_t rotate(uint32_t value,unsigned n)
{ return (value>>n)|(value<<(32u-n)); }
static uint32_t be32(const uint8_t *p)
{ return (uint32_t)p[0]<<24|(uint32_t)p[1]<<16|(uint32_t)p[2]<<8|p[3]; }
static void compress(uint32_t *sha)
{
    uint32_t *w=sha+8;
    uint8_t *block=(uint8_t *)(sha+24);
    uint32_t a=sha[0],b=sha[1],c=sha[2],d=sha[3];
    uint32_t e=sha[4],f=sha[5],g=sha[6],h=sha[7];
    for(unsigned i=0;i<64;++i) {
        if(i<16)w[i]=be32(block+4*i);
        else {
            uint32_t x=w[(i+1u)&15u],y=w[(i+14u)&15u];
            w[i&15u]+=(rotate(x,7)^rotate(x,18)^(x>>3))+w[(i+9u)&15u]+
                       (rotate(y,17)^rotate(y,19)^(y>>10));
        }
        uint32_t t1=h+(rotate(e,6)^rotate(e,11)^rotate(e,25))+
                    ((e&f)^(~e&g))+k[i]+w[i&15u];
        uint32_t t2=(rotate(a,2)^rotate(a,13)^rotate(a,22))+((a&b)^(a&c)^(b&c));
        h=g;g=f;f=e;e=d+t1;d=c;c=b;b=a;a=t1+t2;
    }
    sha[0]+=a;sha[1]+=b;sha[2]+=c;sha[3]+=d;
    sha[4]+=e;sha[5]+=f;sha[6]+=g;sha[7]+=h;
}
static bool calculate(w600_ota_stage_reader read,void *context,uint32_t address,uint32_t length,uint32_t *sha)
{
    static const uint32_t initial[8]={0x6a09e667,0xbb67ae85,0x3c6ef372,0xa54ff53a,
                                     0x510e527f,0x9b05688c,0x1f83d9ab,0x5be0cd19};
    for(unsigned i=0;i<8;++i)sha[i]=initial[i];
    uint8_t *block=(uint8_t *)(sha+24);
    uint32_t offset=0;
    while(offset<length) {
        uint32_t n=length-offset;
        if(n>64)n=64;
        if(!read(context,address+offset,block,n))return false;
        offset+=n;
        if(n==64)compress(sha);
    }
    unsigned tail=length&63u;
    block[tail]=0x80;
    for(unsigned i=tail+1;i<64;++i)block[i]=0;
    if(tail>=56) {
        compress(sha);
        for(unsigned i=0;i<64;++i)block[i]=0;
    }
    uint32_t bits=length*8u;
    block[60]=(uint8_t)(bits>>24);block[61]=(uint8_t)(bits>>16);
    block[62]=(uint8_t)(bits>>8);block[63]=(uint8_t)bits;
    compress(sha);return true;
}
bool w600_ota_authorize_image(w600_ota_stage_reader read,void *context,uint32_t stage,
                             uint32_t length,uint32_t revision,uint32_t current_revision,
                             uint32_t scratch[W600_OTA_AUTH_WORDS])
{
    if(!read || !scratch || ((uintptr_t)scratch&3u) || busy || stage!=0x08010800u || length<256u ||
       length>52720u || (length&3u) || !current_revision || revision<=current_revision || revision<46u || revision>255u)
        return false;
    busy=true;
    uint32_t *identity=scratch+40;
    bool valid=read(context,stage+length-64u,identity,64);
    if(valid) {
        for(unsigned i=0;i<4;++i)if(identity[i]!=magic[i])valid=false;
        if(identity[4]!=1 || identity[5]!=revision || identity[6]!=length || identity[7]!=1)valid=false;
    }
    if(valid)valid=calculate(read,context,stage,length-32u,scratch);
    volatile uint32_t difference=0;
    if(valid) {
        for(unsigned i=0;i<32;++i)
            difference|=((scratch[i/4]>>(24u-(i&3u)*8u))&255u)^((uint8_t *)(identity+8))[i];
    }
    busy=false;return valid && difference==0;
}
#ifdef W600_OTA_AUTH_TEST
bool w600_ota_auth_test_sha256(w600_ota_stage_reader read,void *context,uint32_t address,
                              uint32_t length,uint8_t digest[32])
{
    if(!read || !digest || busy || length>0xce00u || address>UINT32_MAX-length)return false;
    static uint32_t scratch[W600_OTA_AUTH_WORDS];
    busy=true;
    bool valid=calculate(read,context,address,length,scratch);
    if(valid)for(unsigned i=0;i<32;++i)digest[i]=(uint8_t)(scratch[i/4]>>(24u-(i&3u)*8u));
    busy=false;return valid;
}
#endif
