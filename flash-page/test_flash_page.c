#define _GNU_SOURCE
#include "flash_page.h"
#include "flash_regs.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

struct reg { uint32_t address, value; };
static struct reg regs[80];
static unsigned nregs, writes, keys, erases, stores, polls, busy_left, operation;
static unsigned faults, last_write, busy_cr_writes, scenario, failures, flash_reads;
static uint32_t core_values[5], flash[0x20000/4];
static uint32_t request_page;
static int busy_forever;
static struct ciu_flash_request *req;
static struct ciu_flash_report *out;
static uint32_t *image;

static uint32_t *reg(uint32_t a)
{
    for (unsigned i=0;i<nregs;++i) if (regs[i].address==a) return &regs[i].value;
    assert(nregs<80); regs[nregs]=(struct reg){a,0}; return &regs[nregs++].value;
}

static uint32_t crc(const uint32_t *p)
{
    uint32_t c=~0u;
    for (unsigned i=0;i<512;++i) {
        c ^= ((const unsigned char *)p)[i];
        for (unsigned bit=0;bit<8;++bit) c=(c>>1)^((0u-(c&1))&0xedb88320u);
    }
    return ~c;
}

uint32_t fp_test_core(unsigned i) { assert(i<5); return core_values[i]; }
uintptr_t fp_test_code(unsigned end) { return end ? 0x20005000u : 0x20004000u; }

uint32_t fp_test_read(uint32_t a)
{
    if (a>=0x20000000u && a<=0x20006ffcu) return *(uint32_t *)(uintptr_t)a;
    if (a>=0x08000000u && a<=0x0801fffcu) {
        assert(!(*reg(FP_SR)&FP_BUSY));
        ++flash_reads;
        if (scenario==19 && flash_reads>384 && a==request_page+508u) return 0;
        return flash[(a-0x08000000u)/4];
    }
    if (a==FP_SR && (*reg(FP_SR)&FP_BUSY)) {
        ++polls;
        if (!busy_forever && busy_left && !--busy_left) {
            *reg(FP_SR)=FP_EOP;
            *reg(FP_CR)&=~FP_ERASE;
            if (scenario==12 || (scenario==13 && operation==2)) *reg(FP_SR)|=FP_ERRORS;
            if (scenario==14) *reg(FP_SR)=0;
        }
    }
    if (a==FP_SYST_CTRL) {
        uint32_t v=*reg(a);
        if ((scenario==10 || scenario==11) && (*reg(FP_SR)&FP_BUSY)) v|=0x10000;
        if (scenario==18 && operation) v=0;
        return v;
    }
    return *reg(a);
}

void fp_test_write(uint32_t a, uint32_t v)
{
    ++writes; last_write=a;
    if (a==FP_KEY) {
        if (v==FP_KEY1 && !keys) { keys=1; return; }
        assert(v==FP_KEY2 && keys==1); keys=2;
        if (scenario!=6) *reg(FP_CR)&=~FP_LOCK;
        return;
    }
    if (a==FP_SR) {
        assert(v==(FP_ERRORS|FP_EOP)); assert(!(*reg(FP_SR)&FP_BUSY));
        if (scenario!=5) *reg(FP_SR)&=~v;
        return;
    }
    if (a==FP_CR) {
        if (*reg(FP_SR)&FP_BUSY) { ++busy_cr_writes; assert(0); }
        assert(!(v & (0x08000000u|0x10u|0x20u|0x03000000u)));
        assert((v&6u)!=6u); /* No option load/erase, fast/mass erase or IRQ. */
        if (v==(FP_LOCK|FP_OPTLOCK)) { if (scenario!=20) *reg(a)=v; return; }
        assert(keys==2 && !(v&FP_LOCK) && (v&FP_OPTLOCK));
        if (scenario==7 && (v&FP_PAGE_MODE)) return;
        if (scenario==8 && (v&FP_PROGRAM)) return;
        *reg(a)=v;
        if (v&FP_ERASE) {
            assert((v&7u)==FP_PAGE_MODE);
            uint32_t page=(v>>8)&511u, address=0x08000000u+page*512u;
            assert(address==request_page && address>=CIU_FLASH_APP_BEGIN && address<CIU_FLASH_APP_END);
            assert(!erases++); operation=1;
            for (unsigned i=0;i<128;++i) flash[page*128+i]=~0u;
            if (scenario==15) flash[page*128+127]=0;
            if (scenario==9) return; /* Start ignored: no fresh EOP. */
            *reg(FP_SR)=FP_BUSY; busy_left=3;
            if (scenario==10 || scenario==11 || scenario==17) busy_forever=1;
        }
        return;
    }
    assert(a>=CIU_FLASH_APP_BEGIN && a<CIU_FLASH_APP_END && !(a&3u));
    assert(*reg(FP_CR)==(FP_OPTLOCK|FP_PROGRAM) && erases==1);
    assert(a>=request_page && a<request_page+512u);
    assert(v!=~0u); ++stores; operation=2;
    flash[(a-0x08000000u)/4]=scenario==16 ? v^1u : v;
    *reg(FP_SR)=FP_BUSY; busy_left=2;
}

static void reset(unsigned kind)
{
    scenario=kind; nregs=writes=keys=erases=stores=polls=busy_left=operation=0;
    faults=last_write=busy_cr_writes=flash_reads=0; busy_forever=0;
    memset((void *)(uintptr_t)0x20000000u,0,0x7000u);
    req=(void *)(uintptr_t)0x20001000u; out=(void *)(uintptr_t)0x20001400u;
    image=(void *)(uintptr_t)0x20005800u;
    core_values[0]=1; core_values[1]=core_values[2]=0; core_values[3]=0x20006f00u;
    core_values[4]=0x20004010u;
    request_page=CIU_FLASH_APP_BEGIN;
    for(unsigned i=0;i<0x20000u/4;++i) flash[i]=0x5a5a5a5au^i;
    for(unsigned i=0;i<128;++i) image[i]=0x12345678u^i;
    *req=(struct ciu_flash_request){request_page,image,crc(flash+(request_page-0x08000000u)/4),crc(image),5,2,100};
    *reg(FP_VTOR)=0x20000000u;
    *(uint32_t *)(uintptr_t)0x20000008u=0x20003003u;
    *(uint32_t *)(uintptr_t)0x2000000cu=0x20003001u;
    *reg(FP_CFG)=0x12; *reg(FP_PLL)=0x30260; *reg(FP_CSR)=FP_READY; *reg(FP_ACR)=2;
    *reg(FP_AHBEN)=0x100; *reg(FP_SYST_CTRL)=5; *reg(FP_SYST_LOAD)=47999;
    *reg(0xe000ed00u)=0x410cc601; *reg(0x1fff03a4u)=2;
    *reg(0x1fff03a8u)=128; *reg(0x1fff03acu)=28;
    *reg(FP_IOPEN)=7; *reg(0x50000400u)=0x50000000;
    *reg(0x50000410u)=*reg(0x50000414u)=0xc000;
    *reg(0x50000000u)=0x40000000; *reg(0x50000800u)=0x100000;
    *reg(FP_CR)=FP_LOCK|FP_OPTLOCK; *reg(FP_OPTR)=0x1aa;
    *reg(FP_WRPAS)=*reg(FP_WRPBS)=511;
}

static void run(enum ciu_flash_status expected, int erased)
{
    uint32_t protected_before[0x2000/4]; memcpy(protected_before,flash,sizeof(protected_before));
    uint32_t reserved_before[0xf800/4]; memcpy(reserved_before,flash+0x10800/4,sizeof(reserved_before));
    enum ciu_flash_status actual=ciu_flash_replace_page(req,out);
    if(actual!=expected) fprintf(stderr,"scenario%u expected%d got%d\n",scenario,expected,actual);
    assert(actual==expected && out->status==(unsigned)expected);
    assert(erases==(unsigned)erased && !busy_cr_writes && keys<=2);
    assert(!memcmp(protected_before,flash,sizeof(protected_before)));
    assert(!memcmp(reserved_before,flash+0x10800/4,sizeof(reserved_before)));
    if(actual==CIU_FLASH_OK) {
        assert(out->locked && out->words_verified==128 && out->observed_final_crc==req->image_crc);
        assert(!memcmp(image,flash+(request_page-0x08000000u)/4,512));
    }
    if(!erased) assert(!stores);
    ++failures;
}

int main(void)
{
    void *mapped=mmap((void *)(uintptr_t)0x20000000u,0x7000u,PROT_READ|PROT_WRITE,
                      MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0);
    assert(mapped==(void *)(uintptr_t)0x20000000u);
    reset(0); run(CIU_FLASH_OK,1); assert(stores==128);
    reset(0); for(unsigned i=0;i<128;++i) image[i]=~0u; req->image_crc=crc(image);
    run(CIU_FLASH_OK,1); assert(!stores);
    reset(0); image[1]=image[127]=~0u; req->image_crc=crc(image); run(CIU_FLASH_OK,1); assert(stores==126);
    reset(0); request_page=req->address=CIU_FLASH_APP_END-512;
    req->expected_old_crc=crc(flash+(request_page-0x08000000u)/4); run(CIU_FLASH_OK,1);
    for(unsigned i=0;i<6;++i) {
        reset(0); req->address=(uint32_t[]){0x08000000,0x08001e00,0x08002004,0x08010800,0x0801f400,0xfffffe00}[i];
        run(CIU_FLASH_REQUEST,0); assert(!writes);
    }
    reset(0); req->expected_old_crc^=1; run(CIU_FLASH_OLD_CRC,0); assert(!writes&&out->locked);
    reset(0); req->image_crc^=1; run(CIU_FLASH_IMAGE_CRC,0); assert(!writes&&out->locked);
    reset(0); req->erase_ticks=0; run(CIU_FLASH_REQUEST,0);
    reset(0); req->program_ticks=0; run(CIU_FLASH_REQUEST,0);
    reset(0); req->poll_limit=0; run(CIU_FLASH_REQUEST,0);
    reset(0); req->erase_ticks=101; run(CIU_FLASH_REQUEST,0); assert(!writes);
    reset(0); req->program_ticks=21; run(CIU_FLASH_REQUEST,0); assert(!writes);
    reset(0); req->poll_limit=1000001; run(CIU_FLASH_REQUEST,0); assert(!writes);
    reset(0); req->erase_ticks=req->program_ticks=req->poll_limit=UINT32_MAX;
    run(CIU_FLASH_REQUEST,0); assert(!writes);
    reset(0); req->image=(void *)(uintptr_t)0x08002000; run(CIU_FLASH_REQUEST,0);
    reset(0); req->image=(void *)req; run(CIU_FLASH_REQUEST,0);
    reset(0); req->image=(void *)out; run(CIU_FLASH_REQUEST,0);
    reset(0); req->image=(void *)(uintptr_t)0x20004000; run(CIU_FLASH_REQUEST,0);
    reset(0); req->image=(void *)(uintptr_t)0x20006c00; run(CIU_FLASH_REQUEST,0);
    reset(0); core_values[0]=0; run(CIU_FLASH_CONTEXT,0);
    reset(0); core_values[1]=1; run(CIU_FLASH_CONTEXT,0);
    reset(0); core_values[2]=2; run(CIU_FLASH_CONTEXT,0);
    reset(0); core_values[4]=0x08002000; run(CIU_FLASH_CONTEXT,0);
    reset(0); *reg(0xe000e100u)=1; run(CIU_FLASH_CONTEXT,0);
    reset(0); *(uint32_t *)(uintptr_t)0x20000008u=0x08003001; run(CIU_FLASH_CONTEXT,0);
    reset(0); *(uint32_t *)(uintptr_t)0x2000000cu=0x20003000; run(CIU_FLASH_CONTEXT,0);
    for(unsigned i=0;i<7;++i) {
        reset(0); uint32_t a=(uint32_t[]){FP_CFG,FP_PLL,FP_ACR,FP_AHBEN,FP_SYST_CTRL,FP_SYST_LOAD,FP_AHBEN}[i];
        *reg(a)=i==6?0x101:0; run(CIU_FLASH_CLOCK,0); assert(!writes);
    }
    reset(0); *reg(FP_CSR)|=0x800; run(CIU_FLASH_CLOCK,0); assert(!writes);
    reset(0); *reg(FP_CSR)&=~0x400; run(CIU_FLASH_CLOCK,0); assert(!writes);
    for(unsigned i=0;i<8;++i) {
        reset(0); uint32_t a=(uint32_t[]){FP_IOPEN,0x50000400,0x50000404,0x50000410,0x50000414,0x50000010,0x50000814,0x50000800}[i];
        *reg(a)=i==2?0xc000:(i==5?0x8000:(i==6?0x400:0));
        run(CIU_FLASH_MOTOR,0); assert(!writes);
    }
    reset(0); *reg(FP_SR)=FP_BUSY; run(CIU_FLASH_CONTROLLER,0); assert(!writes);
    reset(0); *reg(FP_CR)=FP_OPTLOCK; run(CIU_FLASH_CONTROLLER,0); assert(!writes);
    reset(0); *reg(FP_CR)|=6; run(CIU_FLASH_CONTROLLER,0); assert(!writes);
    reset(0); *reg(FP_OPTR)=0; run(CIU_FLASH_CONTROLLER,0); assert(!writes);
    reset(0); *reg(FP_WRPAS)=*reg(FP_WRPAE)=16; run(CIU_FLASH_PROTECTED,0); assert(!writes);
    reset(0); *reg(FP_WRPBS)=15; *reg(FP_WRPBE)=17; run(CIU_FLASH_PROTECTED,0); assert(!writes);
    reset(0); *reg(FP_SR)=FP_EOP|FP_ERRORS; run(CIU_FLASH_OK,1); assert(out->initial_sr==(FP_EOP|FP_ERRORS));
    reset(5); *reg(FP_SR)=FP_ERRORS; run(CIU_FLASH_CONTROLLER,0);
    reset(6); run(CIU_FLASH_CONTROLLER,0); assert(out->locked);
    reset(7); run(CIU_FLASH_CONTROLLER,0); assert(out->locked);
    reset(8); run(CIU_FLASH_CONTROLLER,1); assert(out->locked);
    reset(9); run(CIU_FLASH_NO_EOP,1); assert(out->locked);
    reset(10); run(CIU_FLASH_TIMEOUT,1); assert(!out->locked&&polls<20);
    reset(11); req->erase_ticks=100; req->poll_limit=7; run(CIU_FLASH_POLL_LIMIT,1); assert(!out->locked&&polls<=9);
    reset(12); run(CIU_FLASH_ERROR,1); assert(out->locked);
    reset(13); run(CIU_FLASH_ERROR,1); assert(stores==1&&out->locked);
    reset(14); run(CIU_FLASH_NO_EOP,1); assert(out->locked);
    reset(15); run(CIU_FLASH_ERASE_VERIFY,1); assert(out->locked);
    reset(16); run(CIU_FLASH_WORD_VERIFY,1); assert(stores==1&&out->locked);
    reset(18); run(CIU_FLASH_CLOCK,1);
    reset(17); req->poll_limit=7; run(CIU_FLASH_POLL_LIMIT,1); assert(!out->locked&&polls<=9);
    reset(19); run(CIU_FLASH_FINAL_CRC,1); assert(out->locked);
    reset(20); run(CIU_FLASH_LOCK_FAILED,1); assert(!out->locked);
    reset(0); for(unsigned i=0;i<512;++i) ((unsigned char *)image)[i]=(unsigned char)i;
    req->image_crc=0x1c613576u; run(CIU_FLASH_OK,1); /* Independent Python/binascii fixture. */
    for(unsigned i=0;i<5;++i) {
        reset(0);
        if(i==1) req->image_crc^=1;
        if(i==2) req->expected_old_crc^=1;
        if(i==3) core_values[0]=0;
        if(i==4) *reg(FP_WRPAS)=*reg(FP_WRPAE)=16;
        enum ciu_flash_status expected=(enum ciu_flash_status[]){CIU_FLASH_OK,CIU_FLASH_IMAGE_CRC,
            CIU_FLASH_OLD_CRC,CIU_FLASH_CONTEXT,CIU_FLASH_PROTECTED}[i];
        assert(ciu_flash_check_page(req,out)==expected);
        assert(!writes&&!erases&&!stores&&!keys); ++failures;
    }
    reset(0); assert(ciu_flash_check_page(NULL,out)==CIU_FLASH_REQUEST); assert(!writes); ++failures;
    reset(0); assert(ciu_flash_check_page(req,NULL)==CIU_FLASH_REQUEST); assert(!writes); ++failures;
    reset(0); assert(ciu_flash_check_page(req,(void *)req)==CIU_FLASH_REQUEST); assert(!writes); ++failures;
    reset(0); core_values[3]=0x20000100;
    assert(ciu_flash_check_page(req,out)==CIU_FLASH_REQUEST); assert(!writes); ++failures;
    assert(munmap(mapped,0x7000)==0);
    printf("PASS%u flash-page actual-source scenarios; no device\n",failures);
    return 0;
}
