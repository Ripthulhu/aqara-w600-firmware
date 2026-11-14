#include "settings_flash.h"
#include "../flash-page/flash_regs.h"

#ifdef W600_NV_TEST
uint32_t nv_test_read(uint32_t);
void nv_test_write(uint32_t,uint32_t);
uint32_t nv_test_core(unsigned);
void nv_test_panic(void) __attribute__((noreturn));
#define RD(a) nv_test_read(a)
#define WR(a,v) nv_test_write(a,v)
#define CORE(i) nv_test_core(i)
#define BEGIN 0x20006000u
#define END 0x20006980u
#define PANIC() nv_test_panic()
#else
#define RD(a) (*(volatile const uint32_t *)(uintptr_t)(a))
#define WR(a,v) (*(volatile uint32_t *)(uintptr_t)(a)=(v))
extern uint8_t __nvcode_start[],__nvcode_end[];
#define BEGIN ((uintptr_t)__nvcode_start)
#define END ((uintptr_t)__nvcode_end)
static uint32_t W600_NV_RAM core(unsigned index)
{
    uint32_t v;
    switch(index) {
    case 0:__asm volatile("mrs %0, primask":"=r"(v));break;
    case 1:__asm volatile("mrs %0, ipsr":"=r"(v));break;
    case 2:__asm volatile("mrs %0, control":"=r"(v));break;
    case 3:__asm volatile("mrs %0, msp":"=r"(v));break;
    default:__asm volatile("mov %0, pc":"=r"(v));break;
    }
    return v;
}
#define CORE(i) core(i)
void W600_NV_RAM w600_settings_flash_fault(void)
{
    __asm volatile("cpsid i");
    WR(0x50000828u,1u<<10); WR(0x50000028u,1u<<15); WR(0x50000418u,0xc000u);
    for(;;)__asm volatile("nop");
}
#define PANIC() w600_settings_flash_fault()
#endif

enum { PVD_CR2=0x40007004u,PVD_SR=0x40007014u,PVD_LOW=0x800u,
       NV_IRQ=1u<<27, POLL_LIMIT=1000000u };
static int W600_NV_RAM ram(uintptr_t p,size_t n)
{ return !(p&3u) && n<=0x6000u && p>=0x20000000u && p<=0x20006000u-n; }
static int W600_NV_RAM overlap(uintptr_t a,size_t n,uintptr_t b,size_t m)
{ return a<b+m && b<a+n; }
static uint32_t W600_NV_RAM checksum(const uint32_t *image,uint32_t flash)
{
    uint32_t crc=UINT32_MAX;
    for(unsigned i=0;i<128;++i) {
        uint32_t word=flash ? RD(flash+i*4u) : image[i];
        for(unsigned byte=0;byte<4;++byte) {
            crc^=word&255u;word>>=8;
            for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
        }
    }
    return ~crc;
}
static enum w600_nv_status W600_NV_RAM finish(struct w600_nv_report *out,enum w600_nv_status status,int owned)
{
    out->final_sr=RD(FP_SR);
    if(out->final_sr&FP_BUSY) {out->status=W600_NV_BUSY_FATAL;PANIC();}
    if(owned) {
        WR(FP_CR,FP_LOCK|FP_OPTLOCK);
        if(RD(FP_CR)!=(FP_LOCK|FP_OPTLOCK))status=W600_NV_LOCK;
    }
    out->final_cr=RD(FP_CR);out->status=status;return status;
}
static int W600_NV_RAM stopped(void)
{
    return (RD(FP_IOPEN)&7u)==7u && !(RD(0x40021038u)&~0x08000000u) &&
        RD(0x40021034u)==0x10060000u &&
        (RD(0x50000400u)&0xf0000000u)==0x50000000u &&
        !(RD(0x50000404u)&0xc000u) && (RD(0x50000414u)&0xc000u)==0xc000u &&
        (RD(0x50000410u)&0xc000u)==0xc000u &&
        (RD(0x50000000u)&0xc0000000u)==0x40000000u &&
        !(RD(0x50000004u)&0x8000u) && !(RD(0x50000014u)&0x8000u) && !(RD(0x50000010u)&0x8000u) &&
        (RD(0x50000800u)&0x00300000u)==0x00100000u &&
        !(RD(0x50000804u)&0x400u) && !(RD(0x50000814u)&0x400u) && !(RD(0x50000810u)&0x400u);
}
static enum w600_nv_status W600_NV_RAM wait_done(const struct w600_nv_request *r,
    struct w600_nv_report *out,uint32_t limit)
{
    uint32_t start=*r->ticks;
    for(unsigned i=0;i<POLL_LIMIT;++i) {
        uint32_t sr=RD(FP_SR);
        out->low_voltage|=(RD(PVD_SR)&PVD_LOW)!=0;
        if((uint32_t)(*r->ticks-start)>=limit) return finish(out,W600_NV_TIMEOUT,1);
        if(!(sr&FP_BUSY)) {
            if(sr&FP_ERRORS || !(sr&FP_EOP))return W600_NV_PROGRAM;
            return out->low_voltage ? W600_NV_VOLTAGE : W600_NV_OK;
        }
    }
    return finish(out,W600_NV_TIMEOUT,1);
}
static enum w600_nv_status W600_NV_RAM issue(const struct w600_nv_request *r,
    struct w600_nv_report *out,uint32_t address,uint32_t value,uint32_t limit)
{
    if(RD(PVD_SR)&PVD_LOW){out->low_voltage=1;return W600_NV_VOLTAGE;}
    WR(FP_SR,FP_EOP|FP_ERRORS);
    if(RD(FP_SR)&(FP_EOP|FP_ERRORS|FP_BUSY))return W600_NV_CONTROLLER;
    WR(address,value);
    return wait_done(r,out,limit);
}
static enum w600_nv_status W600_NV_RAM write_page(const struct w600_nv_request *r,
    struct w600_nv_report *out,uint32_t address)
{
    uintptr_t req=(uintptr_t)r,report=(uintptr_t)out;
    if((RD(FP_AHBEN)&0x100u) && (RD(FP_SR)&FP_BUSY))PANIC();
    if(!ram(req,sizeof(*r)) || !ram(report,sizeof(*out)) || overlap(req,sizeof(*r),report,sizeof(*out)))return W600_NV_REQUEST;
    if(!ram((uintptr_t)r->image,512) || !ram((uintptr_t)r->ticks,4) ||
       overlap((uintptr_t)r->image,512,req,sizeof(*r)) || overlap((uintptr_t)r->image,512,report,sizeof(*out)) ||
       overlap((uintptr_t)r->ticks,4,(uintptr_t)r->image,512) || overlap((uintptr_t)r->ticks,4,report,sizeof(*out)) ||
       overlap((uintptr_t)r->ticks,4,req,sizeof(*r)))return W600_NV_REQUEST;
    uint32_t vectors=RD(FP_VTOR),sp=CORE(3);
    if(CORE(0) || CORE(1) || CORE(2) || sp<0x20006e00u || sp>0x20007000u || (sp&7u) ||
       CORE(4)<BEGIN || CORE(4)>=END || !ram(vectors,192) || (vectors&255u) ||
       overlap(vectors,192,req,sizeof(*r)) || overlap(vectors,192,report,sizeof(*out)) ||
       overlap(vectors,192,(uintptr_t)r->image,512) || overlap(vectors,192,(uintptr_t)r->ticks,4) ||
       RD(0xe000e100u)!=NV_IRQ || (RD(0xe000e200u)&~NV_IRQ) || RD(0xe000ed24u) ||
       RD(0xe000ed00u)!=0x410cc601u || RD(0x1fff03a4u)!=2 || RD(0x1fff03a8u)!=128 || RD(0x1fff03acu)!=28)
        return W600_NV_CONTEXT;
    volatile uint32_t *words=(volatile uint32_t *)out;
    for(unsigned i=0;i<sizeof(*out)/4;++i)words[i]=0;
    out->status=W600_NV_CONTEXT;
    for(unsigned i=0;i<4;++i) {
        unsigned index=i<2 ? i+2 : i==2 ? 15 : 43;
        uint32_t entry=RD(vectors+index*4u);
        if(!(entry&1u) || entry<BEGIN || entry>=END)return W600_NV_CONTEXT;
    }
    if(RD(FP_CFG)!=0x12u || RD(FP_PLL)!=0x30260u || (RD(FP_CSR)&FP_READY_MASK)!=FP_READY ||
       (RD(FP_AHBEN)&0x101u)!=0x100u || RD(FP_ACR)!=2 || RD(FP_SYST_LOAD)!=47999u ||
        (RD(FP_SYST_CTRL)&7u)!=7u)return out->status=W600_NV_CLOCK;
    if(!stopped())return out->status=W600_NV_MOTOR;
    uint32_t pvd_age=*r->ticks-r->pvd_ready_at;
    if(RD(PVD_CR2)!=9 || pvd_age<2 || pvd_age>1000 || (RD(PVD_SR)&PVD_LOW))return out->status=W600_NV_VOLTAGE;
    uint32_t page=(address-0x08000000u)/512u;
    if(RD(FP_SR)&FP_BUSY)return finish(out,W600_NV_CONTROLLER,0);
    if(RD(FP_CR)!=(FP_LOCK|FP_OPTLOCK) || (RD(FP_OPTR)&255u)!=0xaau)return out->status=W600_NV_CONTROLLER;
    for(unsigned i=0;i<2;++i) {
        uint32_t first=RD(FP_WRPAS+i*8u)&511u,last=RD(FP_WRPAE+i*8u)&511u;
        if(first<=page && page<=last)return out->status=W600_NV_PROTECTED;
    }
    if(checksum(NULL,address)!=r->old_crc || checksum(r->image,0)!=r->image_crc)return out->status=W600_NV_CRC;
    WR(FP_KEY,FP_KEY1);WR(FP_KEY,FP_KEY2);
    if(RD(FP_CR)!=FP_OPTLOCK)return finish(out,W600_NV_CONTROLLER,1);
    uint32_t erase=FP_OPTLOCK|FP_PAGE_MODE|(page<<8);
    WR(FP_CR,erase);
    if(RD(FP_CR)!=erase)return finish(out,W600_NV_CONTROLLER,1);
    out->erases=1;
    enum w600_nv_status status=issue(r,out,FP_CR,erase|FP_ERASE,100);
    if(status!=W600_NV_OK)return finish(out,status,1);
    for(unsigned i=0;i<128;++i)if(RD(address+i*4u)!=UINT32_MAX)return finish(out,W600_NV_VERIFY,1);
    WR(FP_CR,FP_OPTLOCK|FP_PROGRAM);
    if(RD(FP_CR)!=(FP_OPTLOCK|FP_PROGRAM))return finish(out,W600_NV_CONTROLLER,1);
    for(unsigned i=0;i<128;++i) {
        uint32_t value=r->image[i];
        if(value!=UINT32_MAX) {
            ++out->stores;status=issue(r,out,address+i*4u,value,20);
            if(status!=W600_NV_OK)return finish(out,status,1);
        }
        if(RD(address+i*4u)!=value)return finish(out,W600_NV_VERIFY,1);
        ++out->verified;
    }
    return finish(out,checksum(NULL,address)==r->image_crc ? W600_NV_OK : W600_NV_CRC,1);
}

enum w600_nv_status W600_NV_RAM w600_settings_flash_write(const struct w600_nv_request *r,
    struct w600_nv_report *out)
{
    if((RD(FP_AHBEN)&0x100u) && (RD(FP_SR)&FP_BUSY))PANIC();
    if(!ram((uintptr_t)r,sizeof(*r)) || r->slot>1)return W600_NV_REQUEST;
    return write_page(r,out,0x0800ee00u+r->slot*512u);
}

enum w600_nv_status W600_NV_RAM w600_ota_flash_page_write(const struct w600_nv_request *r,
    struct w600_nv_report *out,uint32_t address)
{
    if((RD(FP_AHBEN)&0x100u) && (RD(FP_SR)&FP_BUSY))PANIC();
    if((address&511u) || !((address>=0x08010800u && address<0x0801d600u) ||
       address==0x0801fc00u || address==0x0801fe00u || address==0x08001e00u))
        return W600_NV_REQUEST;
    return write_page(r,out,address);
}
