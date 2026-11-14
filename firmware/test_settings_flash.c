#define _GNU_SOURCE
#include "settings_flash.h"
#include "../flash-page/flash_regs.h"
#include <assert.h>
#include <stdbool.h>
#include <setjmp.h>
#include <stdio.h>
#include <string.h>
#include <sys/mman.h>

struct reg {uint32_t address,value;};
static struct reg regs[80];
static unsigned nregs,keys,erases,stores,cr_busy_writes,busy_left,uart_services,mode;
static uint32_t allowed_page;
static uint32_t flash[32768],core_values[5];
static volatile uint32_t *ticks=(void *)0x20000800u;
static struct w600_nv_request *request=(void *)0x20000400u;
static struct w600_nv_report *report=(void *)0x20000500u;
static uint32_t *image=(void *)0x20001000u;
static jmp_buf panic_return;
static uint32_t *reg(uint32_t a)
{
    for(unsigned i=0;i<nregs;++i)if(regs[i].address==a)return &regs[i].value;
    assert(nregs<80);regs[nregs]=(struct reg){a,0};return &regs[nregs++].value;
}
static uint32_t crc(const uint32_t *p)
{
    uint32_t c=~0u;
    for(unsigned i=0;i<512;++i) {
        c^=((const uint8_t *)p)[i];
        for(unsigned b=0;b<8;++b)c=c&1u ? (c>>1)^0xedb88320u : c>>1;
    }
    return ~c;
}
uint32_t nv_test_core(unsigned i){assert(i<5);return core_values[i];}
void nv_test_panic(void){longjmp(panic_return,1);}
uint32_t nv_test_read(uint32_t a)
{
    if(a>=0x20000000u && a<0x20007000u)return *(uint32_t *)(uintptr_t)a;
    if(a>=0x08000000u && a<0x08020000u) {
        assert(!(*reg(FP_SR)&FP_BUSY));
        return flash[(a-0x08000000u)/4];
    }
    if(a==0x40007014u && ((mode==4 && erases) || (mode==5 && stores>=2)))return 0x800u;
    if(a==FP_SR && (*reg(FP_SR)&FP_BUSY)) {
        assert(!core_values[0] && *reg(0xe000e100u)==(1u<<27));
        ++uart_services;
        if(mode!=9)++*ticks;
        if(mode!=8 && mode!=9 && busy_left && !--busy_left) {
            *reg(FP_SR)=mode==2 ? 0 : FP_EOP;
            if(mode==1 && stores==2)*reg(FP_SR)|=FP_ERRORS;
            *reg(FP_CR)&=~FP_ERASE;
        }
    }
    return *reg(a);
}
void nv_test_write(uint32_t a,uint32_t value)
{
    if(a==FP_SR) {*reg(a)&=~value;return;}
    if(a==FP_KEY) {
        if((keys++&1u)==0)assert(value==FP_KEY1);
        else {assert(value==FP_KEY2);if(mode!=10)*reg(FP_CR)=FP_OPTLOCK;}
        return;
    }
    if(a==FP_CR) {
        if(*reg(FP_SR)&FP_BUSY)++cr_busy_writes;
        assert(!(*reg(FP_SR)&FP_BUSY));
        *reg(a)=value;
        if(mode==6 && value==(FP_LOCK|FP_OPTLOCK))*reg(a)=FP_OPTLOCK;
        if(value&FP_ERASE) {
            uint32_t page=(value>>8)&511u;
            assert(page==allowed_page/512);
            ++erases;
            for(unsigned i=0;i<128;++i)flash[page*128u+i]=UINT32_MAX;
            if(mode==3)flash[page*128u]=0;
            *reg(FP_SR)=FP_BUSY;busy_left=3;
        }
        return;
    }
    assert(a>=0x08000000u+allowed_page && a<0x08000200u+allowed_page && !(a&3u));
    assert(!(*reg(FP_SR)&FP_BUSY) && *reg(FP_CR)==(FP_OPTLOCK|FP_PROGRAM));
    uint32_t base=0x08000000u+allowed_page;
    assert(a==base+480u+stores*4u);
    ++stores;flash[(a-0x08000000u)/4]&=value;
    if(mode==7 && stores==2)flash[(a-0x08000000u)/4]^=1;
    *reg(FP_SR)=FP_BUSY;busy_left=1;
}
static void setup(unsigned slot)
{
    memset((void *)0x20000000u,0,0x7000);
    nregs=keys=erases=stores=cr_busy_writes=busy_left=uart_services=mode=0;
    allowed_page=0xee00u+slot*512u;
    for(unsigned i=0;i<32768;++i)flash[i]=0xa5a5a5a5u;
    const struct reg initial[]={
        {FP_AHBEN,0x100},{FP_CFG,0x12},{FP_PLL,0x30260},{FP_CSR,FP_READY},{FP_ACR,2},
        {FP_SYST_LOAD,47999},{FP_SYST_CTRL,7},{FP_VTOR,0x20002000},{FP_CR,FP_LOCK|FP_OPTLOCK},
        {FP_OPTR,0xaa},{FP_WRPAS,511},{FP_WRPAE,0},{FP_WRPBS,511},{FP_WRPBE,0},
        {0xe000e100,1u<<27},{0xe000ed00,0x410cc601},{0x1fff03a4,2},{0x1fff03a8,128},{0x1fff03ac,28},
        {FP_IOPEN,7},{0x40021034,0x10060000},{0x40007004,9},
        {0x50000400,0x50000000},{0x50000410,0xc000},{0x50000414,0xc000},
        {0x50000000,0x40000000},{0x50000800,0x00100000}
    };
    for(unsigned i=0;i<sizeof(initial)/sizeof(initial[0]);++i)*reg(initial[i].address)=initial[i].value;
    core_values[0]=core_values[1]=core_values[2]=0;core_values[3]=0x20006f00;core_values[4]=0x20006100;
    const unsigned handlers[]={2,3,15,43};
    for(unsigned i=0;i<4;++i)*(uint32_t *)(uintptr_t)(0x20002000+4*handlers[i])=0x20006041;
    for(unsigned i=0;i<128;++i)image[i]=UINT32_MAX;
    for(unsigned i=120;i<128;++i)image[i]=0x10203040u+i;
    *ticks=10;
    *request=(struct w600_nv_request){image,ticks,crc(flash+(0xee00+slot*512)/4),crc(image),8,slot};
}
static void refused(enum w600_nv_status wanted)
{assert(w600_settings_flash_write(request,report)==wanted);assert(!keys && !erases && !stores);}
int main(void)
{
    assert(mmap((void *)0x20000000u,0x7000,PROT_READ|PROT_WRITE,
        MAP_PRIVATE|MAP_ANONYMOUS|MAP_FIXED_NOREPLACE,-1,0)==(void *)0x20000000u);
    for(unsigned slot=0;slot<2;++slot) {
        setup(slot);
        assert(w600_settings_flash_write(request,report)==W600_NV_OK);
        assert(report->status==W600_NV_OK && erases==1 && stores==8 && report->verified==128 && uart_services>=11);
        assert(!cr_busy_writes && report->final_cr==(FP_LOCK|FP_OPTLOCK));
        assert(!memcmp(flash+(0xee00+slot*512)/4,image,512));
        for(unsigned i=0;i<32768;++i)if(i<(0xee00+slot*512)/4 || i>=(0xf000+slot*512)/4)
            assert(flash[i]==0xa5a5a5a5u);
    }
    const struct reg reject[]={
        {FP_AHBEN,0x101},{FP_CFG,0},{FP_PLL,0},{FP_CSR,0},{FP_ACR,0},
        {FP_SYST_CTRL,5},{FP_SYST_LOAD,0},{0x40021034,0x10060004},{0x40021038,0x200},
        {0x50000410,0},{0x50000414,0},{0x50000010,0x8000},{0x50000810,0x400},
        {0x40007004,7},{0x40007014,0x800},{0xe000e100,0},{0xe000e200,1},
        {FP_VTOR,0x08002000},{0xe000ed00,0},{0x1fff03a4,0},{0x1fff03a8,256},{0x1fff03ac,0}
    };
    for(unsigned i=0;i<sizeof(reject)/sizeof(reject[0]);++i) {
        setup(0);*reg(reject[i].address)=reject[i].value;
        assert(w600_settings_flash_write(request,report)!=W600_NV_OK && !keys && !erases && !stores);
    }
    for(unsigned i=0;i<5;++i) {
        setup(0);core_values[i]=i<3 ? 1 : 0;
        refused(W600_NV_CONTEXT);
    }
    for(uint32_t sp=0x20006000u;sp<0x20006e00u;sp+=0x100u) {
        setup(0);core_values[3]=sp;refused(W600_NV_CONTEXT);
    }
    for(unsigned index=0;index<4;++index) {
        const unsigned at[]={2,3,15,43};setup(0);
        *(uint32_t *)(uintptr_t)(0x20002000+4*at[index])=0x080020c1;
        refused(W600_NV_CONTEXT);
    }
    setup(0);request->slot=2;refused(W600_NV_REQUEST);
    setup(0);request->old_crc^=1;refused(W600_NV_CRC);
    setup(0);request->image_crc^=1;refused(W600_NV_CRC);
    setup(0);request->pvd_ready_at=9;refused(W600_NV_VOLTAGE);
    setup(0);request->pvd_ready_at=11;refused(W600_NV_VOLTAGE);
    setup(0);*reg(FP_WRPAS)=*reg(FP_WRPAE)=0xee00/512;refused(W600_NV_PROTECTED);
    setup(0);*reg(FP_OPTR)=0;refused(W600_NV_CONTROLLER);
    setup(0);uint32_t vector=*(uint32_t *)0x20002000;
    assert(w600_settings_flash_write(request,(void *)0x20002000)==W600_NV_CONTEXT);
    assert(*(uint32_t *)0x20002000==vector && !keys);
    setup(0);assert(w600_settings_flash_write(request,(void *)image)==W600_NV_REQUEST && !keys);
    for(unsigned fault=1;fault<=10;++fault) {
        setup(0);mode=fault;
        if(!setjmp(panic_return)) {
            enum w600_nv_status result=w600_settings_flash_write(request,report);
            assert(fault!=8 && fault!=9 && result!=W600_NV_OK);
            if(fault==4 || fault==5)assert(result==W600_NV_VOLTAGE);
            if(fault==6)assert(result==W600_NV_LOCK);
        } else assert(fault==8 || fault==9);
        assert(!cr_busy_writes);
        if(fault!=8 && fault!=9 && fault!=6)assert(*reg(FP_CR)==(FP_LOCK|FP_OPTLOCK));
        for(unsigned i=0xf000/4;i<0xf200/4;++i)assert(flash[i]==0xa5a5a5a5u);
    }
    setup(0);*reg(FP_SR)=FP_BUSY;
    if(!setjmp(panic_return)){(void)w600_settings_flash_write(request,report);assert(0);}
    assert(!keys && !cr_busy_writes);
    /* Every stage page plus metadata pages; no active/settings/adjacent writes. */
    for(uint32_t address=0x08000000u;address<0x08020000u;address+=512u) {
        setup(0); allowed_page=address-0x08000000u;
        request->old_crc=crc(flash+allowed_page/4);
        bool accepted=(address>=0x08010800u && address<0x0801d600u) ||
            address==0x0801fc00u || address==0x0801fe00u || address==0x08001e00u;
        enum w600_nv_status status=w600_ota_flash_page_write(request,report,address);
        assert(status==(accepted ? W600_NV_OK : W600_NV_REQUEST));
        assert(erases==(unsigned)accepted && stores==(accepted ? 8u : 0u));
        for(unsigned i=0;i<32768;++i)if(!accepted || i<allowed_page/4 || i>=(allowed_page+512)/4)
            assert(flash[i]==0xa5a5a5a5u);
    }
    setup(0);assert(w600_ota_flash_page_write(request,report,0x08010801u)==W600_NV_REQUEST && !keys);
    setup(0);allowed_page=0x10800u;request->old_crc=crc(flash+allowed_page/4);mode=8;
    if(!setjmp(panic_return)){(void)w600_ota_flash_page_write(request,report,0x08010800u);assert(0);}
    assert(!cr_busy_writes);
    puts("PASS settings FLASH worker: two exact pages, commit-last ordering, IRQ-enabled waits, context/voltage/CRC/WRP guards, write faults and fatal busy/stalled-clock paths; mock MMIO only");
}
