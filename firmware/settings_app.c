#include "settings_flash.h"
#include "settings_app.h"
#include "ciu32l071.h"

extern const uintptr_t w600_vectors[48];
extern uint8_t __settings_page0[],__settings_page1[];
void w600_settings_tick_handler(void);
void W600_UART3_4_Handler(void);

struct w600_settings_service w600_settings_state;
struct w600_nv_report w600_settings_write_report;
static struct w600_settings_workspace workspace;
static uint32_t vectors[48] __attribute__((aligned(256)));
static struct w600_nv_request request;
static bool ota_taken;

static uint32_t checksum(const uint8_t *p)
{
    uint32_t crc=UINT32_MAX;
    for(unsigned i=0;i<512;++i) {
        crc^=p[i];
        for(unsigned bit=0;bit<8;++bit)crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
static bool read_page(void *context,unsigned slot,uint8_t out[512])
{
    (void)context;
    if(slot>1 || !out || (FLASH->SR&0x10000u) ||
       (uintptr_t)__settings_page0!=0x0800ee00u ||
       (uintptr_t)__settings_page1!=0x0800f000u)return false;
    const volatile uint8_t *page=(const volatile uint8_t *)(__settings_page0+slot*512u);
    for(unsigned i=0;i<512;++i)out[i]=page[i];
    return true;
}
static bool replace_at(uint32_t address,uint32_t old_crc,const uint8_t image[512],bool ota)
{
    if(__get_PRIMASK() || __get_IPSR() || __get_CONTROL() || !image ||
       SCB->VTOR!=(uint32_t)w600_vectors || RCC->APB1EN!=0x60000u ||
       (FLASH->SR&0x10000u))return false;
    /* Own PVD only while all foreground peripheral owners are idle. */
    RCC->APB1EN|=RCC_APB1EN_PMUEN;__DSB();
    if(PMU->CR1 || PMU->CR2 || PMU->CR3) {
        RCC->APB1EN&=~RCC_APB1EN_PMUEN;__DSB();return false;
    }
    PMU->CR2=9u;__DSB();
    request.image=(const uint32_t *)image;
    request.old_crc=old_crc;request.image_crc=checksum(image);
    request.pvd_ready_at=*request.ticks;
    unsigned polls=0;
    while((uint32_t)(*request.ticks-request.pvd_ready_at)<2u && ++polls<200000u)__NOP();
    bool ok=false;
    if(polls<200000u && PMU->CR2==9u) {
        __disable_irq();
        SCB->VTOR=(uint32_t)vectors;__DSB();__ISB();
        __enable_irq();
        ok=(ota ? w600_ota_flash_page_write(&request,&w600_settings_write_report,address) :
                  w600_settings_flash_write(&request,&w600_settings_write_report))==W600_NV_OK;
        __disable_irq();
        SCB->VTOR=(uint32_t)w600_vectors;__DSB();__ISB();
        __enable_irq();
    }
    /* Level bits are protected while PVD is enabled: disable, then clear. */
    PMU->CR2=8u;__DSB();
    PMU->CR2=0;__DSB();
    if(PMU->CR2)ok=false;
    RCC->APB1EN&=~RCC_APB1EN_PMUEN;__DSB();
    return ok && RCC->APB1EN==0x60000u;
}
static bool replace_page(void *context,unsigned slot,uint32_t old_crc,const uint8_t image[512])
{
    (void)context;
    if(ota_taken || slot>1)return false;
    request.slot=slot;
    return replace_at(0x0800ee00u+slot*512u,old_crc,image,false);
}
static const struct w600_settings_io io={NULL,read_page,replace_page};

bool w600_settings_app_init(struct w600_endpoint *e,const volatile uint32_t *ticks)
{
    if(__get_PRIMASK()!=1 || __get_IPSR() || __get_CONTROL() || !ticks || request.ticks)return false;
    request.ticks=ticks;
    for(unsigned i=0;i<48;++i)vectors[i]=(uint32_t)w600_vectors[i];
    vectors[2]=vectors[3]=(uint32_t)w600_settings_flash_fault;
    vectors[15]=(uint32_t)w600_settings_tick_handler;
    vectors[43]=(uint32_t)W600_UART3_4_Handler;
    return w600_settings_service_init(&w600_settings_state,&io,&workspace,e);
}
void w600_settings_app_poll(const struct w600_endpoint *e,uint32_t now,bool ready)
{
    if(ota_taken)return;
    w600_settings_service_poll(&w600_settings_state,&io,&workspace,e,now,ready);
}
uint32_t (*w600_settings_app_ota_takeover(void))[128]
{
    if(__get_PRIMASK() || __get_IPSR() || __get_CONTROL() || !request.ticks)return NULL;
    ota_taken=true;
    return workspace.pages;
}
bool w600_settings_app_ota_replace(uint32_t address,uint32_t old_crc,const uint32_t image[128])
{
    if(!ota_taken)return false;
    return replace_at(address,old_crc,(const uint8_t *)image,true);
}
