/* Replacement application core; persistent builds accept explicit calibration. */
#include <stddef.h>
#include <stdint.h>
#include "radio.h"
#include "../replacement-control/w600_buttons.h"
#ifdef W600_MOTOR_SERVICE
#include "motor_service.h"
#include "temperature_adc.h"
#ifdef W600_FLASH_APPLICATION
#include "valve_calibration.h"
#include "settings_flash.h"
#include "settings_app.h"
#include "sleep_clock.h"
#include "stop_mode.h"
#include "vbat_adc.h"
#ifdef W600_OTA_RECEIVER
#include "ota_flash.h"
#include "ota_auth.h"
#endif
#endif
#endif
#ifdef W600_DISPLAY_BENCH
#include "display_tx.h"
#include "../replacement-control/w600_display.h"

static uint8_t display_status(const struct w600_endpoint *endpoint)
{
    /* Stock display byte2: BD4C error bit2, 06DE8 child-lock bit4. */
    uint8_t status=endpoint->child_locked ? 0x10u : 0;
    if (endpoint->calibration_error ||
        (endpoint->control_error>=W600_CONTROL_TEMPERATURE &&
         endpoint->control_error<=W600_CONTROL_DRIVER)) status|=0x04u;
    return status;
}
static void target_frame(const struct w600_endpoint *endpoint, uint8_t frame[4])
{
    for (unsigned i=0;i<4;++i) frame[i]=0;
    frame[2]=display_status(endpoint);
    if (!(endpoint->known&W600_KNOWN_TARGET) || endpoint->target_raw>9990u) {
        frame[0]=frame[1]=0x04u; /* Centre segment from the existing digit map. */
        return;
    }
    uint32_t raw=endpoint->target_raw, decic=0;
    /* Bounded by999 iterations, only when a changed target can be submitted. */
    while (raw>=10u) { raw-=10u; ++decic; }
    (void)w600_display_encode(decic,false,frame);
}
#endif

struct app_report {
    uint32_t magic, version, stage, error, motor_disabled, tick;
    uint32_t pairing_attempts, pairing_request_accepted;
    uint32_t radio_error, reply_count, management_count, pairing_stage;
    uint32_t reserved[20];
};
static struct w600_radio radio;
#ifdef W600_OTA_RECEIVER
static struct w600_ota ota;
static bool ota_owned;
#endif
static struct w600_buttons button;
volatile struct app_report app_report __attribute__((section(".appreport"),used));
static volatile uint32_t app_ticks;
#ifdef W600_DISPLAY_BENCH
static uint32_t display_activity_at;
static bool display_awake;
static bool display_should_show(uint32_t now,uint8_t events)
{
    if (events&W600_BUTTON_WAKE) {
        display_activity_at=now; display_awake=true;
    }
    if ((uint32_t)(now-display_activity_at)>=10000u) display_awake=false;
    return display_awake;
}
#endif

static uint8_t button_poll(uint32_t now, int pressed_mask)
{
    uint8_t events=w600_buttons_poll(&button,now,(uint8_t)pressed_mask);
    struct w600_endpoint *e=&radio.endpoint;
    if (events&W600_BUTTON_LOCK) {
        e->child_locked=!e->child_locked;
        e->known|=W600_KNOWN_CHILD_LOCK;
        (void)w600_radio_report_control(&radio);
    }
    if (events&W600_BUTTON_PAIR) {
        e->child_locked=false;
        e->known|=W600_KNOWN_CHILD_LOCK;
        (void)w600_radio_report_control(&radio);
        ++app_report.pairing_attempts;
        app_report.pairing_request_accepted=(uint32_t)w600_radio_request_pairing(&radio,now);
    }
    if (e->child_locked || !(events&6u)) return events;
    bool known=(e->known&W600_KNOWN_TARGET) && e->target_raw>=500u && e->target_raw<=3000u;
    uint16_t target=known ? e->target_raw : 2000u;
    if ((events&6u)==W600_BUTTON_DECREASE) target=target<=550u ? 500u : (uint16_t)(target-50u);
    if ((events&6u)==W600_BUTTON_INCREASE) target=target>=2950u ? 3000u : (uint16_t)(target+50u);
    if ((!known || target!=e->target_raw) && w600_endpoint_set_target(e,target))
        (void)w600_radio_report_target(&radio);
    return events;
}

#ifndef W600_APP_HOST_TEST
#include "ciu32l071.h"
#include "../startup/w600_board.h"
#include "../startup/w600_motor_stop.h"
#ifdef W600_DISPLAY_BENCH
#include "../startup/w600_display_board.h"
static struct w600_display_tx display;
static uint32_t display_target=UINT32_MAX;
static uint32_t display_submitted, display_completed;
static int display_failure_handled;

static int display_poll(uint32_t now,uint8_t events)
{
    enum w600_display_tx_phase before=display.phase;
    if (!w600_display_tx_poll(&display,now)) return 0;
    if (before!=W600_DISPLAY_TX_IDLE && display.phase==W600_DISPLAY_TX_IDLE)
        ++display_completed;
    uint32_t target=(radio.endpoint.known&W600_KNOWN_TARGET) && radio.endpoint.target_raw<=9990u ?
        radio.endpoint.target_raw : 0x10001u;
    bool visible=display_should_show(now,events);
    if (!visible) target=0x10000u;
    uint8_t status=visible ? display_status(&radio.endpoint) : 0;
    bool changed=target!=display_target || status!=display.frame[2];
    if (display.phase==W600_DISPLAY_TX_IDLE && changed) {
        uint8_t frame[4]={0};
        if (visible) target_frame(&radio.endpoint,frame);
        if (!w600_display_tx_submit(&display,frame,now)) return 0;
        display_target=target; ++display_submitted;
    }
    return 1;
}

static void display_publish(void)
{
    app_report.reserved[0]=1;
    app_report.reserved[1]=display.error;
    app_report.reserved[2]=display.phase;
    app_report.reserved[3]=display_submitted;
    app_report.reserved[4]=display_completed;
    app_report.reserved[5]=display_target;
}
#endif
#ifdef W600_RAW_PROBE_BENCH
#include "motor_probe.h"
static struct motor_probe probe;
volatile struct motor_probe_snapshot probe_report __attribute__((section(".probereport"),used));
_Static_assert(sizeof(probe_report)<=512,"Raw bench report slot");
#endif

#ifdef W600_MOTOR_SERVICE
static struct w600_motor_service motor;
#ifdef W600_FLASH_APPLICATION
static struct w600_valve_calibration valve;
volatile struct w600_mount_report mounting_report;
static uint32_t valve_reported;
static uint32_t valve_reported_at;
static bool valve_source_reported_known;
static bool motor_power_ready;
static struct w600_app_time control_time;
static struct w600_sleep_clock sleep_clock;
static uint32_t sleep_clock_attempt_at;
static bool sleep_clock_attempted;
/* shortcut: bench-only until radio wake lead time and real STOP recovery pass. */
volatile uint32_t w600_stop_bench_enable;
static struct w600_stop_report stop_report;
static struct w600_vbat_adc vbat;
static bool vbat_attempted;
volatile uint32_t vbat_report[32];
static bool vbat_idle(void)
{
    return !vbat.board.owned && !vbat.feedback.owns_adc && !vbat.cleanup_failed &&
        !(vbat.phase>=W600_VBAT_ADC_INIT && vbat.phase<=W600_VBAT_ADC_CLEANUP);
}
#endif
volatile uint32_t motor_report[32] __attribute__((section(".probereport"),used));
static uint32_t motor_begin_used;
static struct w600_temperature_adc temperature;
static uint32_t temperature_last_attempt;
static bool temperature_started, temperature_valid;
#ifdef W600_FLASH_APPLICATION
static bool app_idle_ready(void)
{
    return motor_power_ready && motor.stop_confirmed && !motor.owned &&
        !motor.board.owned && !motor.cleanup_failed &&
        (motor.phase==W600_MOTOR_IDLE || motor.phase==W600_MOTOR_DONE || motor.phase==W600_MOTOR_FAILED) &&
        !radio.endpoint.calibration_busy && !radio.endpoint.calibration_pending &&
        !radio.endpoint.moving && !temperature.owned && !temperature.board.owned &&
        !temperature.cleanup_failed && vbat_idle() &&
        !(temperature.phase>=W600_TADC_INIT && temperature.phase<=W600_TADC_CLEANUP) &&
        !display_failure_handled && display.phase==W600_DISPLAY_TX_IDLE &&
        radio.tx_phase==W600_RADIO_IDLE && !radio.reply_pending && !radio.management_pending &&
        !radio.temperature_pending && !radio.control_pending && !radio.target_pending &&
        radio.rx_head==radio.rx_tail && !radio.decoder.used;
}
#ifdef W600_OTA_RECEIVER
static bool ota_quiesce(void *context)
{
    (void)context;
    if(!motor_power_ready || !motor.stop_confirmed || motor.owned || motor.board.owned ||
       motor.cleanup_failed || radio.endpoint.calibration_busy || radio.endpoint.moving ||
       radio.endpoint.calibration_pending || temperature.owned || temperature.board.owned ||
       temperature.cleanup_failed || !vbat_idle() || sleep_clock.owned || display_failure_handled ||
       display.phase!=W600_DISPLAY_TX_IDLE || radio.tx_phase!=W600_RADIO_IDLE ||
       (temperature.phase>=W600_TADC_INIT && temperature.phase<=W600_TADC_CLEANUP))return false;
    if(!ota_owned && !w600_ota_flash_set_workspace(w600_settings_app_ota_takeover()))return false;
    ota_owned=true;
    radio.endpoint.control_enabled=false;radio.endpoint.mode=0;radio.endpoint.mode_known=true;
    radio.endpoint.cancel_calibration=true;
    radio.temperature_pending=radio.control_pending=radio.target_pending=false;
    return true;
}
static uint32_t ota_page_crc(const uint32_t *page)
{
    uint32_t crc=UINT32_MAX;
    for(unsigned i=0;i<512;++i) {
        crc^=((const uint8_t *)page)[i];
        for(unsigned b=0;b<8;++b)crc=(crc>>1)^((0u-(crc&1u))&0xedb88320u);
    }
    return ~crc;
}
static bool ota_stage_read(void *context,uint32_t address,void *out,size_t length)
{
    (void)context;
    if(!out || address<W600_OTA_STAGE || length>W600_OTA_MAX_IMAGE ||
       address-W600_OTA_STAGE>W600_OTA_MAX_IMAGE-length)return false;
    if(FLASH->SR&0x10000u)return false;
    uint8_t *p=out;
    const volatile uint8_t *from=(const volatile uint8_t *)(uintptr_t)address;
    for(size_t i=0;i<length;++i)p[i]=from[i];
    return true;
}
static bool ota_stage_write(void *context,uint32_t address,const uint32_t page[128])
{
    if((address&511u) || address<W600_OTA_STAGE ||
       address>=W600_OTA_STAGE+W600_OTA_MAX_IMAGE || !ota_quiesce(context) ||
       (FLASH->SR&0x10000u))return false;
    return w600_ota_flash_write_page(address,ota_page_crc((const uint32_t *)(uintptr_t)address),page);
}
static bool ota_authorize(void *context,uint32_t address,uint32_t length,uint32_t revision)
{
    return w600_ota_authorize_image(ota_stage_read,context,address,length,revision,
                                    W600_CONTROLLER_REVISION,ota.page);
}
static bool ota_commit(void *context,uint32_t length,uint16_t crc,uint32_t revision)
{
    return ota_quiesce(context) && w600_ota_flash_commit(length,crc,revision);
}
static void ota_failed_restart(uint32_t now,bool transport_failed)
{
    if(ota_owned && ota.state==W600_OTA_FAILED &&
       (transport_failed || w600_ota_restart_ready(&ota,now)) &&
       motor.stop_confirmed && !motor.owned && !motor.board.owned &&
       !motor.cleanup_failed && !temperature.owned && !temperature.board.owned &&
       vbat_idle() && !(FLASH->SR&0x10000u) && w600_ota_flash_restart_safe()) {
        __disable_irq();NVIC_SystemReset();
    }
}
static const struct w600_ota_ops ota_ops={ota_quiesce,ota_stage_write,ota_stage_read,
                                        ota_authorize,ota_commit};
#endif
#endif
static void motor_publish(void)
{
    motor_report[0]=0x57364d53u; motor_report[1]=2;
    motor_report[2]=motor.phase; motor_report[3]=motor.error;
    motor_report[4]=motor.stop_confirmed; motor_report[5]=motor.samples;
    motor_report[6]=motor.last_mv; motor_report[7]=motor.homing.state;
    motor_report[8]=motor.homing.error; motor_report[9]=motor.homing.guard.error;
    motor_report[10]=motor.owned; motor_report[11]=motor.board.owned;
    motor_report[12]=motor.cleanup_failed; motor_report[13]=motor_begin_used;
    motor_report[14]=motor.direction; motor_report[15]=motor.last_ms;
    motor_report[16]=motor.feedback.sample.error;
    struct w600_motor_time time;
    motor_report[22]=(uint32_t)w600_motor_service_time(&motor,&time);
    motor_report[17]=time.now_ticks;
    motor_report[18]=time.on_lower; motor_report[19]=time.on_upper;
    motor_report[20]=time.sample_on_lower; motor_report[21]=time.sample_on_upper;
}
static void temperature_publish(void)
{
    const struct w600_temperature_adc_sample *s=&temperature.sample;
    app_report.reserved[7]=1;
    app_report.reserved[8]=temperature.phase;
    app_report.reserved[9]=s->error;
    app_report.reserved[10]=temperature_valid;
    app_report.reserved[11]=s->sequence;
    app_report.reserved[12]=s->completed_at_ms;
    app_report.reserved[13]=s->temperature.filtered_adc;
    app_report.reserved[14]=(uint32_t)(int32_t)s->temperature.sensor_decic;
    app_report.reserved[15]=(uint32_t)(int32_t)s->temperature.adjusted_decic;
    app_report.reserved[16]=(uint32_t)s->raw[0]|(uint32_t)s->raw[1]<<16;
    app_report.reserved[17]=(uint32_t)s->raw[2]|(uint32_t)s->raw[3]<<16;
    app_report.reserved[18]=s->raw[4];
    app_report.reserved[19]=temperature.cleanup_failed;
}
static void temperature_poll(uint32_t now)
{
    bool busy=temperature.phase>=W600_TADC_INIT && temperature.phase<=W600_TADC_CLEANUP;
    if (!busy && !temperature.sample.error && (motor.phase==W600_MOTOR_IDLE ||
        motor.phase==W600_MOTOR_DONE || motor.phase==W600_MOTOR_FAILED) &&
        motor.stop_confirmed && !motor.owned && !motor.board.owned && !motor.cleanup_failed &&
        !display_failure_handled && !radio.endpoint.calibration_busy && !radio.endpoint.moving &&
#ifdef W600_FLASH_APPLICATION
        !sleep_clock.owned && vbat_idle() &&
#endif
        !radio.endpoint.calibration_pending && now>=1000u &&
        (!temperature_started || (uint32_t)(now-temperature_last_attempt)>=60000u)) {
        temperature_started=true; temperature_last_attempt=now;
        temperature_valid=false;
        (void)w600_endpoint_local_temperature(&radio.endpoint,0,false);
        (void)w600_temperature_adc_begin(&temperature,now,0);
    }
    struct w600_temperature_adc_sample sample;
    enum w600_temperature_adc_status status=w600_temperature_adc_poll(&temperature,now,&sample);
    if (status==W600_TADC_SAMPLE) {
        temperature_valid=w600_endpoint_local_temperature(&radio.endpoint,sample.temperature.adjusted_decic,true);
        if (temperature_valid) (void)w600_radio_report_temperature(&radio);
    }
    if (temperature.sample.error || (temperature_valid &&
        (uint32_t)(now-temperature.sample.completed_at_ms)>=120000u)) {
        temperature_valid=false;
        (void)w600_endpoint_local_temperature(&radio.endpoint,0,false);
    }
    temperature_publish();
}
static void temperature_abort(uint32_t now)
{
    w600_temperature_adc_abort(&temperature);
    (void)w600_temperature_adc_poll(&temperature,now,0);
    temperature_valid=false;
    (void)w600_endpoint_local_temperature(&radio.endpoint,0,false);
    temperature_publish();
}
#ifdef W600_FLASH_APPLICATION
static void vbat_publish(void)
{
    const struct w600_vbat_adc_sample *s=&vbat.sample;
    vbat_report[0]=0x57364241u; vbat_report[1]=1;
    vbat_report[2]=vbat.phase; vbat_report[3]=s->error; vbat_report[4]=s->valid;
    vbat_report[5]=s->millivolts; vbat_report[6]=s->factory_bgr;
    vbat_report[7]=s->started_at_ms; vbat_report[8]=s->completed_at_ms;
    vbat_report[9]=s->sequence; vbat_report[10]=vbat.board.owned;
    vbat_report[11]=vbat.feedback.owns_adc; vbat_report[12]=vbat.cleanup_failed;
    vbat_report[13]=s->raw.oldest_at_ms; vbat_report[14]=s->raw.completed_at_ms;
    vbat_report[15]=s->raw.sequence; vbat_report[16]=s->raw.error;
    vbat_report[17]=s->raw.valid; vbat_report[18]=s->raw.input;
    for (unsigned i=0;i<3;++i) {
        vbat_report[19+3*i]=s->raw.pair[i].bgr;
        vbat_report[20+3*i]=s->raw.pair[i].signal;
        vbat_report[21+3*i]=s->raw.pair[i].acquired_at_ms;
    }
}
static void vbat_poll(uint32_t now)
{
    /* One diagnostic acquisition per boot; it is not yet public battery data. */
    if (!vbat_attempted && now>=1500u && !sleep_clock.owned && app_idle_ready()) {
        vbat_attempted=true;
        (void)w600_vbat_adc_begin(&vbat,now);
    }
    (void)w600_vbat_adc_poll(&vbat,now,0);
    vbat_publish();
}
static void vbat_abort(uint32_t now)
{
    w600_vbat_adc_abort(&vbat);
    (void)w600_vbat_adc_poll(&vbat,now,0);
    vbat_publish();
}
#endif
#endif

#if defined(W600_RAW_PROBE_BENCH) || defined(W600_MOTOR_SERVICE)

static int probe_power_init(void)
{
    uint32_t am=GPIOA->MODE,at=GPIOA->OTYPE,ap=GPIOA->PUPD,ao=GPIOA->ODR;
    uint32_t bm=GPIOB->MODE,bt=GPIOB->OTYPE,bp=GPIOB->PUPD,bo=GPIOB->ODR;
    if (__get_PRIMASK()!=1 || __get_IPSR() || __get_CONTROL() ||
        (RCC->IOPEN&7u)!=7u || (am&0xc00u)!=0xc00u ||
        (bm&0x03000003u)!=0x03000003u || (at&0x20u) || (bt&0x1001u) ||
        (ap&0xc00u) || (bp&0x03000003u) || (ao&0x20u) || (bo&0x1001u)) return 0;
    /* Previously qualified HHL context, not an inferred motor-rail enable. */
    GPIOA->BSR=0x20u; GPIOB->BSR=1u; GPIOB->BR=0x1000u;
    GPIOA->OTYPE&=~0x20u; GPIOB->OTYPE&=~0x1001u;
    GPIOA->PUPD=(GPIOA->PUPD&~0xc00u)|0x800u;
    GPIOB->PUPD=(GPIOB->PUPD&~0x03000003u)|0x01000002u;
    __DSB();
    if (GPIOA->ODR!=(ao|0x20u) || GPIOB->ODR!=((bo|1u)&~0x1000u) ||
        GPIOA->OTYPE!=(at&~0x20u) || GPIOB->OTYPE!=(bt&~0x1001u) ||
        GPIOA->PUPD!=((ap&~0xc00u)|0x800u) ||
        GPIOB->PUPD!=((bp&~0x03000003u)|0x01000002u)) return 0;
    GPIOA->MODE=(GPIOA->MODE&~0xc00u)|0x400u;
    GPIOB->MODE=(GPIOB->MODE&~0x03000003u)|0x01000001u;
    __DSB();
    return GPIOA->MODE==((am&~0xc00u)|0x400u) &&
        GPIOB->MODE==((bm&~0x03000003u)|0x01000001u) &&
        GPIOA->OTYPE==(at&~0x20u) && GPIOB->OTYPE==(bt&~0x1001u) &&
        GPIOA->PUPD==((ap&~0xc00u)|0x800u) &&
        GPIOB->PUPD==((bp&~0x03000003u)|0x01000002u) &&
        GPIOA->ODR==(ao|0x20u) && GPIOB->ODR==((bo|1u)&~0x1000u) &&
        (GPIOA->IDR&0x20u) && (GPIOB->IDR&0x1001u)==1u;
}

#ifdef W600_RAW_PROBE_BENCH
static void probe_publish(void) { motor_probe_snapshot(&probe,&probe_report); }
#endif
#endif

static struct w600_board_status board_status;
extern const uintptr_t w600_vectors[48];

static int motor_off(void)
{
    uint32_t mask=__get_PRIMASK();
    __disable_irq();
    int stopped=w600_motor_stop();
    __set_PRIMASK(mask);
    return stopped;
}

static void fatal(uint32_t error,uint32_t radio_error) __attribute__((noreturn));
static void fatal(uint32_t error,uint32_t radio_error)
{
    __disable_irq();
    uint32_t stopped=(uint32_t)motor_off();
    if (RCC->IOPEN&1u) GPIOA->BSR=1u<<11;
    if (RCC->APB1EN&RCC_APB1EN_UART3EN) UART3->CR1=0;
#ifdef W600_DISPLAY_BENCH
    if (RCC->APB1EN&RCC_APB1EN_UART2EN) UART2->CR1=0;
    if (RCC->IOPEN&1u) GPIOA->BSR=1u<<4;
#endif
    SysTick->CTRL=0;
    volatile uint32_t *words=(volatile uint32_t *)&app_report;
    for (unsigned i=0;i<sizeof(app_report)/4;++i) words[i]=0;
    app_report.magic=0x57364150u; app_report.version=1;
    app_report.stage=0xe0; app_report.error=error;
    app_report.motor_disabled=stopped; app_report.radio_error=radio_error;
    __DSB();
    for (;;) __NOP();
}

void w600_reset_fault(void) { fatal(1,0); }

int w600_reset_early(void)
{
    /* This path runs before data/BSS initialization. */
    uint32_t initial_iopen=RCC->IOPEN;
    if (!motor_off()) return 0;
    return __get_PRIMASK()==1 && !__get_IPSR() && !__get_CONTROL() &&
        SCB->VTOR==(uint32_t)w600_vectors && !SCB->ICSR && !SCB->SHCSR &&
        !NVIC->ISER[0] && !NVIC->ISPR[0] && !(SysTick->CTRL&7u) &&
        SCB->CPUID==0x410cc601u &&
        *(volatile const uint32_t *)0x1fff03a4u==2u &&
        *(volatile const uint32_t *)0x1fff03a8u==128u &&
        *(volatile const uint32_t *)0x1fff03acu==28u &&
#ifdef W600_FLASH_APPLICATION
        !(initial_iopen&~7u) && RCC->CFG==0x12u && RCC->PLLCFG==0x30260u &&
#else
        !initial_iopen && !RCC->CFG && RCC->PLLCFG==0x20260u &&
#endif
        !RCC->CLKSEL &&
        RCC->AHBEN==0x100u && !RCC->APB1EN && !RCC->APB2EN &&
#ifdef W600_FLASH_APPLICATION
        FLASH->ACR==2u &&
#else
        !FLASH->ACR &&
#endif
        !FLASH->SR && FLASH->CR==0xc0000000u &&
        !IWDG->CFG && !IWDG->WINR && !IWDG->CNT && !IWDG->CNT;
}

void W600_SysTick_Handler(void)
{
#ifdef W600_MOTOR_SERVICE
    w600_motor_service_tick(&motor,++app_ticks);
#ifdef W600_FLASH_APPLICATION
    w600_valve_calibration_tick(&valve,app_ticks*10u);
#endif
#elif defined(W600_RAW_PROBE_BENCH)
    motor_probe_tick(&probe,++app_ticks);
#else
    if (!motor_off()) fatal(8,0);
    ++app_ticks;
#endif
}

#ifdef W600_FLASH_APPLICATION
void W600_NV_RAM w600_settings_tick_handler(void) { ++app_ticks; }
W600_NV_RAM
#endif
void W600_UART3_4_Handler(void) { w600_radio_irq(&radio,app_ticks); }

void w600_app_main(void)
{
    volatile uint32_t *words=(volatile uint32_t *)&app_report;
    for (unsigned i=0;i<sizeof(app_report)/4;++i) words[i]=0;
    app_report.magic=0x57364150u; app_report.version=1;
    app_report.stage=1; app_report.motor_disabled=1;
    if (!w600_board_init(&board_status)) fatal(2,0);
#ifdef W600_RAW_PROBE_BENCH
    if (!probe_power_init() || !motor_probe_init(&probe)) fatal(9,0);
    probe_publish();
#endif
#ifdef W600_MOTOR_SERVICE
    if (!probe_power_init()) fatal(9,0);
#endif
#ifdef W600_DISPLAY_BENCH
    if (!w600_display_board_init() || !w600_display_tx_init(&display,0)) {
        (void)w600_display_board_stop();
        fatal(11,0);
    }
    display_publish();
#endif
    GPIOA->MODE&=~(3u<<4); GPIOA->PUPD&=~(3u<<4);
    GPIOB->MODE&=~((3u<<16)|(3u<<2));
    GPIOB->PUPD=(GPIOB->PUPD&~((3u<<16)|(3u<<2)))|(1u<<16);
    if ((GPIOA->MODE&(3u<<4)) || (GPIOA->PUPD&(3u<<4)) ||
        (GPIOB->MODE&((3u<<16)|(3u<<2))) ||
        (GPIOB->PUPD&((3u<<16)|(3u<<2)))!=(1u<<16)) fatal(3,0);
    if (!w600_radio_init(&radio,0)) fatal(4,radio.error);
#ifdef W600_MOTOR_SERVICE
    for (unsigned i=0;i<32;++i) motor_report[i]=0;
    if (!w600_motor_service_init(&motor,&app_ticks)) fatal(12,0);
#ifdef W600_FLASH_APPLICATION
    if (!w600_valve_calibration_init(&valve,&motor,&radio.endpoint)) fatal(13,0);
    if (!w600_settings_app_init(&radio.endpoint,&app_ticks)) fatal(15,0);
#ifdef W600_OTA_RECEIVER
    if(!w600_ota_flash_init(&app_ticks,ota_authorize,NULL))fatal(18,0);
    w600_ota_init(&ota,&ota_ops,NULL);radio.ota=&ota;
    radio.reply=(struct w600_frame){.sequence=radio.next_sequence++,.kind=5,.length=5,
        .payload={0x20,2,3,(uint8_t)(W600_CONTROLLER_REVISION>>8),(uint8_t)W600_CONTROLLER_REVISION}};
    radio.reply_pending=true;
#endif
    if (radio.endpoint.known&W600_KNOWN_TARGET) (void)w600_radio_report_target(&radio);
#endif
    motor_publish();
#endif
    NVIC_SetPriority(UART3_4_IRQn,0);
    NVIC_ClearPendingIRQ(UART3_4_IRQn);
    NVIC_EnableIRQ(UART3_4_IRQn);
    if (SysTick_Config(48000)) fatal(5,0);
    /* Receive bytes before the longer timer bookkeeping can overrun the UART. */
    NVIC_SetPriority(SysTick_IRQn,1);
    if (NVIC_GetPriority(UART3_4_IRQn)!=0 || NVIC_GetPriority(SysTick_IRQn)!=1) fatal(5,0);
    app_report.stage=2;
    __DMB(); __enable_irq();
    for (;;) {
#ifdef W600_MOTOR_SERVICE
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        uint32_t now=app_ticks;
#ifdef W600_FLASH_APPLICATION
        if (sleep_clock.owned && (!app_idle_ready() ||
            (sleep_clock.arming && w600_stop_bench_enable!=1u))) {
            if (!w600_sleep_clock_cancel(&sleep_clock)) fatal(16,0);
            sleep_clock_attempt_at=now;
        }
        w600_sleep_clock_poll(&sleep_clock);
        if (sleep_clock.cleanup_failed) fatal(16,0);
#endif
#ifdef W600_MOTOR_SERVICE_BENCH
        if (!motor_begin_used && now>=250u) {
            motor_begin_used=1;
            (void)w600_motor_service_begin(&motor,W600_MOTOR_DIRECTION);
        }
#endif
        w600_motor_service_poll(&motor);
#ifdef W600_FLASH_APPLICATION
        __set_PRIMASK(mask); __disable_irq();
        w600_valve_calibration_feedback(&valve);
#endif
        /* Keep controller/report work out of the sub-millisecond ADC window. */
        if (w600_motor_service_urgent(&motor)) {
            __set_PRIMASK(mask);
            continue;
        }
        /* Let pending UART bytes through between independently serialized operations. */
        __set_PRIMASK(mask); __disable_irq();
#ifdef W600_FLASH_APPLICATION
        if (!w600_app_clock(&app_ticks,&control_time)) fatal(14,0);
        uint32_t control_ticks=control_time.ms*10u+
            (control_time.cycles-control_time.ms*48000u)/4800u;
        if (control_time.ms>=101u) motor_power_ready=true;
        bool sensor_busy=!motor_power_ready || display_failure_handled || sleep_clock.owned ||
            (temperature.phase>=W600_TADC_INIT && temperature.phase<=W600_TADC_CLEANUP) || !vbat_idle();
#ifdef W600_OTA_RECEIVER
        sensor_busy|=ota_owned;
#endif
        if (display_failure_handled) {
            radio.endpoint.calibration_pending=false;
            radio.endpoint.cancel_calibration=true;
        }
        __set_PRIMASK(mask); __disable_irq();
        /* A pending SysTick may have advanced the sequencer past this snapshot. */
        if ((uint32_t)(valve.sequence.last_now-control_ticks)<10u)
            control_ticks=valve.sequence.last_now;
#ifdef W600_OTA_RECEIVER
        if(!ota_owned) {
#endif
            w600_valve_mounting_poll(&valve,control_ticks,sensor_busy,&mounting_report);
            w600_valve_calibration_poll(&valve,control_ticks,sensor_busy);
#ifdef W600_OTA_RECEIVER
        }
#endif
        __set_PRIMASK(mask); __disable_irq();
        uint32_t valve_state=(uint32_t)radio.endpoint.calibrated |
            (uint32_t)radio.endpoint.calibration_busy<<1 |
            (uint32_t)radio.endpoint.moving<<2 | (uint32_t)radio.endpoint.opening_known<<3 |
            (uint32_t)radio.endpoint.mode<<4 | (uint32_t)radio.endpoint.mode_known<<6 |
            (uint32_t)radio.endpoint.external_use<<7 |
            (uint32_t)radio.endpoint.calibration_error<<8 |
            (uint32_t)radio.endpoint.control_error<<16 |
            (uint32_t)radio.endpoint.opening_permille<<20;
        bool source_known=(radio.endpoint.known&W600_KNOWN_EXTERNAL_USE)!=0;
        if ((valve_state!=valve_reported || source_known!=valve_source_reported_known) &&
            (!radio.endpoint.moving || source_known!=valve_source_reported_known ||
             ((valve_state^valve_reported)&0x000fffffu) ||
             (uint32_t)(now-valve_reported_at)>=1000u) && w600_radio_report_control(&radio)) {
            valve_reported=valve_state; valve_reported_at=now;
            valve_source_reported_known=source_known;
        }
#endif
        __set_PRIMASK(mask); __disable_irq();
        motor_publish(); app_report.motor_disabled=motor.stop_confirmed;
        __set_PRIMASK(mask); __disable_irq();
        int urgent=w600_motor_service_urgent(&motor);
#ifdef W600_OTA_RECEIVER
        if(ota_owned)urgent=0;
        if(!ota_owned) {
#endif
        if (!urgent) temperature_poll(now);
#ifdef W600_FLASH_APPLICATION
        if (!urgent) vbat_poll(now);
#endif
#ifdef W600_OTA_RECEIVER
        }
#endif
        __set_PRIMASK(mask);
        /* UART IRQs keep receiving during the bounded1ms sampling gap. */
        if (urgent) continue;
        now=app_ticks;
#elif defined(W600_RAW_PROBE_BENCH)
        uint32_t mask=__get_PRIMASK(); __disable_irq();
        uint32_t now=app_ticks;
        if (!probe.report.begin_used && probe.report.state==MOTOR_PROBE_SERVICE_IDLE &&
            !probe.report.error && now>=250u) {
            app_report.motor_disabled=0;
            if (!motor_probe_begin(&probe,now,2)) {
                motor_probe_abort(&probe,now); app_report.error=10;
            }
        }
        if ((probe.report.state>=MOTOR_PROBE_SERVICE_INITIALIZING &&
             probe.report.state<=MOTOR_PROBE_SERVICE_CLEANUP) ||
            (probe.report.state==MOTOR_PROBE_SERVICE_FAILED && !probe.cleanup_attempted)) {
            app_report.motor_disabled=0;
            motor_probe_foreground(&probe,now);
        }
        if (probe_report.last_tick_ms!=now || probe_report.state!=probe.report.state)
            probe_publish();
        app_report.motor_disabled=probe.report.stop_confirmed;
        __set_PRIMASK(mask);
#else
        uint32_t now=app_ticks;
        if (!motor_off()) fatal(6,0);
#endif
        if (!w600_radio_poll(&radio,now)) {
#ifdef W600_OTA_RECEIVER
            ota_failed_restart(now,true);
#endif
#ifdef W600_MOTOR_SERVICE
            __disable_irq(); temperature_abort(app_ticks);
#ifdef W600_FLASH_APPLICATION
            vbat_abort(app_ticks);
#endif
            if (motor.phase!=W600_MOTOR_DONE && motor.phase!=W600_MOTOR_FAILED)
                w600_motor_service_abort(&motor);
            w600_motor_service_poll(&motor);
            motor_publish();
#endif
#ifdef W600_RAW_PROBE_BENCH
            __disable_irq(); motor_probe_abort(&probe,app_ticks); probe_publish();
#endif
            fatal(7,radio.error);
        }
#ifdef W600_OTA_RECEIVER
        ota_failed_restart(now,false);
        if(ota.state==W600_OTA_COMMITTED) {
            if(!ota_quiesce(NULL))fatal(19,0);
            __disable_irq();NVIC_SystemReset();
        }
#endif
        uint32_t inputs_b=GPIOB->IDR,inputs_a=GPIOA->IDR;
        uint8_t pressed=(uint8_t)((!(inputs_b&(1u<<8)) ? W600_BUTTON_PRIMARY : 0) |
            (!(inputs_a&(1u<<2)) ? W600_BUTTON_DECREASE : 0) |
            (!(inputs_b&(1u<<1)) ? W600_BUTTON_INCREASE : 0));
        uint8_t button_events;
#ifdef W600_OTA_RECEIVER
        if(ota_owned)button_events=0;
        else
#endif
            button_events=button_poll(now,pressed);
#ifdef W600_DISPLAY_BENCH
        if (
#ifdef W600_OTA_RECEIVER
            !ota_owned &&
#endif
            !display_poll(app_ticks,button_events) && !display_failure_handled) {
            uint32_t saved=__get_PRIMASK(); __disable_irq();
#ifdef W600_MOTOR_SERVICE
            temperature_abort(app_ticks);
#ifdef W600_FLASH_APPLICATION
            vbat_abort(app_ticks);
            motor_calibration_abort(&valve.sequence);
            radio.endpoint.calibration_pending=false;
            radio.endpoint.cancel_calibration=true;
            radio.endpoint.control_enabled=false;
#endif
            if (motor.phase!=W600_MOTOR_DONE && motor.phase!=W600_MOTOR_FAILED)
                w600_motor_service_abort(&motor);
            w600_motor_service_poll(&motor); motor_publish();
            app_report.reserved[6]=(motor.owned || motor.board.owned || temperature.owned || temperature.board.owned
#ifdef W600_FLASH_APPLICATION
                || !vbat_idle()
#endif
                ) ? 0u :
                (uint32_t)w600_display_board_stop();
#else
            motor_probe_abort(&probe,app_ticks); probe_publish();
            /* Keep UART2's clock if motor/ADC ownership could not be released. */
            app_report.reserved[6]=probe.report.cleanup_pending ? 0u :
                (uint32_t)w600_display_board_stop();
#endif
            display_failure_handled=1;
            __set_PRIMASK(saved);
        }
        display_publish();
#endif
#ifndef W600_DISPLAY_BENCH
        (void)button_events;
#endif
#ifdef W600_FLASH_APPLICATION
        bool settings_ready=app_idle_ready() && !sleep_clock.owned;
#ifdef W600_OTA_RECEIVER
        settings_ready&=!ota_owned;
#endif
        w600_settings_app_poll(&radio.endpoint,app_ticks,settings_ready);
#endif
        app_report.tick=now;
        app_report.reply_count=radio.tx_reply_count;
        app_report.management_count=radio.tx_management_count;
        app_report.pairing_stage=(uint32_t)radio.pairing.stage;
#ifdef W600_FLASH_APPLICATION
        uint32_t idle_mask=__get_PRIMASK(); __disable_irq();
        if (
#ifdef W600_OTA_RECEIVER
            !ota_owned &&
#endif
            app_idle_ready() && sleep_clock.phase==W600_SC_OFF &&
            (!sleep_clock_attempted || (uint32_t)(app_ticks-sleep_clock_attempt_at)>=1000u)) {
            sleep_clock_attempted=true; sleep_clock_attempt_at=app_ticks;
            (void)w600_sleep_clock_begin(&sleep_clock,&app_ticks);
        }
        if (
#ifdef W600_OTA_RECEIVER
            !ota_owned &&
#endif
            w600_stop_bench_enable==1u && app_idle_ready() && display_target==0x10000u &&
            !pressed && (uint32_t)(app_ticks-display_activity_at)>=10000u) {
            if (sleep_clock.phase==W600_SC_DONE)
                (void)w600_sleep_clock_arm(&sleep_clock,&app_ticks);
            if (sleep_clock.phase==W600_SC_ARMED) {
                enum w600_stop_result result=w600_stop_mode(&sleep_clock,&app_ticks,&stop_report);
                if (!w600_sleep_clock_cancel(&sleep_clock) || result==W600_STOP_FAILED) fatal(17,stop_report.error);
            }
        }
        if (app_idle_ready()) (void)w600_app_idle();
        __set_PRIMASK(idle_mask);
#endif
    }
}

void *memcpy(void *to,const void *from,size_t n)
{
    uint8_t *d=to; const uint8_t *s=from;
    for (size_t i=0;i<n;++i) d[i]=s[i];
    return to;
}
void *memset(void *to,int value,size_t n)
{
    uint8_t *d=to;
    for (size_t i=0;i<n;++i) d[i]=(uint8_t)value;
    return to;
}
void *memmove(void *to,const void *from,size_t n)
{
    uint8_t *d=to; const uint8_t *s=from;
    if ((uintptr_t)d<(uintptr_t)s) for (size_t i=0;i<n;++i) d[i]=s[i];
    else for (size_t i=n;i>0;--i) d[i-1]=s[i-1];
    return to;
}
#endif
