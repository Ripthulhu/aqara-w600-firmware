#include "radio.h"
#include <string.h>
#ifdef W600_FLASH_APPLICATION
#include "settings_flash.h"
#define RADIO_RAM W600_NV_RAM
#else
#define RADIO_RAM
#endif
#ifdef W600_RADIO_TEST
#include "radio_test_mmio.h"
#else
#include "ciu32l071.h"
#define RADIO_RDR() ((uint8_t)UART3->RDR)
#define RADIO_TDR(byte) (UART3->TDR=(byte))
#define RADIO_ICR(bits) (UART3->ICR=(bits))
#define RADIO_WAKE(high) ((high) ? (void)(GPIOA->BSR=1u<<11) : (void)(GPIOA->BR=1u<<11))
#endif

#define HALF UINT32_C(0x80000000)
enum { PA11=1u<<11,
       RADIO_CR1=USART_CR1_UE|USART_CR1_RE|USART_CR1_TE|
                 USART_CR1_RXNEIE|USART_CR1_PEIE };
_Static_assert(sizeof(struct w600_radio)<=3072,"Radio state exceeds3KiB");
_Static_assert(UART3_4_IRQn==27,"Unexpected shared UART vector");

static void RADIO_RAM latch_error(struct w600_radio *r,enum w600_radio_error error)
{
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if(!r->error) r->error=error;
    __set_PRIMASK(mask);
}

static bool set_wake(bool high)
{
    if(!(RCC->IOPEN&RCC_IOPEN_GPIOAEN)) return false;
    RADIO_WAKE(high); __DSB();
    uint32_t expected=high ? PA11 : 0;
    return (GPIOA->MODE&(3u<<22))==(1u<<22) && !(GPIOA->OTYPE&PA11) &&
           (GPIOA->ODR&PA11)==expected && (GPIOA->IDR&PA11)==expected;
}

void w600_radio_stop(struct w600_radio *r)
{
    if(!r || !r->initialized) return;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    if(r->wire_position && r->tx_phase!=W600_RADIO_IDLE &&
       r->tx_phase!=W600_RADIO_STOPPED && !r->error)
        r->error=W600_RADIO_STOPPED_PARTIAL;
    r->tx_phase=W600_RADIO_STOPPED;
    if(!set_wake(true) && !r->error) r->error=W600_RADIO_CONTEXT;
    if(RCC->APB1EN&RCC_APB1EN_UART3EN) {
        UART3->CR1=0; UART3->CR3=0;
        __DSB();
        if((UART3->CR1 || UART3->CR3) && !r->error) r->error=W600_RADIO_CONTEXT;
    } else if(!r->error) r->error=W600_RADIO_CONTEXT;
    r->management_pending=false; r->reply_pending=false; r->temperature_pending=false;
    r->control_pending=false; r->target_pending=false;
    __set_PRIMASK(mask);
}

static bool fail(struct w600_radio *r,enum w600_radio_error error)
{
    latch_error(r,error);
    w600_radio_stop(r);
    return false;
}

static bool clock_ok(struct w600_radio *r,uint32_t now,bool count_poll)
{
    uint32_t delta=now-r->last_now;
    if(delta>=HALF) return fail(r,W600_RADIO_CLOCK);
    if(delta) r->stalled_polls=0;
    else if(count_poll && ++r->stalled_polls>=W600_RADIO_STALLED_POLLS)
        return fail(r,W600_RADIO_CLOCK);
    r->last_now=now;
    return true;
}

bool w600_radio_init(struct w600_radio *r,uint32_t now)
{
    if(!r) return false;
    *r=(struct w600_radio){0};
    r->last_now=now; r->next_sequence=1;
    if(!__get_PRIMASK() || __get_IPSR() || __get_CONTROL() ||
       !(RCC->IOPEN&RCC_IOPEN_GPIOAEN) || !(RCC->APB1EN&RCC_APB1EN_UART3EN) ||
       (GPIOA->MODE&(3u<<22))!=(1u<<22) || (GPIOA->OTYPE&PA11) ||
       !(GPIOA->ODR&PA11) || !(GPIOA->IDR&PA11) ||
       UART3->CR1!=0xdu || UART3->CR2 || UART3->CR3 || UART3->BRR!=417u) {
        r->error=W600_RADIO_CONTEXT; return false;
    }
    w600_endpoint_init(&r->endpoint); w600_pairing_init(&r->pairing);
    w600_decoder_reset(&r->decoder);
    r->initialized=true;
    if(UART3->ISR&15u) {
        r->uart_errors=UART3->ISR&15u;
        return fail(r,W600_RADIO_UART);
    }
    RADIO_ICR(15u);
    UART3->CR3=USART_CR3_EIE; UART3->CR1=RADIO_CR1;
    __DSB();
    if(UART3->CR1!=RADIO_CR1 || UART3->CR3!=USART_CR3_EIE)
        return fail(r,W600_RADIO_CONTEXT);
    return true;
}

void RADIO_RAM w600_radio_irq(struct w600_radio *r,uint32_t now)
{
    if(!r || !r->initialized || r->tx_phase==W600_RADIO_STOPPED) return;
    if(!(RCC->APB1EN&RCC_APB1EN_UART3EN)) {
        latch_error(r,W600_RADIO_CONTEXT);
        return;
    }
    uint32_t status=UART3->ISR;
    if(status&USART_ISR_RXNE) {
        uint8_t byte=RADIO_RDR();
        uint32_t head=r->rx_head;
        if((uint32_t)(head-r->rx_tail)>=W600_RADIO_RING_BYTES) {
            latch_error(r,W600_RADIO_RX_OVERFLOW);
        } else if(!r->error) {
            r->ring[head&(W600_RADIO_RING_BYTES-1u)]=byte;
            r->last_rx_at=now;
            __DMB(); r->rx_head=head+1u;
        }
    }
    if(status&15u) {
        r->uart_errors|=status&15u;
        RADIO_ICR(status&15u);
        latch_error(r,W600_RADIO_UART);
    }
}

/* Called only AFTER the unchanged endpoint has validated the whole batch. */
static void acknowledge_response52(struct w600_radio *r)
{
    if(r->request.kind!=2) return;
    size_t offset=0, result=0;
    while(offset<r->request.length) {
        const uint8_t *p=r->request.payload+offset;
        uint8_t type=p[2];
        size_t width;
        if(type<=2) width=1;
        else if(type==3 || type==4) width=2;
        else if(type==0x0b) width=(size_t)p[3]+1;
        else width=4;
        if(p[0]==0 && p[1]==0x52) r->reply.payload[result]=type==0 ? 0 : 3;
        offset+=3+width; result+=3;
    }
}

bool w600_radio_request_pairing(struct w600_radio *r,uint32_t now)
{
    if(!r || !r->initialized || r->error || r->tx_phase==W600_RADIO_STOPPED) return false;
    if(__get_IPSR() || !clock_ok(r,now,false)) return false;
#ifdef W600_OTA_RECEIVER
    if(r->ota && w600_ota_active(r->ota))return false;
#endif
    if(r->tx_phase!=W600_RADIO_IDLE || r->reply_pending || r->management_pending ||
       r->rx_head!=r->rx_tail || r->decoder.used) return false;
    r->pairing_events|=w600_pairing_update(&r->pairing,NULL,now);
    if(!w600_pairing_prepare(&r->pairing,r->next_sequence,now,&r->management)) return false;
    if(++r->next_sequence==0) r->next_sequence=1;
    r->pairing_events=0;
    r->management_pending=true; r->management_sent=false;
    return true;
}

bool w600_radio_report_temperature(struct w600_radio *r)
{
    if(!r || !r->initialized || r->error || r->tx_phase==W600_RADIO_STOPPED ||
       __get_IPSR() || !(r->endpoint.known&W600_KNOWN_LOCAL_TEMP)) return false;
    r->temperature_pending=true;
    return true;
}

bool w600_radio_report_control(struct w600_radio *r)
{
    if(!r || !r->initialized || r->error || r->tx_phase==W600_RADIO_STOPPED ||
       __get_IPSR()) return false;
    r->control_pending=true;
    return true;
}

bool w600_radio_report_target(struct w600_radio *r)
{
    if(!r || !r->initialized || r->error || r->tx_phase==W600_RADIO_STOPPED ||
       __get_IPSR() || !(r->endpoint.known&W600_KNOWN_TARGET)) return false;
    r->target_pending=true;
    return true;
}

static bool cancel_unsent(struct w600_radio *r)
{
    if(!set_wake(true)) return fail(r,W600_RADIO_CONTEXT);
    r->management_pending=false;
    r->tx_phase=W600_RADIO_IDLE; r->wire_length=0; r->wire_position=0;
    ++r->management_cancel_count;
    return true;
}

static bool start_tx(struct w600_radio *r,bool management,uint32_t now)
{
    if(r->error) return fail(r,(enum w600_radio_error)r->error);
    if(management && r->pairing.stage!=W600_PAIRING_PREPARED) {
        return cancel_unsent(r);
    }
    const struct w600_frame *frame=management ? &r->management : &r->reply;
    size_t written=0;
    if(!w600_encode(frame,r->wire,sizeof r->wire,&written)) return fail(r,W600_RADIO_ENCODE);
    r->wire_length=(uint16_t)written; r->wire_position=0; r->tx_management=management;
    r->tx_temperature=false; r->tx_control=false; r->tx_target=false;
#ifdef W600_OTA_RECEIVER
    r->tx_ota=!management && r->reply_ota;
#endif
    if(r->any_tx_completed && (uint32_t)(now-r->last_tx_at)<100u) {
        r->tx_phase=W600_RADIO_SPACING; r->tx_phase_at=now;
    } else {
        if(!set_wake(false)) return fail(r,W600_RADIO_CONTEXT);
        r->tx_phase=W600_RADIO_WAKE; r->tx_phase_at=now;
    }
    return true;
}

static bool service_tx(struct w600_radio *r,uint32_t now)
{
    if(r->error) return fail(r,(enum w600_radio_error)r->error);
    if(r->tx_phase==W600_RADIO_IDLE) return true;
    if(r->tx_temperature && !(r->endpoint.known&W600_KNOWN_LOCAL_TEMP) && !r->wire_position) {
        if(!set_wake(true)) return fail(r,W600_RADIO_CONTEXT);
        r->tx_phase=W600_RADIO_IDLE; r->wire_length=0; r->tx_temperature=false;
        return true;
    }
    if(r->tx_management && r->pairing.stage!=W600_PAIRING_PREPARED) {
        if(r->wire_position) return fail(r,W600_RADIO_EXPIRED_PARTIAL);
        return cancel_unsent(r);
    }
    if(r->tx_phase==W600_RADIO_SPACING) {
        if((uint32_t)(now-r->last_tx_at)<100u) return true;
        if(!set_wake(false)) return fail(r,W600_RADIO_CONTEXT);
        r->tx_phase=W600_RADIO_WAKE; r->tx_phase_at=now;
    }
    if(r->tx_phase==W600_RADIO_WAKE) {
        if((uint32_t)(now-r->tx_phase_at)<3u) return true;
        RADIO_ICR(USART_ICR_TCCF);
        r->tx_started_at=now; r->tx_phase=W600_RADIO_BYTES;
    }
    if((uint32_t)(now-r->tx_started_at)>=50u) return fail(r,W600_RADIO_TX_TIMEOUT);
    if(r->tx_phase==W600_RADIO_BYTES) {
        for(unsigned budget=0;budget<W600_RADIO_TX_BUDGET && r->wire_position<r->wire_length;
            ++budget) {
            uint32_t status=UART3->ISR;
            if(r->error) return fail(r,(enum w600_radio_error)r->error);
            if(!(status&USART_ISR_TXE)) return true;
            RADIO_TDR(r->wire[r->wire_position]);
            ++r->wire_position; ++r->tx_byte_count;
            if(r->error) return fail(r,(enum w600_radio_error)r->error);
        }
        if(r->wire_position<r->wire_length) return true;
        r->tx_phase=W600_RADIO_TC;
    }
    uint32_t status=UART3->ISR;
    if(r->error) return fail(r,(enum w600_radio_error)r->error);
    if(!(status&USART_ISR_TC)) return true;
    uint32_t mask=__get_PRIMASK(); __disable_irq();
    w600_radio_irq(r,now);
    if(r->error || !set_wake(true)) {
        __set_PRIMASK(mask);
        return fail(r,r->error ? (enum w600_radio_error)r->error : W600_RADIO_CONTEXT);
    }
    r->last_tx_at=now; r->any_tx_completed=true;
    if(r->tx_management) {
        /* Include any already-pending RDR byte in the pre-completion cutoff. */
        __DMB(); r->management_rx_cutoff=r->rx_head;
        ++r->tx_management_count; r->management_pending=false; r->management_sent=true;
    } else if(r->tx_temperature) { ++r->tx_temperature_count; r->tx_temperature=false; }
    else if(r->tx_control) { ++r->tx_control_count; r->tx_control=false; }
    else if(r->tx_target) { ++r->tx_target_count; r->tx_target=false; }
    else {
        ++r->tx_reply_count; r->reply_pending=false;
#ifdef W600_OTA_RECEIVER
        if(r->tx_ota && r->ota)w600_ota_sent(r->ota,now);
        r->tx_ota=false; r->reply_ota=false;
#endif
    }
    r->tx_phase=W600_RADIO_IDLE; r->wire_position=0;
    __set_PRIMASK(mask);
    return true;
}

bool w600_radio_poll(struct w600_radio *r,uint32_t now)
{
    if(!r || !r->initialized || r->tx_phase==W600_RADIO_STOPPED) return false;
    if(r->error) return fail(r,(enum w600_radio_error)r->error);
    if(__get_IPSR()) return fail(r,W600_RADIO_CONTEXT);
    if(!clock_ok(r,now,true)) return false;
    if(!(RCC->IOPEN&RCC_IOPEN_GPIOAEN) || !(RCC->APB1EN&RCC_APB1EN_UART3EN) ||
       UART3->CR1!=RADIO_CR1 || UART3->CR2 || UART3->CR3!=USART_CR3_EIE ||
       UART3->BRR!=417u || (GPIOA->MODE&(3u<<22))!=(1u<<22) ||
       (GPIOA->OTYPE&PA11)) return fail(r,W600_RADIO_CONTEXT);
    r->pairing_events|=w600_pairing_update(&r->pairing,NULL,now);
    if(r->pairing.stage==W600_PAIRING_CLOCK_ERROR) return fail(r,W600_RADIO_CLOCK);
    if(r->management_pending && r->pairing.stage!=W600_PAIRING_PREPARED &&
       r->tx_phase==W600_RADIO_IDLE && !cancel_unsent(r)) return false;
    if(!service_tx(r,now)) return false;
    if(r->tx_phase!=W600_RADIO_IDLE) return true;
#ifdef W600_OTA_RECEIVER
    if(r->ota) {
        w600_ota_poll(r->ota,now,&r->next_sequence);
        if(r->ota->pending) {
            r->reply=r->ota->tx; r->reply_pending=true; r->reply_ota=true;
            return start_tx(r,false,now);
        }
    }
#endif
    bool needs_input=false;
    for(unsigned budget=0;budget<W600_RADIO_POLL_BYTES && !r->reply_pending;++budget) {
        if(r->error) return fail(r,(enum w600_radio_error)r->error);
        uint32_t tail=r->rx_tail;
        bool decoded=w600_decoder_next(&r->decoder,&r->request);
        if(!decoded) {
            if(tail==r->rx_head) { needs_input=true; break; }
            __DMB(); uint8_t byte=r->ring[tail&(W600_RADIO_RING_BYTES-1u)];
            __DMB(); r->rx_tail=tail+1u;
            decoded=w600_decoder_feed(&r->decoder,byte,&r->request);
        }
        if(decoded) {
            if(r->error) return fail(r,(enum w600_radio_error)r->error);
            ++r->rx_frame_count;
            uint32_t first=r->rx_tail-r->decoder.used-((uint32_t)r->request.length+10u);
            if(r->management_sent && (uint32_t)(first-r->management_rx_cutoff)<HALF)
                r->pairing_events|=w600_pairing_update(&r->pairing,&r->request,now);
            uint32_t old_target=(r->endpoint.known&W600_KNOWN_TARGET) ?
                r->endpoint.target_raw : 0x10000u;
#ifdef W600_OTA_RECEIVER
            if(r->ota && w600_ota_duplicate_start(r->ota,&r->request)) {
                r->reply=(struct w600_frame){.sequence=r->request.sequence,.kind=3,.length=3,
                    .payload={0,0x40,0}};
                r->reply_pending=true;r->reply_ota=false;
                continue;
            }
            if(r->ota && w600_ota_process(r->ota,&r->request,now,&r->next_sequence)) {
                if(r->ota->pending) {
                    r->reply=r->ota->tx; r->reply_pending=true; r->reply_ota=true;
                }
                continue;
            }
            if(r->ota && w600_ota_active(r->ota) && r->request.kind==2) {
                /* Parse normally for bounded record/status encoding, but discard all writes. */
                struct w600_endpoint copy=r->endpoint;
                if(w600_endpoint_process(&copy,&r->request,&r->reply)) {
                    for(unsigned i=0;i<r->reply.length;i+=3)r->reply.payload[i]=1;
                    acknowledge_response52(r); r->reply_pending=true;
                } else ++r->rx_ignored_count;
                continue;
            }
#endif
            if(w600_endpoint_process(&r->endpoint,&r->request,&r->reply)) {
                if((r->endpoint.known&W600_KNOWN_TARGET) && r->endpoint.target_raw!=old_target) {
                    ++r->target_change_count; r->last_target_change_at=now;
                }
                acknowledge_response52(r); r->reply_pending=true;
            } else ++r->rx_ignored_count;
        }
    }
    uint32_t idle=now-r->last_rx_at;
    if(needs_input && r->rx_tail==r->rx_head && r->decoder.used && idle<HALF && idle>=100u) {
        w600_decoder_reset(&r->decoder); ++r->partial_reset_count;
    }
    if(r->reply_pending) return start_tx(r,false,now);
#ifdef W600_OTA_RECEIVER
    if(r->ota && w600_ota_active(r->ota))return true;
#endif
    if(r->management_pending) return start_tx(r,true,now);
    if(r->control_pending) {
        r->control_pending=false;
        r->reply.sequence=r->next_sequence; r->reply.kind=5; r->reply.length=0;
        /* False means failure to the radio; unreferenced at boot is not failure. */
        const struct w600_endpoint *e=&r->endpoint;
        bool drive_fault=e->control_error>=W600_CONTROL_NO_MOTION &&
                         e->control_error<=W600_CONTROL_DRIVER;
        if(!e->calibration_busy && !e->calibration_pending &&
           (e->calibrated || e->calibration_error || drive_fault)) {
            r->reply.payload[0]=7; r->reply.payload[1]=0x0b; r->reply.payload[2]=0;
            r->reply.payload[3]=(uint8_t)r->endpoint.calibrated;
            r->reply.length=4;
        }
        bool reference_unknown=!e->opening_known && !e->calibrated &&
            !e->calibration_busy && !e->calibration_pending &&
            !e->calibration_error && !drive_fault;
        if(r->endpoint.opening_known || reference_unknown) {
            unsigned at=r->reply.length;
            /* Custom extension: reserved value, never a physical opening.
             * The coordinator adapter clears cached position/calibration. */
            uint16_t opening=reference_unknown ? UINT16_MAX : r->endpoint.opening_permille;
            r->reply.payload[at]=7; r->reply.payload[at+1]=0x1c; r->reply.payload[at+2]=3;
            r->reply.payload[at+3]=(uint8_t)(opening>>8); r->reply.payload[at+4]=(uint8_t)opening;
            r->reply.length=(uint16_t)(at+5);
        }
        if(r->endpoint.mode_known) {
            unsigned at=r->reply.length;
            r->reply.payload[at]=7; r->reply.payload[at+1]=3; r->reply.payload[at+2]=0;
            r->reply.payload[at+3]=(uint8_t)(r->endpoint.mode!=0);
            r->reply.length=(uint16_t)(at+4);
        }
        if(r->endpoint.known&W600_KNOWN_EXTERNAL_USE) {
            unsigned at=r->reply.length;
            r->reply.payload[at]=7; r->reply.payload[at+1]=4; r->reply.payload[at+2]=1;
            r->reply.payload[at+3]=r->endpoint.external_use;
            r->reply.length=(uint16_t)(at+4);
        }
        if(r->endpoint.known&W600_KNOWN_CHILD_LOCK) {
            unsigned at=r->reply.length;
            r->reply.payload[at]=7; r->reply.payload[at+1]=0x18; r->reply.payload[at+2]=0;
            r->reply.payload[at+3]=(uint8_t)r->endpoint.child_locked;
            r->reply.length=(uint16_t)(at+4);
        }
        if(!r->reply.length) return true;
        if(++r->next_sequence==0) r->next_sequence=1;
        if(!start_tx(r,false,now)) return false;
        r->tx_control=true;
        return true;
    }
    if(r->target_pending) {
        r->target_pending=false;
        if(!(r->endpoint.known&W600_KNOWN_TARGET)) return true;
        uint16_t raw=r->endpoint.target_raw;
        r->reply.sequence=r->next_sequence; r->reply.kind=5; r->reply.length=5;
        r->reply.payload[0]=7; r->reply.payload[1]=0; r->reply.payload[2]=4;
        r->reply.payload[3]=(uint8_t)(raw>>8); r->reply.payload[4]=(uint8_t)raw;
        if(++r->next_sequence==0) r->next_sequence=1;
        if(!start_tx(r,false,now)) return false;
        r->tx_target=true;
        return true;
    }
    if(r->temperature_pending) {
        r->temperature_pending=false;
        if(!(r->endpoint.known&W600_KNOWN_LOCAL_TEMP)) return true;
        uint16_t raw=r->endpoint.local_temp_raw;
        r->reply.sequence=r->next_sequence; r->reply.kind=5; r->reply.length=5;
        r->reply.payload[0]=7; r->reply.payload[1]=6; r->reply.payload[2]=4;
        r->reply.payload[3]=(uint8_t)(raw>>8); r->reply.payload[4]=(uint8_t)raw;
        if(++r->next_sequence==0) r->next_sequence=1;
        if(!start_tx(r,false,now)) return false;
        r->tx_temperature=true;
    }
    return true;
}
