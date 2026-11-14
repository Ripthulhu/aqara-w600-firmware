#include "radio.h"
#include "radio_test_mmio.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

struct radio_test_rcc radio_test_rcc;
uint32_t radio_test_mask,radio_test_ipsr,radio_test_control;
static struct radio_test_uart uart;
static struct radio_test_gpio gpio;
static struct w600_radio r;
static uint8_t rdr,output[8192];
static size_t output_size,stall_after;
static unsigned auto_tc,wake_low,wake_high,rdr_count,cases;
static unsigned ignore_low,ignore_high,fail_tdr_at,fail_tc_read;
#ifdef W600_OTA_RECEIVER
static struct w600_ota ota;
static bool ota_ready;
static bool ota_stop(void *c) { (void)c; return ota_ready; }
static bool ota_write(void *c,uint32_t a,const uint32_t p[128])
{ (void)c;(void)a;(void)p;return false; }
static bool ota_read(void *c,uint32_t a,void *p,size_t n)
{ (void)c;(void)a;(void)p;(void)n;return false; }
static bool ota_auth(void *c,uint32_t a,uint32_t n,uint32_t v)
{ (void)c;(void)a;(void)n;(void)v;return false; }
static bool ota_commit(void *c,uint32_t n,uint16_t crc,uint32_t v)
{ (void)c;(void)n;(void)crc;(void)v;return false; }
static const struct w600_ota_ops ota_ops={ota_stop,ota_write,ota_read,ota_auth,ota_commit};
#endif

struct radio_test_uart *radio_test_uart(void)
{
    assert(radio_test_rcc.APB1EN&RCC_APB1EN_UART3EN);
    if(fail_tc_read && r.tx_phase==W600_RADIO_TC) {
        fail_tc_read=0; r.error=W600_RADIO_UART;
    }
    return &uart;
}
struct radio_test_gpio *radio_test_gpio(void)
{ assert(radio_test_rcc.IOPEN&RCC_IOPEN_GPIOAEN); return &gpio; }
uint8_t radio_test_rdr(void)
{ ++rdr_count; uart.ISR&=~(uint32_t)USART_ISR_RXNE; return rdr; }
void radio_test_tdr(uint8_t byte)
{
    assert(!(gpio.ODR&(1u<<11)) && (uart.ISR&USART_ISR_TXE));
    assert(output_size<sizeof output); output[output_size++]=byte;
    uart.ISR&=~(uint32_t)USART_ISR_TC;
    if(auto_tc) uart.ISR|=USART_ISR_TC;
    if(output_size>=stall_after) uart.ISR&=~(uint32_t)USART_ISR_TXE;
    if(fail_tdr_at && output_size==fail_tdr_at) {
        uart.ISR|=8; w600_radio_irq(&r,r.last_now);
    }
}
void radio_test_icr(uint32_t bits) { uart.ISR&=~bits; }
void radio_test_wake(bool high)
{
    assert(radio_test_rcc.IOPEN&RCC_IOPEN_GPIOAEN);
    if((high && ignore_high) || (!high && ignore_low)) return;
    if(high) { gpio.ODR|=1u<<11; ++wake_high; }
    else { gpio.ODR&=~(1u<<11); ++wake_low; }
    gpio.IDR=gpio.ODR;
}

static void setup(uint32_t now)
{
    memset(&r,0,sizeof r); memset(&uart,0,sizeof uart); memset(&gpio,0,sizeof gpio);
    radio_test_rcc=(struct radio_test_rcc){1,0x40000};
    gpio.MODE=1u<<22; gpio.ODR=gpio.IDR=1u<<11;
    uart.CR1=13; uart.BRR=417; uart.ISR=0x6000c0;
    radio_test_mask=1; radio_test_ipsr=radio_test_control=0;
    output_size=0; stall_after=sizeof output; auto_tc=1;
    wake_low=wake_high=rdr_count=0;
    ignore_low=ignore_high=fail_tdr_at=fail_tc_read=0;
    assert(w600_radio_init(&r,now));
    assert(r.initialized && !r.error && uart.CR1==0x12d && uart.CR3==1);
    assert(radio_test_mask==1 && !output_size && !wake_low);
    radio_test_mask=0;
}
static void byte_irq(uint8_t byte,uint32_t now)
{
    assert(!(uart.ISR&USART_ISR_RXNE));
    rdr=byte; uart.ISR|=USART_ISR_RXNE;
    radio_test_ipsr=43; w600_radio_irq(&r,now); radio_test_ipsr=0;
}
static size_t wire(const struct w600_frame *f,uint8_t *bytes)
{ size_t n=0; assert(w600_encode(f,bytes,W600_MAX_FRAME,&n)); return n; }
static void inject(const struct w600_frame *f,uint32_t now)
{
    uint8_t bytes[W600_MAX_FRAME]; size_t n=wire(f,bytes);
    for(size_t i=0;i<n;++i) byte_irq(bytes[i],now);
}
static void poll(uint32_t now) { assert(w600_radio_poll(&r,now)); }
static uint32_t idle(uint32_t now)
{
    for(unsigned i=0;i<500;++i,++now) {
        poll(now);
        if(r.tx_phase==W600_RADIO_IDLE && !r.reply_pending && !r.management_pending && !r.temperature_pending && !r.control_pending &&
           r.rx_tail==r.rx_head) return now;
    }
    assert(!"No idle completion"); return 0;
}
static struct w600_frame output_frame(size_t offset)
{
    struct w600_decoder d={0}; struct w600_frame f={0}; unsigned found=0;
    for(size_t i=offset;i<output_size;++i) {
        if(w600_decoder_feed(&d,output[i],&f)) { ++found; break; }
    }
    assert(found==1); return f;
}
static void management_done(uint32_t now)
{
    assert(w600_radio_request_pairing(&r,now));
    poll(now); assert(r.tx_phase==W600_RADIO_WAKE && !output_size);
    poll(now+2); assert(!output_size);
    poll(now+3); assert(r.management_sent && r.tx_management_count==1);
    assert(!r.tx_reply_count && r.tx_phase==W600_RADIO_IDLE);
}
static struct w600_frame ack(uint8_t sequence)
{ return (struct w600_frame){.sequence=sequence,.kind=3,.length=3,.payload={0,0,0x51}}; }

int main(void)
{
    struct w600_frame target={.sequence=41,.kind=2,.length=5,.payload={7,0,4,8,0x34}};
    setup(0);inject(&target,10);poll(10);
    assert(r.target_change_count==1 && r.last_target_change_at==10 && r.endpoint.target_raw==2100);
    uint32_t target_done=idle(11);
    inject(&target,target_done+1);idle(target_done+1);
    assert(r.target_change_count==1 && r.last_target_change_at==10);++cases;
    setup(0);assert(w600_endpoint_set_target(&r.endpoint,2100));
    inject(&target,10);idle(10);
    assert(!r.target_change_count && !r.last_target_change_at);++cases;
    setup(0);target.payload[4]=0x66;inject(&target,123);idle(123);
    assert(r.target_change_count==1 && r.last_target_change_at==123 && r.endpoint.target_raw==2150);++cases;
    setup(0);r.endpoint.control_enabled=true;
    target.payload[3]=0xff;target.payload[4]=0xff;inject(&target,10);idle(10);
    assert(!r.target_change_count && !r.last_target_change_at && !(r.endpoint.known&W600_KNOWN_TARGET));++cases;
    setup(0xfffffff0u);target.payload[3]=8;target.payload[4]=0x34;inject(&target,2);idle(2);
    assert(r.target_change_count==1 && r.last_target_change_at==2);++cases;
    setup(0);const struct w600_frame target_read={.sequence=41,.kind=1,.length=2,.payload={7,0}};
    inject(&target_read,10);idle(10);
    assert(!r.target_change_count && !r.last_target_change_at);++cases;
    setup(0); poll(1); poll(100); assert(!output_size && r.pairing.stage==W600_PAIRING_IDLE); ++cases;
    setup(0); r.next_sequence=0x21; management_done(0);
    const uint8_t golden[]={0x55,5,2,0x21,5,0,4,0x7a,0,0x51,0,1,0xf9,4};
    struct w600_frame first=output_frame(0);
    assert(first.kind==5 && first.sequence==0x21 && first.length==4 &&
           !memcmp(first.payload,"\0\x51\0\1",4));
    uint8_t canonical[W600_MAX_FRAME]; size_t canonical_n=wire(&first,canonical);
    assert(canonical_n==14 && !memcmp(output,canonical,14));
    assert(!memcmp(output,golden,sizeof golden));
    poll(9000); assert(output_size==14 && !w600_radio_request_pairing(&r,9001));
    poll(10000); assert(r.pairing.stage==W600_PAIRING_WINDOW_ENDED);
    assert(w600_radio_request_pairing(&r,10001)); poll(10001); poll(10004);
    assert(r.tx_management_count==2 && r.management.sequence==0x22); ++cases;

    setup(0); assert(w600_radio_request_pairing(&r,0)); poll(0); poll(10000);
    assert(!output_size && r.management_cancel_count==1 && (gpio.ODR&(1u<<11)));
    assert(r.tx_phase==W600_RADIO_IDLE && !r.management_pending); ++cases;
    setup(0); const struct w600_frame read={.sequence=33,.kind=1,.length=2,.payload={7,0xb}};
    inject(&read,0); assert(!r.rx_frame_count && !output_size);
    uint32_t done=idle(0); size_t reply_bytes=output_size;
    assert(r.tx_reply_count==1 && !r.tx_management_count);
    assert(w600_radio_request_pairing(&r,done+1)); poll(done+1);
    assert(r.tx_phase==W600_RADIO_SPACING); poll(done+10001);
    assert(output_size==reply_bytes && r.management_cancel_count==1 && !r.error); ++cases;
    setup(0); stall_after=1; assert(w600_radio_request_pairing(&r,0)); poll(0); poll(3);
    assert(output_size==1 && !r.management_sent);
    assert(!w600_radio_poll(&r,10000) && r.error==W600_RADIO_EXPIRED_PARTIAL);
    assert(!uart.CR1 && !uart.CR3 && (gpio.ODR&(1u<<11))); ++cases;
    setup(0); auto_tc=0; assert(w600_radio_request_pairing(&r,0)); poll(0); poll(3);
    assert(output_size==14 && r.tx_phase==W600_RADIO_TC && !r.management_sent);
    assert(!w600_radio_poll(&r,53) && r.error==W600_RADIO_TX_TIMEOUT); ++cases;
    setup(0); assert(w600_radio_request_pairing(&r,0)); poll(0);
    for(unsigned i=1;i<W600_RADIO_STALLED_POLLS;++i) {
        bool ok=w600_radio_poll(&r,0);
        assert(ok==(i+1<W600_RADIO_STALLED_POLLS));
    }
    assert(r.error==W600_RADIO_CLOCK && !output_size); ++cases;
    setup(10); assert(!w600_radio_poll(&r,9) && r.error==W600_RADIO_CLOCK); ++cases;
    setup(0xfffffffeu); management_done(0xfffffffeu);
    poll(2); assert(!r.error && r.tx_management_count==1); ++cases;

    setup(0); auto_tc=0; assert(w600_radio_request_pairing(&r,0)); poll(0); poll(3);
    struct w600_frame a=ack(1); inject(&a,4); uart.ISR|=USART_ISR_TC; poll(4);
    assert(r.management_sent && !r.pairing.ack_seen && r.rx_ignored_count==1);
    inject(&a,5); poll(5); assert(r.pairing.ack_seen && r.pairing.ack_at==5);
    assert(output_size==14 && r.tx_reply_count==0); ++cases;
    setup(0); assert(w600_radio_request_pairing(&r,0)); inject(&a,0); poll(0);
    poll(3); assert(!r.pairing.ack_seen && r.management_sent); ++cases;
    setup(0); assert(w600_radio_request_pairing(&r,0)); poll(0); inject(&a,1);
    poll(3); assert(!r.pairing.ack_seen && r.management_sent); ++cases;
    setup(0); auto_tc=0; assert(w600_radio_request_pairing(&r,0)); poll(0); poll(3);
    uint8_t a_bytes[W600_MAX_FRAME]; size_t a_n=wire(&a,a_bytes);
    for(size_t i=0;i<5;++i) byte_irq(a_bytes[i],4);
    uart.ISR|=USART_ISR_TC; poll(4);
    for(size_t i=5;i<a_n;++i) byte_irq(a_bytes[i],5);
    poll(5); assert(!r.pairing.ack_seen && r.rx_frame_count==1); ++cases;
    setup(0); r.rx_head=r.rx_tail=0xfffffff8u; management_done(0);
    inject(&a,4); poll(4); assert(r.pairing.ack_seen && r.rx_head<32); ++cases;
    setup(0); management_done(0);
    struct w600_frame k7={.sequence=71,.kind=7,.length=5,.payload={0,0x52,8,0,255}};
    inject(&k7,4); poll(4);
    assert(r.pairing.response_seen && r.pairing.response_kind==7 &&
           r.pairing.response_status==8 && r.pairing.response_value==255);
    assert(output_size==14 && !r.tx_reply_count); ++cases;
    setup(0); management_done(0);
    struct w600_frame response={.sequence=55,.kind=2,.length=4,.payload={0,0x52,0,0}};
    inject(&response,4); done=idle(4);
    assert(r.pairing.response_seen && r.pairing.response_kind==2 && r.tx_reply_count==1);
    struct w600_frame reply=output_frame(14);
    assert(reply.kind==3 && reply.sequence==55 && reply.length==3 &&
           !memcmp(reply.payload,"\0\0\x52",3)); ++cases;
    setup(0); inject(&response,0); idle(0);
    reply=output_frame(0); assert(reply.payload[0]==0 && !r.pairing.response_seen); ++cases;
    setup(0); management_done(0); poll(10000);
    inject(&response,10001); idle(10001);
    reply=output_frame(14); assert(reply.payload[0]==0 && !r.pairing.response_seen); ++cases;
    setup(0);
    struct w600_frame mixed={.sequence=66,.kind=2,.length=9,
                            .payload={7,5,4,8,0x32,0,0x52,0,17}};
    inject(&mixed,0); idle(0); reply=output_frame(0);
    assert(r.endpoint.external_temp_raw==2098 && r.endpoint.external_use==1);
    assert(reply.length==6 && !memcmp(reply.payload,"\0\7\5\0\0\x52",6)); ++cases;
    setup(0); response.payload[2]=1; inject(&response,0); idle(0);
    reply=output_frame(0); assert(reply.payload[0]==3); response.payload[2]=0; ++cases;
    setup(0); mixed.length=10; mixed.payload[9]=7;
    inject(&mixed,0); idle(0);
    assert(!r.endpoint.known && !r.tx_reply_count && !output_size && r.rx_ignored_count==1); ++cases;

    setup(0); for(unsigned i=0;i<512;++i) byte_irq(0,0);
    poll(0); assert(r.rx_tail==W600_RADIO_POLL_BYTES && !r.rx_frame_count); ++cases;
    setup(0); for(unsigned i=0;i<513;++i) byte_irq(0,0);
    assert(r.error==W600_RADIO_RX_OVERFLOW && rdr_count==513);
    assert(!w600_radio_poll(&r,1) && !uart.CR1); ++cases;
    setup(0); uart.ISR|=8; byte_irq(0xaa,0);
    assert(r.uart_errors==8 && r.error==W600_RADIO_UART && !(uart.ISR&15));
    assert(!w600_radio_poll(&r,1) && r.error==W600_RADIO_UART); ++cases;
    setup(0); byte_irq(0x55,0); poll(0); assert(r.decoder.used==1);
    assert(!w600_radio_request_pairing(&r,1)); poll(100);
    assert(!r.decoder.used && r.partial_reset_count==1); ++cases;
    setup(0); byte_irq(0x55,1); poll(0); assert(r.decoder.used==1 && !r.partial_reset_count); ++cases;
    setup(0); inject(&read,0); assert(!w600_radio_request_pairing(&r,0));
    poll(0); assert(!w600_radio_request_pairing(&r,1)); ++cases;

    /* Bad outer CRC leaves two complete, valid inner frames in the decoder. */
    setup(0); uint8_t inner[W600_MAX_FRAME]; size_t inner_n=wire(&read,inner);
    struct w600_frame outer={.sequence=9,.kind=99,.length=(uint16_t)(2*inner_n)};
    memcpy(outer.payload,inner,inner_n); memcpy(outer.payload+inner_n,inner,inner_n);
    uint8_t outer_wire[W600_MAX_FRAME]; size_t outer_n=wire(&outer,outer_wire);
    outer_wire[outer_n-1]^=1;
    for(size_t i=0;i<outer_n;++i) byte_irq(outer_wire[i],0);
    for(uint32_t t=0;t<250;++t) poll(t);
    assert(r.rx_frame_count==2 && r.tx_reply_count==2 && !r.error); ++cases;

    /* Cutoff applies to inner frame bytes, not to an invalid enclosing header. */
    setup(0); auto_tc=0; assert(w600_radio_request_pairing(&r,0)); poll(0); poll(3);
    inner_n=wire(&a,inner); outer.length=(uint16_t)(2*inner_n);
    memcpy(outer.payload,inner,inner_n); memcpy(outer.payload+inner_n,inner,inner_n);
    outer_n=wire(&outer,outer_wire); outer_wire[outer_n-1]^=1;
    for(size_t i=0;i<8;++i) byte_irq(outer_wire[i],4);
    uart.ISR|=USART_ISR_TC; poll(4);
    for(size_t i=8;i<outer_n;++i) byte_irq(outer_wire[i],5);
    poll(5); assert(r.rx_frame_count==2 && r.pairing.ack_seen && r.pairing.ack_at==5); ++cases;
    setup(0); r.next_sequence=255; management_done(0);
    poll(10000); assert(w600_radio_request_pairing(&r,10001));
    assert(r.management.sequence==1 && r.next_sequence==2); ++cases;

    /* Duplicate0052 is ACKed record-by-record but not promoted to one observation. */
    setup(0); management_done(0);
    struct w600_frame duplicate={.sequence=9,.kind=2,.length=8,
                                .payload={0,0x52,0,0,0,0x52,0,1}};
    inject(&duplicate,4); idle(4); reply=output_frame(14);
    assert(reply.length==6 && reply.payload[0]==0 && reply.payload[3]==0 &&
           !r.pairing.response_seen); ++cases;

    setup(0); inject(&read,0); poll(0); poll(3);
    size_t first_n=output_size; inject(&read,4); poll(4); idle(5);
    assert(r.tx_reply_count==2 && output_size==2*first_n); ++cases;
    setup(0); w600_radio_stop(&r);
    assert(r.tx_phase==W600_RADIO_STOPPED && !r.error && !uart.CR1 && !uart.CR3);
    assert(!w600_radio_poll(&r,0) && !w600_radio_request_pairing(&r,0)); ++cases;
    setup(0); stall_after=1; assert(w600_radio_request_pairing(&r,0)); poll(0); poll(3);
    w600_radio_stop(&r); assert(r.error==W600_RADIO_STOPPED_PARTIAL); ++cases;
    setup(0); radio_test_rcc.APB1EN=0;
    w600_radio_irq(&r,0); assert(r.error==W600_RADIO_CONTEXT);
    assert(!w600_radio_poll(&r,0)); ++cases;
    setup(0); radio_test_rcc.IOPEN=0;
    assert(!w600_radio_poll(&r,0) && r.error==W600_RADIO_CONTEXT); ++cases;
    setup(0); uart.CR3=0; assert(!w600_radio_poll(&r,0)); ++cases;
    setup(0); radio_test_ipsr=43;
    assert(!w600_radio_poll(&r,0) && r.error==W600_RADIO_CONTEXT); radio_test_ipsr=0; ++cases;
    setup(0); r.error=W600_RADIO_UART;
    assert(!w600_radio_poll(&r,0) && r.error==W600_RADIO_UART && !radio_test_mask); ++cases;

    setup(0); w600_radio_stop(&r); uart.CR1=13; uart.CR3=0; radio_test_mask=0;
    assert(!w600_radio_init(&r,0) && r.error==W600_RADIO_CONTEXT); ++cases;
    setup(0); w600_radio_stop(&r); uart.CR1=13; uart.CR3=0; uart.ISR|=8; radio_test_mask=1;
    assert(!w600_radio_init(&r,0) && r.error==W600_RADIO_UART); ++cases;
    setup(0); ignore_low=1; assert(w600_radio_request_pairing(&r,0));
    assert(!w600_radio_poll(&r,0) && r.error==W600_RADIO_CONTEXT && !output_size); ++cases;
    setup(0); assert(w600_radio_request_pairing(&r,0)); poll(0); ignore_high=1;
    assert(!w600_radio_poll(&r,3) && r.error==W600_RADIO_CONTEXT &&
           !r.management_sent && !r.tx_management_count); ++cases;
    setup(0); assert(w600_radio_request_pairing(&r,0)); poll(0); fail_tdr_at=1;
    assert(!w600_radio_poll(&r,3) && r.error==W600_RADIO_UART && output_size==1 &&
           !r.tx_management_count && !uart.CR1 && (gpio.ODR&(1u<<11))); ++cases;
    setup(0); inject(&read,0); poll(0); fail_tdr_at=1;
    assert(!w600_radio_poll(&r,3) && r.error==W600_RADIO_UART && output_size==1 &&
           !r.tx_reply_count); ++cases;
    setup(0); auto_tc=0; assert(w600_radio_request_pairing(&r,0)); poll(0); poll(3);
    fail_tc_read=1;
    assert(!w600_radio_poll(&r,4) && r.error==W600_RADIO_UART &&
           !r.management_sent && !r.tx_management_count && !uart.CR1); ++cases;
    setup(0); management_done(0); inject(&a,4); poll(4);
    assert(r.pairing_events&W600_PAIRING_ACK_OBSERVED);
    poll(10000); assert(w600_radio_request_pairing(&r,10001));
    assert(!r.pairing_events && !r.pairing.ack_seen && !r.management_sent); ++cases;
    assert(!w600_radio_init(NULL,0) && !w600_radio_poll(NULL,0) &&
           !w600_radio_request_pairing(NULL,0));
    w600_radio_irq(NULL,0); w600_radio_stop(NULL); ++cases;

    setup(0); assert(!w600_radio_report_temperature(NULL) && !w600_radio_report_temperature(&r)); ++cases;
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,214,true));
    assert(w600_radio_report_temperature(&r) && !output_size);
    poll(0); assert(r.tx_temperature && !r.temperature_pending && !output_size);
    poll(2); assert(!output_size); idle(3); first=output_frame(0);
    assert(first.kind==5 && first.sequence==1 && first.length==5 &&
           !memcmp(first.payload,"\7\6\4\10\x5c",5));
    assert(r.tx_temperature_count==1 && !r.tx_reply_count && !r.tx_management_count);
    poll(1000); assert(output_size==15 && !r.tx_temperature); ++cases;

    /* Unsent samples coalesce; a reading is not allowed to change source/target. */
    setup(0); r.endpoint.target_raw=2000; r.endpoint.external_temp_raw=1900;
    r.endpoint.external_use=1; r.endpoint.known=7;
    for(int16_t v=200;v<215;++v) {
        assert(w600_endpoint_local_temperature(&r.endpoint,v,true));
        assert(w600_radio_report_temperature(&r));
    }
    idle(0); first=output_frame(0);
    assert(first.payload[3]==8 && first.payload[4]==0x5c && r.tx_temperature_count==1 &&
           r.endpoint.target_raw==2000 && r.endpoint.external_temp_raw==1900 && r.endpoint.external_use==1); ++cases;

    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,-50,true));
    assert(w600_radio_report_temperature(&r)); idle(0); first=output_frame(0);
    assert(first.payload[3]==0xfe && first.payload[4]==0x0c); ++cases;
    setup(0); r.next_sequence=255; assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); done=idle(0); first=output_frame(0);
    assert(first.sequence==255 && r.next_sequence==1);
    assert(w600_radio_report_temperature(&r)); idle(done+1); first=output_frame(15);
    assert(first.sequence==1 && r.next_sequence==2 && r.tx_temperature_count==2); ++cases;

    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); inject(&read,0); idle(0);
    first=output_frame(0); reply=output_frame(14);
    assert(first.kind==4 && first.sequence==read.sequence && reply.kind==5 &&
           r.tx_reply_count==1 && r.tx_temperature_count==1); ++cases;
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); assert(w600_radio_request_pairing(&r,0));
    idle(0); first=output_frame(0); reply=output_frame(14);
    assert(first.kind==5 && first.length==4 && first.payload[1]==0x51 &&
           reply.length==5 && reply.payload[1]==6 && r.tx_management_count==1 && r.tx_temperature_count==1); ++cases;

    /* Clearing availability cancels only a frame that has not sent a byte. */
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r));
    assert(w600_endpoint_local_temperature(&r.endpoint,0,false)); idle(0);
    assert(!output_size && !r.temperature_pending && !r.tx_temperature_count); ++cases;
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); poll(0);
    assert(w600_endpoint_local_temperature(&r.endpoint,0,false)); poll(1);
    assert(!output_size && r.tx_phase==W600_RADIO_IDLE && !r.tx_temperature && (gpio.ODR&(1u<<11))); ++cases;
    setup(0); stall_after=1; assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); poll(0); poll(3); assert(output_size==1);
    assert(w600_endpoint_local_temperature(&r.endpoint,0,false));
    stall_after=sizeof output; uart.ISR|=USART_ISR_TXE; idle(4);
    assert(output_size==15 && r.tx_temperature_count==1 && !r.error); ++cases;

    /* A new sample during an in-flight report survives that report's completion. */
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); poll(0);
    assert(w600_endpoint_local_temperature(&r.endpoint,210,true));
    assert(w600_radio_report_temperature(&r)); idle(3);
    first=output_frame(0); reply=output_frame(15);
    assert(first.payload[4]==0xd0 && reply.payload[4]==0x34 &&
           r.tx_temperature_count==2 && !r.tx_reply_count && !r.tx_temperature); ++cases;
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); poll(0); fail_tdr_at=1;
    assert(!w600_radio_poll(&r,3) && r.error==W600_RADIO_UART &&
           !r.temperature_pending && !r.tx_temperature_count); ++cases;
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    radio_test_ipsr=43; assert(!w600_radio_report_temperature(&r)); radio_test_ipsr=0;
    w600_radio_stop(&r); assert(!w600_radio_report_temperature(&r)); ++cases;
    setup(0); auto_tc=0; assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); poll(0); poll(3);
    assert(output_size==15 && !r.tx_temperature_count && r.tx_phase==W600_RADIO_TC);
    assert(!w600_radio_poll(&r,53) && r.error==W600_RADIO_TX_TIMEOUT && !r.tx_temperature_count); ++cases;
    setup(0); assert(w600_endpoint_local_temperature(&r.endpoint,200,true));
    assert(w600_radio_report_temperature(&r)); done=idle(0);
    assert(w600_radio_report_temperature(&r)); poll(done+1); assert(r.tx_phase==W600_RADIO_SPACING);
    assert(w600_endpoint_local_temperature(&r.endpoint,0,false)); poll(done+2);
    assert(output_size==15 && r.tx_phase==W600_RADIO_IDLE && r.tx_temperature_count==1); ++cases;
    setup(0); assert(!w600_radio_report_control(NULL));
    r.endpoint.control_enabled=true;
    assert(w600_radio_report_control(&r)); idle(0);first=output_frame(0);
    assert(first.kind==5 && first.length==5 &&
        !memcmp(first.payload,(uint8_t[]){7,0x1c,3,0xff,0xff},5));
    assert(r.tx_control_count==1 && !r.tx_reply_count); ++cases;
    for(unsigned error=W600_CONTROL_NEEDS_CALIBRATION;error<=W600_CONTROL_TEMPERATURE;++error) {
        setup(0);r.endpoint.control_error=(uint8_t)error;r.endpoint.mode_known=true;
        assert(w600_radio_report_control(&r));idle(0);first=output_frame(0);
        assert(first.kind==5 && first.length==9 &&
            !memcmp(first.payload,(uint8_t[]){7,0x1c,3,0xff,0xff,7,3,0,0},9));
        ++cases;
    }
    for(unsigned mode=0;mode<=2;++mode) {
        setup(0);r.endpoint.mode=(uint8_t)mode;r.endpoint.mode_known=true;
        assert(w600_radio_report_control(&r));idle(0);first=output_frame(0);
        assert(first.kind==5 && first.length==9 &&
               !memcmp(first.payload,(uint8_t[]){7,0x1c,3,0xff,0xff,7,3,0,(uint8_t)(mode!=0)},9));
        assert(!r.endpoint.opening_known && !r.endpoint.known);++cases;
    }
    for(unsigned source=0;source<=1;++source) {
        setup(0);r.endpoint.external_use=(uint8_t)source;
        r.endpoint.known=W600_KNOWN_EXTERNAL_USE;
        assert(w600_radio_report_control(&r));idle(0);first=output_frame(0);
        assert(first.kind==5 && first.length==9 &&
               !memcmp(first.payload,(uint8_t[]){7,0x1c,3,0xff,0xff,7,4,1,(uint8_t)source},9));
        ++cases;
    }
    setup(0);r.endpoint.external_use=1;
    assert(w600_radio_report_control(&r));idle(0);first=output_frame(0);
    assert(first.kind==5 && first.length==5 &&
        !memcmp(first.payload,(uint8_t[]){7,0x1c,3,0xff,0xff},5));++cases;
    /* Faults and pending/busy calibration keep their own reports, not the idle sentinel. */
    for(unsigned error=W600_CONTROL_NO_MOTION;error<=W600_CONTROL_DRIVER;++error) {
        setup(0);r.endpoint.control_error=(uint8_t)error;
        assert(w600_radio_report_control(&r));idle(0);first=output_frame(0);
        assert(first.kind==5 && first.length==4 &&
            !memcmp(first.payload,(uint8_t[]){7,0x0b,0,0},4));++cases;
    }
    setup(0);r.endpoint.calibration_pending=true;
    assert(w600_radio_report_control(&r));idle(0);
    assert(!output_size);++cases;
    setup(0);r.endpoint.calibration_busy=true;
    r.endpoint.mode_known=true;r.endpoint.known=W600_KNOWN_EXTERNAL_USE;r.endpoint.external_use=1;
    assert(w600_radio_report_control(&r));idle(0);first=output_frame(0);
    assert(first.kind==5 && first.length==8 &&
        !memcmp(first.payload,(uint8_t[]){7,3,0,0,7,4,1,1},8));++cases;
    for(unsigned flags=0;flags<8;++flags) {
        setup(0);r.endpoint.calibration_busy=true;
        r.endpoint.opening_known=(flags&1)!=0;r.endpoint.opening_permille=175;
        r.endpoint.mode_known=(flags&2)!=0;r.endpoint.mode=1;
        r.endpoint.known=(flags&4) ? W600_KNOWN_EXTERNAL_USE : 0;r.endpoint.external_use=1;
        uint8_t expected[13];unsigned length=0;
        if(flags&1) { memcpy(expected+length,(uint8_t[]){7,0x1c,3,0,175},5);length+=5; }
        if(flags&2) { memcpy(expected+length,(uint8_t[]){7,3,0,1},4);length+=4; }
        if(flags&4) { memcpy(expected+length,(uint8_t[]){7,4,1,1},4);length+=4; }
        assert(w600_radio_report_control(&r));idle(0);
        if(length) {
            first=output_frame(0);
            assert(first.kind==5 && first.length==length && !memcmp(first.payload,expected,length));
            assert(r.tx_control_count==1);
        } else assert(!output_size && !r.tx_control_count);
        ++cases;
    }
    setup(0);r.endpoint.mode_known=true;
    r.endpoint.known=W600_KNOWN_EXTERNAL_USE;r.endpoint.external_use=1;
    r.endpoint.calibration_busy=true;
    assert(w600_radio_report_control(&r));idle(0);
    r.endpoint.calibration_busy=false;r.endpoint.calibrated=true;
    assert(w600_radio_report_control(&r));idle(10);reply=output_frame(18);
    assert(reply.kind==5 && reply.length==12 &&
        !memcmp(reply.payload,(uint8_t[]){7,0x0b,0,1,7,3,0,0,7,4,1,1},12));++cases;
    setup(0);r.endpoint.calibration_busy=true;
    assert(w600_radio_report_control(&r));idle(0);
    assert(!output_size && !r.tx_control_count && !r.control_pending);++cases;
    r.endpoint.calibration_busy=false;
    r.endpoint.calibration_pending=true;r.endpoint.calibration_error=1;
    assert(w600_radio_report_control(&r));idle(10);
    assert(!output_size && !r.tx_control_count && !r.control_pending);++cases;
    r.endpoint.calibration_pending=false;
    assert(w600_radio_report_control(&r));idle(20);first=output_frame(0);
    assert(first.kind==5 && first.length==4 &&
        !memcmp(first.payload,(uint8_t[]){7,0x0b,0,0},4));++cases;
    setup(0); r.endpoint.control_enabled=true; r.endpoint.calibrated=true;
    r.endpoint.opening_known=true; r.endpoint.opening_permille=1000;
    assert(w600_radio_report_control(&r)); poll(0);
    r.endpoint.calibrated=false; r.endpoint.opening_known=false;
    r.endpoint.control_error=W600_CONTROL_NO_MOTION;
    assert(w600_radio_report_control(&r)); idle(3);
    first=output_frame(0); reply=output_frame(19);
    assert(first.kind==5 && first.length==9 && !memcmp(first.payload,(uint8_t[]){7,0x0b,0,1,7,0x1c,3,3,0xe8},9));
    assert(reply.kind==5 && reply.length==4 && !reply.payload[3]);
    assert(r.tx_control_count==2 && !r.tx_reply_count && !r.tx_control); ++cases;
    setup(0); r.endpoint.control_enabled=true;r.endpoint.mode_known=true;assert(w600_radio_report_control(&r));
    inject(&read,0); idle(0); first=output_frame(0);
    assert(first.kind==4 && output_frame(first.length+10).kind==5); ++cases;
    setup(0); r.endpoint.control_enabled=true;r.endpoint.mode_known=true;assert(w600_radio_report_control(&r));
    auto_tc=0; poll(0); poll(3);
    assert(!w600_radio_poll(&r,53) && r.error==W600_RADIO_TX_TIMEOUT && !r.tx_control_count && !r.control_pending); ++cases;
    setup(0); assert(!w600_radio_report_target(NULL) && !w600_radio_report_target(&r)); ++cases;
    setup(0); r.endpoint.control_enabled=true; assert(w600_endpoint_set_target(&r.endpoint,2050));
    assert(w600_radio_report_target(&r)); idle(0); first=output_frame(0);
    assert(first.kind==5 && first.length==5 && !memcmp(first.payload,(uint8_t[]){7,0,4,8,2},5));
    assert(r.tx_target_count==1 && !r.tx_reply_count && !r.endpoint.mode); ++cases;
    setup(0); r.endpoint.control_enabled=true;
    for(unsigned v=2000;v<=2200;v+=50) {
        assert(w600_endpoint_set_target(&r.endpoint,(uint16_t)v)); assert(w600_radio_report_target(&r));
    }
    idle(0); first=output_frame(0);
    assert(first.payload[3]==8 && first.payload[4]==0x98 && r.tx_target_count==1); ++cases;
    setup(0); r.endpoint.control_enabled=true; assert(w600_endpoint_set_target(&r.endpoint,2000));
    assert(w600_radio_report_target(&r)); poll(0);
    assert(w600_endpoint_set_target(&r.endpoint,2100)); assert(w600_radio_report_target(&r)); idle(3);
    first=output_frame(0); reply=output_frame(15);
    assert(first.payload[4]==0xd0 && reply.payload[4]==0x34 && r.tx_target_count==2 && !r.tx_reply_count); ++cases;
    setup(0); assert(w600_endpoint_set_target(&r.endpoint,2000));
    assert(w600_radio_report_target(&r)); inject(&read,0); idle(0);
    first=output_frame(0); reply=output_frame(first.length+10);
    assert(first.kind==4 && reply.kind==5 && reply.payload[1]==0 && r.tx_reply_count==1 && r.tx_target_count==1); ++cases;
    setup(0); assert(w600_endpoint_set_target(&r.endpoint,2000));
    assert(w600_radio_report_target(&r)); assert(w600_radio_request_pairing(&r,0)); idle(0);
    first=output_frame(0); reply=output_frame(first.length+10);
    assert(first.payload[1]==0x51 && reply.payload[1]==0 && r.tx_management_count==1 && r.tx_target_count==1); ++cases;
    setup(0); assert(w600_endpoint_set_target(&r.endpoint,2000));
    assert(w600_radio_report_target(&r)); w600_radio_stop(&r);
    assert(!r.target_pending && !w600_radio_report_target(&r)); ++cases;
    setup(0); assert(w600_endpoint_set_target(&r.endpoint,2000));
    radio_test_ipsr=43; assert(!w600_radio_report_target(&r)); radio_test_ipsr=0; ++cases;
    setup(0); assert(w600_endpoint_set_target(&r.endpoint,2000));
    assert(w600_radio_report_target(&r)); auto_tc=0; poll(0); poll(3);
    assert(!w600_radio_poll(&r,53) && r.error==W600_RADIO_TX_TIMEOUT && !r.tx_target_count); ++cases;
    setup(0); assert(w600_endpoint_set_target(&r.endpoint,2000));
    r.next_sequence=255; assert(w600_radio_report_target(&r)); done=idle(0);
    assert(output_frame(0).sequence==255 && r.next_sequence==1);
    assert(w600_radio_report_target(&r)); idle(done+1);
    assert(output_frame(15).sequence==1 && r.tx_target_count==2); ++cases;
    setup(0);r.endpoint.known=W600_KNOWN_CHILD_LOCK;r.endpoint.child_locked=true;
    assert(w600_radio_report_control(&r));idle(0);first=output_frame(0);
    assert(first.kind==5 && first.length==9 &&
        !memcmp(first.payload,(uint8_t[]){7,0x1c,3,0xff,0xff,7,0x18,0,1},9));++cases;
#ifdef W600_OTA_RECEIVER
    setup(0);w600_ota_init(&ota,&ota_ops,NULL);r.ota=&ota;ota_ready=true;
    struct w600_frame start={.sequence=71,.kind=2,.length=4,.payload={0x40,0,0,1}};
    inject(&start,0);done=idle(0);first=output_frame(0);
    assert(first.sequence==71 && first.kind==3 && first.length==3 && !first.payload[0]);
    assert(ota.state==W600_OTA_HEADER && !ota.pending && ota.waiting);
    reply=output_frame(13);
    assert(reply.kind==6 && reply.length==11 && reply.payload[0]==0x40 && reply.payload[1]==1);
    assert(ota.waiting && ota.state==W600_OTA_HEADER && !w600_radio_request_pairing(&r,done));++cases;
    uint32_t original_sent=ota.sent_at;
    uint8_t original_sequence=ota.tx.sequence;
    size_t duplicate_at=output_size;
    inject(&start,done+1);done=idle(done+1);reply=output_frame(duplicate_at);
    assert(reply.kind==3 && reply.sequence==71 && !reply.payload[0] && ota.waiting &&
        ota.tx.sequence==original_sequence && ota.sent_at==original_sent && !ota.retries);++cases;
    uint16_t saved_target=r.endpoint.target_raw;
    size_t before_ota=output_size;inject(&target,done+1);done=idle(done+1);reply=output_frame(before_ota);
    assert(reply.kind==3 && reply.payload[0]==1 && r.endpoint.target_raw==saved_target);++cases;
    before_ota=output_size;inject(&read,done+1);done=idle(done+1);reply=output_frame(before_ota);
    assert(reply.kind==4 && ota.state==W600_OTA_HEADER);++cases;
    uint8_t sequence=ota.tx.sequence;
    poll(ota.sent_at+W600_OTA_REPLY_MS);
    done=idle(ota.sent_at+W600_OTA_REPLY_MS+1);
    assert(ota.tx.sequence==sequence && ota.retries==1 && ota.waiting);++cases;
    setup(0);w600_ota_init(&ota,&ota_ops,NULL);r.ota=&ota;ota_ready=false;
    inject(&start,0);idle(0);first=output_frame(0);
    assert(first.payload[0] && !w600_ota_active(&ota));++cases;
#endif
    printf("production radio: %u cases PASS; state=%zu bytes\n",cases,sizeof r);
    return 0;
}
