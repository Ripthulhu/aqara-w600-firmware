#include "w600_pairing.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static unsigned cases;
static struct w600_pairing p;
static struct w600_frame report;

static void start(uint8_t sequence, uint32_t now)
{
    w600_pairing_init(&p);
    assert(w600_pairing_prepare(&p, sequence, now, &report));
}
static void unchanged_start(uint8_t sequence, uint32_t now)
{
    unsigned char before[sizeof p], output[sizeof report];
    memcpy(before, &p, sizeof p); memcpy(output, &report, sizeof report);
    assert(!w600_pairing_prepare(&p, sequence, now, &report));
    assert(!memcmp(before, &p, sizeof p) && !memcmp(output, &report, sizeof report));
    ++cases;
}

int main(void)
{
    static const uint8_t wire_expected[] = {
        0x55,5,2,0x21,5,0,4,0x7a,0,0x51,0,1,0xf9,4
    };
    start(0x21, 100);
    uint8_t wire[W600_MAX_FRAME]; size_t used = 0;
    assert(w600_encode(&report, wire, sizeof wire, &used));
    assert(used == sizeof wire_expected && !memcmp(wire, wire_expected, used));
    struct w600_decoder decoder; struct w600_frame decoded;
    w600_decoder_reset(&decoder);
    for (size_t i = 0; i < used; ++i)
        assert(w600_decoder_feed(&decoder, wire[i], &decoded) == (i + 1 == used));
    assert(decoded.sequence == 0x21 && decoded.kind == 5 && decoded.length == 4 &&
           !memcmp(decoded.payload, report.payload, 4));
    ++cases;
    unchanged_start(0x22, 101); unchanged_start(0x21, 101); unchanged_start(0, 101);
    assert(w600_pairing_update(&p, NULL, 10099) == 0);
    assert(p.stage == W600_PAIRING_PREPARED && p.prepared_at == 100);
    assert(w600_pairing_update(&p, NULL, 10100) == W600_PAIRING_WINDOW_EXPIRED);
    assert(p.stage == W600_PAIRING_WINDOW_ENDED && !p.ack_seen && !p.response_seen);
    ++cases;
    unchanged_start(0x21, 10101); unchanged_start(0x22, 10099);
    assert(w600_pairing_prepare(&p, 0x22, 10101, &report));
    assert(p.prepared_at == 10101 && p.stage == W600_PAIRING_PREPARED);
    ++cases;

    struct w600_frame ack = {.sequence = 0x22, .kind = 3, .length = 0};
    assert(w600_pairing_update(&p, &ack, 10102) == W600_PAIRING_ACK_OBSERVED);
    assert(p.ack_seen && p.ack_at == 10102 && p.stage == W600_PAIRING_PREPARED);
    assert(w600_pairing_update(&p, &ack, 10103) == 0 && p.ack_at == 10102);
    assert(p.prepared_at == 10101);
    ++cases;
    start(0x21, 0); ack.sequence = 0x22;
    assert(w600_pairing_update(&p, &ack, 1) == 0 && !p.ack_seen); ++cases;
    ack.sequence = 0x21; ack.length = W600_MAX_PAYLOAD;
    memset(ack.payload, 0xA5, sizeof ack.payload);
    assert(w600_pairing_update(&p, &ack, 2) == W600_PAIRING_ACK_OBSERVED);
    assert(p.stage == W600_PAIRING_PREPARED); ++cases;
    assert(w600_pairing_update(&p, NULL, 10000) == W600_PAIRING_WINDOW_EXPIRED);
    assert(p.ack_seen && p.ack_at == 2); ++cases;

    struct w600_frame result = {.sequence = 0x99, .kind = 2, .length = 4,
                                .payload = {0,0x52,0,0}};
    for (unsigned value = 0; value <= 255; value += 255) {
        start(0x21, 100); result.payload[3] = (uint8_t)value;
        assert(w600_pairing_update(&p, &result, 101) == W600_PAIRING_RESPONSE_OBSERVED);
        assert(p.response_seen && p.response_at == 101 && p.response_value == value &&
               p.response_sequence == 0x99 && p.response_kind == 2 && !p.response_has_status);
        assert(p.stage == W600_PAIRING_PREPARED && !p.ack_seen);
        result.payload[3] ^= 1;
        assert(w600_pairing_update(&p, &result, 102) == 0 && p.response_at == 101 &&
               p.response_value == value);
        ++cases;
    }
    /* A late old0052 is indistinguishable on wire: record it, never attribute it. */
    assert(w600_pairing_update(&p, NULL, 10100) == W600_PAIRING_WINDOW_EXPIRED);
    assert(w600_pairing_prepare(&p, 0x22, 10101, &report));
    assert(w600_pairing_update(&p, &result, 10102) == W600_PAIRING_RESPONSE_OBSERVED);
    assert(p.stage == W600_PAIRING_PREPARED && p.response_sequence != p.sequence); ++cases;

    struct w600_frame compound = {.sequence = 7, .kind = 2, .length = 13,
        .payload = {7,5,4,8,0x32, 0,0x52,0,0xA5, 7,4,1,1}};
    start(0x21, 0);
    assert(w600_pairing_update(&p, &compound, 1) == W600_PAIRING_RESPONSE_OBSERVED);
    assert(p.response_value == 0xA5); ++cases;
    struct w600_frame kind7 = {.sequence = 3, .kind = 7, .length = 5,
                               .payload = {0,0x52,8,0,1}};
    start(0x21, 0);
    assert(w600_pairing_update(&p, &kind7, 1) == W600_PAIRING_RESPONSE_OBSERVED);
    assert(p.response_has_status && p.response_status == 8 && p.response_value == 1 &&
           p.stage == W600_PAIRING_PREPARED && !p.ack_seen); ++cases;

    const struct w600_frame malformed[] = {
        {.kind=2,.length=0}, {.kind=2,.length=1,.payload={0}},
        {.kind=2,.length=2,.payload={0,0x52}}, {.kind=2,.length=3,.payload={0,0x52,0}},
        {.kind=2,.length=4,.payload={0,0x52,1,0}},
        {.kind=2,.length=8,.payload={0,0x52,0,1,0,0x52,0,0}},
        {.kind=2,.length=5,.payload={0,0x52,0,1,0}},
        {.kind=2,.length=8,.payload={0,0x52,0,1,7,5,0x0B,255}},
        {.kind=2,.length=4,.payload={0,0x52,0x0C,0}},
        {.kind=7,.length=4,.payload={0,0x52,0,0}},
        {.kind=7,.length=5,.payload={0,0x52,0,1,0}},
        {.kind=7,.length=6,.payload={0,0x52,0,0,0,0}},
        {.kind=7,.length=5,.payload={0,0x51,0,0,0}},
        {.kind=1,.length=2,.payload={0,0x52}},
        {.sequence=0x21,.kind=5,.length=4,.payload={0,0x52,0,0}},
        {.sequence=0x21,.kind=3,.length=W600_MAX_PAYLOAD+1},
        {.kind=2,.length=UINT16_MAX}
    };
    for (size_t i = 0; i < sizeof malformed / sizeof *malformed; ++i) {
        start(0x21, 0);
        assert(w600_pairing_update(&p, &malformed[i], 1) == 0 &&
               !p.response_seen && !p.ack_seen && p.stage == W600_PAIRING_PREPARED);
        ++cases;
    }
    /* Strings/fixed types in unrelated records have known boundaries. */
    const uint8_t types[] = {0,1,2,3,4,5,7,8,0x0B};
    for (size_t i = 0; i < sizeof types; ++i) {
        start(0x21, 0);
        unsigned width = types[i] <= 2 || types[i] == 0x0B ? 1 : types[i] <= 4 ? 2 : 4;
        struct w600_frame frame = {.kind=2,.length=(uint16_t)(7+width),
                                  .payload={0,0x52,0,1,0x12,0x34}};
        frame.payload[6] = types[i];
        assert(w600_pairing_update(&p, &frame, 1) == W600_PAIRING_RESPONSE_OBSERVED);
        ++cases;
    }

    start(0x21, UINT32_MAX - 4999);
    uint32_t began = p.prepared_at;
    assert(w600_pairing_update(&p, NULL, began + 9999) == 0);
    assert(w600_pairing_update(&p, &ack, began + 10000) == W600_PAIRING_WINDOW_EXPIRED);
    assert(!p.ack_seen); ++cases;
    start(0x21, 100);
    assert(w600_pairing_update(&p, &result, 99) == W600_PAIRING_BAD_CLOCK);
    assert(p.stage == W600_PAIRING_CLOCK_ERROR && !p.response_seen);
    unchanged_start(0x22, 101);
    assert(w600_pairing_update(&p, NULL, 102) == 0); ++cases;
    start(0x21, 0);
    assert(w600_pairing_update(&p, NULL, UINT32_C(0x80000000)) == W600_PAIRING_BAD_CLOCK);
    ++cases;
    start(0x21, 0);
    assert(w600_pairing_update(&p, &result, 10000) == W600_PAIRING_WINDOW_EXPIRED);
    assert(!p.response_seen);
    assert(w600_pairing_update(&p, &result, 10001) == 0 && !p.response_seen); ++cases;

    w600_pairing_init(NULL); w600_pairing_init(&p);
    assert(w600_pairing_update(NULL, &result, 0) == 0);
    assert(w600_pairing_update(&p, &result, 0) == 0 && p.stage == W600_PAIRING_IDLE);
    assert(!w600_pairing_prepare(NULL, 1, 0, &report));
    assert(!w600_pairing_prepare(&p, 1, 0, NULL));
    unchanged_start(0, 0); ++cases;
    puts("No network, UART, motor, settings or retry callbacks exist in this component.");
    printf("PASS %u pairing-bookkeeping cases; no hardware tested\n", cases);
}
