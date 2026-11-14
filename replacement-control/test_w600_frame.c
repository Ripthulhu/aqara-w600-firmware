#include "w600_frame.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static const uint8_t captured[3][14] = {
    {0x55,5,2,0x51,2,0,4,0x4d,7,0x14,0,0,0x78,4},
    {0x55,5,2,0x52,2,0,4,0x4c,7,0x18,0,1,0x8b,0x2c},
    {0x55,5,2,0x53,2,0,4,0x4b,7,0x18,0,0,0x1b,0x55}
};

static void captured_equal(const struct w600_frame *frame, unsigned index)
{
    assert(frame->sequence == captured[index][3] && frame->kind == 2 && frame->length == 4);
    assert(memcmp(frame->payload, captured[index] + 8, 4) == 0);
}

static unsigned feed(struct w600_decoder *decoder, const uint8_t *wire, size_t size,
                     struct w600_frame *out)
{
    unsigned frames = 0;
    for (size_t i = 0; i < size; ++i) {
        if (w600_decoder_feed(decoder, wire[i], out))
            ++frames;
        assert(decoder->used < W600_MAX_FRAME);
    }
    return frames;
}

static void captured_fixtures(void)
{
    struct w600_decoder decoder;
    struct w600_frame out;
    uint8_t encoded[W600_MAX_FRAME];
    for (unsigned index = 0; index < 3; ++index) {
        for (size_t split = 0; split <= sizeof captured[index]; ++split) {
            w600_decoder_reset(&decoder);
            unsigned count = feed(&decoder, captured[index], split, &out);
            count += feed(&decoder, captured[index] + split, sizeof captured[index] - split, &out);
            assert(count == 1);
            captured_equal(&out, index);
            assert(!w600_decoder_next(&decoder, &out));
        }
        size_t written;
        assert(w600_encode(&out, encoded, sizeof encoded, &written));
        assert(written == sizeof captured[index]);
        assert(memcmp(encoded, captured[index], written) == 0);
    }
    w600_decoder_reset(&decoder);
    for (unsigned index = 0; index < 3; ++index) {
        assert(feed(&decoder, captured[index], sizeof captured[index], &out) == 1);
        captured_equal(&out, index);
    }
}

static void lengths_and_magic_payload(void)
{
    struct w600_decoder decoder;
    struct w600_frame frame = {.sequence = 255, .kind = 255}, out;
    uint8_t wire[W600_MAX_FRAME];
    size_t written;
    const unsigned lengths[] = {0, 2, 256};
    for (unsigned i = 0; i < 3; ++i) {
        frame.length = (uint16_t)lengths[i];
        for (unsigned n = 0; n < frame.length; ++n)
            frame.payload[n] = (uint8_t)n;
        if (frame.length == 256)
            memcpy(frame.payload + 100, captured[0], sizeof captured[0]);
        assert(w600_encode(&frame, wire, sizeof wire, &written));
        assert(written == (size_t)frame.length + 10);
        w600_decoder_reset(&decoder);
        assert(feed(&decoder, wire, written, &out) == 1);
        assert(out.sequence == 255 && out.kind == 255 && out.length == frame.length);
        assert(memcmp(out.payload, frame.payload, frame.length) == 0);
    }
    frame.length = 257;
    assert(!w600_encode(&frame, wire, sizeof wire, &written) && written == 0);
    frame.length = 256;
    memset(wire, 0xa5, sizeof wire);
    assert(!w600_encode(&frame, wire, 265, &written));
    assert(wire[0] == 0xa5 && wire[265] == 0xa5 && written == 0);
    assert(!w600_encode(NULL, wire, sizeof wire, &written));
    assert(!w600_encode(&frame, NULL, sizeof wire, &written));
    assert(!w600_encode(&frame, wire, sizeof wire, NULL));
}

static void damaged_and_oversize(void)
{
    struct w600_decoder decoder;
    struct w600_frame out;
    uint8_t bad[14];
    for (unsigned kind = 0; kind < 2; ++kind) {
        w600_decoder_reset(&decoder);
        memcpy(bad, captured[0], sizeof bad);
        bad[kind ? 13 : 7] ^= 1;
        assert(feed(&decoder, bad, sizeof bad, &out) == 0);
        assert(feed(&decoder, captured[1], sizeof captured[1], &out) == 1);
        captured_equal(&out, 1);
    }
    const uint16_t lengths[] = {257, 65535};
    for (unsigned n = 0; n < 2; ++n) {
        uint8_t header[] = {0x55,5,2,0xaa,2,0,0,0};
        header[5] = (uint8_t)(lengths[n] >> 8); header[6] = (uint8_t)lengths[n];
        uint8_t sum = 0;
        for (unsigned i = 0; i < 7; ++i)
            sum = (uint8_t)(sum + header[i]);
        header[7] = (uint8_t)(0u - sum);
        w600_decoder_reset(&decoder);
        assert(feed(&decoder, header, sizeof header, &out) == 0);
        assert(feed(&decoder, captured[2], sizeof captured[2], &out) == 1);
        captured_equal(&out, 2);
    }
    w600_decoder_reset(&decoder);
    const uint8_t garbage[] = {0,0x55,0x55,5,2,0x7f};
    assert(feed(&decoder, garbage, sizeof garbage, &out) == 0);
    assert(feed(&decoder, captured[0], sizeof captured[0], &out) == 1);
    captured_equal(&out, 0);
}

static void truncated_and_buffered_recovery(void)
{
    struct w600_decoder decoder;
    struct w600_frame frame = {.sequence = 1, .kind = 5, .length = 256}, out;
    uint8_t wire[W600_MAX_FRAME];
    size_t written;
    assert(w600_encode(&frame, wire, sizeof wire, &written));
    w600_decoder_reset(&decoder);
    assert(feed(&decoder, wire, 12, &out) == 0);
    w600_decoder_reset(&decoder); /* Caller-reported idle gap abandons a truncated frame. */
    assert(feed(&decoder, captured[0], sizeof captured[0], &out) == 1);
    captured_equal(&out, 0);

    w600_decoder_reset(&decoder);
    assert(feed(&decoder, captured[0], 11, &out) == 0);
    assert(feed(&decoder, captured[1], sizeof captured[1], &out) == 1);
    captured_equal(&out, 1); /* Enough bytes arrived to reject the truncated candidate's CRC. */

    frame.length = 28;
    memcpy(frame.payload, captured[0], 14);
    memcpy(frame.payload + 14, captured[1], 14);
    assert(w600_encode(&frame, wire, sizeof wire, &written));
    w600_decoder_reset(&decoder);
    assert(feed(&decoder, wire, written, &out) == 1);
    assert(out.sequence == 1 && out.kind == 5 && out.length == 28); /* Never preempt valid outer. */
    wire[written - 1] ^= 1;
    w600_decoder_reset(&decoder);
    assert(feed(&decoder, wire, written, &out) == 1);
    captured_equal(&out, 0);
    assert(w600_decoder_next(&decoder, &out));
    captured_equal(&out, 1);
    assert(!w600_decoder_next(&decoder, &out));
}

static void bounded_garbage(void)
{
    struct w600_decoder decoder;
    struct w600_frame out;
    w600_decoder_reset(&decoder);
    uint32_t random = 1;
    for (unsigned i = 0; i < 10000; ++i) {
        random = random * 1664525u + 1013904223u;
        if (w600_decoder_feed(&decoder, (uint8_t)(random >> 24), &out))
            assert(out.length <= W600_MAX_PAYLOAD);
        assert(decoder.used < W600_MAX_FRAME);
    }
    w600_decoder_reset(&decoder);
    assert(feed(&decoder, captured[2], sizeof captured[2], &out) == 1);
    captured_equal(&out, 2);
}

int main(void)
{
    assert(w600_crc16((const uint8_t *)"123456789", 9) == 0x4b37);
    captured_fixtures();
    lengths_and_magic_payload();
    damaged_and_oversize();
    truncated_and_buffered_recovery();
    bounded_garbage();
    puts("W600 frame codec tests passed");
    return 0;
}
