#include "w600_frame.h"
#include <string.h>

uint16_t w600_crc16(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xffff;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (unsigned bit = 0; bit < 8; ++bit)
            crc = (uint16_t)((crc >> 1) ^ ((crc & 1u) ? 0xa001u : 0));
    }
    return crc;
}

bool w600_encode(const struct w600_frame *frame, uint8_t *wire,
                 size_t capacity, size_t *written)
{
    if (written)
        *written = 0;
    if (!frame || !wire || !written || frame->length > W600_MAX_PAYLOAD ||
        capacity < (size_t)frame->length + 10)
        return false;
    wire[0] = 0x55; wire[1] = 5; wire[2] = 2;
    wire[3] = frame->sequence; wire[4] = frame->kind;
    wire[5] = (uint8_t)(frame->length >> 8); wire[6] = (uint8_t)frame->length;
    uint8_t sum = 0;
    for (unsigned i = 0; i < 7; ++i)
        sum = (uint8_t)(sum + wire[i]);
    wire[7] = (uint8_t)(0u - sum);
    memcpy(wire + 8, frame->payload, frame->length);
    uint16_t crc = w600_crc16(wire, (size_t)frame->length + 8);
    wire[frame->length + 8] = (uint8_t)(crc >> 8);
    wire[frame->length + 9] = (uint8_t)crc;
    *written = (size_t)frame->length + 10;
    return true;
}

void w600_decoder_reset(struct w600_decoder *decoder)
{
    decoder->used = 0;
}

static void discard(struct w600_decoder *decoder, size_t count)
{
    decoder->used = (uint16_t)(decoder->used - count);
    memmove(decoder->bytes, decoder->bytes + count, decoder->used);
}

bool w600_decoder_next(struct w600_decoder *decoder, struct w600_frame *out)
{
    size_t start = 0;
    if (decoder->used > W600_MAX_FRAME) {
        w600_decoder_reset(decoder);
        return false;
    }
    while ((size_t)decoder->used - start >= 3) {
        const uint8_t *wire = decoder->bytes + start;
        if (wire[0] != 0x55 || wire[1] != 5 || wire[2] != 2) {
            ++start;
            continue;
        }
        size_t available = decoder->used - start;
        if (available < 8)
            break;
        size_t length = (size_t)wire[5] * 256 + wire[6];
        uint8_t sum = 0;
        for (unsigned i = 0; i < 8; ++i)
            sum = (uint8_t)(sum + wire[i]);
        if (length > W600_MAX_PAYLOAD || sum != 0) {
            ++start;
            continue;
        }
        if (available < length + 10)
            break;
        uint16_t crc = (uint16_t)((uint16_t)wire[length + 8] << 8) | wire[length + 9];
        if (w600_crc16(wire, length + 8) != crc) {
            ++start;
            continue;
        }
        out->sequence = wire[3]; out->kind = wire[4]; out->length = (uint16_t)length;
        memcpy(out->payload, wire + 8, length);
        discard(decoder, start + length + 10);
        return true;
    }
    if (start)
        discard(decoder, start);
    return false;
}

bool w600_decoder_feed(struct w600_decoder *decoder, uint8_t byte,
                      struct w600_frame *out)
{
    if (decoder->used >= W600_MAX_FRAME)
        w600_decoder_reset(decoder);
    decoder->bytes[decoder->used++] = byte;
    return w600_decoder_next(decoder, out);
}
