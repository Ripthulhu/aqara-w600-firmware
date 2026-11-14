#ifndef W600_FRAME_H
#define W600_FRAME_H
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

enum { W600_MAX_PAYLOAD = 256, W600_MAX_FRAME = W600_MAX_PAYLOAD + 10 };
struct w600_frame {
    uint8_t sequence, kind;
    uint16_t length;
    uint8_t payload[W600_MAX_PAYLOAD];
};
struct w600_decoder {
    uint16_t used;
    uint8_t bytes[W600_MAX_FRAME];
};

uint16_t w600_crc16(const uint8_t *data, size_t length);
bool w600_encode(const struct w600_frame *frame, uint8_t *wire,
                 size_t capacity, size_t *written);
void w600_decoder_reset(struct w600_decoder *decoder);
bool w600_decoder_next(struct w600_decoder *decoder, struct w600_frame *out);
bool w600_decoder_feed(struct w600_decoder *decoder, uint8_t byte,
                      struct w600_frame *out);
#endif
