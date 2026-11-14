#include "w600_pairing.h"

#define HALF_RANGE UINT32_C(0x80000000)
#define WINDOW_MS UINT32_C(10000)

void w600_pairing_init(struct w600_pairing *p)
{
    if (p) *p = (struct w600_pairing){0};
}

bool w600_pairing_prepare(struct w600_pairing *p, uint8_t sequence, uint32_t now,
                          struct w600_frame *report)
{
    if (!p || !report || !sequence ||
        (p->stage != W600_PAIRING_IDLE && p->stage != W600_PAIRING_WINDOW_ENDED) ||
        (p->stage != W600_PAIRING_IDLE &&
         (sequence == p->sequence || (uint32_t)(now - p->last_now) >= HALF_RANGE)))
        return false;
    *report = (struct w600_frame){.sequence = sequence, .kind = 5, .length = 4,
                                  .payload = {0, 0x51, 0, 1}};
    *p = (struct w600_pairing){.stage = W600_PAIRING_PREPARED, .sequence = sequence,
                              .prepared_at = now, .last_now = now};
    return true;
}

static size_t value_width(uint8_t type, const uint8_t *value, size_t remaining)
{
    if (type <= 2) return 1;
    if (type == 3 || type == 4) return 2;
    if (type == 5 || type == 7 || type == 8) return 4;
    if (type == 0x0b) return remaining ? (size_t)value[0] + 1 : 0;
    return 0;
}

static bool response52(const struct w600_frame *frame, uint8_t *value)
{
    if (frame->kind == 7) {
        if (frame->length != 5 || frame->payload[0] != 0 ||
            frame->payload[1] != 0x52 || frame->payload[3] != 0)
            return false;
        *value = frame->payload[4];
        return true;
    }
    if (frame->kind != 2 || !frame->length) return false;
    size_t offset = 0;
    bool found = false;
    while (offset < frame->length) {
        size_t remaining = frame->length - offset;
        if (remaining < 4) return false;
        const uint8_t *record = frame->payload + offset;
        size_t width = value_width(record[2], record + 3, remaining - 3);
        if (!width || width > remaining - 3) return false;
        if (record[0] == 0 && record[1] == 0x52) {
            if (found || record[2] != 0) return false;
            *value = record[3]; found = true;
        }
        offset += 3 + width;
    }
    return found;
}

unsigned w600_pairing_update(struct w600_pairing *p, const struct w600_frame *frame,
                             uint32_t now)
{
    if (!p || p->stage == W600_PAIRING_IDLE || p->stage == W600_PAIRING_CLOCK_ERROR)
        return 0;
    if ((uint32_t)(now - p->last_now) >= HALF_RANGE) {
        p->stage = W600_PAIRING_CLOCK_ERROR;
        return W600_PAIRING_BAD_CLOCK;
    }
    p->last_now = now;
    if (p->stage != W600_PAIRING_PREPARED) return 0;
    if ((uint32_t)(now - p->prepared_at) >= WINDOW_MS) {
        p->stage = W600_PAIRING_WINDOW_ENDED;
        return W600_PAIRING_WINDOW_EXPIRED;
    }
    if (!frame || frame->length > W600_MAX_PAYLOAD) return 0;
    if (frame->kind == 3 && frame->sequence == p->sequence && !p->ack_seen) {
        p->ack_seen = true; p->ack_at = now;
        return W600_PAIRING_ACK_OBSERVED;
    }
    uint8_t value = 0;
    if (!p->response_seen && response52(frame, &value)) {
        p->response_seen = true; p->response_at = now;
        p->response_value = value; p->response_sequence = frame->sequence;
        p->response_kind = frame->kind; p->response_has_status = frame->kind == 7;
        p->response_status = frame->kind == 7 ? frame->payload[2] : 0;
        return W600_PAIRING_RESPONSE_OBSERVED;
    }
    return 0;
}
