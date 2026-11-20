#include "w600_endpoint.h"

void w600_endpoint_init(struct w600_endpoint *state)
{
    if (state)
        *state = (struct w600_endpoint){0};
}

bool w600_endpoint_set_target(struct w600_endpoint *state,uint16_t raw)
{
    if (!state || (state->control_enabled && (raw<500 || raw>3000))) return false;
    if (state->control_enabled && (!(state->known&W600_KNOWN_TARGET) || state->target_raw!=raw))
        ++state->control_updates;
    state->target_raw=raw;
    state->known|=W600_KNOWN_TARGET;
    return true;
}

bool w600_endpoint_local_temperature(struct w600_endpoint *state,int16_t decic,bool valid)
{
    if (!state || (valid && (decic < -50 || decic > 595))) return false;
    if (valid) {
        state->local_temp_raw=(uint16_t)(decic*10);
        state->known|=W600_KNOWN_LOCAL_TEMP;
    } else state->known&=(uint8_t)~W600_KNOWN_LOCAL_TEMP;
    return true;
}

static uint16_t be16(const uint8_t *p)
{
    return (uint16_t)((uint16_t)p[0] << 8) | p[1];
}

static unsigned read_field(const struct w600_endpoint *state, uint16_t id,
                           uint8_t *type, uint16_t *value)
{
    switch (id) {
    case 0x2002:
        *type=3;*value=W600_CONTROLLER_REVISION;
        return 2;
    case 0x0718:
        *type = 0; *value = state->child_locked;
        return 1;
    case 0x0703:
        *type = 0; *value = state->mode!=0;
        return 1;
    case 0x070b:
        *type = 0; *value = state->calibrated;
        return 1;
    case 0x071c:
        if (!state->opening_known) return 0;
        *type = 3; *value = state->opening_permille;
        return 2;
    case 0x071d:
        *type = 1; *value = state->mode;
        return 1;
    case 0x0700:
        if (!(state->known & W600_KNOWN_TARGET))
            return 0;
        *type = 4; *value = state->target_raw;
        return 2;
    case 0x0704:
        if (!(state->known & W600_KNOWN_EXTERNAL_USE))
            return 0;
        *type = 1; *value = state->external_use;
        return 1;
    case 0x0705:
        if (!(state->known & W600_KNOWN_EXTERNAL_TEMP))
            return 0;
        *type = 4; *value = state->external_temp_raw;
        return 2;
    case 0x0706:
        if (!(state->known & W600_KNOWN_LOCAL_TEMP)) return 0;
        *type = 4; *value = state->local_temp_raw;
        return 2;
    default:
        return 0;
    }
}

static uint8_t write_field(struct w600_endpoint *state, uint16_t id,
                           uint8_t type, const uint8_t *value)
{
    switch (id) {
    case 0x0718:
        if (type!=0 || value[0]>1) return 3;
        state->child_locked=value[0]!=0;
        state->known|=W600_KNOWN_CHILD_LOCK;
        return 0;
    case 0x0703:
    case 0x071d:
        if (type != (id == 0x0703 ? 0 : 1) || value[0] > (id == 0x0703 ? 1 : 2) ||
            (!state->control_enabled && value[0])) return 3;
        if (value[0] && !state->calibrated) {
            state->mode_rejected=true;
            return 3;
        }
        if (state->control_enabled && (!state->mode_known || state->mode!=value[0])) ++state->control_updates;
        state->mode=value[0];
        if (state->control_enabled) state->mode_known=true;
        if (!value[0] && state->control_enabled && (state->calibration_pending || state->calibration_busy)) {
            state->cancel_calibration=true; state->calibration_pending=false;
        }
        return 0;
    case 0x0700:
        return type==4 && w600_endpoint_set_target(state,be16(value)) ? 0 : 3;
    case 0x0704:
        if (type != 1 || value[0] > 1)
            return 3;
        if (state->control_enabled && state->external_use!=value[0]) ++state->control_updates;
        state->external_use = value[0];
        state->known |= W600_KNOWN_EXTERNAL_USE;
        return 0;
    case 0x0705:
        if (type != 4 || (state->control_enabled && ((int16_t)be16(value)<-500 || (int16_t)be16(value)>6000)))
            return 3;
        state->external_temp_raw = be16(value);
        if (state->control_enabled && !state->external_use) ++state->control_updates;
        state->external_use = 1;
        state->known |= W600_KNOWN_EXTERNAL_TEMP | W600_KNOWN_EXTERNAL_USE;
        if (state->control_enabled) ++state->external_updates;
        return 0;
    case 0x0709:
        if (!state->control_enabled || type != 0) return 3;
        if (!value[0] || state->calibration_seen) return 0;
        if (state->calibration_busy || state->calibration_pending || state->moving) return 3;
        state->calibration_pending=true; state->cancel_calibration=false;
        state->calibration_seen=true;
        state->calibrated=false; state->opening_known=false;
        state->calibration_error=0;
        return 0;
    case 0x0706:
    case 0x070b:
    case 0x071c:
    case 0x0714:
    case 0x070c:
    case 0x070d:
    case 0x070f:
        return 3;
    default:
        return 1;
    }
}

static size_t write_width(uint8_t type, const uint8_t *value, size_t remaining)
{
    switch (type) {
    case 0: case 1: case 2: return 1;
    case 3: case 4: return 2;
    case 5: case 7: case 8: return 4;
    case 0x0b: return remaining ? (size_t)value[0] + 1 : 0;
    default: return 0; /* Composite type0C has no established record boundary. */
    }
}

bool w600_endpoint_process(struct w600_endpoint *state,
                           const struct w600_frame *request,
                           struct w600_frame *reply)
{
    if (!state || !request || !reply || request->length == 0 ||
        request->length > W600_MAX_PAYLOAD ||
        (request->kind != 1 && request->kind != 2))
        return false;
    if (request->kind == 1 && ((request->length & 1u) || request->length > 20))
        return false; /* Stock reads process at most ten IDs. */

    const uint16_t length=request->length;
    const uint8_t kind=request->kind,sequence=request->sequence;
    /* Validate every boundary before touching intent or reply. Write replies
     * shrink in place; only the at-most20 read-ID bytes need an alias copy. */
    if (kind==2) {
        for (size_t offset=0;offset<length;) {
            size_t remaining=length-offset;
            if (remaining<4) return false;
            const uint8_t *record=request->payload+offset;
            size_t width=write_width(record[2],record+3,remaining-3);
            if (!width || width>remaining-3) return false;
            offset+=3+width;
        }
    }
    uint8_t ids[20];
    if (kind==1) for (unsigned i=0;i<length;++i) ids[i]=request->payload[i];
    struct w600_endpoint next=*state;
    if (!next.request_seen || kind!=2 || sequence!=next.last_request_sequence)
        next.calibration_seen=false;
    size_t used=0;
    for (size_t offset=0;offset<length;) {
        if (kind == 1) {
            uint16_t id = be16(ids + offset), value = 0;
            uint8_t type = 0;
            unsigned width = read_field(&next, id, &type, &value);
            reply->payload[used++] = (uint8_t)(id >> 8);
            reply->payload[used++] = (uint8_t)id;
            reply->payload[used++] = width ? 0 : 1;
            reply->payload[used++] = type;
            if (width == 2)
                reply->payload[used++] = (uint8_t)(value >> 8);
            if (width)
                reply->payload[used++] = (uint8_t)value;
            offset += 2;
        } else {
            const uint8_t *record = request->payload + offset;
            size_t width = write_width(record[2], record + 3, length-offset-3);
            uint16_t id = be16(record);
            uint8_t status=write_field(&next,id,record[2],record+3);
            reply->payload[used++] = status;
            reply->payload[used++] = (uint8_t)(id >> 8);
            reply->payload[used++] = (uint8_t)id;
            offset += 3 + width;
        }
    }
    reply->sequence=sequence; reply->kind=kind==1 ? 4 : 3;
    reply->length=(uint16_t)used;
    /* Only consecutive retries of the same calibration transaction are idempotent. */
    if (next.control_enabled) {
        next.request_seen=true; next.last_request_sequence=sequence;
    }
    *state = next;
    return true;
}
