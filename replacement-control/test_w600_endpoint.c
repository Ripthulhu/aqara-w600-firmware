#include "w600_endpoint.h"
#include <assert.h>
#include <stdio.h>
#include <string.h>

static struct w600_frame request(uint8_t kind, uint8_t sequence,
                                 const uint8_t *payload, size_t length)
{
    struct w600_frame frame = {.sequence = sequence, .kind = kind,
                               .length = (uint16_t)length};
    assert(length <= W600_MAX_PAYLOAD);
    if (length)
        memcpy(frame.payload, payload, length);
    return frame;
}

static struct w600_frame expect(struct w600_endpoint *state,
                                const struct w600_frame *input,
                                uint8_t kind, const uint8_t *payload, size_t length)
{
    struct w600_frame reply;
    memset(&reply, 0xa5, sizeof reply);
    assert(w600_endpoint_process(state, input, &reply));
    assert(reply.sequence == input->sequence && reply.kind == kind && reply.length == length);
    assert(memcmp(reply.payload, payload, length) == 0);
    uint8_t wire[W600_MAX_FRAME];
    size_t written;
    assert(w600_encode(&reply, wire, sizeof wire, &written));
    assert(written == length + 10);
    return reply;
}

static void no_reply(struct w600_endpoint *state, const struct w600_frame *input)
{
    struct w600_endpoint original = *state;
    struct w600_frame reply, untouched;
    memset(&reply, 0xa5, sizeof reply);
    untouched = reply;
    assert(!w600_endpoint_process(state, input, &reply));
    assert(memcmp(state, &original, sizeof original) == 0);
    assert(memcmp(&reply, &untouched, sizeof reply) == 0);
}

static void initial_truth_and_errors(void)
{
    struct w600_endpoint state;
    w600_endpoint_init(&state);
    assert(!state.known && !state.external_use && !state.target_raw && !state.external_temp_raw);
    const uint8_t ids[] = {7,0x0b,7,3,7,0x1d,7,0,7,4,7,5,7,0x1c,0xab,0xcd};
    const uint8_t expected[] = {
        7,0x0b,0,0,0, 7,3,0,0,0, 7,0x1d,0,1,0,
        7,0,1,0, 7,4,1,0, 7,5,1,0, 7,0x1c,1,0, 0xab,0xcd,1,0
    };
    struct w600_frame input = request(1, 0xff, ids, sizeof ids);
    expect(&state, &input, 4, expected, sizeof expected);
    const uint8_t mixed[] = {7,0x0b,0xab,0xcd,7,0x1d};
    const uint8_t mixed_reply[] = {7,0x0b,0,0,0, 0xab,0xcd,1,0, 7,0x1d,0,1,0};
    input = request(1, 0, mixed, sizeof mixed);
    expect(&state, &input, 4, mixed_reply, sizeof mixed_reply);
    assert(!state.known);
}

static void motor_inhibited_writes(void)
{
    struct w600_endpoint state;
    w600_endpoint_init(&state);
    const uint8_t payload[] = {
        7,3,0,0, 7,0x1d,1,0, 7,3,0,1, 7,0x1d,1,2,
        7,9,0,1, 7,9,0,0, 7,0x1c,3,3,0xe8, 7,0x0b,0,1,
        7,0x14,0,1, 7,0x18,0,1, 7,0x0c,0x0b,0,
        7,0x0d,0,0, 0x40,0,0,1, 1,0x0c,0,1
    };
    const uint8_t expected[] = {
        0,7,3, 0,7,0x1d, 3,7,3, 3,7,0x1d, 3,7,9, 3,7,9,
        3,7,0x1c, 3,7,0x0b, 3,7,0x14, 0,7,0x18, 3,7,0x0c,
        3,7,0x0d, 1,0x40,0, 1,1,0x0c
    };
    struct w600_frame input = request(2, 17, payload, sizeof payload);
    expect(&state, &input, 3, expected, sizeof expected);
    assert(state.known==W600_KNOWN_CHILD_LOCK && state.child_locked);
    initial_truth_and_errors();
}

static void raw_storage_and_duplicate_writes(void)
{
    struct w600_endpoint state;
    w600_endpoint_init(&state);
    const uint8_t payload[] = {7,0,4,0,0, 7,5,4,0xff,0xff, 7,4,1,0};
    const uint8_t expected[] = {0,7,0,0,7,5,0,7,4};
    struct w600_frame input = request(2, 3, payload, sizeof payload);
    struct w600_frame first = expect(&state, &input, 3, expected, sizeof expected);
    assert(state.known == 7 && state.target_raw == 0 && state.external_temp_raw == 0xffff && !state.external_use);
    struct w600_endpoint original = state;
    struct w600_frame duplicate = expect(&state, &input, 3, expected, sizeof expected);
    assert(memcmp(&state, &original, sizeof state) == 0);
    assert(memcmp(&first, &duplicate, sizeof first) == 0);
    const uint8_t ids[] = {7,0,7,4,7,5,7,3,7,0x0b,7,0x1d};
    const uint8_t readback[] = {7,0,0,4,0,0, 7,4,0,1,0, 7,5,0,4,0xff,0xff,
                              7,3,0,0,0, 7,0x0b,0,0,0, 7,0x1d,0,1,0};
    input = request(1, 4, ids, sizeof ids);
    expect(&state, &input, 4, readback, sizeof readback);
    const uint8_t external[] = {7,5,4,7,0x8c};
    const uint8_t external_reply[] = {0,7,5};
    input = request(2, 0x35, external, sizeof external);
    expect(&state, &input, 3, external_reply, sizeof external_reply);
    assert(state.external_temp_raw == 0x078c && state.external_use == 1);
    const uint8_t invalid_types[] = {7,0,0,1, 7,4,1,2, 7,5,3,7,0x88, 7,3,1,0, 7,0x1d,0,0};
    const uint8_t rejected[] = {3,7,0,3,7,4,3,7,5,3,7,3,3,7,0x1d};
    original = state;
    input = request(2, 5, invalid_types, sizeof invalid_types);
    expect(&state, &input, 3, rejected, sizeof rejected);
    assert(memcmp(&state, &original, sizeof state) == 0);
}

static void whole_batch_validation(void)
{
    struct w600_endpoint state;
    w600_endpoint_init(&state);
    const uint8_t partial[] = {7,0,4,0x12,0x34, 7,5,4,7};
    struct w600_frame input = request(2, 1, partial, sizeof partial);
    no_reply(&state, &input);
    assert(!state.known);
    const uint8_t unsupported_composite[] = {7,0,4,0x12,0x34, 7,0x0c,0x0c,0};
    input = request(2, 2, unsupported_composite, sizeof unsupported_composite);
    no_reply(&state, &input);
    const uint8_t trailing[] = {7,0,4,0x12,0x34, 0};
    input = request(2, 3, trailing, sizeof trailing);
    no_reply(&state, &input);
    const uint8_t blob_overrun[] = {7,0,4,0x12,0x34, 0xab,0xcd,0x0b,2,0};
    input = request(2, 4, blob_overrun, sizeof blob_overrun);
    no_reply(&state, &input);
    input = request(1, 5, partial, 3);
    no_reply(&state, &input);
    input = request(1, 6, NULL, 0);
    no_reply(&state, &input);
    input.kind = 2;
    no_reply(&state, &input);
    input.length = 257;
    no_reply(&state, &input);
}

static void bounded_batches_and_types(void)
{
    struct w600_endpoint state;
    w600_endpoint_init(&state);
    struct w600_frame input = {.sequence = 9, .kind = 2, .length = 256}, reply;
    for (unsigned i = 0; i < 64; ++i) {
        memcpy(input.payload + 4*i, (uint8_t[]){0xab,0xcd,0,1}, 4);
    }
    assert(w600_endpoint_process(&state, &input, &reply) && reply.length == 192);
    for (unsigned i = 0; i < 64; ++i)
        assert(memcmp(reply.payload + 3*i, (uint8_t[]){1,0xab,0xcd}, 3) == 0);
    input.payload[0] = 0xab; input.payload[1] = 0xcd; input.payload[2] = 0x0b;
    input.payload[3] = 252;
    assert(w600_endpoint_process(&state, &input, &reply) && reply.length == 3);
    input.payload[3] = 253;
    no_reply(&state, &input);
    const uint8_t types[] = {0,1,2,3,4,5,7,8};
    const uint8_t widths[] = {1,1,1,2,2,4,4,4};
    for (unsigned i = 0; i < sizeof types; ++i) {
        input.payload[2] = types[i]; input.length = (uint16_t)(3 + widths[i]);
        assert(w600_endpoint_process(&state, &input, &reply));
        --input.length;
        no_reply(&state, &input);
    }
    input.kind = 1; input.length = 20;
    for (unsigned i = 0; i < 10; ++i)
        memcpy(input.payload + 2*i, (uint8_t[]){7,0x0b}, 2);
    assert(w600_endpoint_process(&state, &input, &reply) && reply.length == 50);
    input.length = 22;
    no_reply(&state, &input);
}

static void real_wire_fixtures_and_no_ack_loops(void)
{
    static const uint8_t received[3][14] = {
        {0x55,5,2,0x51,2,0,4,0x4d,7,0x14,0,0,0x78,4},
        {0x55,5,2,0x52,2,0,4,0x4c,7,0x18,0,1,0x8b,0x2c},
        {0x55,5,2,0x53,2,0,4,0x4b,7,0x18,0,0,0x1b,0x55}
    };
    static const uint8_t input_wire[] = {0x55,5,2,0x35,2,0,5,0x68,7,5,4,7,0x8c,0x35,0x6b};
    static const uint8_t reply_wire[] = {0x55,5,2,0x35,3,0,3,0x69,0,7,5,0xdd,0x74};
    struct w600_endpoint state;
    struct w600_decoder decoder;
    struct w600_frame input, reply;
    w600_endpoint_init(&state);
    for (unsigned fixture = 0; fixture < 3; ++fixture) {
        w600_decoder_reset(&decoder);
        unsigned frames = 0;
        for (unsigned i = 0; i < sizeof received[fixture]; ++i)
            frames += w600_decoder_feed(&decoder, received[fixture][i], &input);
        assert(frames == 1 && w600_endpoint_process(&state, &input, &reply));
        assert(reply.kind == 3 && reply.sequence == received[fixture][3] && reply.length == 3);
        assert(reply.payload[0] == (fixture ? 0 : 3) && reply.payload[1] == 7 && reply.payload[2] == received[fixture][9]);
        assert(state.known==(fixture ? W600_KNOWN_CHILD_LOCK : 0));
        assert(state.child_locked==(fixture==1)); /* Incoming fixtures, not recorded TX. */
    }
    w600_decoder_reset(&decoder);
    unsigned count = 0;
    for (unsigned i = 0; i < sizeof input_wire; ++i)
        count += w600_decoder_feed(&decoder, input_wire[i], &input);
    assert(count == 1 && w600_endpoint_process(&state, &input, &reply));
    uint8_t wire[W600_MAX_FRAME]; size_t written;
    assert(w600_encode(&reply, wire, sizeof wire, &written));
    assert(written == sizeof reply_wire && memcmp(wire, reply_wire, written) == 0);
    /* Both bytestrings are pinned SRAM examples, not a new bidirectional capture. */
    no_reply(&state, &reply);
    input.kind = 7;
    no_reply(&state, &input);
    input.kind = 5;
    no_reply(&state, &input);
    input.kind = 4;
    no_reply(&state, &input);
    input.kind = 255;
    no_reply(&state, &input);
    const uint8_t off[] = {7,3,0,0};
    input = request(2, 0, off, sizeof off);
    assert(w600_endpoint_process(&state, &input, &input));
    assert(input.kind == 3 && input.sequence == 0 && input.length == 3);
    assert(memcmp(input.payload, (uint8_t[]){0,7,3}, 3) == 0);
}

static void randomized_bounds(void)
{
    struct w600_endpoint state;
    w600_endpoint_init(&state);
    uint32_t random = 0x12345678;
    for (unsigned iteration = 0; iteration < 20000; ++iteration) {
        struct w600_frame input, reply, unchanged;
        random = random * 1664525u + 1013904223u;
        input.kind = (uint8_t)(random >> 28);
        input.sequence = (uint8_t)random;
        input.length = (uint16_t)(random % 258);
        for (unsigned i = 0; i < W600_MAX_PAYLOAD; ++i) {
            random = random * 1664525u + 1013904223u;
            input.payload[i] = (uint8_t)(random >> 24);
        }
        struct w600_endpoint before = state;
        memset(&reply, 0xa5, sizeof reply); unchanged = reply;
        if (w600_endpoint_process(&state, &input, &reply)) {
            assert(reply.sequence == input.sequence && reply.length <= W600_MAX_PAYLOAD);
            assert(reply.kind == (input.kind == 1 ? 4 : 3));
            assert((state.known & ~23u) == 0 && state.external_use <= 1);
        } else {
            assert(memcmp(&state, &before, sizeof state) == 0);
            assert(memcmp(&reply, &unchanged, sizeof reply) == 0);
        }
    }
    struct w600_frame input = request(1, 1, (uint8_t[]){7,0x0b}, 2), reply;
    w600_endpoint_init(NULL);
    assert(!w600_endpoint_process(NULL, &input, &reply));
    assert(!w600_endpoint_process(&state, NULL, &reply));
    assert(!w600_endpoint_process(&state, &input, NULL));
}

static void local_temperature_availability(void)
{
    struct w600_endpoint s={.target_raw=2100,.external_temp_raw=2200,
        .known=7,.external_use=1};
    struct w600_frame input=request(1,42,(uint8_t[]){7,6},2);
    expect(&s,&input,4,(uint8_t[]){7,6,1,0},4);
    for (int t=-50;t<=595;++t) {
        assert(w600_endpoint_local_temperature(&s,(int16_t)t,true));
        uint16_t raw=(uint16_t)(t*10);
        uint8_t expected[]={7,6,0,4,(uint8_t)(raw>>8),(uint8_t)raw};
        expect(&s,&input,4,expected,sizeof expected);
        assert(s.target_raw==2100 && s.external_temp_raw==2200 && s.external_use==1 && s.known==15);
    }
    struct w600_endpoint previous=s;
    assert(!w600_endpoint_local_temperature(NULL,200,true));
    assert(!w600_endpoint_local_temperature(&s,-51,true));
    assert(!w600_endpoint_local_temperature(&s,596,true));
    assert(!memcmp(&s,&previous,sizeof s));
    struct w600_frame write=request(2,43,(uint8_t[]){7,6,4,7,0xd0},5);
    expect(&s,&write,3,(uint8_t[]){3,7,6},3);
    assert(!memcmp(&s,&previous,sizeof s));
    assert(w600_endpoint_local_temperature(&s,0,false));
    expect(&s,&input,4,(uint8_t[]){7,6,1,0},4);
    assert(s.known==7 && s.external_use==1);
}

static void calibration_intent_is_transactional(void)
{
    struct w600_endpoint s={.control_enabled=true};
    struct w600_frame in=request(2,20,(uint8_t[]){7,9,0,1,7},5);
    no_reply(&s,&in);
    assert(!s.calibration_pending && !s.request_seen);
    in=request(2,20,(uint8_t[]){7,9,0,1},4);
    expect(&s,&in,3,(uint8_t[]){0,7,9},3);
    assert(s.calibration_pending && !s.calibrated && !s.opening_known);
    s.calibration_pending=false; s.calibration_busy=true;
    expect(&s,&in,3,(uint8_t[]){0,7,9},3);
    assert(!s.calibration_pending && s.calibration_busy);
    s.calibration_busy=false; s.calibrated=true; s.opening_known=true;
    s.opening_permille=1000;
    expect(&s,&in,3,(uint8_t[]){0,7,9},3);
    assert(!s.calibration_pending && s.calibrated); /* Late consecutive retry. */
    in.sequence=21;
    expect(&s,&in,3,(uint8_t[]){0,7,9},3);
    assert(s.calibration_pending && !s.calibrated && !s.opening_known);
    s.calibration_pending=false; s.calibration_busy=true;
    in.sequence=22;
    expect(&s,&in,3,(uint8_t[]){3,7,9},3);
    assert(!s.calibration_pending && s.calibration_busy);
    in=request(2,23,(uint8_t[]){7,3,0,0},4);
    expect(&s,&in,3,(uint8_t[]){0,7,3},3);
    assert(s.cancel_calibration);
}

static void modes_ranges_and_aliases(void)
{
    struct w600_endpoint s={.control_enabled=true};
    struct w600_frame in=request(2,30,(uint8_t[]){7,0,4,8,0x98,7,5,4,7,0xd0},10);
    expect(&s,&in,3,(uint8_t[]){0,7,0,0,7,5},6);
    assert(!s.mode && !s.mode_known && s.external_updates==1);
    in=request(2,31,(uint8_t[]){7,3,0,1,7,0x1d,1,2,7,5,4,7,0xd0},13);
    uint32_t before=s.control_updates;
    expect(&s,&in,3,(uint8_t[]){3,7,3,3,7,0x1d,0,7,5},9);
    assert(!s.mode && s.mode_rejected && s.control_updates==before && s.external_updates==2);
    s.mode_rejected=false;
    s.calibrated=true;
    in=request(2,31,(uint8_t[]){7,3,0,1,7,0x1d,1,2},8);
    assert(w600_endpoint_process(&s,&in,&in));
    assert(in.kind==3 && in.length==6 && !memcmp(in.payload,(uint8_t[]){0,7,3,0,7,0x1d},6));
    assert(s.mode==2 && s.mode_known && !s.mode_rejected);
    in=request(1,32,(uint8_t[]){7,3,7,0x1d,7,0},6);
    assert(w600_endpoint_process(&s,&in,&in));
    assert(in.kind==4 && in.length==16 && !memcmp(in.payload,
        (uint8_t[]){7,3,0,0,1,7,0x1d,0,1,2,7,0,0,4,8,0x98},16));
    uint32_t update=s.control_updates;
    in=request(2,33,(uint8_t[]){7,0,4,1,0xf3,7,0,4,0x0b,0xb9,7,5,4,0x17,0x71},15);
    expect(&s,&in,3,(uint8_t[]){3,7,0,3,7,0,3,7,5},9);
    assert(s.control_updates==update && s.target_raw==2200 && s.external_temp_raw==2000);
    in=request(2,34,(uint8_t[]){7,0x1d,1,3},4);
    expect(&s,&in,3,(uint8_t[]){3,7,0x1d},3);
    assert(s.mode==2);
    in=request(2,35,(uint8_t[]){7,3,0,0},4);
    expect(&s,&in,3,(uint8_t[]){0,7,3},3);
    assert(!s.mode);
    in=request(2,36,(uint8_t[]){7,0,4,8,0x98,7,5,4,7,0xd0},10);
    expect(&s,&in,3,(uint8_t[]){0,7,0,0,7,5},6);
    assert(!s.mode && s.external_updates==3); /* Same sensor value refreshes its age, never enables heat. */
}

int main(void)
{
    for (unsigned type=0;type<3;++type) for (unsigned value=0;value<256;++value) {
        struct w600_endpoint s={0};
        struct w600_frame in=request(2,1,(uint8_t[]){7,0x18,(uint8_t)type,(uint8_t)value},4);
        bool valid=type==0 && value<=1;
        expect(&s,&in,3,(uint8_t[]){valid ? 0 : 3,7,0x18},3);
        assert(s.child_locked==(valid && value) && s.known==(valid ? W600_KNOWN_CHILD_LOCK : 0));
        in=request(1,2,(uint8_t[]){7,0x18},2);
        expect(&s,&in,4,(uint8_t[]){7,0x18,0,0,s.child_locked},5);
        assert(!s.control_updates && !s.external_updates && !s.mode && !s.calibration_pending);
    }
    assert(!w600_endpoint_set_target(NULL,2000));
    for(unsigned raw=0;raw<=UINT16_MAX;++raw) {
        struct w600_endpoint state={.control_enabled=true,.known=7,.target_raw=2000,
            .mode=0,.mode_known=true,.external_use=1,.external_temp_raw=1950,
            .calibration_busy=true,.control_updates=42,.external_updates=19};
        struct w600_endpoint expected=state;
        bool valid=raw>=500 && raw<=3000;
        assert(w600_endpoint_set_target(&state,(uint16_t)raw)==valid);
        if(valid) {expected.target_raw=(uint16_t)raw;expected.control_updates+=(raw!=2000);}
        assert(!memcmp(&state,&expected,sizeof state));
    }
    initial_truth_and_errors();
    motor_inhibited_writes();
    raw_storage_and_duplicate_writes();
    whole_batch_validation();
    bounded_batches_and_types();
    real_wire_fixtures_and_no_ack_loops();
    randomized_bounds();
    local_temperature_availability();
    calibration_intent_is_transactional();
    modes_ranges_and_aliases();
    puts("W600 motor-inhibited endpoint tests passed");
    return 0;
}
