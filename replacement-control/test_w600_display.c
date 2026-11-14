#include "w600_display.h"
#include <assert.h>
#include <limits.h>
#include <stdio.h>
#include <string.h>

static unsigned cases;
static const uint8_t normal[10] = {0x7b,0x60,0x37,0x75,0x6c,0x5d,0x5f,0x70,0x7f,0x7d};
static const uint8_t rotated[10] = {0x7b,0x0a,0x37,0x1f,0x4e,0x5d,0x7d,0x0b,0x7f,0x5f};

static void expected(uint32_t value, bool flip, uint8_t byte2, uint8_t flags,
                     uint8_t out[4])
{
    const uint8_t *glyph = flip ? rotated : normal;
    uint8_t a = value >= 100u ? glyph[value / 100u] : 0;
    uint8_t b = glyph[(value % 100u) / 10u];
    out[0] = flip ? b : a; out[1] = flip ? a : b; out[2] = byte2;
    unsigned fraction = value % 10u != 0u;
    out[3] = (uint8_t)((flags & 0xe8u) | (flip ? 2u | (fraction << 2) : 0x10u | fraction));
}
static void check(uint32_t value, bool flip, uint8_t byte2, uint8_t flags,
                  const uint8_t result[4])
{
    uint8_t guarded[6] = {0x96,0xa5,0x5a,byte2,flags,0x69};
    assert(w600_display_encode(value,flip,guarded+1));
    assert(!memcmp(guarded+1,result,4));
    assert(guarded[0]==0x96 && guarded[5]==0x69);
    ++cases;
}
int main(void)
{
    static const uint32_t values[] = {50,200,205};
    static const uint8_t vectors[2][3][4] = {
        {{0x00,0x5d,0x00,0x10},{0x37,0x7b,0x00,0x10},{0x37,0x7b,0x00,0x11}},
        {{0x5d,0x00,0x00,0x02},{0x7b,0x37,0x00,0x02},{0x7b,0x37,0x00,0x06}}
    };
    for (unsigned flip=0;flip<2;++flip) {
        for (unsigned i=0;i<3;++i) check(values[i],flip!=0,0,0,vectors[flip][i]);
        /* Exhausts both digit positions,0/9/10/99/100/999 and every tenths value. */
        for (uint32_t value=0;value<1000;++value) {
            uint8_t result[4]; expected(value,flip!=0,0,0,result);
            check(value,flip!=0,0,0,result);
        }
        /* Every combination of caller status byte and fourth-byte flags. */
        for (unsigned byte2=0;byte2<256;++byte2) for (unsigned flags=0;flags<256;++flags) {
            uint8_t result[4]; expected(205,flip!=0,(uint8_t)byte2,(uint8_t)flags,result);
            check(205,flip!=0,(uint8_t)byte2,(uint8_t)flags,result);
        }
        static const uint32_t invalid[] = {1000,1001,65535,UINT32_MAX};
        for (unsigned i=0;i<sizeof invalid/sizeof invalid[0];++i) {
            uint8_t frame[4]={0xa5,0x5a,0x96,0x69}, before[4];
            memcpy(before,frame,4);
            assert(!w600_display_encode(invalid[i],flip!=0,frame));
            assert(!memcmp(frame,before,4)); ++cases;
        }
        uint8_t a[4]={0},b[4]={0},c[4]={0};
        assert(w600_display_encode(201,flip!=0,a));
        assert(w600_display_encode(205,flip!=0,b));
        assert(w600_display_encode(209,flip!=0,c));
        assert(!memcmp(a,b,4) && !memcmp(b,c,4)); ++cases;
    }
    assert(!w600_display_encode(0,false,NULL)); ++cases;
    assert(!w600_display_encode(999,true,NULL)); ++cases;
    assert(!w600_display_encode(UINT32_MAX,true,NULL)); ++cases;
    printf("PASS %u display-renderer cases; data bytes only, no hardware/transport\n",cases);
}
