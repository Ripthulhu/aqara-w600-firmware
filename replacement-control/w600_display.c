#include "w600_display.h"

bool w600_display_encode(uint32_t decic, bool flipped, uint8_t frame[4])
{
    /* Stock FLASH6320104b, glyph digits0..9 at0800E74F and0800E764. */
    static const uint8_t digits[2][10] = {
        {0x7b,0x60,0x37,0x75,0x6c,0x5d,0x5f,0x70,0x7f,0x7d},
        {0x7b,0x0a,0x37,0x1f,0x4e,0x5d,0x7d,0x0b,0x7f,0x5f}
    };
    if (!frame || decic > 999u) return false;
    /* At most nine subtractions per digit; no Cortex-M0 division helper. */
    unsigned tens = 0, ones = 0;
    uint32_t remainder = decic;
    while (remainder >= 100u) { remainder -= 100u; ++tens; }
    while (remainder >= 10u) { remainder -= 10u; ++ones; }
    const uint8_t *glyph = digits[flipped ? 1 : 0];
    uint8_t high = decic >= 100u ? glyph[tens] : 0;
    uint8_t low = glyph[ones];
    uint8_t marker = flipped ? (uint8_t)(2u | ((remainder != 0u) ? 4u : 0u)) :
                              (uint8_t)(0x10u | ((remainder != 0u) ? 1u : 0u));
    frame[0] = flipped ? low : high;
    frame[1] = flipped ? high : low;
    frame[3] = (uint8_t)((frame[3] & 0xe8u) | marker);
    return true;
}
