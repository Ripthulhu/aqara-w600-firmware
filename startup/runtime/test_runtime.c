#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "w600_runtime.h"

int main(void)
{
    const uint8_t source[] = {0x31, 0x82, 0x00, 0xff, 0x44, 0x12, 0x09};
    uint8_t data[19], bss[21], noinit[17];
    unsigned cases = 0;
    for (size_t n = 0; n <= sizeof source; ++n) {
        memset(data, 0xa5, sizeof data);
        memset(bss, 0x7e, sizeof bss);
        memset(noinit, 0x61, sizeof noinit);
        w600_runtime_apply(data + 1, source, n, bss + 3, n + 1);
        assert(!memcmp(data + 1, source, n));
        for (size_t i = 0; i < sizeof data; ++i)
            if (i < 1 || i >= 1 + n) assert(data[i] == 0xa5);
        for (size_t i = 0; i < sizeof bss; ++i)
            assert(bss[i] == (i >= 3 && i < n + 4 ? 0 : 0x7e));
        for (size_t i = 0; i < sizeof noinit; ++i) assert(noinit[i] == 0x61);
        ++cases;
    }
    w600_runtime_apply(NULL, NULL, 0, NULL, 0);
    ++cases;
    memset(data, 0xa5, sizeof data);
    w600_runtime_apply(data, source, sizeof source, NULL, 0);
    assert(!memcmp(data, source, sizeof source) && data[sizeof source] == 0xa5);
    ++cases;
    memset(bss, 0x7e, sizeof bss);
    w600_runtime_apply(NULL, NULL, 0, bss, sizeof bss);
    for (size_t i = 0; i < sizeof bss; ++i) assert(!bss[i]);
    ++cases;
    assert(source[0] == 0x31 && source[6] == 0x09);
    printf("Runtime memory: %u scenarios passed\n", cases);
}
