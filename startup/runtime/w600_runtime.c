#include <stdint.h>
#include "w600_runtime.h"

void w600_runtime_apply(void *data, const void *load, size_t data_size,
                        void *bss, size_t bss_size)
{
    volatile uint8_t *to = data;
    const volatile uint8_t *from = load;
    for (size_t i = 0; i < data_size; ++i) to[i] = from[i];
    to = bss;
    for (size_t i = 0; i < bss_size; ++i) to[i] = 0;
}
