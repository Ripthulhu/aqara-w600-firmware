#ifndef W600_RUNTIME_H
#define W600_RUNTIME_H
#include <stddef.h>

/* Linker-owned spans: writable nonoverlapping .data/.bss, flash .data source.
 * Zero-sized spans may be null. No globals, library calls or IRQ setup needed. */
void w600_runtime_apply(void *data, const void *load, size_t data_size,
                        void *bss, size_t bss_size);
#endif
