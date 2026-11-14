ENTRY(ciu_flash_replace_page)
MEMORY { RAM (rwx) : ORIGIN = 0x20002000, LENGTH = 0x4000 }
SECTIONS {
  .text : { __flash_page_start = .; *(.text*) *(.rodata*) . = ALIGN(4); __flash_page_end = .; } > RAM
  .forbidden : { *(.data*) *(.bss*) *(COMMON) } > RAM
  /DISCARD/ : { *(.comment*) *(.ARM.attributes*) *(.ARM.exidx*) *(.ARM.extab*) }
}
ASSERT(SIZEOF(.forbidden) == 0, "No hidden mutable storage")
ASSERT(__flash_page_end <= 0x20006000, "SRAM code overflow")
