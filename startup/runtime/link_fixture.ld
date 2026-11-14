ENTRY(W600_Reset_Handler)
MEMORY {
 FLASH (rx) : ORIGIN = 0x08002000, LENGTH = 0xe800
 RAM (rw) : ORIGIN = 0x20000000, LENGTH = 0x6c00
}
__stack_top = 0x20007000;
SECTIONS {
 .vectors ORIGIN(FLASH) : { KEEP(*(.vectors)) } >FLASH
 .text : { *(.text*) *(.rodata*) } >FLASH
 .data : {
  __data_start = .; *(.data*) __data_end = .;
 } >RAM AT>FLASH
 __data_load = LOADADDR(.data);
 __data_size = SIZEOF(.data);
 .bss (NOLOAD) : {
  __bss_start = .; *(.bss*) *(COMMON) __bss_end = .;
 } >RAM
 __bss_size = SIZEOF(.bss);
 .noinit (NOLOAD) : { *(.noinit*) } >RAM
 __ram_end = .;
 /DISCARD/ : { *(.comment) *(.note*) *(.ARM.exidx*) *(.ARM.extab*) }
}
ASSERT(ADDR(.vectors)==0x08002000 && SIZEOF(.vectors)==192,"Vector layout")
ASSERT((__stack_top&7)==0 && __ram_end<=0x20006c00,"Stack reserve")
ASSERT(__data_start>=ORIGIN(RAM) && __data_end<=__bss_start,"Data/BSS overlap")
ASSERT(__bss_end<=ADDR(.noinit),"BSS overlaps noinit")
ASSERT(__data_load>=ORIGIN(FLASH) && __data_load+__data_size<=ORIGIN(FLASH)+LENGTH(FLASH),"Data source")
