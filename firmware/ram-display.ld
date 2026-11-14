ENTRY(W600_Reset_Handler)
MEMORY {
 CODE (rwx) : ORIGIN = 0x20001000, LENGTH = 0x5800
 DATA (rw) : ORIGIN = 0x20000000, LENGTH = 0x1000
 PROBE (rw) : ORIGIN = 0x20006800, LENGTH = 0x200
 REPORT (rw) : ORIGIN = 0x20006a00, LENGTH = 0x200
}
__stack_top = 0x20007000;
SECTIONS {
 .vectors ORIGIN(CODE) : { KEEP(*(.vectors)) } >CODE
 .text : { *(.text*) *(.rodata*) } >CODE
 .data : {
  __data_start = .; *(.data*) __data_end = .;
 } >DATA AT>CODE
 __data_load = LOADADDR(.data);
 __data_size = SIZEOF(.data);
 .bss (NOLOAD) : {
  __bss_start = .; *(.bss*) *(COMMON) __bss_end = .;
 } >DATA
 __bss_size = SIZEOF(.bss);
 .probereport ORIGIN(PROBE) (NOLOAD) : { KEEP(*(.probereport)) } >PROBE
 .appreport ORIGIN(REPORT) (NOLOAD) : { KEEP(*(.appreport)) } >REPORT
 /DISCARD/ : { *(.comment) *(.note*) *(.ARM.exidx*) *(.ARM.extab*) }
}
ASSERT(SIZEOF(.vectors)==192 && ADDR(.vectors)==0x20001000,"Vector layout")
ASSERT(__data_end<=__bss_start && __bss_end<=0x20001000,"Data/BSS overlap")
ASSERT(__data_load+__data_size<=0x20006800,"Code/data source overlaps probe report")
ASSERT(SIZEOF(.probereport)>0 && SIZEOF(.probereport)<=512,"Raw probe report layout")
ASSERT(SIZEOF(.appreport)==128,"Application report layout")
ASSERT((__stack_top&7)==0 && ORIGIN(REPORT)+LENGTH(REPORT)<=0x20006c00,"Stack reserve")
