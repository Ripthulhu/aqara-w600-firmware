ENTRY(W600_Reset_Handler)
MEMORY {
 CODE (rwx) : ORIGIN = 0x20000c00, LENGTH = 0x5d80
 DATA (rw) : ORIGIN = 0x20000000, LENGTH = 0xc00
 PROBE (rw) : ORIGIN = 0x20006980, LENGTH = 0x80
 REPORT (rw) : ORIGIN = 0x20006a00, LENGTH = 0x200
}
__stack_top = 0x20007000;
PHDRS {
 code PT_LOAD FLAGS(5);
 data PT_LOAD FLAGS(6);
 probe PT_LOAD FLAGS(6);
 report PT_LOAD FLAGS(6);
}
SECTIONS {
 .vectors ORIGIN(CODE) : { KEEP(*(.vectors)) } >CODE :code
 .text : { *(.text*) *(.rodata*) } >CODE :code
 .data : {
  __data_start = .; *(.data*) __data_end = .;
 } >DATA AT>CODE :data
 __data_load = LOADADDR(.data);
 __data_size = SIZEOF(.data);
 .bss (NOLOAD) : {
  __bss_start = .; *(.bss*) *(COMMON) __bss_end = .;
 } >DATA :data
 __bss_size = SIZEOF(.bss);
 .probereport ORIGIN(PROBE) (NOLOAD) : { KEEP(*(.probereport)) } >PROBE :probe
 .appreport ORIGIN(REPORT) (NOLOAD) : { KEEP(*(.appreport)) } >REPORT :report
 /DISCARD/ : { *(.comment) *(.note*) *(.ARM.exidx*) *(.ARM.extab*) }
}
ASSERT(SIZEOF(.vectors)==192 && ADDR(.vectors)==0x20000c00,"Vector layout")
ASSERT(__data_end<=__bss_start && __bss_end<=0x20000c00,"Data/BSS overlap")
ASSERT(__data_load+__data_size<=0x20006980,"Code/data source overlaps motor report")
ASSERT(SIZEOF(.probereport)==128 && ADDR(.probereport)==0x20006980,"Motor service report layout")
ASSERT(SIZEOF(.appreport)==128,"Application report layout")
ASSERT((__stack_top&7)==0 && ORIGIN(REPORT)+LENGTH(REPORT)<=0x20006c00,"Stack reserve")
