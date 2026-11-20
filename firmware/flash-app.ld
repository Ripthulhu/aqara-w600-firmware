ENTRY(W600_Reset_Handler)
MEMORY {
 CODE (rx) : ORIGIN = 0x08002000, LENGTH = 0xce00
 DATA (rw) : ORIGIN = 0x20000000, LENGTH = 0x6000
 NVCODE (rx) : ORIGIN = 0x20006000, LENGTH = 0x980
 PROBE (rw) : ORIGIN = 0x20006980, LENGTH = 0x80
 REPORT (rw) : ORIGIN = 0x20006a00, LENGTH = 0x200
}
__stack_top = 0x20007000;
__settings_page0 = 0x0800ee00;
__settings_page1 = 0x0800f000;
SECTIONS {
 .vectors ORIGIN(CODE) : { KEEP(*(.vectors)) } >CODE
 .text : { *(.text*) *(.rodata*) } >CODE
 .nvcode : {
  __nvcode_start = .; KEEP(*(.nvcode)) __nvcode_end = .;
 } >NVCODE AT>CODE
 __nvcode_load = LOADADDR(.nvcode);
 __nvcode_size = SIZEOF(.nvcode);
 .data : {
  __data_start = .; *(.data*) __data_end = .;
 } >DATA AT>CODE
 __data_load = LOADADDR(.data);
 __data_size = SIZEOF(.data);
 .ota_identity : { KEEP(*(.ota_identity)) } >CODE
 .bss (NOLOAD) : {
  __bss_start = .; *(.bss*) *(COMMON) __bss_end = .;
 } >DATA
 __bss_size = SIZEOF(.bss);
 .probereport ORIGIN(PROBE) (NOLOAD) : { KEEP(*(.probereport)) } >PROBE
 .appreport ORIGIN(REPORT) (NOLOAD) : { KEEP(*(.appreport)) } >REPORT
 .recovery (NOLOAD) : { KEEP(*(.recovery)) } >REPORT
 /DISCARD/ : { *(.comment) *(.note*) *(.ARM.exidx*) *(.ARM.extab*) }
}
ASSERT(ADDR(.vectors)==0x08002000 && SIZEOF(.vectors)==192,"Application vectors")
ASSERT(__data_end<=__bss_start && __bss_end<=0x20006000,"RAM overlaps settings worker")
ASSERT(__nvcode_start==0x20006000 && __nvcode_end<=0x20006980,"Settings worker overlaps reports")
ASSERT(__data_load+__data_size<=0x0800ee00,"Application crosses active-image boundary")
ASSERT(SIZEOF(.ota_identity)==64 && ADDR(.ota_identity)>=__data_load+__data_size &&
       ADDR(.ota_identity)+64<=0x08002000+52720,"OTA identity/image limit")
ASSERT(__data_load+__data_size<=__settings_page0 &&
       (__settings_page0&511)==0 && __settings_page1==__settings_page0+512 &&
       __settings_page1+512<=0x08010800,"Settings pages overlap code/staging")
ASSERT(SIZEOF(.probereport)==128 && ADDR(.probereport)==0x20006980,"Motor report layout")
ASSERT(SIZEOF(.appreport)==128,"Application report layout")
ASSERT(ADDR(.recovery)==0x20006a80 && SIZEOF(.recovery)==8,"Recovery record layout")
ASSERT((__stack_top&7)==0 && ORIGIN(REPORT)+LENGTH(REPORT)<=0x20006c00,"Stack reserve")
