#ifndef CIU_FLASH_REGS_H
#define CIU_FLASH_REGS_H
/* RM1005 section5; checked against the vendor header by verify_regs.c. */
#define FP_ACR 0x40022000u
#define FP_KEY 0x40022008u
#define FP_SR 0x40022010u
#define FP_CR 0x40022014u
#define FP_OPTR 0x40022020u
#define FP_WRPAS 0x40022038u
#define FP_WRPAE 0x4002203cu
#define FP_WRPBS 0x40022040u
#define FP_WRPBE 0x40022044u
#define FP_BUSY 0x00010000u
#define FP_EOP 0x01000000u
#define FP_ERRORS 0x0000001cu
#define FP_LOCK 0x80000000u
#define FP_OPTLOCK 0x40000000u
#define FP_PAGE_MODE 2u
#define FP_ERASE 8u
#define FP_PROGRAM 1u
#define FP_KEY1 0xe57a1a85u
#define FP_KEY2 0x7c6e8391u
#define FP_CFG 0x40021008u
#define FP_CSR 0x40021000u
#define FP_READY_MASK 0x03001c00u
#define FP_READY 0x03000400u
#define FP_PLL 0x4002100cu
#define FP_IOPEN 0x4002102cu
#define FP_AHBEN 0x40021030u
#define FP_SYST_CTRL 0xe000e010u
#define FP_SYST_LOAD 0xe000e014u
#define FP_VTOR 0xe000ed08u
#endif
