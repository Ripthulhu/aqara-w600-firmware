#ifndef CIU32_STARTUP_H
#define CIU32_STARTUP_H

enum ciu32_startup_result {
    CIU32_STARTUP_OK,
    CIU32_STARTUP_BUSY,
    CIU32_STARTUP_MOTOR_OFF_FAILED,
    CIU32_STARTUP_TRIM_INVALID,
    CIU32_STARTUP_FLASH_TIMEOUT,
    CIU32_STARTUP_RCH_TIMEOUT,
    CIU32_STARTUP_RCH_SWITCH_TIMEOUT,
    CIU32_STARTUP_PLL_STOP_TIMEOUT,
    CIU32_STARTUP_CONFIG_FAILED,
    CIU32_STARTUP_PLL_TIMEOUT,
    CIU32_STARTUP_PLL_OUTPUT_FAILED,
    CIU32_STARTUP_PLL_SWITCH_TIMEOUT,
    CIU32_STARTUP_VERIFY_FAILED
};

/* Early startup only: privileged Thread/MSP, masked IRQs, SysTick off,
 * APB/DMA clocks off (DBG allowed).
 * BUSY makes no register writes. Otherwise commands motor off and checks its
 * readback; hardware failure can prevent that. Do not run timed peripherals
 * on failure. Factory trims are checked, never overwritten. */
enum ciu32_startup_result ciu32_startup_48mhz(void);

#endif
