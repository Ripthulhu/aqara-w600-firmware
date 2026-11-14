.syntax unified
.cpu cortex-m0plus
.thumb
.section .vectors,"a",%progbits
.balign 256
.global w600_vectors
w600_vectors:
    .word __stack_top, W600_Reset_Handler
    .word W600_Default_Handler, W600_Default_Handler
    .word 0, 0, 0, 0, 0, 0, 0
    .word W600_Default_Handler
    .word 0, 0
    .word W600_Default_Handler, W600_SysTick_Handler
    .rept 27
    .word W600_Default_Handler
    .endr
    .word W600_UART3_4_Handler
    .rept 4
    .word W600_Default_Handler
    .endr

.section .text.W600_Reset_Handler,"ax",%progbits
.balign 4
.global W600_Reset_Handler
.type W600_Reset_Handler,%function
.thumb_func
W600_Reset_Handler:
    cpsid i
    movs r0, #0
    msr control, r0
    isb
    ldr r0, =__stack_top
    msr msp, r0
    ldr r0, =w600_vectors
    ldr r1, =0xe000ed08
    str r0, [r1]
    dsb
    isb
    bl w600_reset_early
    cmp r0, #0
    beq W600_Default_Handler
#ifdef W600_FLASH_APPLICATION
    ldr r0, =__nvcode_start
    ldr r1, =__nvcode_load
    ldr r2, =__nvcode_size
    ldr r3, =__bss_start
    sub sp, #8
    movs r4, #0
    str r4, [sp]
    bl w600_runtime_apply
    add sp, #8
#endif
    ldr r0, =__data_start
    ldr r1, =__data_load
    ldr r2, =__data_size
    ldr r3, =__bss_start
    sub sp, #8
    ldr r4, =__bss_size
    str r4, [sp]
    bl w600_runtime_apply
    add sp, #8
    bl w600_app_main
    b W600_Default_Handler
.size W600_Reset_Handler, .-W600_Reset_Handler
.ltorg

.section .text.W600_Default_Handler,"ax",%progbits
.balign 4
.global W600_Default_Handler
.type W600_Default_Handler,%function
.thumb_func
W600_Default_Handler:
    cpsid i
    bl w600_reset_fault
1:  b 1b
.size W600_Default_Handler, .-W600_Default_Handler
