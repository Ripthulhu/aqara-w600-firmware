#ifndef W600_DISPLAY_TX_TEST_MMIO_H
#define W600_DISPLAY_TX_TEST_MMIO_H
#include <stdint.h>
struct display_test_rcc { uint32_t CFG, PLLCFG, CSR1, AHBEN, IOPEN, APB1EN; };
struct display_test_gpio { uint32_t MODE, OTYPE, PUPD, AFL; };
struct display_test_uart { uint32_t CR1, CR2, CR3, BRR, ISR; };
struct display_test_nvic { uint32_t ISER[1], ISPR[1]; };
struct display_test_flash { uint32_t ACR; };
extern struct display_test_rcc display_test_rcc;
extern struct display_test_nvic display_test_nvic;
extern uint32_t display_test_mask, display_test_ipsr, display_test_control;
struct display_test_gpio *display_test_gpio(void);
struct display_test_uart *display_test_uart(void);
struct display_test_flash *display_test_flash(void);
void display_test_icr(uint32_t bits);
void display_test_tdr(uint8_t byte);
#define RCC (&display_test_rcc)
#define NVIC (&display_test_nvic)
#define GPIOB display_test_gpio()
#define UART2 display_test_uart()
#define FLASH display_test_flash()
#define DISPLAY_TX_ICR(bits) display_test_icr(bits)
#define DISPLAY_TX_TDR(byte) display_test_tdr(byte)
#define __get_PRIMASK() display_test_mask
#define __get_IPSR() display_test_ipsr
#define __get_CONTROL() display_test_control
#define __DSB() ((void)0)
enum { UART2_IRQn=26, RCC_IOPEN_GPIOBEN=2, RCC_APB1EN_UART2EN=0x20000,
       RCC_AHBEN_FLASHEN=0x100, RCC_CSR1_RCHRDY=0x400,
       RCC_CSR1_PLLON=0x1000000, RCC_CSR1_PLLRDY=0x2000000,
       RCC_CSR1_RCHDIV=0x1800, FLASH_ACR_LATENCY=3,
       FLASH_ACR_LATENCY_2CLK=2, USART_CR1_UE=1, USART_CR1_TE=8,
       USART_CR1_PTS=0x200, USART_CR1_PEN=0x400, USART_CR1_WL=0x1000,
       USART_ISR_RXNE=0x20, USART_ISR_TC=0x40, USART_ISR_TXE=0x80,
       USART_ISR_TEACK=0x200000, USART_ISR_REACK=0x400000,
       USART_ICR_TCCF=0x40 };
#endif
