#ifndef W600_RADIO_TEST_MMIO_H
#define W600_RADIO_TEST_MMIO_H
#include <stdint.h>
struct radio_test_uart { uint32_t CR1,CR2,CR3,BRR,ISR; };
struct radio_test_gpio { uint32_t MODE,OTYPE,ODR,IDR; };
struct radio_test_rcc { uint32_t IOPEN,APB1EN; };
extern struct radio_test_rcc radio_test_rcc;
extern uint32_t radio_test_mask,radio_test_ipsr,radio_test_control;
struct radio_test_uart *radio_test_uart(void);
struct radio_test_gpio *radio_test_gpio(void);
uint8_t radio_test_rdr(void);
void radio_test_tdr(uint8_t byte);
void radio_test_icr(uint32_t bits);
void radio_test_wake(bool high);
#define RCC (&radio_test_rcc)
#define UART3 radio_test_uart()
#define GPIOA radio_test_gpio()
#define RADIO_RDR() radio_test_rdr()
#define RADIO_TDR(byte) radio_test_tdr(byte)
#define RADIO_ICR(bits) radio_test_icr(bits)
#define RADIO_WAKE(high) radio_test_wake(high)
#define __get_PRIMASK() radio_test_mask
#define __get_IPSR() radio_test_ipsr
#define __get_CONTROL() radio_test_control
#define __disable_irq() ((void)(radio_test_mask=1))
#define __set_PRIMASK(mask) ((void)(radio_test_mask=(mask)))
#define __DMB() ((void)0)
#define __DSB() ((void)0)
enum { UART3_4_IRQn=27, RCC_IOPEN_GPIOAEN=1,
       RCC_APB1EN_UART3EN=0x40000, USART_CR1_UE=1,USART_CR1_RE=4,
       USART_CR1_TE=8,USART_CR1_RXNEIE=0x20,USART_CR1_PEIE=0x100,
       USART_CR3_EIE=1, USART_ISR_RXNE=0x20,USART_ISR_TC=0x40,
       USART_ISR_TXE=0x80,USART_ICR_TCCF=0x40 };
#endif
