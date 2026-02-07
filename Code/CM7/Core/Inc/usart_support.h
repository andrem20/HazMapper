#ifndef INC_USART_SUPPORT_H_
#define INC_USART_SUPPORT_H_

/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include "cmsis_os2.h"
#include "stdint.h"

/* MACRO definition BEGIN ----------------------------------------------------------------- */
#define CARRIAGE_RETURN '\r'
// Transmission Buffer
#define TX_BUFFER_SIZE 1024

/* Transmission Task Handler */
extern osThreadId_t uartTransmit;

void uartTransmitTask(void *argument);

int print_message_to_UART(uint8_t* TxPTR, uint16_t Msize);

int printf(const char *format, ...);

#endif /* INC_USART_SUPPORT_H_ */
