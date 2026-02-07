#ifndef INC_ESP_DRIVER_H_
#define INC_ESP_DRIVER_H_

/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include <stdint.h>
#include "cmsis_os2.h"
#include "semphr.h"
#include "usart.h"
/* Platform Includes END ------------------------------------------------------------------ */

/* MACROS Definition BEGIN ---------------------------------------------------------------- */
#define RX_BUFFER_SIZE 1024
/* MACROS Definition END ------------------------------------------------------------------ */

typedef enum{
	SLOW = 30,
	FAST = 100
}SPEED_MODE;

extern SPEED_MODE speed_mode;

/* ESP_processBuffer Task Handler */
extern osThreadId_t ESP_processBuffer;
extern SemaphoreHandle_t readInputbufferMutex;
extern QueueHandle_t on_off_Queue;
extern QueueHandle_t movementQueue;

extern UART_HandleTypeDef *esp_uart;

extern uint8_t  rx_buffer[RX_BUFFER_SIZE];
extern volatile uint16_t rx_write, rx_read, rx_count;
extern uint8_t  rx_byte;

void ESP_init(UART_HandleTypeDef *huart);
void ESP_processBufferTask(void *argument);

#endif /* INC_ESP_DRIVER_H_ */
