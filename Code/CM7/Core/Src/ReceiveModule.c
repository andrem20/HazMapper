/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "usart.h"
/* Platform Includes END ------------------------------------------------------------------ */

/* User Includes BEGIN -------------------------------------------------------------------- */
#include <usart_support.h>
#include "main.h"

#include "esp_driver.h"
#include "DD_MotorControl.h"
/* User Includes END ---------------------------------------------------------------------- */


/* Receive Callback Handler*/
// ISR calls HAL_UART_RxCpltCallback when reception has ended
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(huart == esp_uart){
		uint16_t next = (rx_write + 1) % RX_BUFFER_SIZE;
		if((rx_write + 1) % RX_BUFFER_SIZE != rx_read)
			rx_write = (next != rx_read)? next : rx_write;

		HAL_UART_Receive_IT(esp_uart, &rx_buffer[rx_write], 1);

		vTaskNotifyGiveFromISR(ESP_processBuffer, &xHigherPriorityTaskWoken);

	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}




