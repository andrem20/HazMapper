/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "task.h"
#include <stdio.h>
#include "main.h"
#include "usart.h"

/* Platform Includes END ------------------------------------------------------------------ */

/* User Includes BEGIN -------------------------------------------------------------------- */
#include <usart_support.h>
/* User Includes END ---------------------------------------------------------------------- */

/*
 * This is the Transmission Module
 * It is based on assynchronous transmission
 * Implements:
 * 	- Circular array
 * 	- Write/Read Index
 * 	- Callback triggered by interuption of end of transmission
 * */

char TxBuffer[TX_BUFFER_SIZE];
volatile uint16_t TxWriteIdx = 0;  // Write index
volatile uint16_t TxReadIdx = 0;   // Read index

/* TASKS ---------------------------------------------------------------------------------- */
void uartTransmitTask(void *argument){ 	// previous UART_Send(void)

	for(;;){
		/* Waits Notification in Blocked State */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

	/* Check if data is actually available in the buffer */
		uint16_t number_characters;

		if (TxWriteIdx >= TxReadIdx) {
			number_characters = TxWriteIdx - TxReadIdx;  // Normal case
		} else {
			number_characters = TX_BUFFER_SIZE - (TxReadIdx);  // Wrap-around case
		}
		//is there more data to send? is previous transmission over?
		if (number_characters > 0) {
			HAL_UART_Transmit_IT(&huart2, (uint8_t*)&TxBuffer[TxReadIdx], number_characters);
			// Move read index forward: assures there are no indirections by using % TX_BUFFERSIZE
			TxReadIdx = (TxReadIdx + number_characters) % TX_BUFFER_SIZE;
		}
		osThreadYield();
	}
}

/* Functions ----------------------------------------------------------------------------- */
int print_message_to_UART(uint8_t* TxPTR, uint16_t Msize) {
    for (uint16_t i = 0; i < Msize; i++) {
        uint16_t nextWriteIdx = (TxWriteIdx + 1) % TX_BUFFER_SIZE;

        // Buffer overflow protection
        // assures that in wrap-around case both indexes do not overlap
        if (nextWriteIdx == TxReadIdx) {
            return -1;
        }

        TxBuffer[TxWriteIdx] = *TxPTR++;
        TxWriteIdx = nextWriteIdx;
    }

    xTaskNotifyGive(uartTransmit); 	 // Try sending the new data

    return 0;
}

int _write(int file, char *ptr, int len) {
	print_message_to_UART((uint8_t*)ptr, len);
	return len;
}

/* Transmit Callback */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
#ifndef USE_CAMERA
	vTaskNotifyGiveFromISR(uartTransmit, &xHigherPriorityTaskWoken);
#else
	xSemaphoreGiveFromISR(binSyncUART);
#endif
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

