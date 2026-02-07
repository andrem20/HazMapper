
/* ESP_DRIVER
 *
 * 	bridge for communication between STM32 and ESP
 * 	Simple parser for 2-byte frames: [TYPE][VALUE]
 * 	Implements a circular buffer (rx_buffer) with indices rx_write and rx_read.
 * 	Received bytes (one by one) are stored via interrupt, and frames are parsed in the main loop.
 */

/* Platform Includes BEGIN -------------------------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include <stdbool.h>
/* Platform Includes END ---------------------------------------------------------------------------------- */

/* User Includes BEGIN ------------------------------------------------------------------------------------ */
#include "esp_driver.h"
#include "DD_MotorControl.h"
#include "movement_control.h"
#include "UltrasonicSensors.h"
/* User Includes END -------------------------------------------------------------------------------------- */



/* MACROS which define reception "protocol" BEGIN --------------------------------------------------------- */
#define FRAME_SIZE 2 // type + value
#define MAX_TYPE 3	// maximum number of commands allowed
#define MAX_PWM_VALUE 255
#define MAX_NUM_DIRECTIONS 10
#define MAX_NUM_SPEED_MODE 2

#define ESP_CMD_ON_OFF 0x00
#define ESP_CMD_DIRECTION 0x01
#define ESP_CMD_SPEED 0x02
#define ESP_CMD_SPEED_MODE 0x03
/* MACROS which define reception "protocol" END ----------------------------------------------------------- */

UART_HandleTypeDef *esp_uart = NULL;

uint8_t  rx_buffer[RX_BUFFER_SIZE];
volatile uint16_t rx_write = 0, rx_read = 0, rx_count = 0;
uint8_t  rx_byte;

/*
 * void ESP_init(UART_HandleTypeDef *huart)
 * 	- Initialize the ESP UART driver.
 * 	- param: huart Pointer to the UART handle used for ESP32 communication (UART2)
 */
void ESP_init(UART_HandleTypeDef *huart){
	esp_uart = huart;
	rx_write = rx_read = rx_count = 0;
	HAL_UART_Receive_IT(esp_uart, &rx_buffer[rx_write], 1);
}

/*
 * static bool is_valid_frame(uint8_t type, uint8_t value)
 * 	- Validates type and value range before processing
 */
static bool is_valid_frame(uint8_t type, uint8_t value) {
	if(type > MAX_TYPE) return false;
	switch (type) {
	case ESP_CMD_ON_OFF:
		return (value == 0 || value == 1);  		// ON/OFF must be 0 or 1
	case ESP_CMD_DIRECTION:
		return (value <= MAX_NUM_DIRECTIONS);	// max direction index
	case ESP_CMD_SPEED:
		return (value <= MAX_PWM_VALUE);  		// any uint8_t is fine for speed
	case ESP_CMD_SPEED_MODE:
		return (value <= MAX_NUM_SPEED_MODE);
	default:   return false;
	}
}
/*
 * static void handle_frame( uint8_t type, uint8_t value)
 * 	- split frame and parse command;
 */
static void handle_frame( uint8_t type, uint8_t value){
	switch (type){
	case ESP_CMD_ON_OFF:
		//turn_on_off_f(value);
		BaseType_t result = xQueueSend(on_off_Queue, &value, pdMS_TO_TICKS(20));
		if (result != pdPASS) {
			uint8_t a = 0;
		}
		break;
	case ESP_CMD_DIRECTION:
		//update_movement((direction)value);
		configASSERT( movementQueue != NULL );
		MovementCommand cmd = {.dir = (direction)value, .force_stop = false};
		BaseType_t result1 = xQueueSend(movementQueue, &cmd, 0);
		if (result1 != pdPASS) {
			uint8_t a = 0;
		}
		break;
	case ESP_CMD_SPEED:
		// 0 -255 convert to 0- 100
		uint8_t speed = (value * (uint8_t)speed_mode)/255;
		update_speed((int)speed);
		break;
	case ESP_CMD_SPEED_MODE:
		if(!value)speed_mode = SLOW;
		else speed_mode = FAST;
	default:
		break;
	}
}



/* TASKS ----------------------------------------------------------------------------------------------- */
/* ESP_processBuffer AKA Reception Task */

/*
 * void ESP_processBuffer(void *argument)
 * 	- Triggered by task notification sent from interrupt
 * 	- Process any pending frames in the reception buffer.
 *	- Should be called regularly in the main loop.
 */
void ESP_processBufferTask(void *argument){


	for(;;){
		/* Waits Notification in Blocked State */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		//printf("trigger\n");
		/* Check if there was actually something received */
		if ((rx_write - rx_read + RX_BUFFER_SIZE) % RX_BUFFER_SIZE >= FRAME_SIZE){
			uint8_t type = rx_buffer[rx_read];
			uint8_t value = rx_buffer[(rx_read + 1) % RX_BUFFER_SIZE];

			if(is_valid_frame(type, value))
				handle_frame(type, value);
			else
				printf("[ESP] Discard frame: type=0x%02X val=0x%02X\n", type, value);

			rx_read = (rx_read + FRAME_SIZE) % RX_BUFFER_SIZE;
			rx_count -= FRAME_SIZE;
		}
		vTaskDelay(pdMS_TO_TICKS(10));
	}
}




