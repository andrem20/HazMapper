#ifndef INC_BATTERY_H_
#define INC_BATTERY_H_

/* Platform Includes -------------------------------------------------------------------------- */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "queue.h"

#include <stdint.h>

/* User Defined Macros -------------------------------------------------------------------- */
#define V_BATTERY 12
#define ADC_DMA_BUFFER_SIZE 5

/* User Defined Variables (External) ------------------------------------------------------ */
extern volatile uint32_t sum_samples;
extern volatile uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];
extern volatile float vbat;

extern osThreadId_t SensorTemp;
extern QueueHandle_t batteryQueue;
/* Defined Structures --------------------------------------------------------------------- */
typedef enum {
	BATTERY_OK = 0,
	BATTERY_LOW = 1
} Battery_state_t;

/* Functions ------------------------------------------------------------------------------ */
void Battery_Task(void* argument);

#endif /* INC_BATTERY_H_ */
