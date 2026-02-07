/* Platform Includes ---------------------------------------------------------------------- */
#include "adc.h"
#include "gpio.h"
#include "tim.h"
/* User Includes -------------------------------------------------------------------------- */
#include "battery.h"
#include "usart_support.h"
/* User Defined Macros -------------------------------------------------------------------- */
#define R1 50000.0f
#define R2 10000.0f
#define V_REF 3.3f
#define V_BATERY_LOW_LEVEL 2
#define BATTERY_SAMPLING_TIME_MS 10e3
#define ADC_MAX 4095U

/* User Defined Variables (External) ------------------------------------------------------ */
uint8_t battery_level;
/* Functions  ----------------------------------------------------------------------------- */
void Battery_init(void){
	sum_samples = 0;
}

void Battery_start_ADC_conversion(void){
	HAL_ADC_Start_DMA(&hadc3, (uint32_t*)adc_dma_buffer, ADC_DMA_BUFFER_SIZE);
}
/*
static float Battery_convertRawValue(uint16_t raw_value){
	// convertemos o valor do ADC para V
	float v_adc = (float)raw_value * VREF / ADC_MAX;
	return v_adc;
	//return (uint16_t)(v_adc*(R1 + R2)/R2);
}

void Battery_process(uint16_t* raw_value){
	//float val = battery_raw_value;
	volatile float vbat = Battery_convertRawValue(&raw_value);

	if(vbat >= V_BATERY_LOW_LEVEL){
		HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
	}
	else{
		HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
		HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
	}
}
*/
/* TASKS ---------------------------------------------------------------------------------- */

void Battery_Task(void* argument){
	uint16_t raw_value;
	Battery_init ();
	TickType_t xLastWakeTime = xTaskGetTickCount();;
	const TickType_t xPeriod = pdMS_TO_TICKS(BATTERY_SAMPLING_TIME_MS/4);
	uint8_t n= 0;
	for(;;){
		Battery_start_ADC_conversion();

		/* Waits in Blocked State until Reception of data Sent from ISR */
		xQueueReceive(batteryQueue, &raw_value, portMAX_DELAY );

		/* Convert Raw value to Volts */
		float vbat = (float)raw_value * V_REF / ADC_MAX;

		/* Process Data and Display it to the user through LED toogle */
		if(vbat >= V_BATERY_LOW_LEVEL){
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_RESET);
		}
		else{
			HAL_GPIO_WritePin(RED_LED_GPIO_Port, RED_LED_Pin, GPIO_PIN_SET);
			HAL_GPIO_WritePin(ORANGE_LED_GPIO_Port, ORANGE_LED_Pin, GPIO_PIN_RESET);
		}
		printf("it %d: ticks: %ld vbat: %f\n", n, xLastWakeTime, vbat);
		vTaskDelayUntil(&xLastWakeTime, xPeriod);
		n = (n+1)& sizeof(uint8_t);
	}
}

