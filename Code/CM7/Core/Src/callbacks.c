/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include <sensors_i2c.h>
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "tim.h"
#include "gpio.h"
#include "adc.h"
#include "i2c.h"
/* Platform Includes END ------------------------------------------------------------------ */

/* User-Created Header files include BEGIN ------------------------------------------------ */
#include "usart_support.h"

#include "DD_MotorControl.h"
#include "battery.h"
#include "UltrasonicSensors.h"
#include "camera.h"
/* User-Created Header files include END -------------------------------------------------- */

/* Initialize extern variables in User-Created Header files included BEGIN ---------------- */
motors_id motors [4];
TIM_HandleTypeDef *htim_sampling;
TIM_TypeDef* TIM_sampling;

GPIO_TypeDef *ON_OFF_Port;
uint16_t ON_OFF_Pin;

volatile uint32_t sum_samples;
volatile uint16_t adc_dma_buffer[ADC_DMA_BUFFER_SIZE];

volatile uint8_t buffer[MAX_SIZE_BUFFER];
volatile uint8_t *WritePointer = buffer;
volatile uint8_t *ReadPointer= buffer;

SensorsI2C idSensorI2C;
/* Initialize extern variables in User-Created Header files included END ------------------ */

uint8_t allow_extiB = 1;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if(htim->Instance == TIM_sampling){				// Sampling for DD_MotorControl
		if (htim == htim_sampling)
			vTaskNotifyGiveFromISR(Sampling_and_PID, &xHigherPriorityTaskWoken);
	}
	else if(htim->Instance == TIM17){
		if((GPIOE->IDR & GPIO_PIN_3) != (uint32_t) GPIO_PIN_RESET)
			*WritePointer = 1;
		else
			*WritePointer = 0;

		WritePointer ++;
		WritePointer = buffer + ((WritePointer - buffer) & (MAX_SIZE_BUFFER - 1));
	}

	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (hi2c->Instance == hi2c1.Instance){
		switch(idSensorI2C){
		case HDC1081: vTaskNotifyGiveFromISR(HDC1081_Sensor, &xHigherPriorityTaskWoken); break;
		case CCS811: vTaskNotifyGiveFromISR(CCS811_Sensor, &xHigherPriorityTaskWoken); break;
		default: break;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (hi2c->Instance == hi2c1.Instance){
		switch(idSensorI2C){
		case HDC1081:
			uint16_t raw = (HDC1081_rx_buff[0] << 8) | HDC1081_rx_buff[1];
			xQueueSendFromISR(HDC1081_TempRawDataQueue, &raw, &xHigherPriorityTaskWoken );
			break;
		case CCS811:
			CCS811_received data;
			data.CCS811_eCO2 = (CCS811_rx_buff[0] << 8) | CCS811_rx_buff[1];
			data.CCS811_TVOC= (CCS811_rx_buff[2] << 8) | CCS811_rx_buff[3];
			xQueueSendFromISR(CCS811_DataQueue, &data, &xHigherPriorityTaskWoken );
			break;
		}
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}


void HAL_DCMI_FrameEventCallback(DCMI_HandleTypeDef *hdcmi) {
//	taskENTER_CRITICAL();
	// Só copia buffer para envio se UART livre - Semáforo Cheio -  e buffer pendente vazio
	if (uxSemaphoreGetCount(binSyncUART) && uart_pending_ptr == NULL) {
		uart_pending_ptr = current_dcmi_ptr;

		if (current_dcmi_ptr == frame_buffer_A)
			current_dcmi_ptr = frame_buffer_B;
		else
			current_dcmi_ptr = frame_buffer_A;

		xTaskNotify(Camera, BIT_WAIT_FRAME, eSetValueWithoutOverwrite);
	}
//	taskEXIT_CRITICAL();
//	HAL_GPIO_TogglePin(GPIOB, GPIO_PIN_0);
}
void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc){
//void HAL_DMA_ConvCpltCallback(DMA_HandleTypeDef *hdma){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	if(hadc -> Instance == ADC3){
	// if(hdma->Instance == DMA1_Stream0){
		//HAL_GPIO_TogglePin(GREEN_LED_GPIO_Port, GREEN_LED_Pin);
		sum_samples = 0;
		//sample sum
		for(int i = 1; i < ADC_DMA_BUFFER_SIZE; i++)
			sum_samples += adc_dma_buffer[i];

		uint16_t battery_raw_value = sum_samples/(ADC_DMA_BUFFER_SIZE-1);
		xQueueSendFromISR(batteryQueue, &battery_raw_value, &xHigherPriorityTaskWoken);
	}
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

