/* Platform Includes ---------------------------------------------------------------------- */
#include "i2c.h"
#include "stdio.h"
/* User Includes -------------------------------------------------------------------------- */
#include "sensors_i2c.h"
#include "usart_support.h"

/* MACROS definition ---------------------------------------------------------------------  */
/*CCS811*/
#define CCS811_ADDR (0x5B << 1)

#define CCS811_APP_START_REGISTER 	0xF4		// Config Bootloader START
#define CCS811_MEAS_REGISTER 		0x01
#define CCS811_DATA_REGISTER 		0x02
#define CCS811_MEAS_MODE 			(1<<4)		// 1s/1s
#define CCS811_TEMP_APP_START_READY_MS 		1000

/* User Defined Variables ---------------------------------------------------------------- */
sensor_state_t CCS811_state;
uint8_t CCS811_rx_buff[8];

/* Internally used Functions in Temperature Sensor BEGIN */
static void CCS811_Sensorinit(void);
static void CCS811_SensorStart_Read(void);
static void CCS811_SensorRX_Process(void);
/* Internally used Functions in Temperature Sensor END */

/* Functions ----------------------------------------------------------------------------- */
static void CCS811_Sensorinit(void) {
	CCS811_state = SENSOR_STATE_IDLE;
	xSemaphoreTake(i2cmutex, portMAX_DELAY);
	/* Config Write Action */
	HAL_GPIO_WritePin(GPIOF, GPIO_PIN_6, GPIO_PIN_RESET);
	uint8_t reg = CCS811_APP_START_REGISTER;
	HAL_I2C_Master_Transmit(&hi2c1, CCS811_ADDR, &reg, 1, 1000);

	vTaskDelay(pdMS_TO_TICKS(CCS811_TEMP_APP_START_READY_MS));

	uint8_t mode = CCS811_MEAS_MODE;
	HAL_I2C_Mem_Write(&hi2c1, CCS811_ADDR, CCS811_MEAS_REGISTER, 1, &mode, 1, 1000); //modo de 1 em 1 segundo a leitura
	xSemaphoreGive(i2cmutex);
}

static void CCS811_SensorStart_Read(void) {
	idSensorI2C = CCS811;
	xSemaphoreTake(i2cmutex, portMAX_DELAY);
	uint8_t reg = CCS811_DATA_REGISTER;
	uint8_t ret = HAL_I2C_Master_Transmit_IT(&hi2c1, CCS811_ADDR, &reg, 1);
	if(ret != HAL_OK)
		CCS811_state = SENSOR_STATE_IDLE;
	xSemaphoreGive(i2cmutex);
}


static void CCS811_SensorRX_Process(void) {
	idSensorI2C = CCS811;
	xSemaphoreTake(i2cmutex, portMAX_DELAY);
	uint8_t ret = HAL_I2C_Master_Receive_IT(&hi2c1, CCS811_ADDR, CCS811_rx_buff, 8);
	if(ret != HAL_OK)
		CCS811_state = SENSOR_STATE_IDLE;
	xSemaphoreGive(i2cmutex);
}

/* FSM ----------------------------------------------------------------------------------- */
static void CCS811_SensorFSM(TickType_t* xLastWakeTime) {
	switch (CCS811_state) {
	case SENSOR_STATE_IDLE:
		/* Waits in Blocked State for a interval of time: precision not required */
		vTaskDelayUntil(xLastWakeTime, pdMS_TO_TICKS(SENSOR_SAMPLING_TIME_MS));
		CCS811_SensorStart_Read();
		CCS811_state = SENSOR_STATE_TEMP_TX;
		break;
	case SENSOR_STATE_TEMP_TX:
		/* Waits in Blocked State for Tx callback */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		CCS811_state = SENSOR_STATE_TEMP_RX_PENDING;
		break;
	case SENSOR_STATE_TEMP_RX_PENDING:
		CCS811_SensorRX_Process();
		CCS811_state = SENSOR_STATE_TEMP_RX;
		break;
	case SENSOR_STATE_TEMP_RX:
		/* Waits in Blocked State for Rx callback Data*/
		CCS811_received data;
		xQueueReceive(CCS811_DataQueue, &data, portMAX_DELAY);
		printf("C02: %d\n", data.CCS811_eCO2);
		printf("TVOC: %d\n",  data.CCS811_TVOC);
		CCS811_state = SENSOR_STATE_IDLE;
		break;
	default:
		CCS811_state = SENSOR_STATE_IDLE;
	}
}

/* TASKS ----------------------------------------------------------------------------------------------- */
void CCS811_SensorTask (void* argument){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	CCS811_Sensorinit();
	vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(SENSOR_SAMPLING_TIME_MS/2));
	for(;;)
		CCS811_SensorFSM(&xLastWakeTime);
}
