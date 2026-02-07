/* Platform Includes ---------------------------------------------------------------------- */
#include "sensors_i2c.h"
#include "i2c.h"
/* User Includes -------------------------------------------------------------------------- */
#include "usart_support.h"

/* MACROS definition ---------------------------------------------------------------------  */
/*HDC1081*/
#define HDC1081_ADDR (0x40 << 1)

#define HDC1081_TEMP_REGISTER 	0x00
#define HDC1081_HUM_REGISTER 	0x01
#define HDC1081_CONFIG_REGISTER 0x02

#define HDC1081_TEMP_RES_14BIT 	((0<<10))
#define HDC1081_HUM_RES_14BIT 	((0 << 9)|(0<<8))

#define HDC1081_TEMP_CONV_MS 15U
/* User Defined Variables ---------------------------------------------------------------- */
uint8_t HDC1081_rx_buff[2];
volatile sensor_state_t HDC1081_state = SENSOR_STATE_IDLE;

static volatile float sensor_temperature = 0.0f;

/* Internally used Functions in Temperature Sensor BEGIN */
static void	HDC1081_SensorInit(void);
static void HDC1081_SensorStart_Temperature(void);
static void HDC1081_SensorRX_Process(void);
/* Internally used Functions in Temperature Sensor END */

/* Functions ----------------------------------------------------------------------------- */
static void HDC1081_SensorInit(void){
	HDC1081_state = SENSOR_STATE_IDLE;
	sensor_temperature = 0.0f;

	uint8_t config_device[3];
	config_device[0] = HDC1081_CONFIG_REGISTER;
	uint16_t config = (1 << 12);
	config_device[1] = config >> 8;
	config_device[2] = config & 0xFF;

	xSemaphoreTake(i2cmutex, portMAX_DELAY);
	HAL_I2C_Master_Transmit(&hi2c1, HDC1081_ADDR, config_device, 3, 1500);
	xSemaphoreGive(i2cmutex);

}

static void HDC1081_SensorStart_Temperature(void){
	idSensorI2C = HDC1081;
	xSemaphoreTake(i2cmutex, portMAX_DELAY);
	uint8_t reg = HDC1081_TEMP_REGISTER;
	uint8_t ret = HAL_I2C_Master_Transmit_IT(&hi2c1, HDC1081_ADDR, &reg, 1);
	xSemaphoreGive(i2cmutex);
	if(ret != HAL_OK)
		HDC1081_state = SENSOR_STATE_IDLE;
}

static void HDC1081_SensorRX_Process(void){
	idSensorI2C = HDC1081;
	xSemaphoreTake(i2cmutex, portMAX_DELAY);
	uint8_t ret = HAL_I2C_Master_Receive_IT(&hi2c1, HDC1081_ADDR, HDC1081_rx_buff, 2);
	xSemaphoreGive(i2cmutex);
	if(ret != HAL_OK)
		HDC1081_state = SENSOR_STATE_IDLE;
}

/* FSM ----------------------------------------------------------------------------------- */
void HDC1081_SensorTemperature_FSM(TickType_t* xLastWakeTime){
	switch(HDC1081_state){
	case SENSOR_STATE_IDLE:
		/* Waits in Blocked State for a interval of time: precision not required */
		vTaskDelayUntil(xLastWakeTime, pdMS_TO_TICKS(SENSOR_SAMPLING_TIME_MS));
		HDC1081_SensorStart_Temperature();
		HDC1081_state = SENSOR_STATE_TEMP_TX;
		break;
	case SENSOR_STATE_TEMP_TX:
		/* Waits in Blocked State for Tx callback */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
		HDC1081_state = SENSOR_STATE_TEMP_RX_PENDING;
		break;
	case SENSOR_STATE_TEMP_RX_PENDING:
		/* Waits in Blocked State for conversion time  */
		vTaskDelay(pdMS_TO_TICKS(HDC1081_TEMP_CONV_MS));
		HDC1081_SensorRX_Process();
		HDC1081_state = SENSOR_STATE_TEMP_RX;
		break;
	case SENSOR_STATE_TEMP_RX:
		/* Waits in Blocked State for Rx callback Data */
		uint16_t raw;
		xQueueReceive(HDC1081_TempRawDataQueue, &raw, portMAX_DELAY);
		float temp = ((raw / 65536.0f) * 165.0f) - 40.0f;
		printf("Temperature: %.2f\n ", temp);
		HDC1081_state = SENSOR_STATE_IDLE;
		break;
	default:
		HDC1081_state = SENSOR_STATE_IDLE;
	}
}

/* TASKS ----------------------------------------------------------------------------------------------- */
void HDC1081_SensorTask (void* argument){
	TickType_t xLastWakeTime = xTaskGetTickCount();
	HDC1081_SensorInit();
	for(;;)
		HDC1081_SensorTemperature_FSM(&xLastWakeTime);
}
