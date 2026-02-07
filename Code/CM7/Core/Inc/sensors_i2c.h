#ifndef INC_SENSORS_I2C_H_
#define INC_SENSORS_I2C_H_

/*
 * I2C Interface Sensors
 * 	- HDC1081: Temperature and Humidity
 * 	- CCS811: CO2 and Gas
 */

/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include "cmsis_os2.h"
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "queue.h"
/* MACROS --------------------------------------------------------------------------------- */
/*General*/
#define SENSOR_SAMPLING_TIME_MS 2000
/* Defined Structures --------------------------------------------------------------------- */
typedef enum{
	SENSOR_STATE_IDLE = 0,
	SENSOR_STATE_TEMP_TX,
	SENSOR_STATE_TEMP_RX_PENDING,
	SENSOR_STATE_TEMP_RX
} sensor_state_t;
typedef enum{
	HDC1081 = 0,
	CCS811
}SensorsI2C;
typedef struct{
	uint16_t CCS811_eCO2;
	uint16_t CCS811_TVOC;
}CCS811_received;	//received format for CCS811 DATA

/* External Variables --------------------------------------------------------------------- */
extern uint8_t HDC1081_rx_buff[2];
extern uint8_t CCS811_rx_buff[8];
extern SensorsI2C idSensorI2C;
/* SensorTemp Task Handler */
extern osThreadId_t HDC1081_Sensor;
extern osThreadId_t CCS811_Sensor;
extern osThreadId_t LCD;

extern SemaphoreHandle_t i2cmutex;
extern QueueHandle_t HDC1081_TempRawDataQueue;
extern QueueHandle_t CCS811_DataQueue;
extern QueueHandle_t LCD_DataQueue;
/* Functions ------------------------------------------------------------------------------ */
void HDC1081_SensorTask (void* argument);
void CCS811_SensorTask (void* argument);
void LCD_Task (void* argument);
#endif /* INC_SENSORS_I2C_H_ */
