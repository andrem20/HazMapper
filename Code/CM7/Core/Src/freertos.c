/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "usart_support.h"
#include "esp_driver.h"
#include "DD_MotorControl.h"
#include "UltrasonicSensors.h"
#include "battery.h"
#include "camera.h"
#include "sensors_i2c.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osThreadId_t ESP_processBuffer;
const osThreadAttr_t ESP_processBuffer_attributes = {
  .name = "ESP_processBuffer",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t uartTransmit;
const osThreadAttr_t uartTransmit_attributes = {
  .name = "uartTransmit",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Sampling_and_PID;
const osThreadAttr_t Sampling_and_PID_attributes = {
  .name = "Sampling_and_PID",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t ParkingSensors;
const osThreadAttr_t ParkingSensors_attributes = {
  .name = "ParkingSensors",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t turn_on_off;
const osThreadAttr_t turn_on_off_attributes = {
  .name = "turn_on_off",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t MovementControl;
const osThreadAttr_t MovementControl_attributes = {
  .name = "MovementControl",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t HDC1081_Sensor;
const osThreadAttr_t HDC1081_Sensor_attributes = {
  .name = "SensorTemp",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t BatteryRead;
const osThreadAttr_t BatteryRead_attributes = {
  .name = "BatteryRead",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t CCS811_Sensor;
const osThreadAttr_t CCS811_Sensor_attributes = {
  .name = "SensorGas",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

osThreadId_t Camera;
const osThreadAttr_t Camera_attributes = {
  .name = "Camera",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};
osThreadId_t LCD;
const osThreadAttr_t LCD_attributes = {
  .name = "LCD",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

SemaphoreHandle_t i2cmutex  = NULL;
SemaphoreHandle_t binSyncUART  = NULL;
QueueHandle_t on_off_Queue = NULL;
QueueHandle_t movementQueue = NULL;
QueueHandle_t batteryQueue = NULL;
QueueHandle_t HDC1081_TempRawDataQueue = NULL;
QueueHandle_t CCS811_DataQueue = NULL;
QueueHandle_t LCD_DataQueue = NULL;
/* USER CODE END Variables */
/* Definitions for nothing */
osThreadId_t nothingHandle;
const osThreadAttr_t nothing_attributes = {
  .name = "nothing",
  .stack_size = 128 * 4,
  .priority = (osPriority_t) osPriorityLow,
};
/* Definitions for uartMutex */
osMutexId_t uartMutexHandle;
const osMutexAttr_t uartMutex_attributes = {
  .name = "uartMutex"
};

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void nothingTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */
  /* Create the mutex(es) */
  /* creation of uartMutex */
  uartMutexHandle = osMutexNew(&uartMutex_attributes);

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */

  i2cmutex = xSemaphoreCreateMutex();
  configASSERT(i2cmutex  != NULL);
  binSyncUART = xSemaphoreCreateBinary();
  configASSERT(binSyncUART  != NULL);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
	on_off_Queue = xQueueCreate(20, sizeof(uint8_t));
	configASSERT(on_off_Queue != NULL);
	movementQueue = xQueueCreate(4, sizeof(MovementCommand));
	configASSERT(movementQueue != NULL);
	batteryQueue = xQueueCreate(4, sizeof(uint16_t));
	configASSERT(batteryQueue != NULL);
	HDC1081_TempRawDataQueue = xQueueCreate(10, sizeof(uint16_t));
	configASSERT(HDC1081_TempRawDataQueue != NULL);
	CCS811_DataQueue = xQueueCreate(10, sizeof(CCS811_received));
	configASSERT(CCS811_DataQueue != NULL);

  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of nothing */
  nothingHandle = osThreadNew(nothingTask, NULL, &nothing_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  ESP_processBuffer = osThreadNew (ESP_processBufferTask, NULL, &ESP_processBuffer_attributes);
  uartTransmit = osThreadNew (uartTransmitTask, NULL, &uartTransmit_attributes);
  Sampling_and_PID = osThreadNew (Sampling_and_PID_Task, NULL, &Sampling_and_PID_attributes);
  ParkingSensors = osThreadNew (ParkingSensors_Task, NULL, &ParkingSensors_attributes);
  turn_on_off = osThreadNew (turn_on_off_Task, NULL, &turn_on_off_attributes);
  MovementControl = osThreadNew (MovementControl_Task, NULL, &MovementControl_attributes);
  HDC1081_Sensor = osThreadNew (HDC1081_SensorTask, NULL, &HDC1081_Sensor_attributes);
  BatteryRead = osThreadNew (Battery_Task, NULL, &BatteryRead_attributes);
  CCS811_Sensor = osThreadNew (CCS811_SensorTask, NULL, &CCS811_Sensor_attributes);
#ifdef USE_CAMERA
  Camera = osThreadNew (Camera_Task, NULL, &Camera_attributes);
#endif
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_nothingTask */
/**
  * @brief  Function implementing the nothing thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_nothingTask */
void nothingTask(void *argument)
{
  /* USER CODE BEGIN nothingTask */
  /* Infinite loop */
  for(;;)
  {
    osThreadYield();
  }
  /* USER CODE END nothingTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

