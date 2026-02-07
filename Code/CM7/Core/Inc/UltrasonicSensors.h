#ifndef INC_ULTRASSONICSENSOR_H_
#define INC_ULTRASSONICSENSOR_H_

/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include "cmsis_os2.h"
#include "semphr.h"
/* User Includes ------------------------------------------------------------------------------------------- */
#include "usart_support.h"
#include "DD_MotorControl.h"

#define MAX_SIZE_BUFFER 4096*2

#define SAMPLING_FREQ_US		10

// micro-seconds
#define START_PULSE_PERIOD_ON	1891
#define START_PULSE_PERIOD_OFF	999
#define SMALL_PULSE	101
#define BIG_PULSE	203

//UNIT: TICKS
#define DISCARD_TICKS 20
#define DISCARD_FRAME 300
#define HISTERISES 5

#define NUMBER_OF_BYTES	8
#define NUM_SENSORS 4

typedef enum { E = 0, F, G, H, A, B, C, D } sensors;
typedef enum { IDLE = 0, START, SYNC_H, SYNC_L, FRAME } states_US;

typedef struct{
	sensors Sensor;
	direction dir;
	char posicao [9];
} SensorCar;

/* Ultrassonic Sensors Task Handler */
extern osThreadId_t ParkingSensors;
extern SemaphoreHandle_t obstacleMutex;
extern SemaphoreHandle_t readUSbufferMutex;

extern volatile uint8_t buffer[MAX_SIZE_BUFFER];
extern volatile uint8_t *WritePointer;
extern volatile uint8_t *ReadPointer;

void ParkingSensors_Task (void *argument);

void init_US(void);

#endif /* INC_ULTRASSONICSENSOR_H_ */
