#ifndef INC_DD_MOTORCONTROL_H_
#define INC_DD_MOTORCONTROL_H_

/* Platform Includes BEGIN ----------------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "cmsis_os2.h"
#include "semphr.h"
#include "stdio.h"
#include <stdlib.h>

#include "tim.h"
#include "gpio.h"
/* Platform Includes END ------------------------------------------------------------------------- */

#define USE_PC_CONTROL

#define NUM_COMMANDS 11
#define NUM_DIRECTIONS NUM_COMMANDS -1
#define ms 1e-3

typedef enum {FW = 0, FW_RIGHT, RIGHT, BW_RIGHT, BW, BW_LEFT, LEFT, FW_LEFT, STOP, R_RIGHT, R_LEFT} direction;

typedef enum {OFF = 0, FORWARD, REVERSE} motors_state;

typedef enum {M1 = 0, M2, M3, M4} motor_num;

typedef struct{
	GPIO_TypeDef* GPIO_Port;
	uint16_t Pin;
}PinRef;

typedef struct{
	float Kp;
	float Ki;
	float Kd_h;
	float a;
}PID_parameters;

typedef struct{
	volatile float PWM_value;
	volatile int16_t encoder_current_position,
			encoder_last_position, pulse_diff, encoder_position;
	volatile float speed_rad, speed_rpm,
			position_rad, position_degrees;		/* Variables used indirectly for PID Speed Algorithm */
	volatile float erro_previous, integral; 	/* Variables used directly for PID Speed Algorithm */
}motor_var;

typedef struct{
	TIM_HandleTypeDef *htim_pwm;		/* htim used to generate PWM -------------------------- */
	uint32_t HTIM_PWM_CHANNEL;
}motor_tim_pwm;

typedef struct{
	TIM_HandleTypeDef *htim_encoder;
	uint32_t SENSOR_A_CHANNEL;		/* Encoder A Sensor */
	uint32_t SENSOR_B_CHANNEL;		/* Encoder B Sensor */
}motor_tim_encoder;

typedef struct{
	motors_state state;
	PinRef FORWARD_DRIVE;	/*IN1 IN3*/
	PinRef REVERSE_DRIVE;	/*IN2 IN4*/
	PID_parameters PID;		/* Motor's PID parameters */
	motor_var var;			/* Motor's variables used for sampling*/
	motor_tim_pwm pTim;
	motor_tim_encoder eTim;
}motors_id;

typedef struct{
	direction dir;
	motors_state M_1;
	motors_state M_2;
	motors_state M_3;
	motors_state M_4;
}commands;

/* Used only for queue */
typedef struct{
	direction dir;
	uint8_t force_stop;
}MovementCommand;
/* Sampling_and_PID Task Handler */
extern osThreadId_t Sampling_and_PID;
/* Movement Control Task Handler */
extern osThreadId_t MovementControl;

extern SemaphoreHandle_t updateMutex;
extern QueueHandle_t movementQueue;
/* DEFINE GENERAL PINS AND PORTS BEGIN ------------------------------------------------------------ */
extern GPIO_TypeDef *ON_OFF_Port;
extern uint16_t ON_OFF_Pin;
/* DEFINE GENERAL PINS AND PORTS END -------------------------------------------------------------- */

extern motors_id motors [4];				/* array of motors ------------------------------------ */
extern volatile float setpoint;

extern TIM_HandleTypeDef *htim_sampling; 	/* htim used for sampling ----------------------------- */
extern TIM_TypeDef* TIM_sampling;			/* specify TIM used for sampling ---------------------- */

extern uint8_t ON_OFF_value;				/* 1--> robot is on; 0--> robot is off ---------------- */

extern commands cmd_list [NUM_COMMANDS];

/* DECLARATION OF FUNCTIONS USED TO CONFIGURE PARAMETERS BEGIN (USER MUST USE) -------------------- */
void set_On_Off (GPIO_TypeDef *port, uint16_t pin);
void set_Ports (motor_num M, GPIO_TypeDef *forward, GPIO_TypeDef *reverse);
void set_Pins (motor_num M, uint16_t forward, uint16_t reverse);
void set_PWM_parameters (motor_num M, TIM_HandleTypeDef *htim, uint32_t TIM_CHANNEL, uint16_t ppr, uint16_t freq);
void set_sampling_parameters(TIM_HandleTypeDef *htim, TIM_TypeDef* TIM, uint16_t value);
void set_encoder_parameters(motor_num M, TIM_HandleTypeDef *htim,uint32_t TIM_CHANNEL_A, uint32_t TIM_CHANNEL_B);
void set_pid(motor_num M, float k_p, float k_i, float k_d, float low_pass);
/* DECLARATION OF FUNCTIONS USED TO CONFIGURE PARAMETERS END -------------------------------------- */

/* DECLARATION OF FUNCTIONS USED TO INTERACT WITH THE SYSTEM ITSELF BEGIN ------------------------- */
void turn_on_off_Task (void*argument);
void turn_on_off_f(uint8_t value);
void MovementControl_Task(void *argument);
void update_movement (direction dir);
void update_pwm_ind(motors_id *Motor, float value);
/* DECLARATION OF FUNCTIONS USED TO INTERACT WITH THE SYSTEM ITSELF END --------------------------- */

/* DECLARATION OF FUNCTIONS USED FOR THE PID CONTROL (NEED INTERRRUPTS) BEGIN --------------------- */
void Sampling_and_PID_Task(void *argument);
/* DECLARATION OF FUNCTIONS USED FOR THE PID CONTROL (NEED INTERRRUPTS) END ----------------------- */

#endif /* INC_DD_MOTORCONTROL_H_ */
