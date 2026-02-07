/* Platform Includes BEGIN ---------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "queue.h"

#include "stdio.h"
#include "string.h"
#include "math.h"
#include "usart.h"
/* Platform Includes END ------------------------------------------------------------------ */

/* User Includes BEGIN -------------------------------------------------------------------- */
#include "DD_MotorControl.h"
#include "usart_support.h"
#include "battery.h"
#include "esp_driver.h"
/* User Includes END ---------------------------------------------------------------------- */

/*
 * 	 MOTORS PLACEMENT
 * 		M1	M2		(FRONT)
 * 		M3	M4		(BACK)
 *
 * 		OUT1 -> OUT2 	Forward Drive
 * 		OUT4 -> OUT3
 */


/* DEFINE TIMERS AND TIMERS' RELATED VARIABLES BEGIN -------------------------------------- */
uint16_t SAMPLING_PERIOD = 1;		/* Sampling Period in ms ------------------------------ */
int16_t PULSES = 1;				/* number of PPR in each encoder's channel ---------------- */
uint16_t PWM_FREQ = 1;				/* PWM's best frequency ------------------------------- */
uint32_t ARR_value = 0;
/* DEFINE TIMERS AND TIMERS' RELATED VARIABLES END ---------------------------------------- */

/* DEFINE PID CONTROLLER RELATED VARIABLES BEGIN ------------------------------------------ */
volatile float setpoint = 0;	/* Desired Speed on Every Motor */
volatile int16_t encoder_current_position = 0, encoder_last_position = 0, pulse_diff = 0, encoder_position = 0;
volatile float y = 0, integral = 0, erro_previous = 0;
/* DEFINE PID CONTROLLER RELATED VARIABLES END -------------------------------------------- */

/* OTHER VARIABLES DEFINITION BEGIN ------------------------------------------------------- */
uint8_t ON_OFF_value = 0;
SPEED_MODE speed_mode = SLOW;
/* OTHER VARIABLES DEFINITION END --------------------------------------------------------- */

/* ALLOWED COMMANDS AND RESPECTIVE MOTORS' STATE BEGIN ------------------------------------ */
commands cmd_list [] = {
		//		COMMAND		MOTOR 1		MOTOR 2		MOTOR 3		MOTOR 4
		{FW, 		FORWARD,	FORWARD,	FORWARD,	FORWARD	},
		{FW_RIGHT,	OFF,		FORWARD,	FORWARD,	OFF		},
		{RIGHT,		REVERSE,	FORWARD, 	FORWARD, 	REVERSE	},
		{BW_RIGHT,	REVERSE,	OFF,		OFF,	    REVERSE },
		{BW,		REVERSE,	REVERSE,	REVERSE,	REVERSE	},
		{BW_LEFT,	OFF,		REVERSE,	REVERSE,	OFF		},
		{LEFT,		FORWARD,	REVERSE,	REVERSE,	FORWARD	},
		{FW_LEFT,	FORWARD,	OFF,		OFF,		FORWARD },
		{STOP,		OFF,		OFF,		OFF,		OFF},
		{R_RIGHT,	FORWARD,	REVERSE,	FORWARD,	REVERSE},
		{R_LEFT, 	REVERSE,	FORWARD,	REVERSE,	FORWARD}
};
/* ALLOWED COMMANDS AND RESPECTIVE MOTORS' STATE BEGIN ------------------------------------ */

/* --------------------------------------------------------------------------------------------------------------------------- */

/*
 * void set_On_Off (GPIO_TypeDef *port, uint16_t pin)
 * 	- Sets the port and pin related to system's On/Off
 */
void set_On_Off (GPIO_TypeDef *port, uint16_t pin){
	ON_OFF_Port = port;
	ON_OFF_Pin = pin;

	for(int i = 0; i < 4; i++)
		motors[i].state = OFF;
}

/*
 * void set_Ports (motor_num M, GPIO_TypeDef *forward, GPIO_TypeDef *reverse)
 * 	- Sets the ports related to each motor
 */
void set_Ports (motor_num M, GPIO_TypeDef *forward, GPIO_TypeDef *reverse){
	motors_id *Motor = &motors[(M-1)&0x03];
	Motor->FORWARD_DRIVE.GPIO_Port = forward;
	Motor->REVERSE_DRIVE.GPIO_Port = reverse;
}

/*
 * void set_Pins (motor_num M, uint16_t forward, uint16_t reverse)
 * 	- Sets the pins related to each motor
 */
void set_Pins (motor_num M, uint16_t forward, uint16_t reverse){
	motors_id *Motor = &motors[(M-1)&0x03];
	Motor->FORWARD_DRIVE.Pin = forward;
	Motor->REVERSE_DRIVE.Pin = reverse;
}

/*
 * int set_PWM_parameters(motor_num M,TIM_HandleTypeDef *htim, uint32_t TIM_Channel, uint16_t ppr, uint16_t freq)
 * 	- Sets the PWM related parameters on each motor
 */
void set_PWM_parameters (motor_num M,TIM_HandleTypeDef *htim, uint32_t TIM_Channel, uint16_t ppr, uint16_t freq){
	motors_id *Motor = &motors[(M-1)&0x03];
	Motor->pTim.htim_pwm = htim;
	Motor->pTim.HTIM_PWM_CHANNEL=TIM_Channel;
	PULSES = ppr;
	PWM_FREQ = freq;
	ARR_value = ((SystemCoreClock/(Motor->pTim.htim_pwm->Init.Prescaler + 1))/((int)PWM_FREQ))-1;  //equivale ao número de ticks
	__HAL_TIM_SET_AUTORELOAD(Motor->pTim.htim_pwm, ARR_value);

	HAL_TIM_PWM_Init(Motor->pTim.htim_pwm);
}

/*
 * int set_sampling_period(void)
 * 	- Sets the sampling period desired by the user on the register
 */
void set_sampling_parameters(TIM_HandleTypeDef *htim, TIM_TypeDef* TIM, uint16_t value)
{
	htim_sampling = htim;
	TIM_sampling = TIM;
	SAMPLING_PERIOD = value;

	/*f_timer = SYS / ( (PSC+1)*(ARR+1) )*/
	uint32_t Sampling_Register = (uint32_t)(((SAMPLING_PERIOD * ms * SystemCoreClock) / (htim_sampling->Init.Period + 1)) - 1);
	htim_sampling->Init.Prescaler = Sampling_Register;
	HAL_TIM_Base_Init(htim_sampling);
}

/*
 * void set_encoder_parameters(motor_num M, TIM_HandleTypeDef *htim, uint32_t TIM_CHANNEL_A, uint32_t TIM_CHANNEL_B)
 * 	- Sets encoder parameters on each motor
 */
void set_encoder_parameters(motor_num M, TIM_HandleTypeDef *htim, uint32_t TIM_CHANNEL_A, uint32_t TIM_CHANNEL_B)
{
	motors_id *Motor = &motors[(M-1)&0x03];
	Motor->eTim.htim_encoder = htim;
	Motor->eTim.SENSOR_A_CHANNEL = TIM_CHANNEL_A;
	Motor->eTim.SENSOR_B_CHANNEL = TIM_CHANNEL_B;
	HAL_TIM_Encoder_Start(Motor->eTim.htim_encoder, Motor->eTim.SENSOR_A_CHANNEL | Motor->eTim.SENSOR_B_CHANNEL);
}

/*
 * void set_pid(void* p_argument)
 * 	- Sets the PID controller parameters based on the users input
 */
void set_pid(motor_num M, float k_p, float k_i, float k_d, float low_pass)
{
	motors_id *Motor = &motors[(M-1)&0x03];

	Motor->PID.Kp = k_p;
	Motor->PID.Ki = k_i;
	Motor->PID.a = low_pass;
	Motor->PID.Kd_h = (k_d * (1-Motor->PID.a))/(SAMPLING_PERIOD * ms);
}


void turn_on_off_f (uint8_t value)
{
	ON_OFF_value = value;
	HAL_GPIO_WritePin(ON_OFF_Port, ON_OFF_Pin, ON_OFF_value); // Visual Signal

	if (ON_OFF_value){
		for(int i = 0; i < 4; i++){
			HAL_TIM_PWM_Start(motors[i].pTim.htim_pwm, motors[i].pTim.HTIM_PWM_CHANNEL);

		}
		HAL_TIM_Base_Start_IT(htim_sampling);
	}
	else{
		for (int i = 0; i < 4; i++){	// TURN OFF motors
			motors[i].state = OFF;
			update_pwm_ind(&motors[i], motors[i].var.PWM_value);
			HAL_TIM_PWM_Stop(motors[i].pTim.htim_pwm, motors[i].pTim.HTIM_PWM_CHANNEL);
		}
		HAL_TIM_Base_Stop_IT(htim_sampling);
	}
}


/*
 * void update_pwm_pid (motors_id *Motor, float value)
 * 	- Sets the drive direction on EACH motor individually, since it is called for a specific motor
 * 	- Updates its individual PWM value and State
 * 	- Must receive a value between 0 - 100 for P
 */

void update_pwm_ind(motors_id *Motor, float value)
{

	Motor->var.PWM_value = value;
	//PWM_FREQ = (45 * PWM_value) - 350;
	//ARR_value = ((SystemCoreClock/(motors[i].pTim.htim_pwm->Init.Prescaler + 1))/((int)PWM_FREQ))-1;

	switch (Motor->state){
	case OFF:
		// Default configuration
		Motor->var.PWM_value = 0;
		HAL_GPIO_WritePin(Motor->FORWARD_DRIVE.GPIO_Port, Motor->FORWARD_DRIVE.Pin, RESET);
		HAL_GPIO_WritePin(Motor->REVERSE_DRIVE.GPIO_Port, Motor->REVERSE_DRIVE.Pin, RESET);
		break;
	case FORWARD:
		HAL_GPIO_WritePin(Motor->REVERSE_DRIVE.GPIO_Port, Motor->REVERSE_DRIVE.Pin, RESET);
		HAL_GPIO_WritePin(Motor->FORWARD_DRIVE.GPIO_Port, Motor->FORWARD_DRIVE.Pin, SET);
		break;
	case REVERSE:
		HAL_GPIO_WritePin(Motor->FORWARD_DRIVE.GPIO_Port, Motor->FORWARD_DRIVE.Pin, RESET);
		HAL_GPIO_WritePin(Motor->REVERSE_DRIVE.GPIO_Port, Motor->REVERSE_DRIVE.Pin, SET);
		break;
	default:
		break;
	}

	// PWM register will be the CCR register
	uint16_t PWM_register = ((ARR_value + 1) * (Motor->var.PWM_value)/100);

	__HAL_TIM_SET_COMPARE(Motor->pTim.htim_pwm,Motor->pTim.HTIM_PWM_CHANNEL, PWM_register);

}


/*
 * void Read_Encoder (motors_id *Motor)
 * 	- Reads Encoders' waves in order to get the position and speed data from each one
 */
void Read_Encoder (motors_id *Motor){
	Motor->var.encoder_current_position = __HAL_TIM_GET_COUNTER(Motor->eTim.htim_encoder);

	Motor->var.pulse_diff = Motor->var.encoder_current_position - Motor->var.encoder_last_position;

	// Correção de overflow do contador --> deu uma volta ou mais
	if ( Motor->var.pulse_diff > 32767)
		Motor->var.pulse_diff -= 65536;
	else if ( Motor->var.pulse_diff < -32768)
		Motor->var.pulse_diff += 65536;

	Motor->var.encoder_last_position = Motor->var.encoder_current_position;
	Motor->var.encoder_position = Motor->var.encoder_current_position;

	Motor->var.position_degrees = (Motor->var.encoder_position * 36000.0f) / (PULSES * 4);
	Motor->var.position_rad = (Motor->var.encoder_position * 200.0f * M_PI) / (PULSES * 4);
	Motor->var.speed_rad = (Motor->var.pulse_diff * 200.0f * M_PI) / (PULSES * 4);
	Motor->var.speed_rpm = (Motor->var.pulse_diff * 6000.0f) / (PULSES * 4.0f);
	//printf("Pos: %d\tVel: %.2f RPM\r\n", Motor->var.encoder_position, Motor->var.speed_rpm);
}

/*
 * void ISR_PID_speed(void)
 * 	- Called on every htim_sampling OV
 * 	- Digital PID modified speed algorithm
 */
void ISR_PID_speed(motors_id *Motor){
	y = fabs(Motor->var.speed_rad);
	float error = setpoint - y;
	Motor->var.integral += error * SAMPLING_PERIOD * ms;
	float derivative = (error - Motor->var.erro_previous) / SAMPLING_PERIOD * ms;

	float output = Motor->PID.Kp * error + Motor->PID.Ki * Motor->var.integral + Motor->PID.Kd_h * derivative;

	if (output > (V_BATTERY*speed_mode*0.01)) output = (V_BATTERY*speed_mode*0.01);
	if (output < 0) output = 0;
	float duty_cycle = (output/V_BATTERY)*100;
	update_pwm_ind(Motor, duty_cycle);
	Motor->var.erro_previous = error;

}
/* TASKS ---------------------------------------------------------------------------------- */
void Sampling_and_PID_Task(void *argument){

	for(;;){
		/* Waits Notification in Blocked State */
		ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

		for(int i = 0; i < 4; i++){
			Read_Encoder(&motors[i]);
			ISR_PID_speed(&motors[i]);
		}

	}
}

/*
 * int turn_on_off (uint8_t value)
 * 	- Sets ON or OFF the motors <=> ON or OFF Robot
 */
void turn_on_off_Task (void *argument)
{
	for(;;){

		/* Waits Notification/ Data in Blocked State */
		xQueueReceive(on_off_Queue, &ON_OFF_value, portMAX_DELAY);

		HAL_GPIO_WritePin(ON_OFF_Port, ON_OFF_Pin, ON_OFF_value); // Visual Signal

		if (ON_OFF_value){
			for(int i = 0; i < 4; i++){
				HAL_TIM_PWM_Start(motors[i].pTim.htim_pwm, motors[i].pTim.HTIM_PWM_CHANNEL);

			}
			HAL_TIM_Base_Start_IT(htim_sampling);
		}
		else{
			for (int i = 0; i < 4; i++){	// TURN OFF motors
				motors[i].state = OFF;
				update_pwm_ind(&motors[i], motors[i].var.PWM_value);
				HAL_TIM_PWM_Stop(motors[i].pTim.htim_pwm, motors[i].pTim.HTIM_PWM_CHANNEL);
			}
			HAL_TIM_Base_Stop_IT(htim_sampling);
		}

		vTaskDelay(pdMS_TO_TICKS(10));
	}
}


void MovementControl_Task(void *argument){
	MovementCommand cmd;
	for(;;){
		if (xQueueReceive(movementQueue, &cmd, portMAX_DELAY) == pdPASS)
			update_movement(cmd.dir);
	}
}
