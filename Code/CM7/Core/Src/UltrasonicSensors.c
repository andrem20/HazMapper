/* Platform Includes --------------------------------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"

#include "stdio.h"
#include "gpio.h"
#include "string.h"
#include "stdbool.h"
/* User Includes ------------------------------------------------------------------------------------------- */
#include "UltrasonicSensors.h"
#include "usart_support.h"
#include "DD_MotorControl.h"
/* VARIABLES DECLARATION ----------------------------------------------------------------------------------- */
volatile bool isThereObstacle[NUM_DIRECTIONS] = {false};
direction last_dir = STOP;

uint8_t frame[NUMBER_OF_BYTES];
sensors sensor;

int n_bits = 0;
volatile int num_bits_frame = 0;
int discard_cnt = DISCARD_TICKS;
int discard_frame = DISCARD_FRAME;

/* STATES DECLARATIONS ------------------------------------------------------------------------------------- */
states_US state = IDLE;
states_US next_state = IDLE;

void idle_US(void);
void start_US(void);
void sync_h_US(void);
void sync_l_US(void);
void get_data_US(void);

typedef void(*func_ptr)(void);

SensorCar sensores_e_posicao[]={
		{E,	STOP,	"ND"},
		{F, STOP,	"ND"},
		{G, STOP,	"ND"},
		{H, STOP,	"ND"},
		{A, FW,		"FRENTE"},
		{B, RIGHT,	"DIREITA"},
		{C, BW,		"TRAS"},
		{D, LEFT,	"ESQUERDA"}
};

func_ptr array_states [] ={
		idle_US,
		start_US,
		sync_h_US,
		sync_l_US,
		get_data_US,
};

/* FUNCTIONS ---------------------------------------------------------------------------------------------- */
void init_US(void){
	state =  IDLE;
	next_state = IDLE;
	sensor = E;
}

void switch_state_US(void){
	//	state &= 0x03;
		array_states[state]();
		state = next_state;
}

void idle_US(void){
	if(WritePointer != ReadPointer){
		if(*ReadPointer == 1){
			n_bits ++;
			if(n_bits == (uint8_t)(START_PULSE_PERIOD_ON/SAMPLING_FREQ_US)){
				n_bits = 0;
				next_state = START;
			}else
				next_state = IDLE;
		}
		else{
			n_bits = 0;
			next_state = IDLE;
		}
		ReadPointer ++;
		ReadPointer = buffer + ((ReadPointer - buffer) & (MAX_SIZE_BUFFER - 1));
	}else
		next_state = IDLE;
}
void start_US(void){

	if(WritePointer != ReadPointer){
		if(*ReadPointer == 0){
			n_bits ++;
			if(n_bits >= ((uint8_t)(START_PULSE_PERIOD_OFF/SAMPLING_FREQ_US))-HISTERISES){
				n_bits = 0;
				discard_cnt = DISCARD_TICKS;
				next_state =SYNC_H;
			}else
				next_state = START;
		}else{
			discard_cnt --;
			n_bits = 0;
			if(!discard_cnt){
				discard_cnt = DISCARD_TICKS;
				next_state = IDLE;
			}
			else
				next_state = START;
		}
		ReadPointer ++;
		ReadPointer = buffer + ((ReadPointer - buffer) & (MAX_SIZE_BUFFER - 1));
	}else
		next_state = START;
}
void sync_h_US(void){

	if(WritePointer != ReadPointer){
		if(*ReadPointer == 1){
			n_bits ++;
			if(n_bits >= ((uint8_t)(SMALL_PULSE/SAMPLING_FREQ_US)-HISTERISES)){
				n_bits = 0;
				discard_cnt = DISCARD_TICKS;
				next_state = SYNC_L;
			}else
				next_state = SYNC_H;
		}else{
			discard_cnt --;
			n_bits = 0;
			if(!discard_cnt){
				discard_cnt = DISCARD_TICKS;
				next_state = IDLE;
			}
			else
				next_state = SYNC_H;
		}
		ReadPointer ++;
		ReadPointer = buffer + ((ReadPointer - buffer) & (MAX_SIZE_BUFFER - 1));
	}else
		next_state = SYNC_H;
}
void sync_l_US(void){
	if(WritePointer != ReadPointer){
		if(*ReadPointer == 0){
			n_bits ++;
			if(n_bits == ((uint8_t)(SMALL_PULSE/SAMPLING_FREQ_US)-HISTERISES)){
				n_bits = 0;
				next_state = FRAME;
				discard_cnt = DISCARD_TICKS;
			}else
				next_state = SYNC_L;
		}else{
			discard_cnt --;
			n_bits = 0;
			if(!discard_cnt){
				discard_cnt = DISCARD_TICKS;
				next_state = IDLE;
			}
			else
				next_state = SYNC_L;
		}
		ReadPointer ++;
		ReadPointer = buffer + ((ReadPointer - buffer) & (MAX_SIZE_BUFFER - 1));
	}else
		next_state = SYNC_L;
}

void interpret_data(void){
	SensorCar* sensor_id;
	for ( sensor_id = &sensores_e_posicao[A]; sensor_id < &sensores_e_posicao[A + NUM_SENSORS]; sensor_id++){
		//if(trama[sensor_id] < 150){
		uint8_t distance = frame[sensor_id->Sensor] * 10;
		char msg[20];
		sprintf(msg,"%s : %d\n", sensor_id->posicao, distance);
		isThereObstacle[sensor_id->dir] = (distance <= 30);

		// If as obstacle was detected in the current movement, update the movement to stop
		if(last_dir == sensor_id->dir && (distance <= 30)) {
		    MovementCommand cmd = {.dir = STOP, .force_stop = true};
		    xQueueSend(movementQueue, &cmd, 0);
		}
	print_message_to_UART((uint8_t*)msg, strlen(msg));
		//printf ("Sensor %s: %d cm\n", sensores_e_posicao[sensor_id].posicao , distance);
		//}
	}

}

void get_data_US(void){
	if(WritePointer != ReadPointer){

		if(*ReadPointer == 1){
			n_bits ++;
			next_state = FRAME;
		}else{
			discard_frame--;
			if(!discard_frame){
				n_bits = 0;
				num_bits_frame = 0;
				discard_frame = DISCARD_FRAME;
				next_state = IDLE;
			}else
				next_state = FRAME;


			if(n_bits>=(((uint8_t)(SMALL_PULSE/SAMPLING_FREQ_US))- HISTERISES) && n_bits <= (((uint8_t)(SMALL_PULSE/SAMPLING_FREQ_US))+HISTERISES)){
				n_bits = 0;
				discard_frame = DISCARD_FRAME;

				frame[sensor] &= ~(1 << (0x07 - (num_bits_frame & 0x07)));

				num_bits_frame ++;
				sensor = ((num_bits_frame) >> 3) & 0xFF;

				if(num_bits_frame == NUMBER_OF_BYTES * 8){

					num_bits_frame = 0;
					next_state = IDLE;
					interpret_data();

				}else{
					next_state = FRAME;

				}
			}else if(n_bits <= (((uint8_t)(BIG_PULSE/SAMPLING_FREQ_US))+HISTERISES) && n_bits >= (((uint8_t)(BIG_PULSE/SAMPLING_FREQ_US))- HISTERISES)){
				n_bits = 0;
				discard_frame = DISCARD_FRAME;


				frame[sensor] |= 1 << (0x07 - (num_bits_frame & 0x07));

				num_bits_frame ++;
				sensor = ((num_bits_frame) >> 3) & 0xFF;
				if(num_bits_frame == NUMBER_OF_BYTES * 8){
					num_bits_frame = 0;
					next_state = IDLE;
					interpret_data();
				}else{
					next_state = FRAME;

				}
			}

		}

		ReadPointer ++;
		ReadPointer = buffer + ((ReadPointer - buffer) & (MAX_SIZE_BUFFER - 1));
	}else
		next_state = FRAME;
}

/* TASKS ---------------------------------------------------------------------------------- */
void ParkingSensors_Task (void *argument){
	HAL_TIM_Base_Start_IT(&htim17);	// Sampling for Sensors
	for(;;)
		switch_state_US();
}

