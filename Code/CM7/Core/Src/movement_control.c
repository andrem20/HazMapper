/* Platform Includes BEGIN ----------------------------------------------------------------- */
#include "string.h"
#include "stdio.h"
/* Platform Includes END ------------------------------------------------------------------- */

/* User Includes BEGIN --------------------------------------------------------------------- */
#include "movement_control.h"
#include "DD_MotorControl.h"
#include "usart_support.h"
/* User Includes END ----------------------------------------------------------------------- */

/* MACRO definition BEGIN ------------------------------------------------------------------ */
#define NUM_MOVES 9
#define MAX_SPEED_RAD_S 34.87
/* MACRO definition END -------------------------------------------------------------------- */

#ifdef USE_PC_CONTROL
// Used for keyboard control ONLY
const char* valid_movements [NUM_MOVES] = {"W", "E", "D", "C", "X", "Z", "A", "Q", "S"};
const float valid_pwm [] = {0, 10, 20, 30, 40, 50, 60, 70, 80, 90, 100};
#endif

#ifdef USE_PC_CONTROL
/*
 * static void update_direction(char* input)
 * 	- updates direction based on input string
 * 	- Used only for keyboard input - Wired Connection PC <-> STM
 */

static void update_direction(char* input){
	for (int i = 0; i < NUM_MOVES; i++){
		if(strcmp(input, valid_movements[i]) == 0){
			direction command = i;
			printf("Movement updated\r\n");
			update_movement(cmd_list[command].dir);
			return;
		}
	}
	printf("Invalid direction: %s\n", input);
}

/*
 * void move (char* update)
 * 	- decides if speed or direction is getting an update depending
 *		 on the input of the keyboard
 */
void move (char* update){
	if (!ON_OFF_value) return;

	int val = update[0] - '0';
	if (val >= 17)
		update_direction(update);
	else if (val >= 0 && val < 11)
		update_speed(val);
	else
		printf("Invalid\n");
}
#endif
/*
 * void update_speed(char* speed)
 * 	- updates speed (setpoint - reference variable in PID) based on input value
 */
void update_speed(int speed){
	 setpoint = MAX_SPEED_RAD_S * speed / 100;
	 //setpoint = MAX_SPEED_RAD_S * speed_driver / 100
}





