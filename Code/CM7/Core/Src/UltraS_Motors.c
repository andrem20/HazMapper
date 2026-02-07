/* Platform Includes ------------------------------------------------------------------------------------------- */
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"

#include "stdbool.h"
/* User Includes ----------------------------------------------------------------------------------------------- */
#include "UltrasonicSensors.h"
#include "DD_MotorControl.h"
/* VARIABLES DECLARATION ----------------------------------------------------------------------------------- */
extern bool isThereObstacle[NUM_DIRECTIONS];
extern direction last_dir;

/*
 * void update_movement (direction dir)
 * 	- Updates motor state based on the desired movement direction
 * 	- Calls function update_pwm_ind to update the pwm state
 */
void update_movement (direction dir){

// Check if there is an obstacle in that direction. If so, change dir to STOP, disabling any movement in that direction

	if(isThereObstacle[dir])
		dir = STOP;


	last_dir = dir;
	motors[M1].state = cmd_list[dir].M_1;
	motors[M2].state = cmd_list[dir].M_2;
	motors[M3].state = cmd_list[dir].M_3;
	motors[M4].state = cmd_list[dir].M_4;
	for (int i = 0; i <4; i++)
		update_pwm_ind(&motors[i], motors[i].var.PWM_value);
}
